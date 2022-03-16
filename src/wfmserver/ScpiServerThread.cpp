/***********************************************************************************************************************
*                                                                                                                      *
* wfmserver                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg
	@brief SCPI server. Control plane traffic only, no waveform data.

	SCPI commands supported:

		*IDN?
			Returns a standard SCPI instrument identification string
 */

#include "wfmserver.h"
#include <string.h>
#include <math.h>
#include "../../lib/scpi-server-tools/SCPIServer.h"

//These functions are not yet in the Digilent API headers (will be in the next release)
//Prototypes from email conversation with Attila at Digilent
//TODO: delete this once the next version of the SDK is released
typedef int DwfAnalogCoupling;
const DwfAnalogCoupling DwfAnalogCouplingDC = 0;
const DwfAnalogCoupling DwfAnalogCouplingAC = 1;
DWFAPI int FDwfAnalogInChannelCouplingInfo(HDWF hdwf, int *pfscoupling); // use IsBitSet
DWFAPI int FDwfAnalogInChannelCouplingSet(HDWF hdwf, int idxChannel, DwfAnalogCoupling coupling);
DWFAPI int FDwfAnalogInChannelCouplingGet(HDWF hdwf, int idxChannel, DwfAnalogCoupling *pcoupling);

using namespace std;

//Channel state
map<size_t, bool> g_channelOn;
size_t g_memDepth = 1000000;
int64_t g_sampleInterval = 0;	//in fs

//Copy of state at timestamp of last arm event
map<size_t, bool> g_channelOnDuringArm;
int64_t g_sampleIntervalDuringArm = 0;
size_t g_captureMemDepth = 0;

bool g_triggerArmed = false;
bool g_triggerOneShot = false;
bool g_memDepthChanged = false;

//Trigger state (for now, only simple edge trigger supported)
double g_triggerVoltage = 0;
size_t g_triggerChannel = 0;
size_t g_triggerSampleIndex;
int64_t g_triggerDelay;
double g_triggerDeltaSec;

/*
//Thresholds for MSO pods
size_t g_numDigitalPods = 2;
int16_t g_msoPodThreshold[2][8] = { {0}, {0} };
PICO_DIGITAL_PORT_HYSTERESIS g_msoHysteresis[2] = {PICO_NORMAL_100MV, PICO_NORMAL_100MV};
bool g_msoPodEnabled[2] = {false};
bool g_msoPodEnabledDuringArm[2] = {false};

bool EnableMsoPod(size_t npod);

bool g_lastTriggerWasForced = false;
*/
std::mutex g_mutex;

/**
	@brief Main socket server
 */
void ScpiServerThread()
{
	#ifdef __linux__
	pthread_setname_np(pthread_self(), "ScpiThread");
	#endif

	while(true)
	{
		Socket scpiClient = g_scpiSocket.Accept();
		if(!scpiClient.IsValid())
			break;

		//Create a server object for this connection
		SCPIServer server(scpiClient.Detach());

		//Reset the device to default configuration
		if(!FDwfAnalogInReset(g_hScope))
		{
			LogError("FDwfAnalogInReset failed\n");
			exit(1);
		}

		thread dataThread(WaveformServerThread);

		//Main command loop
		string line;
		string cmd;
		bool query;
		string subject;
		vector<string> args;
		while(true)
		{
			if(!server.RecvCommand(line))
				break;
			LogTrace((line + "\n").c_str());
			server.ParseLine(line, subject, cmd, query, args);

			//Extract channel ID from subject and clamp bounds
			size_t channelId = 0;
			//size_t laneId = 0;
			//bool channelIsDigital = false;
			if(toupper(subject[0]) == 'C')
			{
				channelId = min(static_cast<size_t>(stoi(subject.c_str() + 1) - 1), g_numAnalogInChannels);
				//channelIsDigital = false;
			}
			/*
			else if(isdigit(subject[0]))
			{
				channelId = min(subject[0] - '0', 2) - 1;
				channelIsDigital = true;
				if(subject.length() >= 3)
					laneId = min(subject[2] - '0', 7);
			}
			*/

			if(query)
			{

				//Read ID code
				if(cmd == "*IDN")
					server.SendReply(string("Digilent,") + g_model + "," + g_serial + "," + g_fwver);

				//Get number of channels
				else if(cmd == "CHANS")
					server.SendReply(to_string(g_numAnalogInChannels));

				//Get legal sample rates for the current configuration
				else if(cmd == "RATES")
				{
					double minFreq;
					double maxFreq;
					if(!FDwfAnalogInFrequencyInfo(g_hScope, &minFreq, &maxFreq))
						LogError("FDwfAnalogInFrequencyInfo failed\n");

					//Cap min freq to 1 kHz
					minFreq = max(minFreq, 1000.0);

					//Report sample rates in 1-2-5 steps
					string ret = "";
					double freq = maxFreq;
					while(freq >= minFreq)
					{
						double f1 = freq;
						double f2 = freq / 2;
						double f3 = freq / 5;
						freq /= 10;

						double interval1 = FS_PER_SECOND / f1;
						double interval2 = FS_PER_SECOND / f2;
						double interval3 = FS_PER_SECOND / f3;

						ret += to_string(interval1) + ",";
						ret += to_string(interval2) + ",";
						ret += to_string(interval3) + ",";
					}

					server.SendReply(ret);
				}

				//Get memory depths
				else if(cmd == "DEPTHS")
				{
					int bufsizeMin;
					int bufsizeMax;
					if(!FDwfAnalogInBufferSizeInfo(g_hScope, &bufsizeMin, &bufsizeMax))
						LogError("FDwfAnalogInBufferSizeInfo failed\n");

					//for now, only report max memory depth
					string ret = to_string(bufsizeMax) + ",";

					server.SendReply(ret);
				}

				else
					LogDebug("Unrecognized query received: %s\n", line.c_str());
			}

			else if(cmd == "EXIT")
				break;

			else if(cmd == "ON")
			{
				lock_guard<mutex> lock(g_mutex);
				g_channelOn[channelId] = true;

				if(!FDwfAnalogInChannelEnableSet(g_hScope, channelId, true))
					LogError("FDwfAnalogInChannelEnableSet failed\n");

				//We need to allocate new buffers for this channel
				g_memDepthChanged = true;

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}
			else if(cmd == "OFF")
			{
				lock_guard<mutex> lock(g_mutex);
				g_channelOn[channelId] = true;

				if(!FDwfAnalogInChannelEnableSet(g_hScope, channelId, false))
					LogError("FDwfAnalogInChannelEnableSet failed\n");

				//We need to allocate new buffers for this channel
				g_memDepthChanged = true;

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}
			else if( (cmd == "COUP") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				DwfAnalogCoupling coup;
				if(args[0] == "DC1M")
					coup = DwfAnalogCouplingDC;
				else// if(args[0] == "AC1M")
					coup = DwfAnalogCouplingAC;

				if(!FDwfAnalogInChannelCouplingSet(g_hScope, channelId, coup))
					LogError("FDwfAnalogInChannelCouplingSet failed\n");
			}

			else if( (cmd == "OFFS") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				double requestedOffset = stod(args[0]);
				if(!FDwfAnalogInChannelOffsetSet(g_hScope, channelId, requestedOffset))
					LogError("FDwfAnalogInChannelOffsetSet failed\n");

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "ATTEN") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				double requestedAtten = stod(args[0]);
				if(!FDwfAnalogInChannelAttenuationSet(g_hScope, channelId, requestedAtten))
					LogError("FDwfAnalogInChannelAttenuationSet failed\n");

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "RANGE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				auto range = stod(args[0]);

				if(!FDwfAnalogInChannelRangeSet(g_hScope, channelId, range))
					LogError("FDwfAnalogInChannelRangeSet failed\n");

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "RATE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				int64_t rate = stoull(args[0]);
				if(!FDwfAnalogInFrequencySet(g_hScope, rate))
					LogError("FDwfAnalogInFrequencySet failed\n");
				g_sampleInterval = FS_PER_SECOND / rate;

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "DEPTH") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				g_memDepth = stoull(args[0]);
				if(!FDwfAnalogInBufferSizeSet(g_hScope, g_memDepth))
					LogError("FDwfAnalogInBufferSizeSet failed\n");

				g_memDepthChanged = true;

				//need to re-arm trigger to apply changes
				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "START") || (cmd == "SINGLE") )
			{
				lock_guard<mutex> lock(g_mutex);

				if(g_triggerArmed)
				{
					LogVerbose("Ignoring START command because trigger is already armed\n");
					continue;
				}

				//Make sure we've got something to capture
				bool anyChannels = false;
				for(size_t i=0; i<g_numAnalogInChannels; i++)
				{
					if(g_channelOn[i])
					{
						anyChannels = true;
						break;
					}
				}
				/*
				for(size_t i=0; i<g_numDigitalPods; i++)
				{
					if(g_msoPodEnabled[i])
					{
						anyChannels = true;
						break;
					}
				}
				*/

				if(!anyChannels)
				{
					LogVerbose("Ignoring START command because no channels are active\n");
					continue;
				}

				//Start the capture
				Start();
				g_triggerOneShot = (cmd == "SINGLE");
			}

			else if(cmd == "FORCE")
			{
				lock_guard<mutex> lock(g_mutex);
				Start(true);
			}

			else if(cmd == "STOP")
			{
				lock_guard<mutex> lock(g_mutex);

				Stop();
			}

			else if(subject == "TRIG")
			{
				if( (cmd == "MODE") && (args.size() == 1) )
				{
					if(args[0] == "EDGE")
					{
						if(!FDwfAnalogInTriggerTypeSet(g_hScope, trigtypeEdge))
							LogError("FDwfAnalogInTriggerTypeSet failed\n");
					}

					else
						LogWarning("Unknown trigger mode %s\n", args[0].c_str());
				}

				else if( (cmd == "EDGE:DIR") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					DwfTriggerSlope condition;
					if(args[0] == "RISING")
						condition = DwfTriggerSlopeRise;
					else if(args[0] == "FALLING")
						condition = DwfTriggerSlopeFall;
					else// if(args[0] == "ANY")
						condition = DwfTriggerSlopeEither;

					if(!FDwfAnalogInTriggerConditionSet(g_hScope, condition))
						LogError("FDwfAnalogInTriggerConditionSet failed\n");

					//need to re-arm trigger to apply changes
					if(g_triggerArmed)
						Start();
				}

				else if( (cmd == "LEV") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					g_triggerVoltage = stod(args[0]);

					if(!FDwfAnalogInTriggerLevelSet(g_hScope, g_triggerVoltage))
						LogError("FDwfAnalogInTriggerLevelSet failed\n");

					//need to re-arm trigger to apply changes
					if(g_triggerArmed)
						Start();
				}

				else if( (cmd == "SOU") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					if(!FDwfAnalogInTriggerSourceSet(g_hScope, trigsrcDetectorAnalogIn))
						LogError("FDwfAnalogInTriggerSourceSet failed\n");

					if(!FDwfAnalogInTriggerAutoTimeoutSet(g_hScope, 0))
						LogError("FDwfAnalogInTriggerAutoTimeoutSet failed\n");

					g_triggerChannel = args[0][1] - '1';
					if(!FDwfAnalogInTriggerChannelSet(g_hScope, g_triggerChannel))
						LogError("FDwfAnalogInTriggerChannelSet failed\n");

					//need to re-arm trigger to apply changes
					if(g_triggerArmed)
						Start();
				}

				else if( (cmd == "DELAY") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);
					g_triggerDelay = stoull(args[0]);

					//For single trigger mode, trigger position is WRT midpoint of buffer
					//but the TRIG:DELAY command measures WRT start of buffer.
					int64_t offset_samples = g_memDepth/2;
					int64_t offset_fs = offset_samples * g_sampleInterval;
					int64_t position_fs = offset_fs - g_triggerDelay;

					//After setting trigger time, see what we actually got.
					//Hardware may round it.
					double position_sec_requested = position_fs * SECONDS_PER_FS;
					if(!FDwfAnalogInTriggerPositionSet(g_hScope, position_sec_requested))
						LogError("FDwfAnalogInTriggerPositionSet failed\n");
					double position_sec_actual;
					if(!FDwfAnalogInTriggerPositionGet(g_hScope, &position_sec_actual))
						LogError("FDwfAnalogInTriggerPositionGet failed\n");

					g_triggerDeltaSec = position_sec_actual - position_sec_requested;

					//need to re-arm trigger to apply changes
					if(g_triggerArmed)
						Start();
				}

				else
				{
					LogDebug("Unrecognized trigger command received: %s\n", line.c_str());
					LogIndenter li;
					LogDebug("Command: %s\n", cmd.c_str());
					for(auto arg : args)
						LogDebug("Arg: %s\n", arg.c_str());
				}
			}

			//TODO: bandwidth limiter

			//Unknown
			else
			{
				LogDebug("Unrecognized command received: %s\n", line.c_str());
				LogIndenter li;
				LogDebug("Subject: %s\n", subject.c_str());
				LogDebug("Command: %s\n", cmd.c_str());
				for(auto arg : args)
					LogDebug("Arg: %s\n", arg.c_str());
			}
		}

		//Reset the device to default configuration
		FDwfAnalogInReset(g_hScope);

		LogVerbose("Client disconnected\n");

		g_waveformThreadQuit = true;
		dataThread.join();
		g_waveformThreadQuit = false;
	}
}

void Stop()
{
	FDwfAnalogInConfigure(g_hScope, true, false);
	g_triggerArmed = false;
}

void Start(bool force)
{
	//Save configuration
	g_captureMemDepth = g_memDepth;
	g_channelOnDuringArm = g_channelOn;
	g_sampleIntervalDuringArm = g_sampleInterval;
	/*
	for(size_t i=0; i<g_numDigitalPods; i++)
		g_msoPodEnabledDuringArm[i] = g_msoPodEnabled[i];
	*/

	//Precalculate some stuff we need for trigger interpolation
	g_triggerSampleIndex = g_triggerDelay / g_sampleInterval;

	//Set acquisition mode
	FDwfAnalogInAcquisitionModeSet(g_hScope, acqmodeSingle);

	//Start acquisition
	FDwfAnalogInConfigure(g_hScope, true, true);

	g_triggerArmed = true;
}
