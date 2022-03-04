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

using namespace std;

bool ScpiSend(Socket& sock, const string& cmd);
bool ScpiRecv(Socket& sock, string& str);
void ParseScpiLine(const string& line, string& subject, string& cmd, bool& query, vector<string>& args);

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

/*
//Trigger state (for now, only simple single-channel trigger supported)
int64_t g_triggerDelay = 0;
PICO_THRESHOLD_DIRECTION g_triggerDirection = PICO_RISING;
float g_triggerVoltage = 0;
size_t g_triggerChannel = 0;
size_t g_triggerSampleIndex;

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
	@brief Sends a SCPI reply (terminated by newline)
 */
bool ScpiSend(Socket& sock, const string& cmd)
{
	string tempbuf = cmd + "\n";
	return sock.SendLooped((unsigned char*)tempbuf.c_str(), tempbuf.length());
}

/**
	@brief Reads a SCPI command (terminated by newline or semicolon)
 */
bool ScpiRecv(Socket& sock, string& str)
{
	int sockid = sock;

	char tmp = ' ';
	str = "";
	while(true)
	{
		if(1 != recv(sockid, &tmp, 1, MSG_WAITALL))
			return false;

		if( (tmp == '\n') || ( (tmp == ';') ) )
			break;
		else
			str += tmp;
	}

	return true;
}

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
		Socket client = g_scpiSocket.Accept();
		Socket dataClient(-1);
		LogVerbose("Client connected to control plane socket\n");

		if(!client.IsValid())
			break;
		if(!client.DisableNagle())
			LogWarning("Failed to disable Nagle on socket, performance may be poor\n");

		//Reset the device to default configuration
		FDwfAnalogInReset(g_hScope);

		thread dataThread(WaveformServerThread);

		//Main command loop
		string line;
		string cmd;
		bool query;
		string subject;
		vector<string> args;
		while(true)
		{
			if(!ScpiRecv(client, line))
				break;
			ParseScpiLine(line, subject, cmd, query, args);
			LogTrace((line + "\n").c_str());

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
					ScpiSend(client, string("Digilent,") + g_model + "," + g_serial + "," + g_fwver);

				//Get number of channels
				else if(cmd == "CHANS")
					ScpiSend(client, to_string(g_numAnalogInChannels));

				//Get legal sample rates for the current configuration
				else if(cmd == "RATES")
				{
					double minFreq;
					double maxFreq;
					FDwfAnalogInFrequencyInfo(g_hScope, &minFreq, &maxFreq);

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

					ScpiSend(client, ret);
				}

				//Get memory depths
				else if(cmd == "DEPTHS")
				{
					string ret = "";
					ret = "65536,";			//for now, only 64K supported
					ScpiSend(client, ret);
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

				if(g_triggerArmed)
					Start();
			}
			/*
			else if( (cmd == "COUP") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				if(args[0] == "DC1M")
					g_coupling[channelId] = PICO_DC;
				else if(args[0] == "AC1M")
					g_coupling[channelId] = PICO_AC;
				else if(args[0] == "DC50")
					g_coupling[channelId] = PICO_DC_50OHM;

				UpdateChannel(channelId);
			}
			*/
			else if( (cmd == "OFFS") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				double requestedOffset = stod(args[0]);
				if(!FDwfAnalogInChannelOffsetSet(g_hScope, channelId, requestedOffset))
					LogError("FDwfAnalogInChannelOffsetSet failed\n");

				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "RANGE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				auto range = stod(args[0]);

				if(!FDwfAnalogInChannelRangeSet(g_hScope, channelId, range))
					LogError("FDwfAnalogInChannelRangeSet failed\n");

				/*
				//Update trigger if this is the trigger channel.
				//Trigger is digital and threshold is specified in ADC counts.
				//We want to maintain constant trigger level in volts, not ADC counts.
				if(g_triggerChannel == channelId)
					UpdateTrigger();
				*/

				if(g_triggerArmed)
					Start();
			}

			else if( (cmd == "RATE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				int64_t rate = stoull(args[0]);
				if(!FDwfAnalogInFrequencySet(g_hScope, rate))
					LogError("FDwfAnalogInFrequencySet failed\n");
				g_sampleInterval = 1e15 / rate;

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
				/*
				lock_guard<mutex> lock(g_mutex);

				Stop();
				g_triggerArmed = false;
				*/
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

					if(g_triggerArmed)
						Start();
				}

				else if( (cmd == "LEV") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					if(!FDwfAnalogInTriggerLevelSet(g_hScope, stod(args[0])))
						LogError("FDwfAnalogInTriggerLevelSet failed\n");

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

					if(!FDwfAnalogInTriggerChannelSet(g_hScope, args[0][1] - '1'))
						LogError("FDwfAnalogInTriggerChannelSet failed\n");

					if(g_triggerArmed)
						Start();
				}

				/*
				else if( (cmd == "DELAY") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					g_triggerDelay = stoull(args[0]);
					UpdateTrigger();
				}
				*/
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

/**
	@brief Parses an incoming SCPI command
 */
void ParseScpiLine(const string& line, string& subject, string& cmd, bool& query, vector<string>& args)
{
	//Reset fields
	query = false;
	subject = "";
	cmd = "";
	args.clear();

	string tmp;
	bool reading_cmd = true;
	for(size_t i=0; i<line.length(); i++)
	{
		//If there's no colon in the command, the first block is the command.
		//If there is one, the first block is the subject and the second is the command.
		//If more than one, treat it as freeform text in the command.
		if( (line[i] == ':') && subject.empty() )
		{
			subject = tmp;
			tmp = "";
			continue;
		}

		//Detect queries
		if(line[i] == '?')
		{
			query = true;
			continue;
		}

		//Comma delimits arguments, space delimits command-to-args
		if(!(isspace(line[i]) && cmd.empty()) && line[i] != ',')
		{
			tmp += line[i];
			continue;
		}

		//merge multiple delimiters into one delimiter
		if(tmp == "")
			continue;

		//Save command or argument
		if(reading_cmd)
			cmd = tmp;
		else
			args.push_back(tmp);

		reading_cmd = false;
		tmp = "";
	}

	//Stuff left over at the end? Figure out which field it belongs in
	if(tmp != "")
	{
		if(cmd != "")
			args.push_back(tmp);
		else
			cmd = tmp;
	}
}

/**
	@brief Pushes channel configuration to the instrument
 */
/*
void UpdateChannel(size_t chan)
{
	if(g_pico_type == PICO3000A)
	{
		ps3000aSetChannel(g_hScope, (PS3000A_CHANNEL)chan, g_channelOn[chan],
			(PS3000A_COUPLING)g_coupling[chan], g_range_3000a[chan], -g_offset[chan]);
		ps3000aSetBandwidthFilter(g_hScope, (PS3000A_CHANNEL)chan,
			(PS3000A_BANDWIDTH_LIMITER)g_bandwidth_legacy[chan]);
		if(chan == g_triggerChannel)
			UpdateTrigger();
		return;
	}

	if(g_channelOn[chan])
	{
		ps6000aSetChannelOn(g_hScope, (PICO_CHANNEL)chan,
			g_coupling[chan], g_range[chan], -g_offset[chan], g_bandwidth[chan]);

		//We use software triggering based on raw ADC codes.
		//Any time we change the frontend configuration on the trigger channel, it has to be reconfigured.
		//TODO: handle multi-input triggers
		if(chan == g_triggerChannel)
			UpdateTrigger();
	}
	else
		ps6000aSetChannelOff(g_hScope, (PICO_CHANNEL)chan);
}
*/

/**
	@brief Pushes trigger configuration to the instrument
 */
/*
void UpdateTrigger(bool force)
{
	//Timeout, in microseconds, before initiating a trigger
	//Force trigger is really just a one-shot auto trigger with a 1us delay.
	uint32_t timeout = 0;
	if(force)
	{
		timeout = 1;
		g_lastTriggerWasForced = true;
	}
	else
		g_lastTriggerWasForced = false;

	bool triggerIsAnalog = (g_triggerChannel < g_numChannels);

	//Convert threshold from volts to ADC counts
	float offset = 0;
	if(triggerIsAnalog)
		offset = g_offset[g_triggerChannel];
	float scale = 1;
	if(triggerIsAnalog)
	{
		scale = g_roundedRange[g_triggerChannel] / 32512;
		if(scale == 0)
			scale = 1;
	}
	float trig_code = (g_triggerVoltage - offset) / scale;
	//LogDebug("UpdateTrigger: trig_code = %.0f for %f V, scale=%f\n", round(trig_code), g_triggerVoltage, scale);

	//This can happen early on during initialization.
	//Bail rather than dividing by zero.
	if(g_sampleInterval == 0)
		return;

	//Add delay before start of capture if needed
	int64_t triggerDelaySamples = g_triggerDelay / g_sampleInterval;
	uint64_t delay = 0;
	if(triggerDelaySamples < 0)
		delay = -triggerDelaySamples;

	switch(g_pico_type)
	{
		case PICO3000A:
			ps3000aSetSimpleTrigger(
				g_hScope,
				1,
				(PS3000A_CHANNEL)g_triggerChannel,
				round(trig_code),
				(enPS3000AThresholdDirection)g_triggerDirection, // same as 6000a api
				delay,
				timeout);
			break;

		case PICO6000A:
			if(g_triggerChannel < g_numChannels)
			{
				ps6000aSetSimpleTrigger(
					g_hScope,
					1,
					(PICO_CHANNEL)g_triggerChannel,
					round(trig_code),
					g_triggerDirection,
					delay,
					timeout);
			}
			else
			{
				//Remove old trigger conditions
				ps6000aSetTriggerChannelConditions(
					g_hScope,
					NULL,
					0,
					PICO_CLEAR_ALL);

				//Set up new conditions
				int ntrig = g_triggerChannel - g_numChannels;
				int trigpod = ntrig / 8;
				int triglane = ntrig % 8;
				PICO_CONDITION cond;
				cond.source = static_cast<PICO_CHANNEL>(PICO_PORT0 + trigpod);
				cond.condition = PICO_CONDITION_TRUE;
				ps6000aSetTriggerChannelConditions(
					g_hScope,
					&cond,
					1,
					PICO_ADD);

				//Set up configuration on the selected channel
				PICO_DIGITAL_CHANNEL_DIRECTIONS dirs;
				dirs.channel = static_cast<PICO_PORT_DIGITAL_CHANNEL>(PICO_PORT_DIGITAL_CHANNEL0 + triglane);
				dirs.direction = PICO_DIGITAL_DIRECTION_RISING;				//TODO: configurable
				ps6000aSetTriggerDigitalPortProperties(
					g_hScope,
					cond.source,
					&dirs,
					1);

				//ps6000aSetTriggerDigitalPortProperties doesn't have a timeout!
				//Should we call ps6000aSetTriggerChannelProperties with no elements to do this?
				if(force)
					LogWarning("Force trigger doesn't currently work if trigger source is digital\n");
			}
			break;
	}

	if(g_triggerArmed)
		StartCapture(true);
}

void Stop()
{
	switch(g_pico_type)
	{
		case PICO3000A:
			ps3000aStop(g_hScope);
			break;

		case PICO6000A:
			ps6000aStop(g_hScope);
			break;
	}
}

*/
void Start(bool force)
{
	g_captureMemDepth = g_memDepth;

	/*
	//If previous trigger was forced, we need to reconfigure the trigger to be not-forced now
	if(g_lastTriggerWasForced && !force)
	{
		Stop();
		UpdateTrigger();
	}
	*/

	g_channelOnDuringArm = g_channelOn;
	/*
	for(size_t i=0; i<g_numDigitalPods; i++)
		g_msoPodEnabledDuringArm[i] = g_msoPodEnabled[i];
	*/
	g_sampleIntervalDuringArm = g_sampleInterval;

	//Set acquisition mode
	FDwfAnalogInAcquisitionModeSet(g_hScope, acqmodeSingle);

	//Start acquisition
	FDwfAnalogInConfigure(g_hScope, true, true);

	g_triggerArmed = true;
}
