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

/*
//Channel state
map<size_t, bool> g_channelOn;
map<size_t, PICO_COUPLING> g_coupling;
map<size_t, PICO_CONNECT_PROBE_RANGE> g_range;
map<size_t, enPS3000ARange> g_range_3000a;
map<size_t, double> g_roundedRange;
map<size_t, double> g_offset;
map<size_t, PICO_BANDWIDTH_LIMITER> g_bandwidth;
map<size_t, size_t> g_bandwidth_legacy;
*/
size_t g_memDepth = 1000000;
int64_t g_sampleInterval = 0;	//in fs

/*
//Copy of state at timestamp of last arm event
map<size_t, bool> g_channelOnDuringArm;
*/
int64_t g_sampleIntervalDuringArm = 0;
size_t g_captureMemDepth = 0;
/*map<size_t, double> g_offsetDuringArm;
*/
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
			size_t laneId = 0;
			bool channelIsDigital = false;
			if(isalpha(subject[0]))
			{
				channelId = min(static_cast<size_t>(subject[0] - 'A'), g_numAnalogInChannels);
				channelIsDigital = false;
			}
			else if(isdigit(subject[0]))
			{
				channelId = min(subject[0] - '0', 2) - 1;
				channelIsDigital = true;
				if(subject.length() >= 3)
					laneId = min(subject[2] - '0', 7);
			}

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
					string ret = "";
					ret = "100000000";		//for now, only 100 Msps supported
					ScpiSend(client, ret);
				}

				//Get memory depths
				else if(cmd == "DEPTHS")
				{
					string ret = "";
					ret = "65536";			//for now, only 64K supported
					ScpiSend(client, ret);
				}

				else
					LogDebug("Unrecognized query received: %s\n", line.c_str());
			}

			else if(cmd == "EXIT")
				break;

			/*
			else if(cmd == "ON")
			{
				lock_guard<mutex> lock(g_mutex);

				if(channelIsDigital)
				{
					PICO_CHANNEL podId = (PICO_CHANNEL)(PICO_PORT0 + channelId);
					auto status = ps6000aSetDigitalPortOn(
						g_hScope,
						podId,
						g_msoPodThreshold[channelId],
						8,
						g_msoHysteresis[channelId]);
					if(status != PICO_OK)
						LogError("ps6000aSetDigitalPortOn failed with code %x\n", status);
					else
						g_msoPodEnabled[channelId] = true;
				}
				else
				{
					g_channelOn[channelId] = true;
					UpdateChannel(channelId);
				}

				//We need to allocate new buffers for this channel
				g_memDepthChanged = true;

			}
			else if(cmd == "OFF")
			{
				lock_guard<mutex> lock(g_mutex);

				if(channelIsDigital)
				{
					PICO_CHANNEL podId = (PICO_CHANNEL)(PICO_PORT0 + channelId);
					auto status = ps6000aSetDigitalPortOff(g_hScope, podId);
					if(status != PICO_OK)
						LogError("ps6000aSetDigitalPortOff failed with code %x\n", status);
				}
				else
				{
					g_channelOn[channelId] = false;
					UpdateChannel(channelId);
				}

				//Free the memory used by this channel
				g_memDepthChanged = true;
			}

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
			else if( (cmd == "OFFS") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				double requestedOffset = stod(args[0]);

				double maxoff;
				double minoff;
				float maxoff_f;
				float minoff_f;

				//Clamp to allowed range
				switch(g_pico_type) {
				case PICO3000A:
					ps3000aGetAnalogueOffset(g_hScope, g_range_3000a[channelId], (PS3000A_COUPLING)g_coupling[channelId], &maxoff_f, &minoff_f);
					maxoff = maxoff_f;
					minoff = minoff_f;
					break;
				case PICO6000A:
					ps6000aGetAnalogueOffsetLimits(g_hScope, g_range[channelId], g_coupling[channelId], &maxoff, &minoff);
					break;
				}
				requestedOffset = min(maxoff, requestedOffset);
				requestedOffset = max(minoff, requestedOffset);

				g_offset[channelId] = requestedOffset;
				UpdateChannel(channelId);
			}

			else if( (cmd == "RANGE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				auto range = stod(args[0]);

				//If 50 ohm coupling, cap hardware voltage range to 5V
				if(g_coupling[channelId] == PICO_DC_50OHM)
					range = min(range, 5.0);

				if(range > 100 && g_pico_type == PICO6000A)
				{
					g_range[channelId] = PICO_X1_PROBE_200V;
					g_roundedRange[channelId] = 200;
				}
				else if(range > 50 && g_pico_type == PICO6000A)
				{
					g_range[channelId] = PICO_X1_PROBE_100V;
					g_roundedRange[channelId] = 100;
				}
				else if(range > 20)
				{
					g_range[channelId] = PICO_X1_PROBE_50V;
					g_range_3000a[channelId] = PS3000A_50V;
					g_roundedRange[channelId] = 50;
				}
				else if(range > 10)
				{
					g_range[channelId] = PICO_X1_PROBE_20V;
					g_range_3000a[channelId] = PS3000A_20V;
					g_roundedRange[channelId] = 20;
				}
				else if(range > 5)
				{
					g_range[channelId] = PICO_X1_PROBE_10V;
					g_range_3000a[channelId] = PS3000A_10V;
					g_roundedRange[channelId] = 10;
				}
				else if(range > 2)
				{
					g_range[channelId] = PICO_X1_PROBE_5V;
					g_range_3000a[channelId] = PS3000A_5V;
					g_roundedRange[channelId] = 5;
				}
				else if(range > 1)
				{
					g_range[channelId] = PICO_X1_PROBE_2V;
					g_range_3000a[channelId] = PS3000A_2V;
					g_roundedRange[channelId] = 2;
				}
				else if(range > 0.5)
				{
					g_range[channelId] = PICO_X1_PROBE_1V;
					g_range_3000a[channelId] = PS3000A_1V;
					g_roundedRange[channelId] = 1;
				}
				else if(range > 0.2)
				{
					g_range[channelId] = PICO_X1_PROBE_500MV;
					g_range_3000a[channelId] = PS3000A_500MV;
					g_roundedRange[channelId] = 0.5;
				}
				else if(range > 0.1)
				{
					g_range[channelId] = PICO_X1_PROBE_200MV;
					g_range_3000a[channelId] = PS3000A_200MV;
					g_roundedRange[channelId] = 0.2;
				}
				else if(range >= 0.05)
				{
					g_range[channelId] = PICO_X1_PROBE_100MV;
					g_range_3000a[channelId] = PS3000A_100MV;
					g_roundedRange[channelId] = 0.1;
				}
				else if(range >= 0.02)
				{
					g_range[channelId] = PICO_X1_PROBE_50MV;
					g_range_3000a[channelId] = PS3000A_50MV;
					g_roundedRange[channelId] = 0.05;
				}
				else if(range >= 0.01)
				{
					g_range[channelId] = PICO_X1_PROBE_20MV;
					g_range_3000a[channelId] = PS3000A_20MV;
					g_roundedRange[channelId] = 0.02;
				}
				else
				{
					g_range[channelId] = PICO_X1_PROBE_10MV;
					g_range_3000a[channelId] = PS3000A_10MV;
					g_roundedRange[channelId] = 0.01;
				}

				UpdateChannel(channelId);

				//Update trigger if this is the trigger channel.
				//Trigger is digital and threshold is specified in ADC counts.
				//We want to maintain constant trigger level in volts, not ADC counts.
				if(g_triggerChannel == channelId)
					UpdateTrigger();
			}
			*/
			else if( (cmd == "RATE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				int64_t rate = stoull(args[0]);
				FDwfAnalogInFrequencySet(g_hScope, rate);
				g_sampleInterval = 1e15 / rate;
			}

			else if( (cmd == "DEPTH") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				g_memDepth = stoull(args[0]);
				FDwfAnalogInBufferSizeSet(g_hScope, g_memDepth);
				g_memDepthChanged = true;
			}

			else if( (cmd == "START") || (cmd == "SINGLE") )
			{
				lock_guard<mutex> lock(g_mutex);

				if(g_triggerArmed)
				{
					LogVerbose("Ignoring START command because trigger is already armed\n");
					continue;
				}

				/*
				//Make sure we've got something to capture
				bool anyChannels = false;
				for(size_t i=0; i<g_numChannels; i++)
				{
					if(g_channelOn[i])
					{
						anyChannels = true;
						break;
					}
				}
				for(size_t i=0; i<g_numDigitalPods; i++)
				{
					if(g_msoPodEnabled[i])
					{
						anyChannels = true;
						break;
					}
				}

				if(!anyChannels)
				{
					LogVerbose("Ignoring START command because no channels are active\n");
					continue;
				}
				*/

				//Start the capture
				StartCapture(false);
				g_triggerOneShot = (cmd == "SINGLE");
			}

			else if(cmd == "FORCE")
			{
				/*
				//Clear out any old trigger config
				if(g_triggerArmed)
				{
					Stop();
					g_triggerArmed = false;
				}

				UpdateTrigger(true);
				StartCapture(true, true);
				*/
			}

			else if(cmd == "STOP")
			{
				/*
				lock_guard<mutex> lock(g_mutex);

				Stop();
				g_triggerArmed = false;
				*/
			}

			/*
			else if(subject == "TRIG")
			{
				if( (cmd == "EDGE:DIR") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					if(args[0] == "RISING")
						g_triggerDirection = PICO_RISING;
					else if(args[0] == "FALLING")
						g_triggerDirection = PICO_FALLING;
					else if(args[0] == "ANY")
						g_triggerDirection = PICO_RISING_OR_FALLING;

					UpdateTrigger();
				}

				else if( (cmd == "LEV") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					g_triggerVoltage = stof(args[0]);
					UpdateTrigger();
				}

				else if( (cmd == "SOU") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					if(isalpha(args[0][0]))
					{
						g_triggerChannel = args[0][0] - 'A';
						if(!g_channelOn[g_triggerChannel])
						{
							LogDebug("Trigger channel wasn't on, enabling it\n");
							g_channelOn[g_triggerChannel] = true;
							UpdateChannel(g_triggerChannel);
						}
					}
					else if( (isdigit(args[0][0])) && (args[0].length() == 3) )
					{
						int npod = args[0][0] - '1';
						int nchan = args[0][2] - '0';
						g_triggerChannel = g_numChannels + npod*8 + nchan;

						if(!g_msoPodEnabled[npod])
						{
							LogDebug("Trigger pod wasn't on, enabling it\n");
							EnableMsoPod(npod);
						}
					}

					bool wasOn = g_triggerArmed;
					Stop();

					UpdateTrigger();

					if(wasOn)
						StartCapture(false);
				}

				else if( (cmd == "DELAY") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					g_triggerDelay = stoull(args[0]);
					UpdateTrigger();
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
			*/

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

PICO_STATUS StartInternal()
{
	//Calculate pre/post trigger time configuration based on trigger delay
	int64_t triggerDelaySamples = g_triggerDelay / g_sampleInterval;
	size_t nPreTrigger = min(max(triggerDelaySamples, 0L), (int64_t)g_memDepth);
	size_t nPostTrigger = g_memDepth - nPreTrigger;
	g_triggerSampleIndex = nPreTrigger;

	switch(g_pico_type)
	{
		case PICO3000A:
			// TODO: why the 1
			return ps3000aRunBlock(g_hScope, nPreTrigger, nPostTrigger, g_timebase, 1, NULL, 0, NULL, NULL);

		case PICO6000A:
			return ps6000aRunBlock(g_hScope, nPreTrigger, nPostTrigger, g_timebase, NULL, 0, NULL, NULL);

		default:
			return PICO_OK;
	}
}
*/
void StartCapture(bool stopFirst, bool force)
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

	/*
	g_offsetDuringArm = g_offset;
	g_channelOnDuringArm = g_channelOn;
	for(size_t i=0; i<g_numDigitalPods; i++)
		g_msoPodEnabledDuringArm[i] = g_msoPodEnabled[i];
	*/
	g_sampleIntervalDuringArm = g_sampleInterval;
	/*
	LogTrace("StartCapture stopFirst %d memdepth %zu\n", stopFirst, g_captureMemDepth);

	PICO_STATUS status;
	status = PICO_RESERVED_1;
	if(stopFirst)
		Stop();
	status = StartInternal();
	*/

	//Set acquisition mode
	FDwfAnalogInAcquisitionModeSet(g_hScope, acqmodeSingle);

	//Start acquisition
	FDwfAnalogInConfigure(g_hScope, true, true);

	g_triggerArmed = true;
}
