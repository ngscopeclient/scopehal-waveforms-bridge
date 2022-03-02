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
	@brief Waveform data thread (data plane traffic only, no control plane SCPI)
 */
#include "wfmserver.h"
#include <string.h>

using namespace std;

volatile bool g_waveformThreadQuit = false;
/*float InterpolateTriggerTime(int16_t* buf);

vector<PICO_CHANNEL> g_channelIDs;*/

void WaveformServerThread()
{
	#ifdef __linux__
	pthread_setname_np(pthread_self(), "WaveformThread");
	#endif

	Socket client = g_dataSocket.Accept();
	LogVerbose("Client connected to data plane socket\n");

	if(!client.IsValid())
		return;
	if(!client.DisableNagle())
		LogWarning("Failed to disable Nagle on socket, performance may be poor\n");

	map<size_t, double*> waveformBuffers;
	uint16_t numchans = 0;

	while(!g_waveformThreadQuit)
	{
		if(!g_triggerArmed)
		{
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
			continue;
		}

		//Poll until we have a fully acquired waveform
		while(true)
		{
			lock_guard<mutex> lock(g_mutex);

			//Get status
			DwfState state;
			FDwfAnalogInStatus(g_hScope, true, &state);

			int samplesLeft;
			FDwfAnalogInStatusSamplesLeft(g_hScope, &samplesLeft);
			LogDebug("Samples left: %d\n", samplesLeft);

			if(samplesLeft == 0)
				break;

			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}

		LogDebug("Got a waveform\n");

		{
			lock_guard<mutex> lock(g_mutex);

			//Set up buffers if needed
			if(g_memDepthChanged || waveformBuffers.empty())
			{
				LogTrace("Reallocating buffers\n");

				//Clear out old buffers
				for(size_t i=0; i<g_numAnalogInChannels; i++)
				{
					if(waveformBuffers[i])
					{
						delete[] waveformBuffers[i];
						waveformBuffers[i] = NULL;
					}
				}

				//Set up new ones
				//TODO: Only allocate memory if the channel is actually enabled
				for(size_t i=0; i<g_numAnalogInChannels; i++)
				{
					//Allocate memory if needed
					waveformBuffers[i] = new double[g_captureMemDepth];
					memset(waveformBuffers[i], 0x00, g_captureMemDepth * sizeof(double));
				}

				g_memDepthChanged = false;

			}

			//Download the data from the scope
			for(size_t i=0; i<g_numAnalogInChannels; i++)
			{
				//TODO: only if channel is enabled?

				FDwfAnalogInStatusData(g_hScope, i, waveformBuffers[i], g_captureMemDepth);
			}

			//Figure out how many channels are active in this capture
			numchans = 0;
			/*for(size_t i=0; i<g_numChannels; i++)
			{
				if(g_channelOnDuringArm[i])
					numchans ++;
			}
			for(size_t i=0; i<g_numDigitalPods; i++)
			{
				if(g_msoPodEnabledDuringArm[i])
					numchans ++;
			}
			*/

			//FIXME
			numchans = g_numAnalogInChannels;
		}

		//Send the channel count to the client
		client.SendLooped((uint8_t*)&numchans, sizeof(numchans));

		//Send sample rate to the client
		client.SendLooped((uint8_t*)&g_sampleIntervalDuringArm, sizeof(g_sampleIntervalDuringArm));

		//Interpolate trigger position if we're using an analog level trigger
		//bool triggerIsAnalog = (g_triggerChannel < g_numChannels);
		float trigphase = 0;
		//if(triggerIsAnalog)
		//	trigphase = InterpolateTriggerTime(waveformBuffers[g_triggerChannel]);

		//Send data for each channel to the client
		for(size_t i=0; i<g_numAnalogInChannels; i++)
		{
			//Analog channels
			//if((i < g_numAnalogInChannels) && (g_channelOnDuringArm[i]) )
			{
				//Send channel ID, memory depth, and trigger phase
				client.SendLooped((uint8_t*)&i, sizeof(i));
				client.SendLooped((uint8_t*)&g_captureMemDepth, sizeof(g_captureMemDepth));
				client.SendLooped((uint8_t*)&trigphase, sizeof(trigphase));

				//Send the actual waveform data
				client.SendLooped((uint8_t*)waveformBuffers[i], g_captureMemDepth * sizeof(double));
			}

			/*
			//Digital channels
			else if( (i >= g_numChannels) && (g_msoPodEnabledDuringArm[i - g_numChannels]) )
			{
				client.SendLooped((uint8_t*)&i, sizeof(i));
				client.SendLooped((uint8_t*)&numSamples, sizeof(numSamples));
				client.SendLooped((uint8_t*)&trigphase, sizeof(trigphase));
				client.SendLooped((uint8_t*)waveformBuffers[i], numSamples * sizeof(int16_t));
			}
			*/
		}

		//Re-arm the trigger if doing repeating triggers
		if(g_triggerOneShot)
			g_triggerArmed = false;
		else
		{
			lock_guard<mutex> lock(g_mutex);
			StartCapture(false);
		}
	}

	/*
	//Clean up temporary buffers
	for(auto it : waveformBuffers)
		delete[] it.second;
	*/
}

/*
float InterpolateTriggerTime(int16_t* buf)
{
	if(g_triggerSampleIndex <= 0)
		return 0;

	float trigscale = g_roundedRange[g_triggerChannel] / 32512;
	float trigoff = g_offsetDuringArm[g_triggerChannel];

	float fa = buf[g_triggerSampleIndex-1] * trigscale + trigoff;
	float fb = buf[g_triggerSampleIndex] * trigscale + trigoff;

	//no need to divide by time, sample spacing is normalized to 1 timebase unit
	float slope = (fb - fa);
	float delta = g_triggerVoltage - fa;
	return delta / slope;
}
*/
