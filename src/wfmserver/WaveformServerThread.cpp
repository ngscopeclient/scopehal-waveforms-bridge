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
#include "DigilentSCPIServer.h"

using namespace std;

volatile bool g_waveformThreadQuit = false;
float InterpolateTriggerTime(double* buf);

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

			if(samplesLeft == 0)
				break;

			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}

		int64_t interval;
		size_t depth;
		map<size_t, bool> channelOn;
		{
			lock_guard<mutex> lock(g_mutex);

			interval = g_sampleIntervalDuringArm;
			channelOn = g_channelOnDuringArm;
			depth = g_captureMemDepth;

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
			for(size_t i=0; i<g_numAnalogInChannels; i++)
			{
				if(g_channelOnDuringArm[i])
					numchans ++;
			}
			/*
			for(size_t i=0; i<g_numDigitalPods; i++)
			{
				if(g_msoPodEnabledDuringArm[i])
					numchans ++;
			}
			*/
		}

		//Send the channel count and sample rate to the client
		if(!client.SendLooped((uint8_t*)&numchans, sizeof(numchans)))
			break;
		if(!client.SendLooped((uint8_t*)&g_sampleIntervalDuringArm, sizeof(interval)))
			break;

		//Interpolate trigger position if we're using an analog level trigger
		//bool triggerIsAnalog = (g_triggerChannel < g_numChannels);
		bool triggerIsAnalog = true;
		float trigphase = 0;
		if(triggerIsAnalog)
		{
			//Interpolate zero crossing to get sub-sample precision
			trigphase = -InterpolateTriggerTime(waveformBuffers[g_triggerChannel]) * interval;

			//Correct for set point error
			trigphase += (interval  + g_triggerDeltaSec*FS_PER_SECOND);
		}

		//Send data for each channel to the client
		for(size_t i=0; i<g_numAnalogInChannels; i++)
		{
			//Analog channels
			if((i < g_numAnalogInChannels) && (channelOn[i]) )
			{
				//Send channel ID, memory depth, and trigger phase
				size_t header[2] = {i, g_captureMemDepth};
				if(!client.SendLooped((uint8_t*)&header, sizeof(header)))
					break;
				if(!client.SendLooped((uint8_t*)&trigphase, sizeof(trigphase)))
					break;

				//Send the actual waveform data
				if(!client.SendLooped((uint8_t*)waveformBuffers[i], depth * sizeof(double)))
					break;
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

		{
			lock_guard<mutex> lock(g_mutex);

			//Re-arm the trigger if doing repeating triggers
			if(g_triggerOneShot)
				g_triggerArmed = false;
			else
				DigilentSCPIServer::Start();
		}
	}

	//Clean up temporary buffers
	for(auto it : waveformBuffers)
		delete[] it.second;
}

float InterpolateTriggerTime(double* buf)
{
	if(g_triggerSampleIndex >= g_memDepth-1)
		return 0;

	float fa = buf[g_triggerSampleIndex];
	float fb = buf[g_triggerSampleIndex+1];

	//no need to divide by time, sample spacing is normalized to 1 timebase unit
	float slope = (fb - fa);
	float delta = g_triggerVoltage - fa;
	return delta / slope;
}
