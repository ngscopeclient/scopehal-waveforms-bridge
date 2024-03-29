/***********************************************************************************************************************
*                                                                                                                      *
* wfmserver                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
	@brief Program entry point
 */

#include "wfmserver.h"
#include <signal.h>
#include "DigilentSCPIServer.h"

using namespace std;

void help();

void help()
{
	fprintf(stderr,
			"wfmserver [general options] [USB or IP device options] [logger options]\n"
			"\n"
			"  [general options]:\n"
			"    --help                        : this message...\n"
			"    --scpi-port nnn               : specifies the SCPI control plane port (default 5025)\n"
			"    --waveform-port nnn           : specifies the binary waveform data port (default 5026)\n"
			"  [USB device options]:\n"
			"    --device nnn                  : specifies the device to open if more than one is present\n"
			"    --config nnn                  : specifies the configuration for the device to use\n"
			"  [IP device options]:\n"
			"    --host hostname_or_ip         : hostname or IP address of the embedded server\n"
			"\n"
			"  [logger options]:\n"
			"    levels: ERROR, WARNING, NOTICE, VERBOSE, DEBUG\n"
			"    --quiet|-q                    : reduce logging level by one step\n"
			"    --verbose                     : set logging level to VERBOSE\n"
			"    --debug                       : set logging level to DEBUG\n"
			"    --trace <classname>|          : name of class with tracing messages. (Only relevant when logging level is DEBUG.)\n"
			"            <classname::function>\n"
			"    --logfile|-l <filename>       : output log messages to file\n"
			"    --logfile-lines|-L <filename> : output log messages to file, with line buffering\n"
			"    --stdout-only                 : writes errors/warnings to stdout instead of stderr\n"
	);
}

string g_model;
string g_serial;
string g_fwver;

HDWF g_hScope;
size_t g_numAnalogInChannels = 0;

Socket g_scpiSocket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
Socket g_dataSocket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

void OnQuit(int signal);

int main(int argc, char* argv[])
{
	//Global settings
	Severity console_verbosity = Severity::NOTICE;

	//Parse command-line arguments
	uint16_t scpi_port = 5025;
	uint16_t waveform_port = 5026;
	string host;
	int device = 0;
	int config = 0;
	for(int i=1; i<argc; i++)
	{
		string s(argv[i]);

		//Let the logger eat its args first
		if(ParseLoggerArguments(i, argc, argv, console_verbosity))
			continue;

		if(s == "--help")
		{
			help();
			return 0;
		}

		else if(s == "--scpi-port")
		{
			if(i+1 < argc)
				scpi_port = atoi(argv[++i]);
		}

		else if(s == "--waveform-port")
		{
			if(i+1 < argc)
				waveform_port = atoi(argv[++i]);
		}
		else if(s == "--device")
		{
			if(i+1 < argc)
				device = atoi(argv[++i]);
		}
		else if(s == "--config")
		{
			if(i+1 < argc)
				config = atoi(argv[++i]);
		}
		else if(s == "--host")
		{
			if(i+1 < argc)
				host = argv[++i];
		}

		else
		{
			fprintf(stderr, "Unrecognized command-line argument \"%s\", use --help\n", s.c_str());
			return 1;
		}
	}

	//Set up logging
	g_log_sinks.emplace(g_log_sinks.begin(), new ColoredSTDLogSink(console_verbosity));

	//Dump the Digilent API version
	char version[32] = "";
	if(!FDwfGetVersion(version))
	{
		LogError("FDwfGetVersion failed\n");
		return 1;
	}
	LogDebug("Digilent API %s\n", version);

	if(host.empty())
	{
		//Initial setup: enumerate devices
		LogNotice("Looking for Digilent devices...\n");
		int numDevices;
		if(!FDwfEnum(enumfilterAll, &numDevices))
		{
			LogError("FDwfEnum failed\n");
			return 1;
		}
		LogDebug("%d devices found\n", numDevices);
		if(numDevices == 0)
		{
			LogNotice("No devices found, exiting\n");
			return 0;
		}

		//Print out list of all devices found
		char username[32];
		char devname[32];
		char serial[32];
		for(int i=0; i<numDevices; i++)
		{
			LogIndenter li;

			FDwfEnumUserName(i, username);
			FDwfEnumDeviceName(i, devname);
			FDwfEnumSN(i, serial);
			LogVerbose("[%d] %s (user name %s), serial %s\n", i, devname, username, serial);
		}

		//Print out the selected device
		FDwfEnumUserName(device, username);
		FDwfEnumDeviceName(device, devname);
		FDwfEnumSN(device, serial);
		LogVerbose("Using device %d: %s (user name %s), serial %s\n", device, devname, username, serial);
		g_model = devname;
		g_serial = serial;
		g_fwver = "FIXME";

		//Enum configurations and decide which one to use
		int configsFound;
		LogVerbose("Checking possible device configurations...\n");
		if(!FDwfEnumConfig(device, &configsFound))
		{
			LogError("FDwfEnumConfig failed\n");
			return 1;
		}
		LogDebug("%d configs found\n", configsFound);
		{
			LogIndenter li;
			for(int i=0; i<configsFound; i++)
			{
				LogDebug("Config %d:\n", i);
				LogIndenter li2;

				int analogInCount;
				int analogIOCount;
				int analogOutCount;
				int digitalInCount;
				int digitalOutCount;
				int digitalIOCount;

				int analogInBufferSize;
				int analogOutBufferSize;
				int digitalInBufferSize;
				int digitalOutBufferSize;

				FDwfEnumConfigInfo(i, DECIAnalogInChannelCount, &analogInCount);
				FDwfEnumConfigInfo(i, DECIAnalogOutChannelCount, &analogOutCount);
				FDwfEnumConfigInfo(i, DECIAnalogIOChannelCount, &analogIOCount);
				FDwfEnumConfigInfo(i, DECIDigitalInChannelCount, &digitalInCount);
				FDwfEnumConfigInfo(i, DECIDigitalOutChannelCount, &digitalOutCount);
				FDwfEnumConfigInfo(i, DECIDigitalIOChannelCount, &digitalIOCount);

				FDwfEnumConfigInfo(i, DECIAnalogInBufferSize, &analogInBufferSize);
				FDwfEnumConfigInfo(i, DECIAnalogOutBufferSize, &analogOutBufferSize);
				FDwfEnumConfigInfo(i, DECIDigitalInBufferSize, &digitalInBufferSize);
				FDwfEnumConfigInfo(i, DECIDigitalOutBufferSize, &digitalOutBufferSize);

				LogDebug("Analog in:   %d\n", analogInCount);
				LogDebug("Analog out:  %d\n", analogOutCount);
				LogDebug("Analog IO:   %d\n", analogIOCount);
				LogDebug("Digital in:  %d\n", digitalInCount);
				LogDebug("Digital out: %d\n", digitalOutCount);
				LogDebug("Digital IO:  %d\n", digitalIOCount);

				g_numAnalogInChannels = analogInCount;

				LogDebug("Analog buffer: %d in, %d out\n", analogInBufferSize, analogOutBufferSize);
				LogDebug("Digital buffer: %d in, %d out\n", digitalInBufferSize, digitalOutBufferSize);
			}
		}

		//Open the device
		LogDebug("Opening device %d in config %d\n", device, config);
		if(!FDwfDeviceConfigOpen(device, config, &g_hScope))
		{
			LogError("Failed to open device\n");
			return 1;
		}
	}
	else
	{
		LogDebug("Opening Ethernet device\n");

		//TODO: figure out how to get this info
		g_model = "Analog Discovery Pro 3450";
		g_serial = "Unknown";
		g_fwver = "FIXME";
		g_numAnalogInChannels = 4;

		string connstr = string("ip:") + host + "\nuser:admin\npass:admin\nsecure:1";
		if(!FDwfDeviceOpenEx(connstr.c_str(), &g_hScope))
		{
			LogError("Failed to open device\n");
			return 1;
		}
	}

	//Initialize analog channels
	for(size_t i=0; i<g_numAnalogInChannels; i++)
		g_channelOn[i] = false;

	//Set up signal handlers
	signal(SIGINT, OnQuit);
	signal(SIGPIPE, SIG_IGN);

	//Configure the data plane socket
	g_dataSocket.Bind(waveform_port);
	g_dataSocket.Listen();

	//Launch the control plane socket server
	g_scpiSocket.Bind(scpi_port);
	g_scpiSocket.Listen();

	while(true)
	{
		Socket scpiClient = g_scpiSocket.Accept();
		if(!scpiClient.IsValid())
			break;

		//Create a server object for this connection
		DigilentSCPIServer server(scpiClient.Detach());

		//Launch the data-plane thread
		thread dataThread(WaveformServerThread);

		//Process connections on the socket
		server.MainLoop();

		g_waveformThreadQuit = true;
		dataThread.join();
		g_waveformThreadQuit = false;
	}

	//Done, clean up
	FDwfDeviceClose(g_hScope);

	return 0;
}

void OnQuit(int /*signal*/)
{
	LogNotice("Shutting down...\n");

	lock_guard<mutex> lock(g_mutex);
	FDwfDeviceClose(g_hScope);
	exit(0);
}
