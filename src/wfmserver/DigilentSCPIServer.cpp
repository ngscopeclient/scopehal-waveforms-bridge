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

#include "wfmserver.h"
#include "DigilentSCPIServer.h"

using namespace std;

//These functions are not yet in the Digilent API headers (will be in the next release)
//Prototypes from email conversation with Attila at Digilent
//TODO: delete this once the next version of the SDK is released (and set that as the min version)
typedef int DwfAnalogCoupling;
const DwfAnalogCoupling DwfAnalogCouplingDC = 0;
const DwfAnalogCoupling DwfAnalogCouplingAC = 1;
DWFAPI int FDwfAnalogInChannelCouplingInfo(HDWF hdwf, int *pfscoupling); // use IsBitSet
DWFAPI int FDwfAnalogInChannelCouplingSet(HDWF hdwf, int idxChannel, DwfAnalogCoupling coupling);
DWFAPI int FDwfAnalogInChannelCouplingGet(HDWF hdwf, int idxChannel, DwfAnalogCoupling *pcoupling);

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

DigilentSCPIServer::DigilentSCPIServer(ZSOCKET sock)
	: BridgeSCPIServer(sock)
{
	//Reset the device to default configuration
	if(!FDwfAnalogInReset(g_hScope))
	{
		LogError("FDwfAnalogInReset failed\n");
		exit(1);
	}
}

DigilentSCPIServer::~DigilentSCPIServer()
{
	//Reset the device to default configuration
	FDwfAnalogInReset(g_hScope);
	LogVerbose("Client disconnected\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command parsing

bool DigilentSCPIServer::GetChannelID(const string& subject, size_t& id_out)
{
	//Extract channel ID from subject and clamp bounds
	if(toupper(subject[0]) == 'C')
		id_out = min(static_cast<size_t>(stoi(subject.c_str() + 1) - 1), g_numAnalogInChannels);

	/*
	else if(isdigit(subject[0]))
	{
		channelId = min(subject[0] - '0', 2) - 1;
		channelIsDigital = true;
		if(subject.length() >= 3)
			laneId = min(subject[2] - '0', 7);
	}
	*/

	else
		return false;

	return true;
}

BridgeSCPIServer::ChannelType DigilentSCPIServer::GetChannelType(size_t channel)
{
	if(channel < g_numAnalogInChannels)
		return CH_ANALOG;
	else
		return CH_DIGITAL;
}

bool DigilentSCPIServer::OnQuery(
		const string& line,
		const string& subject,
		const string& cmd)
{
	if(BridgeSCPIServer::OnQuery(line, subject, cmd))
		return true;

	//TODO: handle commands not implemented by the base class
	LogWarning("Unrecognized query received: %s\n", line.c_str());

	return false;
}

string DigilentSCPIServer::GetMake()
{
	return "Digilent";
}

string DigilentSCPIServer::GetModel()
{
	return g_model;
}

string DigilentSCPIServer::GetSerial()
{
	return g_serial;
}

string DigilentSCPIServer::GetFirmwareVersion()
{
	return g_fwver;
}

size_t DigilentSCPIServer::GetAnalogChannelCount()
{
	return g_numAnalogInChannels;
}

vector<size_t> DigilentSCPIServer::GetSampleRates()
{
	vector<size_t> rates;

	double minFreq;
	double maxFreq;
	if(!FDwfAnalogInFrequencyInfo(g_hScope, &minFreq, &maxFreq))
		LogError("FDwfAnalogInFrequencyInfo failed\n");

	//Cap min freq to 1 kHz
	minFreq = max(minFreq, 1000.0);

	//Report sample rates in 1-2-5 steps
	double freq = maxFreq;
	while(freq >= minFreq)
	{
		rates.push_back(freq);
		rates.push_back(freq/2);
		rates.push_back(freq/5);

		freq /= 10;
	}

	return rates;
}

vector<size_t> DigilentSCPIServer::GetSampleDepths()
{
	int bufsizeMin;
	int bufsizeMax;
	if(!FDwfAnalogInBufferSizeInfo(g_hScope, &bufsizeMin, &bufsizeMax))
		LogError("FDwfAnalogInBufferSizeInfo failed\n");

	//for now only report max depth
	vector<size_t> depths;
	depths.push_back(bufsizeMax);
	return depths;
}

bool DigilentSCPIServer::OnCommand(
		const string& line,
		const string& subject,
		const string& cmd,
		const vector<string>& args)
{
	if(BridgeSCPIServer::OnCommand(line, subject, cmd, args))
		return true;

	else if( (cmd == "ATTEN") && (args.size() == 1) )
	{
		lock_guard<mutex> lock(g_mutex);

		size_t channelId;
		if(!GetChannelID(subject, channelId))
			return false;

		double requestedAtten = stod(args[0]);
		if(!FDwfAnalogInChannelAttenuationSet(g_hScope, channelId, requestedAtten))
			LogError("FDwfAnalogInChannelAttenuationSet failed\n");

		//need to re-arm trigger to apply changes
		if(g_triggerArmed)
			Start();
	}

	//Unknown
	else
	{
		LogDebug("Unrecognized command received: %s\n", line.c_str());
		LogIndenter li;
		LogDebug("Subject: %s\n", subject.c_str());
		LogDebug("Command: %s\n", cmd.c_str());
		for(auto arg : args)
			LogDebug("Arg: %s\n", arg.c_str());

		return false;
	}

	return true;
}

void DigilentSCPIServer::AcquisitionStart(bool oneShot)
{
	lock_guard<mutex> lock(g_mutex);

	if(g_triggerArmed)
	{
		LogVerbose("Ignoring START command because trigger is already armed\n");
		return;
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

	//for(size_t i=0; i<g_numDigitalPods; i++)
	//{
	//	if(g_msoPodEnabled[i])
	//	{
	//		anyChannels = true;
	//		break;
	//	}
	//}

	if(!anyChannels)
	{
		LogVerbose("Ignoring START command because no channels are active\n");
		return;
	}

	//Start the capture
	Start();
	g_triggerOneShot = oneShot;
}

void DigilentSCPIServer::AcquisitionForceTrigger()
{
	lock_guard<mutex> lock(g_mutex);
	Start(true);
}

void DigilentSCPIServer::AcquisitionStop()
{
	lock_guard<mutex> lock(g_mutex);
	Stop();
}

void DigilentSCPIServer::SetChannelEnabled(size_t chIndex, bool enabled)
{
	lock_guard<mutex> lock(g_mutex);
	g_channelOn[chIndex] = enabled;

	if(!FDwfAnalogInChannelEnableSet(g_hScope, chIndex, enabled))
		LogError("FDwfAnalogInChannelEnableSet failed\n");

	//We need to allocate new buffers for this channel
	g_memDepthChanged = true;

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetAnalogCoupling(size_t chIndex, const std::string& coupling)
{
	lock_guard<mutex> lock(g_mutex);
	DwfAnalogCoupling coup;
	if(coupling == "DC1M")
		coup = DwfAnalogCouplingDC;
	else// if(coupling == "AC1M")
		coup = DwfAnalogCouplingAC;

	if(!FDwfAnalogInChannelCouplingSet(g_hScope, chIndex, coup))
		LogError("FDwfAnalogInChannelCouplingSet failed\n");
}

void DigilentSCPIServer::SetAnalogRange(size_t chIndex, double range_V)
{
	lock_guard<mutex> lock(g_mutex);
	if(!FDwfAnalogInChannelRangeSet(g_hScope, chIndex, range_V))
		LogError("FDwfAnalogInChannelRangeSet failed\n");

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetAnalogOffset(size_t chIndex, double offset_V)
{
	lock_guard<mutex> lock(g_mutex);

	if(!FDwfAnalogInChannelOffsetSet(g_hScope, chIndex, offset_V))
		LogError("FDwfAnalogInChannelOffsetSet failed\n");

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetDigitalThreshold(size_t /*chIndex*/, double /*threshold_V*/)
{
	//not yet supported
}

void DigilentSCPIServer::SetDigitalHysteresis(size_t /*chIndex*/, double /*hysteresis*/)
{
	//not yet supported
}

void DigilentSCPIServer::SetSampleRate(uint64_t rate_hz)
{
	lock_guard<mutex> lock(g_mutex);

	if(!FDwfAnalogInFrequencySet(g_hScope, rate_hz))
		LogError("FDwfAnalogInFrequencySet failed\n");
	g_sampleInterval = FS_PER_SECOND / rate_hz;

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetSampleDepth(uint64_t depth)
{
	lock_guard<mutex> lock(g_mutex);
	g_memDepth = depth;
	if(!FDwfAnalogInBufferSizeSet(g_hScope, g_memDepth))
		LogError("FDwfAnalogInBufferSizeSet failed\n");

	g_memDepthChanged = true;

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetTriggerDelay(uint64_t delay_fs)
{
	lock_guard<mutex> lock(g_mutex);
	g_triggerDelay = delay_fs;

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

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetTriggerSource(size_t chIndex)
{
	lock_guard<mutex> lock(g_mutex);

	if(!FDwfAnalogInTriggerSourceSet(g_hScope, trigsrcDetectorAnalogIn))
		LogError("FDwfAnalogInTriggerSourceSet failed\n");

	if(!FDwfAnalogInTriggerAutoTimeoutSet(g_hScope, 0))
		LogError("FDwfAnalogInTriggerAutoTimeoutSet failed\n");

	g_triggerChannel = chIndex;
	if(!FDwfAnalogInTriggerChannelSet(g_hScope, g_triggerChannel))
		LogError("FDwfAnalogInTriggerChannelSet failed\n");

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetTriggerLevel(double level_V)
{
	lock_guard<mutex> lock(g_mutex);

	g_triggerVoltage = level_V;
	if(!FDwfAnalogInTriggerLevelSet(g_hScope, g_triggerVoltage))
		LogError("FDwfAnalogInTriggerLevelSet failed\n");

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetTriggerTypeEdge()
{
	lock_guard<mutex> lock(g_mutex);
	if(!FDwfAnalogInTriggerTypeSet(g_hScope, trigtypeEdge))
		LogError("FDwfAnalogInTriggerTypeSet failed\n");

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::SetEdgeTriggerEdge(const std::string& edge)
{
	lock_guard<mutex> lock(g_mutex);

	DwfTriggerSlope condition;
	if(edge == "RISING")
		condition = DwfTriggerSlopeRise;
	else if(edge == "FALLING")
		condition = DwfTriggerSlopeFall;
	else// if(edge == "ANY")
		condition = DwfTriggerSlopeEither;

	if(!FDwfAnalogInTriggerConditionSet(g_hScope, condition))
		LogError("FDwfAnalogInTriggerConditionSet failed\n");

	RestartTriggerIfArmed();
}

void DigilentSCPIServer::Stop()
{
	FDwfAnalogInConfigure(g_hScope, true, false);
	g_triggerArmed = false;
}

void DigilentSCPIServer::Start(bool force)
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
