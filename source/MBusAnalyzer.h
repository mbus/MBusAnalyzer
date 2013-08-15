#ifndef MBUS_ANALYZER_H
#define MBUS_ANALYZER_H

#include <Analyzer.h>
#include "MBusAnalyzerResults.h"
#include "MBusSimulationDataGenerator.h"

class MBusAnalyzerSettings;
class ANALYZER_EXPORT MBusAnalyzer : public Analyzer
{
public:
	MBusAnalyzer();
	virtual ~MBusAnalyzer();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected: //vars
	std::auto_ptr< MBusAnalyzerSettings > mSettings;
	std::auto_ptr< MBusAnalyzerResults > mResults;
	AnalyzerChannelData* mMasterCLK;
	AnalyzerChannelData* mMasterDAT;
	std::vector< AnalyzerChannelData * > mMemberCLKs;
	std::vector< AnalyzerChannelData * > mMemberDATs;

	MBusSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitilized;

	//Serial analysis vars:
	U32 mSampleRateHz;
	U32 mStartOfStopBitOffset;
	U32 mEndOfStopBitOffset;

private: // analysis helpers:
	void Process_IdleToArbitration();
	void Process_ArbitrationToPriorityArbitration();
	void Process_PriorityArbitrationToAddress();
	void Process_AddressToData();
	void Process_DataToInterrupt();
	void Process_InterruptToControl();
	void Process_ControlToIdle();

	bool AdvanceAllTo(U64 sample, bool interruptable=false);

	AnalyzerChannelData* mLastNodeCLK;
	AnalyzerChannelData* mLastNodeDAT;

	std::vector< AnalyzerChannelData * > mNodeCLKs;
	std::vector< AnalyzerChannelData * > mNodeDATs;

	int mTransmitter;
	bool requestBugWorkaround;
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //MBUS_ANALYZER_H
