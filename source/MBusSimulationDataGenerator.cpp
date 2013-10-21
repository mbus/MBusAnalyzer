#include "MBusSimulationDataGenerator.h"
#include "MBusAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

#include <cstdlib>
#include <algorithm> // std::random_shuffle

MBusSimulationDataGenerator::MBusSimulationDataGenerator()
{
}

MBusSimulationDataGenerator::~MBusSimulationDataGenerator()
{
}

void MBusSimulationDataGenerator::Initialize( U32 simulation_sample_rate, MBusAnalyzerSettings* settings )
{
	mSimulationSampleRateHz = simulation_sample_rate;
	mSettings = settings;
	mNodeCount = mSettings->mMemberCount + 1;

	mClockGenerator.Init(400e3, mSimulationSampleRateHz);

	{
		mNodeCLKSimulationDatas.resize(mNodeCount);
		mNodeDATSimulationDatas.resize(mNodeCount);
	}
	mNodeCLKSimulationDatas.at(0) = mMBusSimulationChannels.Add( mSettings->mMasterCLKChannel, simulation_sample_rate, BIT_HIGH);
	mNodeDATSimulationDatas.at(0) = mMBusSimulationChannels.Add( mSettings->mMasterDATChannel, simulation_sample_rate, BIT_HIGH);
	for (int i=1; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i) = mMBusSimulationChannels.Add( mSettings->mMemberCLKChannels[i-1], simulation_sample_rate, BIT_HIGH);
		mNodeDATSimulationDatas.at(i) = mMBusSimulationChannels.Add( mSettings->mMemberDATChannels[i-1], simulation_sample_rate, BIT_HIGH);
	}
}

U64 MBusSimulationDataGenerator::minSampleNumber() {
	U64 min = UINT64_MAX;
	for (int i=0; i<mNodeCLKSimulationDatas.size(); i++)
		min = (min < mNodeCLKSimulationDatas.at(i)->GetCurrentSampleNumber()) ? min : mNodeCLKSimulationDatas.at(i)->GetCurrentSampleNumber();
	for (int i=0; i<mNodeDATSimulationDatas.size(); i++)
		min = (min < mNodeDATSimulationDatas.at(i)->GetCurrentSampleNumber()) ? min : mNodeDATSimulationDatas.at(i)->GetCurrentSampleNumber();
	return min;
}

U32 MBusSimulationDataGenerator::GenerateSimulationData( U64 largest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel )
{
	U64 adjusted_largest_sample_requested = AnalyzerHelpers::AdjustSimulationTargetSample( largest_sample_requested, sample_rate, mSimulationSampleRateHz );

	while( minSampleNumber() < adjusted_largest_sample_requested )
	{
		CreateMBusTransaction(0, 0xA5, 0x12345678, false);
	}

	*simulation_channel = mMBusSimulationChannels.GetArray();
	return mMBusSimulationChannels.GetCount();
}

void MBusSimulationDataGenerator::PropogationDelay() {
	mMBusSimulationChannels.AdvanceAll( std::rand() % 3 + 1 );
}

void MBusSimulationDataGenerator::CreateMBusTransaction(int sender, U8 address, U32 data, bool acked) {
	static int num_calls = 0;
	num_calls++;
	for (int i=0; i < mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusTransaction must be entered with all lines high (Fail CLK)");
		if (mNodeDATSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusTransaction must be entered with all lines high (Fail DAT)");
	}

	// Some space before we start
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(10) );

	{
		std::vector< bool > normal( mNodeCount, false );
		std::vector< bool > priority( mNodeCount, false );

		normal.at(sender) = true;
		CreateMBusArbitration(normal, priority); // Through PrioLatch inclusive

		for (int i=0; i<mNodeCount; i++) {
			if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
				AnalyzerHelpers::Assert("CreateMBusArbitration did not exit with CLK's high?");
		}
	}

	CreateMBusData(sender, address, data); // Through last Data Bit latch inclusive
	for (int i=0; i<mNodeCount; i++)
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusData did not exit with CLK's high?");
	CreateMBusInterrupt(sender); // Through Interrupt Asserted edge inclusive
	for (int i=0; i<mNodeCount; i++)
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusInterrupt did not exit with CLK's high?");
	CreateMBusControl(sender, BIT_HIGH, address & 0xf, (acked) ? BIT_LOW : BIT_HIGH); // Through Begin Idle latch inclusive
	for (int i=0; i<mNodeCount; i++)
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusControl did not exit with CLK's high?");

	// Some space after the end
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(20) );

	// Sanity check: Make sure we left all the lines high
	for (int i=0; i < mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusTransaction should exit with all lines high (Fail CLK)");
		if (mNodeDATSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH)
			AnalyzerHelpers::Assert("CreateMBusTransaction should exit with all lines high (Fail DAT)");
	}
}

void MBusSimulationDataGenerator::CreateMBusArbitration(std::vector< bool > normal, std::vector< bool > priority) {
	std::vector< int > arb_order;
	for (int i=0; i<normal.size(); i++) {
		arb_order.push_back(i);
	}
	std::random_shuffle(arb_order.begin(), arb_order.end());

	// Generate request signal(s)
	for (int i=0; i<arb_order.size(); i++) {
		if (normal.at(i))
			mNodeDATSimulationDatas.at(i)->TransitionIfNeeded( BIT_LOW );
		PropogationDelay();

		// Propogate this node's request
		for (int j=i; j<i+mNodeCount; j++) {
			int k = j % mNodeCount;
			//  Ignore master node as it doesn't forward
			if (k == 0)
				continue;

			if (mNodeDATSimulationDatas.at((k-1)%normal.size())->GetCurrentBitState() == BIT_LOW)
				mNodeDATSimulationDatas.at(k)->TransitionIfNeeded( BIT_LOW );
			PropogationDelay();
		}
	}

	// "t_long"
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(5) );

	// Falling clock to start transaction
	for (int i=0; i<normal.size(); i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Arbitration Edge
	for (int i=0; i<normal.size(); i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Resolve arbitration winner internally
	int arbitrationWinner = -1;
	if (mNodeDATSimulationDatas.at(0)->GetCurrentBitState() == BIT_LOW)
		arbitrationWinner = 0;
	else {
		for (int j=1; j<mNodeCount; j++) {
			if (
					(mNodeDATSimulationDatas.at((j-1)%mNodeCount)->GetCurrentBitState() == BIT_HIGH) &&
					(mNodeDATSimulationDatas.at(j)->GetCurrentBitState() == BIT_LOW)
			   ) {
				if (arbitrationWinner != -1)
					AnalyzerHelpers::Assert("Multiple normal arbitration winners?");
				arbitrationWinner = j;
			}
		}
	}
	if (arbitrationWinner == -1)
		AnalyzerHelpers::Assert("No arbitration winner not currently supported");

	// The timing here is too simple for now (all clocks then all datas),
	// but generating anything more is prohibitively complex for gen 1
	//
	// Prio Drive Edge (CLK)
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Prio Drive Edge (DAT)
	for (int i=0; i<mNodeCount; i++) {
		if (priority.at(i))
			mNodeDATSimulationDatas.at(i)->TransitionIfNeeded( BIT_HIGH );
		PropogationDelay();

		// Propogate this node's request
		for (int j=i; j<i+mNodeCount; j++) {
			int k = j % mNodeCount;
			// Ignore arbitration winner as it doesn't forward
			if (k == arbitrationWinner)
				continue;

			if (mNodeDATSimulationDatas.at((k-1)%mNodeCount)->GetCurrentBitState() == BIT_HIGH)
				mNodeDATSimulationDatas.at(k)->TransitionIfNeeded( BIT_HIGH );
			PropogationDelay();
		}
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Prio Latch Edge
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	/* This isn't actually needed, but would work as of gen 1
	 *
	// Resolve priority arbitration winner internally
	int priorityArbitrationWinner = -1;
	if (mNodeDATSimulationDatas.at(arbitrationWinner)->GetCurrentBitState() == BIT_LOW)
		priorityArbitrationWinner = arbitrationWinner;
	else {
		for (int j=arbitrationWinner; j<arbitrationWinner+mNodeCount; j++) {
			int k = j % mNodeCount;

			if (
					(mNodeDATSimulationDatas.at((k-1)%mNodeCount)->GetCurrentBitState() == BIT_LOW) &&
					(mNodeDATSimulationDatas.at(k)->GetCurrentBitState() == BIT_HIGH)
			   ) {
				if (priorityArbitrationWinner != -1)
					AnalyzerHelpers::Assert("Multiple priority arbitration winners?");
				priorityArbitrationWinner = k;
			}
		}
	}
	 *
	 */
}

void MBusSimulationDataGenerator::CreateMBusBit(int sender, BitState bit) {
	// Simplistic timing / prop still
	//
	// Drive Bit N (CLK)
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Drive Bit N (DAT)
	for (int j=sender; j<sender+mNodeCount; j++) {
		int k = j % mNodeCount;
		mNodeDATSimulationDatas.at(k)->TransitionIfNeeded( bit );
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Latch Bit N
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );
}

void MBusSimulationDataGenerator::CreateMBusData(int sender, U8 address, U32 data) {
	BitExtractor addressBits(address, AnalyzerEnums::MsbFirst, 8);
	BitExtractor dataBits(data, AnalyzerEnums::MsbFirst, 32);

	for (int i=0; i < 8; i++)
		CreateMBusBit(sender, addressBits.GetNextBit());
	for (int i=0; i < 32; i++)
		CreateMBusBit(sender, dataBits.GetNextBit());
}

void MBusSimulationDataGenerator::CreateMBusInterrupt(int interrupter) {
	// Generate blocked CLK pulses
	//
	// Drive Req Int
	for (int i=0; i<interrupter; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Latch Req Int
	for (int i=0; i<interrupter; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Drive Beg Int
	for (int i=0; i<interrupter; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Latch Beg Int
	for (int i=0; i<interrupter; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Master Drives All Data High
	for (int i=0; i<mNodeCount; i++) {
		mNodeDATSimulationDatas.at(i)->TransitionIfNeeded( BIT_HIGH );
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Generate Interrupt Pulses
	//
	const int MBUS_NUM_INTERRUPT_PULSES = 3;
	for (int p=0; p<MBUS_NUM_INTERRUPT_PULSES*2; p++) {
		for (int i=0; i<mNodeCount; i++) {
			mNodeDATSimulationDatas.at(i)->Transition();
			PropogationDelay();
		}
		mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );
	}
}

void MBusSimulationDataGenerator::CreateMBusControl(int interrupter, BitState cb0, int target, BitState cb1) {
	// "Drive" Begin Control
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// "Latch" Begin Control
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Drive CB0 (CLK)
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Drive CB0 (DAT)
	for (int j=interrupter; j < interrupter + mNodeCount; j++) {
		int k = j % mNodeCount;
		mNodeDATSimulationDatas.at(k)->TransitionIfNeeded( cb0 );
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Latch CB0
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Drive CB1 (CLK)
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Drive CB1 (DAT)
	for (int j=target; j < target + mNodeCount; j++) {
		int k = j % mNodeCount;
		mNodeDATSimulationDatas.at(k)->TransitionIfNeeded( cb1 );
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Latch CB1
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// "Drive" Begin Idle (CLK)
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// "Drive" Begin Idle (DAT) [master returns data line to high]
	for (int i=0; i<mNodeCount; i++) {
		mNodeDATSimulationDatas.at(i)->TransitionIfNeeded( BIT_HIGH );
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// "Latch" Begin Idle (CLK)
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// "Latch" Begin Idle (DAT)
	for (int i=0; i<mNodeCount; i++) {
		mNodeDATSimulationDatas.at(i)->TransitionIfNeeded( BIT_HIGH );
		PropogationDelay();
	}
}
