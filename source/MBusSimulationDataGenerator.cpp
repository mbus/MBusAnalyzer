#include "MBusSimulationDataGenerator.h"
#include "MBusAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

#include <iostream>
#include <fstream>
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
	for (size_t i=0; i<mNodeCLKSimulationDatas.size(); i++)
		min = (min < mNodeCLKSimulationDatas.at(i)->GetCurrentSampleNumber()) ? min : mNodeCLKSimulationDatas.at(i)->GetCurrentSampleNumber();
	for (size_t i=0; i<mNodeDATSimulationDatas.size(); i++)
		min = (min < mNodeDATSimulationDatas.at(i)->GetCurrentSampleNumber()) ? min : mNodeDATSimulationDatas.at(i)->GetCurrentSampleNumber();
	return min;
}

U32 MBusSimulationDataGenerator::GenerateSimulationData( U64 largest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel )
{
	U64 adjusted_largest_sample_requested = AnalyzerHelpers::AdjustSimulationTargetSample( largest_sample_requested, sample_rate, mSimulationSampleRateHz );

	while( minSampleNumber() < adjusted_largest_sample_requested )
	{
		U8 data[256];
		data[0] = 0x12; data[1] = 0x34; data[2] = 0x56; data[3] = 0x78;
		CreateMBusTransaction(0, 0xA1, 4, data, false);
		CreateMBusTransaction(0, 0xF00000B2, 4, data, false);
		data[4] = 0x87; data[5] = 0x65; data[6] = 0x43; data[7] = 0x21;
		CreateMBusTransaction(0, 0xC3, 8, data, true);
		CreateMBusTransaction(0, 0xF00000D4, 8, data, true);
		data[4] = 0x9A; data[5] = 0xBC;
		CreateMBusTransaction(0, 0xE5, 6, data, false);

		// Can't generate this sequence if we don't have any member nodes
		if (mNodeCount > 1) {
			mSettings->log_hack << "\nSIM: Before CreateMBusWakeup" << std::endl;
			CreateMBusWakeup(1);
		} else {
			mSettings->log_hack << "\nSIM: Skipping CreateMBusWakeup because only one node" << std::endl;
		}

		// Imager messages: To address 0x17, 1 row / message
		// 8 bits/column x 160 columns = 160 bytes / message
		// There are 160 rows. Closes with a single 32-bit message
		for (int i=0; i < 160; i++) {
			for (int j=0; j < 160; j++)
				data[j] = (j+i) % 255;
			CreateMBusTransaction(0, 0x17, 160, data, false);
		}
		data[0] = 0xa5; data[1] = 0xa5; data[2] = 0xa5; data[3] = 0xa5;
		CreateMBusTransaction(0, 0x17, 4, data, false);
	}

	*simulation_channel = mMBusSimulationChannels.GetArray();
	return mMBusSimulationChannels.GetCount();
}

void MBusSimulationDataGenerator::PropogationDelay() {
	mMBusSimulationChannels.AdvanceAll( std::rand() % 3 + 1 );
}

void MBusSimulationDataGenerator::CreateMBusWakeup(int sender) {
	for (int i=0; i < mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction must be entered with all lines high (Fail CLK)" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusTransaction must be entered with all lines high (Fail CLK)");
		}
		if (mNodeDATSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction must be entered with all lines high (Fail DAT)" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusTransaction must be entered with all lines high (Fail DAT)");
		}
	}

	// Some space before we start
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(50) );

	{
		std::vector< bool > normal( mNodeCount, false );
		std::vector< bool > priority( mNodeCount, false );
		std::vector< bool > wakeup( mNodeCount, false );

		wakeup.at(sender) = true;
		CreateMBusArbitration(normal, priority, wakeup); // Through PrioLatch inclusive

		for (int i=0; i<mNodeCount; i++) {
			if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
				mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusArbitration did not exit with CLK's high?" << std::endl;
				AnalyzerHelpers::Assert("CreateMBusArbitration did not exit with CLK's high?");
			}
		}
	}

	CreateMBusInterrupt(0); // Through Interrupt Asserted edge inclusive
	for (int i=0; i<mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusInterrupt did not exit with CLK's high?" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusInterrupt did not exit with CLK's high?");
		}
	}
	CreateMBusControl(0, BIT_LOW, 0, BIT_LOW); // Through Begin Idle latch inclusive
	for (int i=0; i<mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusControl did not exit with CLK's high?" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusControl did not exit with CLK's high?");
		}
	}

	// Some space after the end
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(20) );

	// Sanity check: Make sure we left all the lines high
	for (int i=0; i < mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction should exit with all lines high (Fail CLK)" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusTransaction should exit with all lines high (Fail CLK)");
		}
		if (mNodeDATSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction should exit with all lines high (Fail DAT)" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusTransaction should exit with all lines high (Fail DAT)");
		}
	}
}

void MBusSimulationDataGenerator::CreateMBusTransaction(int sender, U32 address, U8 num_bytes, U8 data[], bool acked) {
	mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction start" << std::endl;

	for (int i=0; i < mNodeCount; i++) {
		if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction must be entered with all lines high (Fail CLK)" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusTransaction must be entered with all lines high (Fail CLK)");
		}
		if (mNodeDATSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction must be entered with all lines high (Fail DAT)" << std::endl;
			AnalyzerHelpers::Assert("CreateMBusTransaction must be entered with all lines high (Fail DAT)");
		}
	}

	// Some space before we start
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(20) );

	{
		std::vector< bool > normal( mNodeCount, false );
		std::vector< bool > priority( mNodeCount, false );

		normal.at(sender) = true;
		CreateMBusArbitration(normal, priority); // Through PrioLatch inclusive

		for (int i=0; i<mNodeCount; i++) {
			if (mNodeCLKSimulationDatas.at(i)->GetCurrentBitState() != BIT_HIGH) {
				mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusArbitration did not exit with CLK's high?" << std::endl;
				AnalyzerHelpers::Assert("CreateMBusArbitration did not exit with CLK's high?");
			}
		}
	}

	CreateMBusData(sender, address, num_bytes, data); // Through last Data Bit latch inclusive
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

	mSettings->log_hack << "SIM: " << __LINE__ << ": CreateMBusTransaction end" << std::endl;
}

void MBusSimulationDataGenerator::CreateMBusArbitration(std::vector< bool > normal, std::vector< bool > priority) {
	std::vector< bool > wakeup( normal.size(), false );
	CreateMBusArbitration(normal, priority, wakeup);
}

void MBusSimulationDataGenerator::CreateMBusArbitration(std::vector< bool > normal, std::vector< bool > priority, std::vector< bool > wakeup) {
	mSettings->log_hack << "SIM: CreateMBusArbitration Begin" << std::endl;

	std::vector< size_t > arb_order;
	for (size_t i=0; i<normal.size(); i++) {
		arb_order.push_back(i);
	}
	std::random_shuffle(arb_order.begin(), arb_order.end());

	// Generate request signal(s)
	mSettings->log_hack << "SIM: gen request sigs" << std::endl;
	for (size_t i=0; i<arb_order.size(); i++) {
		if (normal.at(i) || wakeup.at(i))
			mNodeDATSimulationDatas.at(i)->TransitionIfNeeded( BIT_LOW );
		PropogationDelay();

		// Propogate this node's request
		for (unsigned j=i; j<i+mNodeCount; j++) {
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
	mSettings->log_hack << "SIM: t_long" << std::endl;
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(5) );

	// Falling clock to start transaction
	mSettings->log_hack << "SIM: fall clock to start tx" << std::endl;
	bool any_request = false;
	for (size_t i=0; i<normal.size(); i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();

		// Deassert wakeup requests; assumes master is node 0
		if (normal.at(i) || any_request) {
			any_request = true;
		} else {
			mNodeDATSimulationDatas.at(i)->TransitionIfNeeded (BIT_HIGH);
		}
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Arbitration Edge
	mSettings->log_hack << "SIM: arb edge" << std::endl;
	for (size_t i=0; i<normal.size(); i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Resolve arbitration winner internally
	mSettings->log_hack << "SIM: resolve arb winner" << std::endl;
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

	// The timing here is too simple for now (all clocks then all datas),
	// but generating anything more is prohibitively complex for gen 1
	//
	// Prio Drive Edge (CLK)
	mSettings->log_hack << "SIM: prio drive edge clk" << std::endl;
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Prio Drive Edge (DAT)
	mSettings->log_hack << "SIM: prio drive edge dat" << std::endl;
	if (arbitrationWinner != -1) {
		// Only deal with prio if someone won arb
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
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Prio Latch Edge
	mSettings->log_hack << "SIM: prio latch edge" << std::endl;
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

	// Reserved Drive Edge (CLK)
	mSettings->log_hack << "SIM: rsv clk" << std::endl;
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	// Reserved Drive Edge (DAT)
	mSettings->log_hack << "SIM: rsv dat" << std::endl;
	for (int i=0; i<mNodeCount; i++) {
		;
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	// Reserved Latch Edge
	mSettings->log_hack << "SIM: rsv latch" << std::endl;
	for (int i=0; i<mNodeCount; i++) {
		mNodeCLKSimulationDatas.at(i)->Transition();
		PropogationDelay();
	}
	mMBusSimulationChannels.AdvanceAll( mClockGenerator.AdvanceByHalfPeriod(1) );

	mSettings->log_hack << "SIM: Gen Arb Done" << std::endl;
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

void MBusSimulationDataGenerator::CreateMBusData(int sender, U32 address, U8 num_bytes, U8 data[]) {
	if (address > 0xff) {
		if ((address & 0xf0000000) != 0xf0000000) {
			mSettings->log_hack << "SIM: " << __LINE__ << ": Big address but top bits not 0xf " << address << std::endl;
			AnalyzerHelpers::Assert("If address is greater than 8 bits, the top 4 bits *must* be 0xf");
		}
		BitExtractor addressBits(address, AnalyzerEnums::MsbFirst, 32);
		for (int i=0; i < 32; i++)
			CreateMBusBit(sender, addressBits.GetNextBit());
	} else {
		BitExtractor addressBits(address, AnalyzerEnums::MsbFirst, 8);
		for (int i=0; i < 8; i++)
			CreateMBusBit(sender, addressBits.GetNextBit());
	}

	for (int b=0; b < num_bytes; b++) {
		BitExtractor dataBits(data[b], AnalyzerEnums::MsbFirst, 8);
		for (int i=0; i < 8; i++)
			CreateMBusBit(sender, dataBits.GetNextBit());
	}
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
