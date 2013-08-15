#include "MBusAnalyzer.h"
#include "MBusAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

MBusAnalyzer::MBusAnalyzer()
:	Analyzer(),
	mSettings( new MBusAnalyzerSettings() ),
	mSimulationInitilized( false )
{
	SetAnalyzerSettings( mSettings.get() );
}

MBusAnalyzer::~MBusAnalyzer()
{
	KillThread();
}

void MBusAnalyzer::WorkerThread()
{
	mResults.reset( new MBusAnalyzerResults( this, mSettings.get() ) );
	SetAnalyzerResults( mResults.get() );
	mResults->AddChannelBubblesWillAppearOn( mSettings->mMasterDATChannel );
	for (int i=0; i < mSettings->mMemberCount; i++) {
		mResults->AddChannelBubblesWillAppearOn( mSettings->mMemberDATChannels[i] );
	}

	mSampleRateHz = GetSampleRate();

	mMasterCLK = GetAnalyzerChannelData( mSettings->mMasterCLKChannel );
	mMasterDAT = GetAnalyzerChannelData( mSettings->mMasterDATChannel );
	{
		mMemberCLKs.resize( mSettings->mMemberCount );
		mMemberDATs.resize( mSettings->mMemberCount );
	}
	for (int i=0; i < mSettings->mMemberCount; i++) {
		mMemberCLKs.at(i) = GetAnalyzerChannelData( mSettings->mMemberCLKChannels[i] );
		mMemberDATs.at(i) = GetAnalyzerChannelData( mSettings->mMemberDATChannels[i] );
	}
	// For convenience, we alias to a generic list of node w/ master at idx 0
	{
		mNodeCLKs.resize( mSettings->mMemberCount + 1 );
		mNodeDATs.resize( mSettings->mMemberCount + 1 );
	}
	mNodeCLKs.at(0) = mMasterCLK;
	mNodeDATs.at(0) = mMasterDAT;
	for (int i=1; i < 1+mSettings->mMemberCount; i++) {
		mNodeCLKs.at(i) = mMemberCLKs.at(i-1);
		mNodeDATs.at(i) = mMemberDATs.at(i-1);
	}

	if (( mMasterCLK->GetBitState() == BIT_LOW ) || ( mMasterDAT->GetBitState() == BIT_LOW))
		AnalyzerHelpers::Assert("Master node clock / data lines must be high at time 0. Analyzer does not support starting in the middle of a transaction");
	for (int i=0; i < mSettings->mMemberCount; i++) {
		if (( mMemberCLKs.at(i)->GetBitState() == BIT_LOW ) || (mMemberDATs.at(i)->GetBitState() == BIT_LOW))
			AnalyzerHelpers::Assert("Member node clock / data lines must be high at time 0. Analyzer does not support starting in the middle of a transaction");
	}

	mLastNodeCLK = mMemberCLKs.at(mMemberCLKs.size()-1);
	mLastNodeDAT = mMemberDATs.at(mMemberDATs.size()-1);

	while (true) {
		Process_IdleToArbitration();
		Process_ArbitrationToPriorityArbitration();
		Process_PriorityArbitrationToAddress();
		Process_AddressToData();
		Process_DataToInterrupt();
		Process_InterruptToControl();
		Process_ControlToIdle();
	}
}

bool MBusAnalyzer::AdvanceAllTo(U64 sample, bool interruptable) {
	bool interrupted = false;

	for (int i=0; i<mNodeCLKs.size(); i++) {
		mNodeCLKs.at(i)->AdvanceToAbsPosition(sample);
		if (mNodeDATs.at(i)->AdvanceToAbsPosition(sample) > 3) {
			if (!interruptable) {
				AnalyzerHelpers::Assert("Unexpected Interrupt: The Analyzer isn't very good at handling Interrupts when they don't occur on clean 32-bit data boundaries. Things are probably garbage after here.");
			}
			interrupted = true;
		}
	}

	return interrupted;
}

#include <stdio.h>
#include <signal.h>

void MBusAnalyzer::Process_IdleToArbitration() {
	Frame frame;
	frame.mFlags = 0;
	frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

	// Advance LastNodeCLK channel to end of t_long
	mLastNodeCLK->AdvanceToNextEdge();

	bool requested[ mNodeDATs.size() ];
	for (int i=0; i < mNodeDATs.size(); i++) requested[i] = false;

	U64  DOUT_Fall[ mNodeDATs.size() ];

	// First find the sample number when each node's DOUT falls
	for (int i=0; i < mNodeDATs.size(); i++) {
		/* It is theoretically possible for DOUTs to never fall until
		 * data transmission, so advancing to the next edge is unsafe.
		 * Instead we peek to the end of t_long to see if the node is
		 * participating.
		 */
		U64 SamplesTo_t_long = mLastNodeCLK->GetSampleNumber() - mNodeDATs[i]->GetSampleNumber();
		if (mNodeDATs[i]->WouldAdvancingCauseTransition(SamplesTo_t_long)) {
			// Node is participating
			mNodeDATs[i]->AdvanceToNextEdge();
			DOUT_Fall[i] = mNodeDATs[i]->GetSampleNumber();

			// Handle master node here, if its DOUT ever fell it requested
			if (i==0)
				requested[0] = true;
		} else {
			// Node is not participating, set a fake fall value past any participants
			DOUT_Fall[i] = mLastNodeCLK->GetSampleNumber()+1;
		}
	}
	// Now go through each member node. If a node's DOUT falls _before_
	// the previous node's DOUT fell then this node is requesting.
	for (int i=1; i < mNodeDATs.size(); i++) {
		if (DOUT_Fall[i] < DOUT_Fall[i-1]) // min i==1, i-1 ref is safe
			requested[i] = true;
	}

	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

	frame.mData1 = 0;
	for (int i=0; i<mNodeDATs.size(); i++) {
		frame.mData1 |= ((U64) (requested[i])) << (i + 32);
	}
	frame.mData2 = 1;
	frame.mType = FrameTypeRequest;

	frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
	mResults->AddFrame(frame);
	mResults->CommitResults();
	ReportProgress( mLastNodeCLK->GetSampleNumber() );
}

void MBusAnalyzer::Process_ArbitrationToPriorityArbitration() {
	Frame frame;
	frame.mFlags = 0;
	frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

	// Latch Arbitration
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

	int arbitrationWinner = -1;
	if (mMasterDAT->GetBitState() == BIT_LOW)
		arbitrationWinner = 0;
	else {
		for (int i=1; i<mNodeDATs.size(); i++) {
			if (
					(mNodeDATs.at((i-1)%mNodeDATs.size())->GetBitState() == BIT_HIGH) &&
					(mNodeDATs.at(i)->GetBitState() == BIT_LOW)
			   ) {
				if (arbitrationWinner != -1)
					frame.mFlags |= MULTIPLE_ARBITRATION_WINNER | DISPLAY_AS_ERROR_FLAG;
				arbitrationWinner = i;
			}
		}
	}
	if (arbitrationWinner == -1)
		AnalyzerHelpers::Assert("Analyzer does not yet support 'No Arbitration Winner' case");
	mTransmitter = arbitrationWinner;

	frame.mData1 = 0;
	for (int i=0; i<mNodeDATs.size(); i++) {
		frame.mData1 |= ((U64) (arbitrationWinner == i)) << (i+32);
	}
	frame.mData2 = 1;
	frame.mType = FrameTypeArbitration;

	// Get to Prio Drive edge before ending this frame
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

	frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
	mResults->AddFrame(frame);
	mResults->CommitResults();
	ReportProgress( mLastNodeCLK->GetSampleNumber() );
}

void MBusAnalyzer::Process_PriorityArbitrationToAddress() {
	Frame frame;
	frame.mFlags = 0;
	frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

	// Latch Prio Drive
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

	int prioArbitrationWinner = -1;
	if (mNodeDATs.at(mTransmitter)->GetBitState() == BIT_LOW)
		prioArbitrationWinner = mTransmitter;
	else {
		int start = (mTransmitter+1) % mNodeDATs.size();
		for (int j=start; j<start+mNodeDATs.size()-1; j++) {
			int k = j % mNodeDATs.size();
			if (
					(mNodeDATs.at((k-1)%mNodeDATs.size())->GetBitState() == BIT_LOW) &&
					(mNodeDATs.at(k)->GetBitState() == BIT_HIGH)
			   ) {
				if (prioArbitrationWinner != -1)
					frame.mFlags |= MULTIPLE_ARBITRATION_WINNER | DISPLAY_AS_ERROR_FLAG;
				prioArbitrationWinner = k;
			}
		}
	}
	if (prioArbitrationWinner != -1)
		mTransmitter = prioArbitrationWinner;

	frame.mData1 = 0;
	for (int i=0; i<mNodeDATs.size(); i++) {
		frame.mData1 |= ((U64) (prioArbitrationWinner == i)) << (i+32);
	}
	frame.mData2 = 1;
	frame.mType = FrameTypePriorityArbitration;

	// Get to Drive Bit 0 edge before ending this frame
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

	frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
	mResults->AddFrame(frame);
	mResults->CommitResults();
	ReportProgress( mLastNodeCLK->GetSampleNumber() );
}

void MBusAnalyzer::Process_AddressToData() {
	Frame frame;
	frame.mFlags = 0;
	frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

	U32 address = 0;

	for (int i=0; i < 8; i++) {
		// Latch Drive Bit N (Address is MSB)
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
		address <<= 1;
		address |= mLastNodeDAT->GetBitState() == BIT_HIGH;
		// Advance to Drive Bit N+1
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
	}

	if ((address & 0xf0) == 0xf0) {
		// This is 32-bit addr
		for (int i=0; i < 24; i++) {
			// Latch Drive Bit N (Address is MSB)
			mLastNodeCLK->AdvanceToNextEdge();
			AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
			address <<= 1;
			address |= mLastNodeDAT->GetBitState() == BIT_HIGH;
			// Advance to Drive Bit N+1
			mLastNodeCLK->AdvanceToNextEdge();
			AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
		}
	}

	frame.mData1 = address;
	frame.mType = FrameTypeAddress;

	frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
	mResults->AddFrame(frame);
	mResults->CommitResults();
	ReportProgress( mLastNodeCLK->GetSampleNumber() );
}

void MBusAnalyzer::Process_DataToInterrupt() {
	bool interrupted = false;
	do {
		Frame frame;
		frame.mFlags = 0;
		frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

		U32 data = 0;

		for (int i=0; i < 31; i++) {
			// Latch Drive Bit N (Data is MSB, currently assume words)
			mLastNodeCLK->AdvanceToNextEdge();
			AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
			data <<= 1;
			data |= mLastNodeDAT->GetBitState() == BIT_HIGH;
			// Advance to Drive Bit N+1
			mLastNodeCLK->AdvanceToNextEdge();
			AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
		}
		// Latch Drive Bit 32
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
		data <<= 1;
		data |= mLastNodeDAT->GetBitState() == BIT_HIGH;
		// Advance to Drive Bit of next word; may interrupt
		mLastNodeCLK->AdvanceToNextEdge();
		interrupted = AdvanceAllTo( mLastNodeCLK->GetSampleNumber(), true );

		frame.mData1 = data;
		frame.mType = FrameTypeData;

		frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
		mResults->AddFrame(frame);
		mResults->CommitResults();
		ReportProgress( mLastNodeCLK->GetSampleNumber() );
	} while (!interrupted);
}

void MBusAnalyzer::Process_InterruptToControl() {
	// nop; this is currently subsumed by the DataToInterrupt; FIXME
}

void MBusAnalyzer::Process_ControlToIdle() {
	// This currently picks up having processed the drive Begin Control CLK edge
	//
	// Silently consume period that should eventually be marked as part of Interrupt:
	// Latch Begin Control:
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
	// Drive Ctrl Bit 0
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

	// Control Bit 0
	{
		Frame frame;
		frame.mFlags = 0;
		frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

		// Latch Ctrl Bit 0
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

		frame.mData1 = mLastNodeDAT->GetBitState() == BIT_HIGH;
		frame.mType = FrameTypeControlBit0;

		// Extend this bubble up to Drive Ctrl Bit 1
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

		frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
		mResults->AddFrame(frame);
		mResults->CommitResults();
		ReportProgress( mLastNodeCLK->GetSampleNumber() );
	}

	// Control Bit 1
	{
		Frame frame;
		frame.mFlags = 0;
		frame.mStartingSampleInclusive = mLastNodeCLK->GetSampleNumber()+1;

		// Latch Ctrl Bit 1
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

		frame.mData1 = mLastNodeDAT->GetBitState() == BIT_HIGH;
		frame.mType = FrameTypeControlBit1;

		// Extend this bubble up to Drive Begin Idle
		mLastNodeCLK->AdvanceToNextEdge();
		AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );

		frame.mEndingSampleInclusive = mLastNodeCLK->GetSampleNumber();
		mResults->AddFrame(frame);
		mResults->CommitResults();
		ReportProgress( mLastNodeCLK->GetSampleNumber() );
	}

	// Silently consume Latch Begin Idle (good enough until we have real Idle solution)
	mLastNodeCLK->AdvanceToNextEdge();
	AdvanceAllTo( mLastNodeCLK->GetSampleNumber() );
}

bool MBusAnalyzer::NeedsRerun()
{
	return false;
}

U32 MBusAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
	if( mSimulationInitilized == false )
	{
		mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
		mSimulationInitilized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 MBusAnalyzer::GetMinimumSampleRateHz()
{
	// "Rule of Thumb" is bit rate * 4; current MBus @ 400 kHz
	return 400e3 * 4;
}

const char* MBusAnalyzer::GetAnalyzerName() const
{
	return "MBus";
}

const char* GetAnalyzerName()
{
	return "MBus";
}

Analyzer* CreateAnalyzer()
{
	return new MBusAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
	delete analyzer;
}
