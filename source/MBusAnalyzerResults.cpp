#include "MBusAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "MBusAnalyzer.h"
#include "MBusAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <iomanip>

MBusAnalyzerResults::MBusAnalyzerResults( MBusAnalyzer* analyzer, MBusAnalyzerSettings* settings )
:	AnalyzerResults(),
	mSettings( settings ),
	mAnalyzer( analyzer )
{
}

MBusAnalyzerResults::~MBusAnalyzerResults()
{
}

int MBusAnalyzerResults::ChannelToIndex(Channel& channel) {
	if (channel == mSettings->mMasterDATChannel)
		return 0;
	for (int i=0; i<mSettings->mMemberCount; i++)
		if (channel == mSettings->mMemberDATChannels[i])
			return i+1;
	AnalyzerHelpers::Assert("Internal Error: Could not find channel index");
}

void MBusAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
	ClearResultStrings();
	Frame frame = GetFrame( frame_index );

	U32 data = (frame.mData1 & (1ULL << (ChannelToIndex( channel ) + 32))) ? frame.mData2 : frame.mData1;

	static bool singleton = false;
	static std::ofstream outfile;
	if (!singleton) {
		outfile.open("MBus_AnalyzerResults.out", std::ios::out | std::ios::trunc);
		singleton = true;
	}

	switch( frame.mType ) {
		case FrameTypeRequest:
			if (frame.mFlags & REQUEST_BUG_WORKAROUND) {
				AddResultString("...");
				AddResultString("Request Phase");
			} else {
				if (data) {
					AddResultString("R");
					AddResultString("Req");
					AddResultString("Requested");
				} else {
					AddResultString("!R");
					AddResultString("!Req");
					AddResultString("Did not Request");
				}
			}
			break;
		case FrameTypeArbitration:
			if (frame.mFlags & NO_ARBITRATION_WINNER) {
				AddResultString("~A");
				AddResultString("No Arb");
				AddResultString("No Arbitration Winner");
			} else if (data) {
				AddResultString("A");
				AddResultString("Arb");
				AddResultString("Won Arbitration");
			} else {
				AddResultString("!A");
				AddResultString("!Arb");
				AddResultString("Lost Arbitration");
				AddResultString("Lost Arbitration (Or Did Not Participate)");
			}
			break;
		case FrameTypePriorityArbitration:
			if (frame.mFlags & NO_ARBITRATION_WINNER) {
				AddResultString("~P");
				AddResultString("No Prio");
				AddResultString("No Priority Arbitration Winner");
			} else if (data) {
				AddResultString("P");
				AddResultString("Pri");
				AddResultString("Won Priority");
				AddResultString("Won Priority Arbitration");
			} else {
				AddResultString("!P");
				AddResultString("!Pri");
				AddResultString("Lost Priority");
				AddResultString("Lost Priority Arbitration");
				AddResultString("Lost Priority (Or Did Not Participate)");
				AddResultString("Lost Priority Arbitration (Or Did Not Participate)");
			}
			break;
		case FrameTypeReservedBit:
			AddResultString("V");
			AddResultString("Rsvd");
			AddResultString("Reserved");
			break;
		case FrameTypeAddress:
			{
				char number_str[64];
				int addr_len = ((data & 0xf0000000) == 0xf0000000) ? 32 : 8;
				AnalyzerHelpers::GetNumberString(data, display_base, addr_len, number_str, 64);

				AddResultString(number_str);

				char prefix_str[64];
				char fu_str[64];
				if (addr_len == 8)
					AnalyzerHelpers::GetNumberString((data >> 4) & 0xf, display_base, 4, prefix_str, 64);
				else
					AnalyzerHelpers::GetNumberString((data >> 4) & 0xfffff, display_base, 20, prefix_str, 64);
				AnalyzerHelpers::GetNumberString(data & 0xf, display_base, 4, fu_str, 64);

				AddResultString(prefix_str, " + ", fu_str);
				AddResultString("Prefix: ", prefix_str, " F.U. Addr: ", fu_str);

				outfile << "Prefix " << prefix_str << " FU_Addr " << fu_str << std::endl;
			}
			break;
		case FrameTypeData:
			{
				char number_str[64];
				AnalyzerHelpers::GetNumberString(data, display_base, 8, number_str, 64);

				AddResultString(number_str);
				AddResultString("Data: ", number_str);

				outfile << "Data " << number_str << std::endl;
			}
			break;
		case FrameTypeInterrupt:
			AddResultString("I");
			AddResultString("Int");
			AddResultString("Interrupt");
			break;
		case FrameTypeControlBit0:
			mCB0 = data;
			if (data) {
				AddResultString("EoM");
				AddResultString("End of Message");
				AddResultString("Control Bit 0: End of Message");
			} else {
				AddResultString("Err");
				AddResultString("Control Bit 0: General Error");
			}
			break;
		case FrameTypeControlBit1:
			if (mCB0) {
				if (data) {
					AddResultString("!K");
					AddResultString("Nak");
					AddResultString("Control Bit 1: Nak");
				} else {
					AddResultString("K");
					AddResultString("Ack");
					AddResultString("Control Bit 1: Ack");
				}
			} else {
				if (data) {
					AddResultString("TX,RX Err");
					AddResultString("TX or RX Node Error");
					AddResultString("Control Bit 1: TX or RX Node Error");
				} else {
					AddResultString("Int");
					AddResultString("Interrupted");
					AddResultString("Control Bit 1: Interrupted");
				}
			}
			break;
		default:
			;//AnalyzerHelpers::Assert("Internal Error: Unknown frame type in GenerateBubbleText?");
	}
}

void MBusAnalyzerResults::GenerateCSVFile( const char* file, DisplayBase display_base )
{
	std::ofstream file_stream( file, std::ios::out );

	U64 trigger_sample = mAnalyzer->GetTriggerSample();
	U32 sample_rate = mAnalyzer->GetSampleRate();

	file_stream << "Time [s], Addr [in hex], Data [in hex]" << std::endl;

	// Handle picking up in the middle of a transaction by ignoring everything until the first address
	bool any_frame = false;
	bool new_frame = true;
	U64 num_frames = GetNumFrames();
	for( U32 i=0; i < num_frames; i++ )
	{
		Frame frame = GetFrame( i );
		U32 data = frame.mData1;

		if (!any_frame) {
			if (frame.mType == FrameTypeAddress) {
				any_frame = true;
			} else {
				continue;
			}
		}

		switch (frame.mType) {
			case FrameTypeAddress:
				{
				new_frame = true;

				char time_str[128];
				AnalyzerHelpers::GetTimeString( frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128 );
				file_stream << time_str << ", ";

				char number_str[64];
				int addr_len = ((data & 0xf0000000) == 0xf0000000) ? 32 : 8;
				AnalyzerHelpers::GetNumberString(data, display_base, addr_len, number_str, 64);
				file_stream << number_str << ", ";
				break;
				}

			case FrameTypeData:
				{
				if (new_frame) {
					new_frame = false;
					file_stream << "0x";
				}

				file_stream << std::hex << std::setw(2) << std::setfill('0') << data;
				break;
				}

			case FrameTypeInterrupt:
				file_stream << std::endl;
		};

		if( UpdateExportProgressAndCheckForCancel( i, num_frames ) == true )
		{
			file_stream.close();
			return;
		}
	}

	file_stream.close();
}

void MBusAnalyzerResults::GenerateOutFile(const char* file, DisplayBase display_base)
{
	std::ofstream file_stream(file, std::ios::out);

	// Handle picking up in the middle of a transaction by ignoring everything until the first address
	bool any_frame = false;
	bool new_frame = true;
	U64 num_frames = GetNumFrames();
	for (U32 i = 0; i < num_frames; i++)
	{
		Frame frame = GetFrame(i);
		U32 data = frame.mData1;

		if (!any_frame) {
			if (frame.mType == FrameTypeAddress) {
				any_frame = true;
			}
			else {
				continue;
			}
		}

		switch (frame.mType) {
		case FrameTypeAddress:
		{
			new_frame = true;

			file_stream << "Address " << std::hex << data << std::endl;
			break;
		}

		case FrameTypeData:
		{
			file_stream << "Data " << std::hex << (U32)data << std::endl;
			break;
		}
		};

		if (UpdateExportProgressAndCheckForCancel(i, num_frames) == true)
		{
			file_stream.close();
			return;
		}
	}

	file_stream.close();
}

void MBusAnalyzerResults::GenerateExportFile(const char* file, DisplayBase display_base, U32 export_type_user_id)
{
	// With apologies to the world for using hard-coded #'s here, see MBusAnalzyerSettings::HACK_FILE_TYPE
	if (export_type_user_id == 0) {
		GenerateCSVFile(file, display_base);
	}
	else if (export_type_user_id == 1) {
		GenerateOutFile(file, display_base);
	}
}


void MBusAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
	Frame frame = GetFrame( frame_index );
	ClearResultStrings();

	char number_str[128];
	AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 8, number_str, 128 );
	AddResultString( number_str );
}

void MBusAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
	ClearResultStrings();
	AddResultString( "not supported" );
}

void MBusAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
	ClearResultStrings();
	AddResultString( "not supported" );
}
