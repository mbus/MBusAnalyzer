#ifndef MBUS_ANALYZER_RESULTS
#define MBUS_ANALYZER_RESULTS

#include <AnalyzerResults.h>

enum MBusFrameType {
	FrameTypeRequest,
	FrameTypeArbitration,
	FrameTypePriorityArbitration,
	FrameTypeReservedBit,
	FrameTypeAddress,
	FrameTypeData,
	FrameTypeInterrupt,
	FrameTypeControlBit0,
	FrameTypeControlBit1,
};

#define MULTIPLE_ARBITRATION_WINNER (1 << 0)
#define NO_ARBITRATION_WINNER		(1 << 1)
#define REQUEST_BUG_WORKAROUND      (1 << 2)

class MBusAnalyzer;
class MBusAnalyzerSettings;

class MBusAnalyzerResults : public AnalyzerResults
{
public:
	MBusAnalyzerResults( MBusAnalyzer* analyzer, MBusAnalyzerSettings* settings );
	virtual ~MBusAnalyzerResults();

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions

protected:  //vars
	MBusAnalyzerSettings* mSettings;
	MBusAnalyzer* mAnalyzer;

private:
	int ChannelToIndex(Channel& channel);
	bool mCB0;
};

#endif //MBUS_ANALYZER_RESULTS
