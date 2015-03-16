#ifndef MBUS_ANALYZER_SETTINGS
#define MBUS_ANALYZER_SETTINGS

#include <fstream>

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

/*
#define MAX_MBUS_NODES 16	// MBus design limits rings to 16 nodes
#define MAX_MBUS_MEMBERS 15
 *
 * Take these down to a smaller number as Saleae's config window isn't as
 * dynamic / flexible as would be required to draw all of these well
 */
#define FUCK_THIS

#ifdef FUCK_THIS
#define MAX_MBUS_NODES 2
#define MAX_MBUS_MEMBERS (MAX_MBUS_NODES - 1)
#else
#define MAX_MBUS_NODES 5
#define MAX_MBUS_MEMBERS (MAX_MBUS_NODES - 1)
#endif

class MBusAnalyzerSettings : public AnalyzerSettings
{
public:
	MBusAnalyzerSettings();
	virtual ~MBusAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	
	Channel mMasterCLKChannel;
	Channel mMasterDATChannel;

	int mMemberCount;
	bool mMemberActive[MAX_MBUS_MEMBERS];
	Channel mMemberCLKChannels[MAX_MBUS_MEMBERS];
	Channel mMemberDATChannels[MAX_MBUS_MEMBERS];

	std::ofstream log_hack;

protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel >   mMasterCLKChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceChannel >   mMasterDATChannelInterface;

	std::auto_ptr< AnalyzerSettingInterfaceInteger >   mMemberCountInterface;
	std::unique_ptr< AnalyzerSettingInterfaceBool >    mMemberActiveInterface[MAX_MBUS_MEMBERS];
	std::unique_ptr< AnalyzerSettingInterfaceChannel > mMemberCLKChannelsInterface[MAX_MBUS_MEMBERS];
	std::unique_ptr< AnalyzerSettingInterfaceChannel > mMemberDATChannelsInterface[MAX_MBUS_MEMBERS];
};

#endif //MBUS_ANALYZER_SETTINGS
