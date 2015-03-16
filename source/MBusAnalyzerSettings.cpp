#include "MBusAnalyzerSettings.h"
#include <AnalyzerHelpers.h>

#include <cassert>
#include <cstdio>

#ifdef WIN32
#define snprintf _snprintf
#endif

MBusAnalyzerSettings::MBusAnalyzerSettings()
:	mMasterCLKChannel( UNDEFINED_CHANNEL ),
	mMasterDATChannel( UNDEFINED_CHANNEL ),
	mMemberCount( 0 )
{
	log_hack.open("C:\\TEMP\\MBusAnalyzerLogHack.txt");
	log_hack << "LOG HACK BEGIN\n\n";
	log_hack.flush();

	mMasterCLKChannelInterface.reset( new AnalyzerSettingInterfaceChannel() );
	mMasterCLKChannelInterface->SetTitleAndTooltip( "Master CLK", "Connect to CLK_OUT of Master" );
	mMasterCLKChannelInterface->SetChannel( mMasterCLKChannel );
	mMasterCLKChannelInterface->SetSelectionOfNoneIsAllowed(false);

	mMasterDATChannelInterface.reset( new AnalyzerSettingInterfaceChannel() );
	mMasterDATChannelInterface->SetTitleAndTooltip( "Master DAT", "Connect to DAT_OUT of Master" );
	mMasterDATChannelInterface->SetChannel( mMasterDATChannel );
	mMasterDATChannelInterface->SetSelectionOfNoneIsAllowed(false);

	AddInterface( mMasterCLKChannelInterface.get() );
	AddInterface( mMasterDATChannelInterface.get() );

	for (int i=0; i < MAX_MBUS_MEMBERS; i++) {
		char title[32];

#ifdef FUCK_THIS
		static_assert(MAX_MBUS_MEMBERS == 1, "Fuck this");
		mMemberActive[i] = true;
		mMemberActiveInterface[i].reset( new AnalyzerSettingInterfaceBool() );
		snprintf(title, 32, "Member %d Enabled", i);
		mMemberActiveInterface[i]->SetTitleAndTooltip( title, "Enable / Disable this Member Node");
		mMemberActiveInterface[i]->SetValue(mMemberActive[i]);
#else
		mMemberActive[i] = false;
		mMemberActiveInterface[i].reset( new AnalyzerSettingInterfaceBool() );
		snprintf(title, 32, "Member %d Enabled", i);
		mMemberActiveInterface[i]->SetTitleAndTooltip( title, "Enable / Disable this Member Node");
		mMemberActiveInterface[i]->SetValue(mMemberActive[i]);
#endif

		mMemberCLKChannels[i] = UNDEFINED_CHANNEL;
		mMemberCLKChannelsInterface[i].reset( new AnalyzerSettingInterfaceChannel() );
		snprintf(title, 32, "Member %d CLK\n", i);
		mMemberCLKChannelsInterface[i]->SetTitleAndTooltip( title, "Connect to CLK_OUT of Member Node");
		mMemberCLKChannelsInterface[i]->SetChannel(mMemberCLKChannels[i]);
		mMemberCLKChannelsInterface[i]->SetSelectionOfNoneIsAllowed(true);

		mMemberDATChannels[i] = UNDEFINED_CHANNEL;
		mMemberDATChannelsInterface[i].reset( new AnalyzerSettingInterfaceChannel() );
		snprintf(title, 32, "Member %d DAT\n", i);
		mMemberDATChannelsInterface[i]->SetTitleAndTooltip( title, "Connect to DAT_OUT of Member Node");
		mMemberDATChannelsInterface[i]->SetChannel(mMemberDATChannels[i]);
		mMemberDATChannelsInterface[i]->SetSelectionOfNoneIsAllowed(true);

#ifndef FUCK_THIS
		AddInterface( mMemberActiveInterface[i].get() );
#endif
		AddInterface( mMemberCLKChannelsInterface[i].get() );
		AddInterface( mMemberDATChannelsInterface[i].get() );
	}

	AddExportOption( 0, "Export as text/csv file" );
	AddExportExtension( 0, "text", "txt" );
	AddExportExtension( 0, "csv", "csv" );
}

MBusAnalyzerSettings::~MBusAnalyzerSettings()
{
}

bool MBusAnalyzerSettings::SetSettingsFromInterfaces()
{
	static_assert(MAX_MBUS_NODES == (MAX_MBUS_MEMBERS + 1), "Bad MAX_MBUS #defines");

	int     MemberCount;
	bool    NodeActive[MAX_MBUS_NODES];
	Channel _NodeChannels[MAX_MBUS_NODES * 2]; // Need uniform array for overlap API
	Channel* NodeCLKChannels = _NodeChannels;
	Channel* NodeDATChannels = _NodeChannels + MAX_MBUS_NODES;

	NodeActive[0] = true;
	NodeCLKChannels[0] = mMasterCLKChannelInterface->GetChannel();
	NodeDATChannels[0] = mMasterDATChannelInterface->GetChannel();

	if (NodeCLKChannels[0] == UNDEFINED_CHANNEL) {
		SetErrorText("Master CLK Channel is required");
		return false;
	} else if (NodeDATChannels[0] == UNDEFINED_CHANNEL) {
		SetErrorText("Master DAT Channel is required");
		return false;
	}

	bool found_inactive = false;
	MemberCount = 0;
	for (int i=1; i < MAX_MBUS_NODES; i++) {
		NodeActive[i] = mMemberActiveInterface[i-1]->GetValue();
		NodeCLKChannels[i] = mMemberCLKChannelsInterface[i-1]->GetChannel();
		NodeDATChannels[i] = mMemberDATChannelsInterface[i-1]->GetChannel();

		if (NodeActive[i] && found_inactive) {
			SetErrorText("All active members must be sequential (no gaps; weakness of my analyzer, sorry)");
			return false;
		} else if (NodeActive[i]) {
			MemberCount++;
		} else {
			found_inactive = true;
		}

		if (NodeActive[i] && (NodeCLKChannels[i] == UNDEFINED_CHANNEL)) {
			SetErrorText("Active member nodes must all have CLK defined. WTF?");
			return false;
		}
		if (NodeActive[i] && (NodeDATChannels[i] == UNDEFINED_CHANNEL)) {
			SetErrorText("Active member nodes must all have DAT defined");
			return false;
		}
	}

	if (AnalyzerHelpers::DoChannelsOverlap(_NodeChannels, MAX_MBUS_NODES * 2)) {
		SetErrorText("All active channels must be unique");
		return false;
	}

	ClearChannels();

	mMemberCount = MemberCount;
	mMasterCLKChannel = NodeCLKChannels[0];
	mMasterDATChannel = NodeDATChannels[0];

	AddChannel( mMasterCLKChannel, "MBus Master CLK", true);
	AddChannel( mMasterDATChannel, "MBus Master DAT", true);

	for (int i=0; i < MAX_MBUS_MEMBERS; i++) {
		mMemberActive[i] = NodeActive[i+1];
		mMemberCLKChannels[i] = NodeCLKChannels[i+1];
		mMemberDATChannels[i] = NodeDATChannels[i+1];

#ifdef FUCK_THIS
		if (mMemberActive[i] != true) {
			log_hack << "SET: Unexpected inactive member" << std::endl;
			AnalyzerHelpers::Assert("How the fuck did this happen here?");
		}
#endif
		AddChannel( mMemberCLKChannels[i], "MBus Member CLK", mMemberActive[i] );
		AddChannel( mMemberDATChannels[i], "MBus Member DAT", mMemberActive[i] );
	}

	return true;
}

void MBusAnalyzerSettings::UpdateInterfacesFromSettings()
{
	mMasterCLKChannelInterface->SetChannel( mMasterCLKChannel );
	mMasterDATChannelInterface->SetChannel( mMasterDATChannel );

	for (int i=0; i < MAX_MBUS_MEMBERS; i++) {
		mMemberActiveInterface[i]->SetValue( mMemberActive[i] );
		mMemberCLKChannelsInterface[i]->SetChannel( mMemberCLKChannels[i] );
		mMemberDATChannelsInterface[i]->SetChannel( mMemberDATChannels[i] );
	}
}

void MBusAnalyzerSettings::LoadSettings( const char* settings )
{
	SimpleArchive text_archive;
	text_archive.SetString( settings );

	text_archive >> mMasterCLKChannel;
	text_archive >> mMasterDATChannel;
	text_archive >> mMemberCount;
	for (int i=0; i < MAX_MBUS_MEMBERS; i++) {
		text_archive >> mMemberActive[i];
		text_archive >> mMemberCLKChannels[i];
		text_archive >> mMemberDATChannels[i];
	}

	ClearChannels();
	AddChannel( mMasterCLKChannel, "MBus Master CLK", true );
	AddChannel( mMasterDATChannel, "MBus Master DAT", true );
	for (int i=0; i < MAX_MBUS_MEMBERS; i++) {
#ifdef FUCK_THIS
		if (mMemberActive[i] != true) {
			log_hack << "SET: " << __LINE__ << ": Unexpected inactive member " << i << std::endl;
			AnalyzerHelpers::Assert("How the fuck did this happen?");
		}
#endif
		AddChannel( mMemberCLKChannels[i], "MBus Member CLK", mMemberActive[i] );
		AddChannel( mMemberDATChannels[i], "MBus Member DAT", mMemberActive[i] );
	}

	UpdateInterfacesFromSettings();
}

const char* MBusAnalyzerSettings::SaveSettings()
{
	SimpleArchive text_archive;

	text_archive << mMasterCLKChannel;
	text_archive << mMasterDATChannel;
	text_archive << mMemberCount;

	for (int i=0; i<mMemberCount; i++) {
		text_archive << mMemberActive[i];
		text_archive << mMemberCLKChannels[i];
		text_archive << mMemberDATChannels[i];
	}

	return SetReturnString( text_archive.GetString() );
}
