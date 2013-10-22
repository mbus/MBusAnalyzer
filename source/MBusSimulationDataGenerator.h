#ifndef MBUS_SIMULATION_DATA_GENERATOR
#define MBUS_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <AnalyzerHelpers.h> // For ClockGenerator
#include <string>
#include <vector>
class MBusAnalyzerSettings;

class MBusSimulationDataGenerator
{
public:
	MBusSimulationDataGenerator();
	~MBusSimulationDataGenerator();

	void Initialize( U32 simulation_sample_rate, MBusAnalyzerSettings* settings );
	U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
	MBusAnalyzerSettings* mSettings;
	U32 mSimulationSampleRateHz;

protected:
	SimulationChannelDescriptorGroup mMBusSimulationChannels;
	std::vector< SimulationChannelDescriptor * > mNodeCLKSimulationDatas;
	std::vector< SimulationChannelDescriptor * > mNodeDATSimulationDatas;

	int mNodeCount;

private:
	void CreateMBusTransaction(int sender, U32 address, U8 num_bytes, U8 data[], bool acked);
	void CreateMBusArbitration(std::vector< bool > normal, std::vector< bool > priority);
	void CreateMBusData(int sender, U32 address, U8 num_bytes, U8 data[]);
	void CreateMBusBit(int sender, BitState bit);
	void CreateMBusInterrupt(int interrupter);
	void CreateMBusControl(int interrupter, BitState cb0, int target, BitState cb1);
	void PropogationDelay();

	ClockGenerator mClockGenerator;

	U64 minSampleNumber();
};
#endif //MBUS_SIMULATION_DATA_GENERATOR
