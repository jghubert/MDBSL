#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>
#include "../MDB_SocialLearning/TraceMemory.h"

using namespace std;
using namespace MDB_Social;

void processTraceMemory(TraceMemory* traceMemory, unsigned long RewardBackpropagationStepSize, unsigned long RewardBackpropagationStepRepeatSize)
{
    // Need to backpropagate the rewards.
        
    double RewardBackpropagationMinimumAbsoluteValue = 0.1;
    double delta = 0.0;
    double last_reward = 0.0;
    double initReward = 0.0;
    unsigned repeatCount = 1;
    TraceMemory::reverse_iterator itend = traceMemory->rend();
    boost::uuids::uuid currentUUID = traceMemory->rbegin()->uuid;
    for (TraceMemory::reverse_iterator it = traceMemory->rbegin(); it != itend; ++it ) {
        if (currentUUID != it->uuid) {
            // We reset the counter because the tested genotype changed.
            currentUUID = it->uuid;
            last_reward = 0.0;
            repeatCount = 1;
        }
        
        if (it->true_reward >= RewardBackpropagationMinimumAbsoluteValue || it->true_reward <= -RewardBackpropagationMinimumAbsoluteValue) {
            it->expected_reward = it->true_reward;
            last_reward = std::fabs(it->true_reward);
            initReward = it->true_reward;
            delta = last_reward / (RewardBackpropagationStepSize*1.0);
            repeatCount = 1;
        }
        else if (last_reward > 1e-6) {  // Problematic as the delta might jump from 1e-6 to -1e-6 and then it wouldn't stop.
            if (repeatCount == RewardBackpropagationStepRepeatSize) {
                last_reward -= delta;
                repeatCount = 1;
            }
            else
                repeatCount++;
            if (last_reward < 1e-6)
                last_reward = 0.0;
            it->expected_reward = std::copysign(last_reward, initReward); // last_reward gets the sign of initReward
        }
        else
            it->expected_reward = 0.0;
            
    }
}


int main(int argc, char* argv[])
{
	if (argc < 5) {
		cout << "Usage: " << argv[0] << " [output file] [trace log file] [Size of the propagation] [Number of repetition of the reward]" << std::endl;
		exit(0);
	}

	// load the trace memory
	TraceMemory traces;
	std::cout << "Loading traces..." << std::endl;
	if (!traces.loadFromFile(argv[2])) {
		cerr << "Error loading the file " << argv[2] << endl;
		exit(1);
	}

	unsigned long stepSize = strtoul(argv[3], NULL, 10);
	if (stepSize == ULLONG_MAX) {
		cerr << "Error with the size of the propagation. Check and try again." << endl;
		exit(1);
	}
	unsigned long repeat = strtoul(argv[4], NULL, 10);
	if (repeat == ULLONG_MAX) {
		cerr << "Error with the number of repeatition of the propagation. Check and try again." << endl;
		exit(1);
	}

	std::cout << "Reformatting traces..." << std::endl;
	processTraceMemory(&traces, stepSize, repeat);

	std::cout << "Saving resulting traces to " << argv[1] << " ...." << std::endl;
	traces.saveToFile(argv[1], true);

	std::cout << "Reformatting completed." << std::endl;
	return 0;
}