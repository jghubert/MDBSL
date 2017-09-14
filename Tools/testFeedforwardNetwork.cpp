/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   testFeedforwardNetwork.cpp
 * Author: julienhubert
 *
 * Created on April 24, 2017, 3:25 PM
 */

#define MDBSL_FIELD_SEPARATOR " "

#include <cstdlib>
#include "../MDB_SocialLearning/FeedforwardNN.h"
#include "../MDB_SocialLearning/Settings.h"
#include "../MDB_SocialLearning/TraceMemory.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace MDB_Social;

const bool useOnlyRewardedTraces = true;
const double rewardThreshold = 0.000001;

void trainNetwork(FeedforwardNN &net, TraceMemory &traces)
{
    std::vector<bool> mask;
    unsigned selectedTraces = 0;
    if (useOnlyRewardedTraces) {
        mask.resize(traces.size(), false);
        unsigned index=0;
        for (auto it = traces.begin(); it != traces.end(); ++it) {
            mask[index] = it->expected_reward >= rewardThreshold;
            selectedTraces += mask[index];
            index++;
        }
    }

    net.resetTrainingSet();
    net.reset();

    TraceMemory::const_iterator it = traces.begin();
    std::vector<double> inputs;
//    inputs.reserve(traces.size());
    inputs.reserve(((*it).inputs.size() ) * traces.size() );
//    inputs.reserve(((*it).inputs.size() + (*it).outputs.size()) * traces.size() );
    std::vector<double> outputs;
    outputs.reserve(traces.size());

    if (!useOnlyRewardedTraces) { 
        for (it = traces.begin(); it != traces.end(); ++it) {
//            inputs.push_back(it->inputs[1]);
            
            for (size_t i = 0; i<it->inputs.size(); ++i)
                inputs.push_back(it->inputs[i]);
//            for (size_t i = 0; i<it->outputs.size(); ++i)
//                inputs.push_back(outputs[i]);
            outputs.push_back(it->expected_reward);
        }
        net.setTrainingSet(traces.size(), inputs, outputs);
    }                
    else {
        unsigned index = 0;
        for (it = traces.begin(); it != traces.end(); ++it) {
            if (mask[index++]) {
//                inputs.push_back(it->inputs[1]);
                for (size_t i = 0; i<it->inputs.size(); ++i)
                    inputs.push_back(it->inputs[i]);
//                            for (size_t i = 0; i<it->outputs.size(); ++i)
//                                inputs.push_back(outputs[i]);
                outputs.push_back(it->expected_reward);
            }
        }
//        for (size_t i=0; i<inputs.size(); ++i)
//            cout << inputs[i] << " " << outputs[i] << endl;
        
        net.setTrainingSet(selectedTraces, inputs, outputs);
    }
    
    net.train();
    std::cout << "    Learning completed." << std::endl;
    
}

void testNeuralNet(FeedforwardNN &net, TraceMemory &traces, const char* filename)
{
    std::cout << "Testing the neural network... ";
    cout.flush();
    std::vector<double> input(1);
    std::vector<double> output(1);
    TraceMemory::const_iterator it = traces.begin();

    ofstream outfile(filename, ios_base::trunc);
    if (!outfile.is_open()) {
        cerr << "Impossible to open the output file for the test." << std::endl;
        exit(1);
    }
    
    std::vector<bool> mask;
    unsigned selectedTraces = 0;
    if (useOnlyRewardedTraces) {
        mask.resize(traces.size(), false);
        unsigned index=0;
        for (it = traces.begin(); it != traces.end(); ++it) {
            mask[index] = it->expected_reward >= rewardThreshold;
            selectedTraces += mask[index];
            index++;
        }
    }

    if (!useOnlyRewardedTraces) { 
        for (it = traces.begin(); it != traces.end(); ++it) {
//            input[0] = it->inputs[1];
            input = it->inputs;
            net.run(input, output);
            for (size_t i=0; i<input.size(); ++i)
                outfile << input[i] << " ";
            outfile << output[0] << std::endl;
        }
    }
    else {
        unsigned index = 0;
        for (it = traces.begin(); it != traces.end(); ++it) {
            if (mask[index++]) {
//                input[0] = it->inputs[1];
                input = it->inputs;
                net.run(input, output);
                for (size_t i=0; i<input.size(); ++i)
                    outfile << input[i] << " ";
                outfile << output[0] << std::endl;
            }
        }
    }
    
    outfile.close();
    std::cout << "DONE" << endl;
    
}

/*
 * 
 */
int main(int argc, char** argv) 
{
    if (argc < 3) {
        std::cerr << "Error: Missing config file. Usage : " << argv[0] << " [config_file] [trace file]" << std::endl;
        exit(1);
    }
     
    Settings* settings = SettingsLibrary::getInstance();
    char* settingfile = argv[1];    
    char* tracefile = argv[2];
    settings->setParameterSources(settingfile, argc, argv);

    TraceMemory traces;
    if (!traces.loadFromFile(tracefile, true)) {
        std::cerr << "ERROR: Failed to load the trace file." << std::endl;
        exit(1);
    }
    else
        std::cout << "Traces loaded successfully." << std::endl;
    
    FeedforwardNN net;
    net.registerParameters("FeedforwardNet");
    
    if (!settings->processAllParameters(settingfile, argc, argv)) {
        std::cerr << "Manager:: Error loading the parameters." << std::endl;
        exit(1);
    }
    
    net.loadParameters("FeedforwardNet");
    net.initializeFromParameters();
    net.setTrainingAlgorithm(FeedforwardNN::RPROP);
    net.setTrainingErrorFunction(FeedforwardNN::TANHFCT);
    
    
    trainNetwork(net, traces);
    if (!net.save("neuralnet.net"))
        cerr << "Failed to save the neural network to file." << endl;
    testNeuralNet(net, traces, "trainingTest.log");
    
    return 0;
}

