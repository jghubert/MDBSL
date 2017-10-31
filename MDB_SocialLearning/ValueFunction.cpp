/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ValueFunction.cpp
 * Author: Julien Hubert
 * 
 * Created on August 3, 2016, 4:50 PM
 */

#include "ValueFunction.h"
#include "Settings.h"
#include "ModelLibrary.hpp"
#include "ResourceLibrary.hpp"
#include "FeedforwardNN.h"
#include <valarray>
#include "RobotID.h"

namespace MDB_Social {

    ValueFunction::ValueFunction()
    {
        settings = RobotID::getSettings();
        resourceLibrary = RobotID::getResourceLibrary();
        
        vf = NULL;
        quality = 0.0;
        reliable = false;
        compressTraceMemoryFlag = false;
        useOnlyRewardedTraces = false;
        rewardThreshold = 0.0;
        vfMergingMode = UNDEFINED;
        learningMode = RANDOM;
        famtester = NULL;
        outputs.resize(1, 0);
    }

    ValueFunction::~ValueFunction() 
    {
        if (vf)
            delete vf;
        for (auto it = additionalVFs.begin(); it != additionalVFs.end(); ++it)
            delete *it;
        additionalVFs.clear();
        
        if (famtester)
            delete famtester;
    }
    
    void ValueFunction::loadParameters()
    {
//        Settings* settings = Settings::getInstance();
        valueFunctionType = settings->value<std::string>("ValueFunction.type").second;
        
        std::string qualityMeasureStr = settings->value<std::string>("ValueFunction.qualityMeasure").second;
        if (qualityMeasureStr == "SIMPLE_RATIO")
            qualityMeasure = SIMPLE_RATIO;
        else if (qualityMeasureStr == "SHANNON_ENTROPY")
            qualityMeasure = SHANNON_ENTROPY;
        else if (qualityMeasureStr == "SENSORY_COVERAGE")
            qualityMeasure = SENSORY_COVERAGE;
        else if (qualityMeasureStr == "MINIMUM_REWARD")
            qualityMeasure = MINIMUM_REWARD;
        else
            std::cerr << "ValueFunction: unknown quality measure selected: " << qualityMeasureStr << std::endl;
        
        simpleRatioQualityThreshold = settings->value<double>("ValueFunction.simple_ratio.qualityRatio").second;
        minimumRewardThreshold = settings->value<unsigned>("ValueFunction.minimum_reward.threshold").second;
        
        compressTraceMemoryFlag = settings->value<bool>("ValueFunction.compressTraceMemory").second;
        
        useOnlyRewardedTraces = settings->value<bool>("ValueFunction.useOnlyRewardedTraces").second;
        rewardThreshold = settings->value<double>("ValueFunction.rewardThreshold").second;
        
        std::string mergemode = settings->value<std::string>("ValueFunction.vfMergingMode").second;
        if (mergemode == "AVERAGE")
            vfMergingMode = AVERAGE;
        else if (mergemode == "SUM")
            vfMergingMode = SUM;
        else if (mergemode == "UNDEFINED")
            vfMergingMode = UNDEFINED;
        else {
            std::cerr << "ValueFunction: Unknown merging mode selected: " << mergemode << std::endl;
            exit(1);
        }
        
        std::string learningModeStr = settings->value<std::string>("ValueFunction.learningMode").second;
        if (learningModeStr == "RANDOM")
            learningMode = RANDOM;
        else if (learningModeStr == "SEQUENCE")
            learningMode = SEQUENCE;
        else {
            std::cerr << "ValueFunction: Unknown learning mode selected: " << learningModeStr << std::endl;
            exit(1);
        }
        
    }

    void ValueFunction::registerParameters()
    {
//        Settings* settings = Settings::getInstance();
        if (settings->registerAndRetrieveParameter<std::string>(valueFunctionType, "ValueFunction.type", std::string("Feedforward"), "Type of the value function to use")) {
            vf = ModelLibrary::getModel(valueFunctionType);
//            vf->setID(robotid);
            vf->setExternalMemory((Memory<NeuralNetworkData>*)resourceLibrary->getValueFunctionMemory());
            vf->registerParameters("ValueFunction");
        }
        settings->registerParameter<std::string>("ValueFunction.qualityMeasure", std::string("SIMPLE_RATIO"), "Quality measure used to evaluate the value function.");
        settings->registerParameter<double>("ValueFunction.simple_ratio.qualityRatio", 0.1, "Value function: Simple Ratio: Minimum success ratio threshold for using the controllers.");
        settings->registerParameter<unsigned>("ValueFunction.minimum_reward.threshold", 50, "Value function: Minimum Reward: Amount of rewarded steps required to switch off the babbling.");
        settings->registerParameter<std::string>("ValueFunction.vfMergingMode", std::string("UNDEFINED"), "Indicate how to merge the outputs of multiple value functions.");
        settings->registerParameter<bool>("ValueFunction.compressTraceMemory", false, "Value function: Compress the traces before learning the value function.");
        
        settings->registerParameter<bool>("ValueFunction.useOnlyRewardedTraces", false, "Value function: Use only rewarded traces for learning.");
        settings->registerParameter<double>("ValueFunction.rewardThreshold", 0.0, "Value function: Minimum reward of a trace to be included in the learning set.");
        settings->registerParameter<std::string>("ValueFunction.learningMode", std::string("RANDOM"), "RANDOM: traces are mixed; SEQUENCE: traces are temporal sequences that should not be mixed.");
    }

    bool ValueFunction::initialise()
    {
        loadParameters();
                
        if (!vf) {
            vf = ModelLibrary::getModel(valueFunctionType);
            vf->setExternalMemory((Memory<NeuralNetworkData>*)resourceLibrary->getValueFunctionMemory());
            vf->registerParameters("ValueFunction");
        }
        vf->loadParameters("ValueFunction");
        vf->initializeFromParameters();
        
        return true;
    }
    
    void ValueFunction::publish()
    {
        vf->publish();
    }
    
    void ValueFunction::loadFromExternalMemory()
    {
        if (!vf->loadFromExternalMemory()) {
            std::cerr << "ValueFunction: Failed to load data from the external memory." << std::endl;
            exit(1);
        }
    }
    
    bool ValueFunction::loadFamiliarityFromTraces(TraceMemory* tm)
    {
        std::vector<bool> mask;
        unsigned selectedTraces = 0;
        mask.resize(tm->size(), false);
        unsigned index=0;
        for (auto it = tm->begin(); it != tm->end(); ++it) {
            mask[index] = it->usedForVFTraining;
            selectedTraces += mask[index];
            index++;
        }        
        if (!famtester)
            famtester = new FamiliarityTester(tm->getMaximumSize(), tm->begin()->inputs.size());            
        if (selectedTraces) 
            famtester->setTrainingSet(tm, mask);
        return selectedTraces > 0;
    }

    void ValueFunction::update()
    {
        std::cout << "Update of the value function..." << std::endl; 
        
        if (compressTraceMemoryFlag) {
            compressTraceMemory();
        }
        else {
            TraceMemory* tm = resourceLibrary->getTraceMemory();

            std::vector<bool> mask;
            unsigned selectedTraces = 0;
            if (useOnlyRewardedTraces) {
                mask.resize(tm->size(), false);
                unsigned index=0;
                for (auto it = tm->begin(); it != tm->end(); ++it) {
                    mask[index] = it->expected_reward >= rewardThreshold;
                    selectedTraces += mask[index];
                    index++;
                }
            }

            if (!famtester) {
                famtester = new FamiliarityTester(tm->getMaximumSize(), tm->begin()->inputs.size());
            }
            famtester->setTrainingSet(tm, mask);
            
            
            
            if (valueFunctionType == "Feedforward") {
                std::cout << "ValueFunction: Learning " << tm->size() << " patterns..." << std::endl;
                FeedforwardNN* nn = static_cast<FeedforwardNN*>(vf);
                nn->resetTrainingSet();
                nn->reset();

                TraceMemory::const_iterator it = tm->begin();
                std::vector<double> inputs;
                inputs.reserve(((*it).inputs.size() + (*it).outputs.size()) * tm->size() );
                std::vector<double> outputs;
                outputs.reserve(tm->size());

                if (!useOnlyRewardedTraces) { 
                    for (it = tm->begin(); it != tm->end(); ++it) {
                        for (size_t i = 0; i<it->inputs.size(); ++i)
                            inputs.push_back(it->inputs[i]);
//                        for (size_t i = 0; i<it->outputs.size(); ++i)
//                            inputs.push_back(outputs[i]);
                        outputs.push_back(it->expected_reward);
                    }
                    nn->setTrainingSet(tm->size(), inputs, outputs);
                }                
                else {
                    unsigned index = 0;
                    for (it = tm->begin(); it != tm->end(); ++it) {
                        if (mask[index++]) {
                            for (size_t i = 0; i<it->inputs.size(); ++i)
                                inputs.push_back(it->inputs[i]);
//                            for (size_t i = 0; i<it->outputs.size(); ++i)
//                                inputs.push_back(outputs[i]);
                            outputs.push_back(it->expected_reward);
                        }
                    }
                    nn->setTrainingSet(selectedTraces, inputs, outputs);
                }
//                TraceMemory::const_iterator it = tm->begin();
//                std::vector<double> inputs((*it).inputs.size() + (*it).outputs.size());
//                std::vector<double> outputs(1);
//                for (it = tm->begin(); it != tm->end(); ++it) {
//                    size_t index = 0;
//                    for (size_t i = 0; i<it->inputs.size(); ++i)
//                        inputs[index++] = it->inputs[i];
//                    for (size_t i = 0; i<it->outputs.size(); ++i)
//                        inputs[index] = outputs[i];
//                    outputs[0] = it->expected_reward;
//                    nn->addToTrainingSet(inputs, outputs);
//                }
                nn->train();
            }
#ifdef USE_CAFFE
            else if (valueFunctionType == "CaffeDeepNet") {
                std::cout << "ValueFunction: Learning " << tm->size() << " patterns..." << std::endl;
                CaffeDeepNet* nn = static_cast<CaffeDeepNet*>(vf);
                
                TraceMemory::const_iterator it = tm->begin();
                std::vector<double> inputs;
                std::vector<double> outputs;

                if (learningMode == RANDOM) {
                    inputs.reserve(((*it).inputs.size() + (*it).outputs.size()) * tm->size() );
                    outputs.reserve(tm->size());

                    if (!useOnlyRewardedTraces) { 
                        for (it = tm->begin(); it != tm->end(); ++it) {
                            for (size_t i = 0; i<it->inputs.size(); ++i)
                                inputs.push_back(it->inputs[i]);
    //                        for (size_t i = 0; i<it->outputs.size(); ++i)
    //                            inputs.push_back(outputs[i]);
                            outputs.push_back(it->expected_reward);
                        }
                        nn->setTrainingSet(tm->size(), inputs, outputs);
                    }                
                    else {
                        unsigned index = 0;
                        for (it = tm->begin(); it != tm->end(); ++it) {
                            if (mask[index++]) {
                                for (size_t i = 0; i<it->inputs.size(); ++i)
                                    inputs.push_back(it->inputs[i]);
    //                            for (size_t i = 0; i<it->outputs.size(); ++i)
    //                                inputs.push_back(outputs[i]);
                                outputs.push_back(it->expected_reward);
                            }
                        }
                        nn->setTrainingSet(selectedTraces, inputs, outputs);
                    }
                    nn->train();
                }
                else if (learningMode == SEQUENCE) {
                    std::vector<std::pair<unsigned, TraceMemory::iterator> > traceInfo = tm->countIndependentTraces();
                    unsigned nbtraces = traceInfo.size();
                    std::vector<std::vector<std::vector<double> > > input_seq(nbtraces);
                    std::vector<std::vector<std::vector<double> > > output_seq(nbtraces);
                    
                    auto itend = tm->end();
                    for (unsigned s=0; s<nbtraces; ++s) {
                        input_seq[s].resize(traceInfo[s].first);
                        output_seq[s].resize(traceInfo[s].first);
                        if (s+1 <= nbtraces)
                            itend = traceInfo[s+1].second;
                        else
                            itend = tm->end();
                        
                        unsigned index = 0;
                        for (auto it = traceInfo[s].second; it != itend; ++it) {
                            input_seq[s][index] = it->inputs;
                            output_seq[s][index] = it->outputs;
                            index++;
                        }
                    }
                    nn->trainSequentially(input_seq, output_seq);
                    
                    
                }
                
            }
#endif
            else {
                std::cerr << "ValueFunction::update: Unknown value function type: " << valueFunctionType << std::endl;
                exit(1);
            }
        }
        std::cout << "    Learning completed." << std::endl;
    }

    double ValueFunction::estimateTrace(Trace& t)
    {
        size_t insize = t.inputs.size();
//        size_t outsize = t.outputs.size();
//        inputs.resize(insize + outsize);
        inputs.resize(insize);
        if (valueFunctionType == "Feedforward") {
            FeedforwardNN* nn = static_cast<FeedforwardNN*>(vf);

            for (unsigned i=0; i<insize; ++i)
                inputs[i] = t.inputs[i];
//            for (unsigned i=0; i<outsize; ++i)
//                inputs[insize+i] = t.outputs[i];

            nn->run(inputs, outputs);
            double localOutput = outputs[0];
            
            if (additionalVFs.size()) {
                std::valarray<double> estimations(additionalVFs.size());
                unsigned index = 0;
                for (auto it = additionalVFs.begin(); it != additionalVFs.end(); ++it) {
                    static_cast<FeedforwardNN*>(*it)->run(inputs,outputs);
                    estimations[index++] = outputs[0];
                }
                
                switch (vfMergingMode) {
                    case AVERAGE:
                        localOutput = (localOutput + estimations.sum()) / (estimations.size() + 1);
                        break;
                    case SUM:
                        localOutput += estimations.sum();
                        break;
                    case UNDEFINED:
                        std::cerr << "ValueFunction: ERROR: The merging mode is set to UNDEFINED. Ignoring the additional value functions." << std::endl;
                        break;
                    default:
                        std::cerr << "ValueFunction: ERROR: The merging mode is not unknown." << std::endl;
                        exit(1);
                }
            }
            
            return localOutput;
        }
#ifdef USE_CAFFE
        else if (valueFunctionType == "CaffeDeepNet") {
            CaffeDeepNet* nn = static_cast<CaffeDeepNet*>(vf);

            for (unsigned i=0; i<insize; ++i)
                inputs[i] = t.inputs[i];
//            for (unsigned i=0; i<outsize; ++i)
//                inputs[insize+i] = t.outputs[i];

            nn->run(1, inputs, outputs);
            double localOutput = outputs[0];
            
            if (additionalVFs.size()) {
                std::valarray<double> estimations(additionalVFs.size());
                unsigned index = 0;
                for (auto it = additionalVFs.begin(); it != additionalVFs.end(); ++it) {
                    static_cast<FeedforwardNN*>(*it)->run(inputs,outputs);
                    estimations[index++] = outputs[0];
                }
                
                switch (vfMergingMode) {
                    case AVERAGE:
                        localOutput = (localOutput + estimations.sum()) / (estimations.size() + 1);
                        break;
                    case SUM:
                        localOutput += estimations.sum();
                        break;
                    case UNDEFINED:
                        std::cerr << "ValueFunction: ERROR: The merging mode is set to UNDEFINED. Ignoring the additional value functions." << std::endl;
                        break;
                    default:
                        std::cerr << "ValueFunction: ERROR: The merging mode is not unknown." << std::endl;
                        exit(1);
                }
            }
            
            return localOutput;
            
        }
#endif   // USE_CAFFE
        else {
            std::cerr << "ValueFunction::estimateTrace: Unknown value function type: " << valueFunctionType << std::endl;
            return -1.0;
        }
    }

    
    void ValueFunction::compressTraceMemory()
    {
        
    }

    double ValueFunction::simpleRatioQualityMeasure()
    {
        TraceMemory* tm = resourceLibrary->getTraceMemory();

        unsigned rewardCount = 0;
        for (TraceMemory::const_iterator it = tm->begin(); it != tm->end(); ++it) {
            rewardCount += it->true_reward > 0.1;
        }

        return rewardCount / (1.0*tm->size());
    }

    unsigned ValueFunction::minimumRewardQualityMeasure()
    {
        TraceMemory* tm = resourceLibrary->getTraceMemory();
        unsigned rewardCount = 0;
        for (TraceMemory::const_iterator it = tm->begin(); it != tm->end(); ++it) {
            rewardCount += it->true_reward > 0.1;
        }
        
        return rewardCount;
    }
    
    double ValueFunction::evaluateTraceMemoryQuality()
    {
        quality = 0.0;
        switch(qualityMeasure) {
        case SIMPLE_RATIO:
            quality = simpleRatioQualityMeasure();
            reliable = (quality >= simpleRatioQualityThreshold);
            break;

        case SHANNON_ENTROPY:
            std::cerr << "ValueFunction: evaluateTraceMemoryQuality: Shannon entropy is not yet implemented." << std::endl;
            break;
            
        case SENSORY_COVERAGE:
            std::cerr << "ValueFunction: evaluateTraceMemoryQuality: sensory coverage is not yet implemented." << std::endl;
            break;
            
        case MINIMUM_REWARD:
            {
            unsigned q = minimumRewardQualityMeasure();
            quality = q;
            reliable = (q >= minimumRewardThreshold);
            }
            break;

        default:
            std::cerr << "ValueFunction: Unknown quality measure selected: " << qualityMeasure << std::endl;
        }
        return quality;
    }
 
    bool ValueFunction::isItReliable() const
    {
        return reliable;
    }
    
    double ValueFunction::getQuality() const
    {
        return quality;
    }

    
    bool ValueFunction::saveValueFunctionToFile(const char* filename)
    {
        return vf->saveModelToFile(filename);
    }
     
    double ValueFunction::computeFamiliarity(Trace& t)
    {
        if (famtester) {
            return famtester->getClosestNeighbour(t);
        }
        else
            return 1e6;
    }
    
    void ValueFunction::addImportedValueFunction(ValueFunctionMemory* vfmem)
    {
        // TODO: Consider different types of models between robots.
        Model* addvf;
        addvf = ModelLibrary::getModel(valueFunctionType);
        addvf->setExternalMemory(vfmem);
        addvf->loadParameters("ValueFunction");
        addvf->initializeFromParameters();
        
        addvf->setExternalMemory(NULL);   // We don't use the publish function on them for now.

        additionalVFs.push_back(addvf);
    }
    
    void ValueFunction::clearImportedValueFunction()
    {
         for (auto it = additionalVFs.begin(); it != additionalVFs.end(); ++it)
            delete *it;
        additionalVFs.clear();
   }

   
}