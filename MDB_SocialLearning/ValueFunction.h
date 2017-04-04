/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ValueFunction.h
 * Author: Julien Hubert
 *
 * Created on August 3, 2016, 4:50 PM
 */

#ifndef VALUEFUNCTION_H
#define VALUEFUNCTION_H

#include <string>
#include "TraceMemory.h"
#include "Model.hpp"
#include "RobotID.h"
#include "FamiliarityTester.h"

namespace MDB_Social {

    class ValueFunction: public RobotID {
    public:
        enum QualityEvaluationFunction {
            SIMPLE_RATIO = 0,
            SHANNON_ENTROPY = 1,
            SENSORY_COVERAGE = 2,
            MINIMUM_REWARD = 3
        };

    private:
        // Here are the parameters for the value function.
        std::string valueFunctionType;
        
        // A value function has a model
        Model* vf;
        std::vector<double> inputs;
        std::vector<double> outputs;

        FamiliarityTester* famtester;
        
        void loadParameters();
        
        unsigned qualityMeasure;
        double quality;
        bool reliable;
        double simpleRatioQualityThreshold;
        unsigned minimumRewardThreshold;
        bool useOnlyRewardedTraces;
        double rewardThreshold;

        bool compressTraceMemoryFlag;
        
        double simpleRatioQualityMeasure();
        unsigned minimumRewardQualityMeasure();
        
    public:
        ValueFunction();
        virtual ~ValueFunction();

        void registerParameters();

        bool initialise();
        
        void publish();
        
        void update();
        double estimateTrace(Trace& t);

        void compressTraceMemory();
        double evaluateTraceMemoryQuality();     // This is a fast version just counting what's inside the memory. 

        bool isItReliable() const;
        double getQuality() const;
        
        void loadFromExternalMemory();
        
        bool saveValueFunctionToFile(const char* filename);
        
        virtual void setID(std::string& _id);
        
        double computeFamiliarity(Trace& t);
        
    };

}
#endif /* VALUEFUNCTION_H */

