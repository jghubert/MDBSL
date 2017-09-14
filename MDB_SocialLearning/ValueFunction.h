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
#include "FamiliarityTester.h"
#include "ValueFunctionMemory.hpp"
#include <list>

namespace MDB_Social {

    class Settings;
    class ResourceLibraryData;
    
    class ValueFunction
    {
    public:
        enum QualityEvaluationFunction {
            SIMPLE_RATIO = 0,
            SHANNON_ENTROPY = 1,
            SENSORY_COVERAGE = 2,
            MINIMUM_REWARD = 3
        };
        
        enum MultipleVFMergingMode {
            AVERAGE = 0,
            SUM = 1,
            UNDEFINED    // Used to deactivate the function and cause an error.
        };

        enum LearningMode {
            SEQUENCE = 0,   // Learn the sequences by keeping the traces from different robots separated.
            RANDOM = 1
        };
        
    private:
        Settings* settings;
        ResourceLibraryData* resourceLibrary;

        // Here are the parameters for the value function.
        std::string valueFunctionType;
        
        // A value function has a model
        Model* vf;
        std::list<Model*> additionalVFs;
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
        unsigned vfMergingMode;
        unsigned learningMode;

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
        bool loadFamiliarityFromTraces(TraceMemory* tm);
        
        bool saveValueFunctionToFile(const char* filename);
        
        double computeFamiliarity(Trace& t);
        
        void addImportedValueFunction(ValueFunctionMemory* addvf);
        void clearImportedValueFunction();
        
    };

}
#endif /* VALUEFUNCTION_H */

