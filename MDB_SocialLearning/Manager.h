/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Manager.h
 * Author: Julien Hubert
 *
 * Created on June 14, 2016, 6:08 PM
 */

#ifndef MANAGER_H
#define MANAGER_H

#include "Settings.h"
#include "ValueFunction.h"
#include "ValueFunctionMemory.hpp"
#include "Policy.h"
#include "ResourceLibrary.hpp"
#include "TraceMemory.h"
#include "PolicyMemory.hpp"
#include "RobotID.h"
#include "SocialManagerClient.h"

namespace MDB_Social {

    class SocialManager;
    
    class Manager: public RobotID {
    private:
        
//        Settings* settings;
//        ResourceLibrary* resourceLibrary;
        
        // Value Function
        ValueFunction* vf;
        
        // Babbling
        // Policy
        Policy* policyGA;
        
        // Memories
        ValueFunctionMemory* vfMemory;
        TraceMemory* traceMemory;
        PolicyMemory* policyMemory;
        
        // Social Manager Client
        SocialManagerClient* smc;
        bool socialMode;
        
        unsigned RewardBackpropagationStepSize;
        unsigned RewardBackpropagationStepRepeatSize;
        
        // logging
        int loggingFrequency;
        bool logValueFunction;
        bool logPolicy;
        bool logTraces;
        std::string logFilenamePrefix; 
        
        bool testIndividual;
        bool testGeneration;
        int individualToTest;
        int generationToTest;
        bool testIndividualNoValueFunction;
        bool testIndividualNoTrace;
        
        unsigned traceMemoryMaximumSize;
        
        bool deactivateValueFunction;
        bool deactivateBabbling;
        
        int stepCounter;
        int experimentCycles;

        std::string workingDirectory;
        
        bool createWorkingDirectory(const char* foldername);
        bool changeWorkingDirectory(const char* dir);
        
    public:
        Manager(const char* settingfile, int argc, char* argv[], std::string ID="Default");
        ~Manager();

        void reset();
        void step();
        void runExperiment();
        void runIndividualTest();
        void runGenerationTest();

        
        void processTraceMemory();
        void processValueFunction();

        void registerParameters() const;
        void loadParameters();
        
        bool loadMemoriesFromFiles(unsigned generation, bool loadTraces = false);
        
        ValueFunctionMemory* getVFMemory();
        TraceMemory* getTraceMemory();
        PolicyMemory* getPolicyMemory();

        void initializeLocalConnection(SocialManager* sm);
        void setSocialMode(bool mode);
        bool getSocialMode() const;
        
        void log() const;
        
        virtual void setID(std::string& _id);
        
        void operator()();
        
    };

    
}

#endif /* MANAGER_H */
