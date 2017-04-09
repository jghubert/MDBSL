/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Manager.cpp
 * Author: Julien Hubert
 * 
 * Created on June 14, 2016, 6:08 PM
 */

#include "Manager.h"
#include "FitnessLibrary.hpp"
#include "RandomGenerators.h"
#include <iostream>
#include <errno.h>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include "SocialManager.h"
#include "SocialManagerClient.h"

// Manager should initialize the structures: Memories, value function, policy evolution and babbling
// It should also load the settings

using namespace boost::filesystem;

namespace MDB_Social {

    Manager::Manager(const char* settingfile, int argc, char* argv[], std::string _ID)
        : RobotID(_ID)
    {
//        setID(_ID);
        std::cout << "Manager: robotid = " << getID() << std::endl;
        
        RandomGenerator::InitializeRandomSeed();
        
        smc = NULL;
        socialMode = false;
        testIndividualNoValueFunction = false;
        testIndividualNoTrace = false;
        
//        settings = SettingsLibrary::getInstance(ID);
        settings->setParameterSources(settingfile, argc, argv);
        resourceLibrary = ResourceLibrary::getInstance(robotid);

        traceMemory = new TraceMemory();
        traceMemory->setID(robotid);
        vfMemory = new ValueFunctionMemory();
        vfMemory->setID(robotid);
        policyMemory = new PolicyMemory();
        policyMemory->setID(robotid);
        resourceLibrary->setTraceMemory(traceMemory);
        resourceLibrary->setValueFunctionMemory(vfMemory);
        resourceLibrary->setPolicyMemory(policyMemory);

        traceMemoryMaximumSize = 0;
        deactivateValueFunction = false;
        deactivateBabbling = false;
        
        vf = new ValueFunction();
        vf->setID(robotid);
        vf->registerParameters();

        policyGA = new Policy();
        policyGA->setID(robotid);
        policyGA->registerParameters();
        
        this->registerParameters();
        
        // Let's load the settings from the file and the arguments
        if (!settings->processAllParameters(settingfile, argc, argv)) {
            std::cerr << "Manager:: Error loading the parameters." << std::endl;
            exit(1);
        }
        
        loadParameters();
        
        traceMemory->setMaximumSize(traceMemoryMaximumSize);
        
        if (!deactivateValueFunction)
            vf->initialise();
        policyGA->initialise();
        
        resourceLibrary->setPolicy(policyGA);
        resourceLibrary->setValueFunction(vf);
        
        stepCounter = 0;
        
        
        
    }
    
    Manager::~Manager()
    {
        delete vf;
        delete policyGA;
        delete traceMemory;
        delete policyMemory;
        delete resourceLibrary;
        delete settings;
        if (smc)
            delete smc;
    }

    void Manager::reset()
    {
        traceMemory->clear();
        stepCounter = 0;
    }
    
    void Manager::registerParameters() const
    {
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<unsigned>("Trace.RewardBackpropagationStepSize", 20, "Number of steps over which to propagate a received reward.");
        
        settings->registerParameter<int>("Log.loggingFrequency", -1, "Number of updates between logging the memories. -1 logs only before exiting.");
        settings->registerParameter<bool>("Log.logValueFunction", false, "Activates the logging of the value functions");
        settings->registerParameter<bool>("Log.logPolicy", false, "Activates the logging of the policies");
        settings->registerParameter<bool>("Log.logTraces", false, "Activates the logging of the traces");
        settings->registerParameter<std::string>("Log.logFilenamePrefix", "log", "Prefix for the files generated to store the logs.");
        
        settings->registerParameter<bool>("General.testIndividual", false, "Test a previously trained individual.");
        settings->registerParameter<int>("General.Test.individual", -1, "Which individual to test. -1 designates the best individual.");
        settings->registerParameter<int>("General.Test.generation", 0, "Which generation to test.");
        settings->registerParameter<bool>("General.Test.noValueFunction", false, "Do not load the value function for the test");
        settings->registerParameter<bool>("General.Test.noTrace", false, "Do not load the traces for the test");
        settings->registerParameter<unsigned>("General.traceMemoryMaximumSize", 0, "Maximum size for the trace memory (0 = unlimited).");
        settings->registerParameter<bool>("General.deactivateValueFunction", false, "Do not use the value function if set to true.");
        settings->registerParameter<bool>("General.deactivateBabbling", false, "Do not use babbling if set to true.");
        settings->registerParameter<std::string>("General.workingDirectory", std::string("."), "Set the working directory of the robot. In social mode, the default is the ID of the robot.");
        
        settings->registerParameter<int>("General.experimentCycles", 0, "Number of cycles of controller-value function optimization to run.");
    }

    void Manager::loadParameters()
    {
//        Settings* settings = Settings::getInstance();
        RewardBackpropagationStepSize = settings->value<unsigned>("Trace.RewardBackpropagationStepSize").second;
        
        loggingFrequency = settings->value<int>("Log.loggingFrequency").second;
        logValueFunction = settings->value<bool>("Log.logValueFunction").second;
        logPolicy = settings->value<bool>("Log.logPolicy").second;
        logTraces = settings->value<bool>("Log.logTraces").second;
        logFilenamePrefix = settings->value<std::string>("Log.logFilenamePrefix").second;
        
        testIndividual = settings->value<bool>("General.testIndividual").second;
        individualToTest = settings->value<int>("General.Test.individual").second;
        generationToTest = settings->value<int>("General.Test.generation").second;
        testIndividualNoValueFunction = settings->value<bool>("General.Test.noValueFunction").second;
        testIndividualNoTrace = settings->value<bool>("General.Test.noTrace").second;
        traceMemoryMaximumSize = settings->value<unsigned>("General.traceMemoryMaximumSize").second;
        deactivateValueFunction = settings->value<bool>("General.deactivateValueFunction").second;
        deactivateBabbling = settings->value<bool>("General.deactivateBabbling").second;
        experimentCycles = settings->value<int>("General.experimentCycles").second;
        workingDirectory = settings->value<std::string>("General.workingDirectory").second;
        if (workingDirectory.back() == '/')
            workingDirectory.erase(workingDirectory.size()-1);
        if (!changeWorkingDirectory(workingDirectory.c_str()))
            exit(1);
    }

    
    void Manager::step()
    {
        if (stepCounter == 0)
            log();
        else
            policyGA->socialPreProcessing();

        policyGA->run();
//        vf->update();
        
        processTraceMemory();
        
        if (!deactivateValueFunction)
            processValueFunction();
        
        stepCounter++;

        policyGA->publish();
        
        if (!deactivateValueFunction)
            vf->publish();

        policyGA->socialPostProcessing();
        
        log();
        
//        traceMemory->clear();
        
    }
    
    void Manager::operator()()
    {
        runExperiment();
    }

    void Manager::runExperiment()
    {
        if (testIndividual) {  // We just test an individual
            std::cout << "Testing individual number " << individualToTest << " from cycle " << generationToTest << std::endl;
            runIndividualTest();
        }
        else {
            // Don't need this part. Should have done when taking care of the socialMode
//            if (socialMode) {
//                resourceLibrary->setWorkingDirectory(getID());
////                changeWorkingDirectory(getID().c_str());
//            }

            stepCounter = 0;
            std::cout << "Running experiment for " << experimentCycles << " cycles..." << std::endl;

            policyGA->publish();
            if (!deactivateValueFunction)
                vf->publish();

            if (!deactivateBabbling)
                policyGA->setBabblingRecommendation(true);   // Because it's the first time we launch the experiment and have no value function yet
            
            while (stepCounter < experimentCycles)
                step();            
        }
        
    }
    
    void Manager::runIndividualTest()
    {
        // requirement: the fitness is loaded.
        // TODO: load the backup files (loadMemoriesFromFile)
        //       Call the GAFitness with the selected genotype
        
        if (!loadMemoriesFromFiles(generationToTest, !testIndividualNoTrace)) {   // No need for the traces
            std::cerr << "Failed to load the memories." << std::endl;
            exit(1);
        }
        
        // Select the genotype and launch the fitness
        GAFitness* fit = policyGA->getFitnessFunction();
        if (individualToTest < policyMemory->size()) {
            double fitval = fit->evaluateFitness((*policyMemory)[individualToTest]);
            std::cout << "The fitness of the individual (index = " << individualToTest << "; evaluated fitness = " << (*policyMemory)[individualToTest].getFitness() << ") is " << fitval << std::endl;
        }
        else {
            std::cerr << "Manager: Error with the index of the individual to test. The memory contains only " << policyMemory->size() << " individuals." << std::endl;
            exit(1);
        }
        
    }

    
    void Manager::processTraceMemory()
    {
        // Need to backpropagate the rewards.
        
        traceMemory->trimMemory();
        
        double delta = 0.0;
        double last_reward = 0.0;;
        TraceMemory::reverse_iterator itend = traceMemory->rend();
        for (TraceMemory::reverse_iterator it = traceMemory->rbegin(); it != itend; ++it ) {
            if (it->true_reward > 1e-6) {
                it->expected_reward = it->true_reward;
                last_reward = it->true_reward;
                delta = last_reward / (RewardBackpropagationStepSize*1.0);
            }
            else if (last_reward > 1e-6) {
                last_reward -= delta;
                if (last_reward < 1e-6)
                    last_reward = 0.0;
                it->expected_reward = last_reward;
            }
            else
                it->expected_reward = 0.0;
        }
    }
    
    void Manager::processValueFunction()
    {
        // Get the data from the memory and learn it
        
        vf->evaluateTraceMemoryQuality();
        if (vf->isItReliable()) {
            std::cout << "Manager: Traces for the value function are reliable." << std::endl;
            policyGA->setBabblingRecommendation(false);
            vf->update();
        }
        else {
            std::cout << "Manager: Traces for the value function are unreliable (quality = "<< vf->getQuality() <<"). Babbling recommended." << std::endl;
            if (!deactivateBabbling)
                policyGA->setBabblingRecommendation(true);
        }
            
    }

    
    bool Manager::loadMemoriesFromFiles(unsigned generation, bool loadTraces)
    {
        std::cout << "Manager:: Loading memories: ";
        
        std::string filename;
        std::string cwd = resourceLibrary->getWorkingDirectory();
        
        if ((testIndividual && !testIndividualNoValueFunction)) {
            filename = cwd + "/" + logFilenamePrefix + ".value_function.step_" + std::to_string(generation) + ".log";
            if (!vfMemory->loadFromFile(filename.c_str())) {
                std::cerr << "Manager::loadMemoriesFromFiles: An error occurred when loading the value function from the file " << filename << std::endl;
                return false;
            }
            else {
                std::cout << "Value Function ; ";
                vf->loadFromExternalMemory();
            }
        }
        
        filename = cwd + "/" + logFilenamePrefix + ".policy.step_" + std::to_string(generation) + ".log";
        if (!policyMemory->loadFromFile(filename.c_str())) {
            std::cerr << "Manager::loadMemoriesFromFiles: An error occurred when loading the policies from the file " << filename << std::endl;
            return false;
        }
        else
            std::cout << "Policies ; ";
        
        if (loadTraces) {
            filename = cwd + "/" + logFilenamePrefix + ".traces.step_" + std::to_string(generation) + ".log";
            if (!traceMemory->loadFromFile(filename.c_str())) {
                std::cerr << "Manager::loadMemoriesFromFiles: An error occurred when loading the traces from the file " << filename << std::endl;
                return false;
            }
            else {
                std::cout << " Traces.";
                if (vf) {
                    std::cout << "Manager: Loading familiarity from traces: ";
                    std::cout << vf->loadFamiliarityFromTraces(traceMemory) << std::endl;
                }
            }
        }
        std::cout << std::endl;
        
        return true;
    }
    
    
    void Manager::log() const
    {
        if (stepCounter % loggingFrequency == 0) {
            std::string filename;
            std::string cwd = resourceLibrary->getWorkingDirectory();
            
            if (logValueFunction && !deactivateValueFunction) {
                filename = cwd + "/" + logFilenamePrefix + ".value_function.step_" + std::to_string(stepCounter) + ".log";
                if (!vfMemory->saveToFile(filename.c_str()))
                    std::cerr << "Manager::log: An error occurred when logging the value function in the file " << filename << std::endl;
            }
            if (logPolicy) {
                filename = cwd + "/" + logFilenamePrefix + ".policy.step_" + std::to_string(stepCounter) + ".log";
                if (!policyMemory->saveToFile(filename.c_str()))
                    std::cerr << "Manager::log: An error occurred when logging the policies in the file " << filename << std::endl;
            }
            
            if (logTraces) {
                filename = cwd + "/" + logFilenamePrefix + ".traces.step_" + std::to_string(stepCounter) + ".log";
                if (!traceMemory->saveToFile(filename.c_str()))
                    std::cerr << "Manager::log: An error occurred when logging the traces in the file " << filename << std::endl;
            }
        }
    }

    void Manager::setID(std::string& _id)
    {
        RobotID::setID(_id);
        vf->setID(_id);
        policyGA->setID(_id);
        vfMemory->setID(_id);
        traceMemory->setID(_id);
        policyMemory->setID(_id);
    }

    ValueFunctionMemory* Manager::getVFMemory()
    {
        return vfMemory;
    }
    
    TraceMemory* Manager::getTraceMemory()
    {
        return traceMemory;
    }
    
    PolicyMemory* Manager::getPolicyMemory()
    {
        return policyMemory;
    }
    
    void Manager::initializeLocalConnection(SocialManager* sm)
    {
        if (!smc) {
            smc = new SocialManagerClient();
        }

        smc->setupLocalConnection(sm);
        
        resourceLibrary->setSocialManagerClient(smc);
    }
    
    void Manager::setSocialMode(bool mode)
    {
        socialMode = mode;
        if (socialMode) {
            createWorkingDirectory(getID().c_str());
            workingDirectory += std::string("/") + getID();
            resourceLibrary->setWorkingDirectory(workingDirectory);
        }
    }
    
    bool Manager::getSocialMode() const
    {
        return socialMode;
    }
    
    bool Manager::createWorkingDirectory(const char* foldername)
    {
        bool ret = true;
        path p(foldername);
        if (!exists(p)) {
            ret = create_directory(p);
            if (!ret) {
                std::cerr << "Manager (robotid = " << getID() << "): Error creating the working directory. Directory " << foldername << " cannot be created and does not exist." << std::endl;
                exit (1);
            }
        }
        ret = is_directory(p);
        if (!ret) {
            std::cerr << "Manager (robotid = " << getID() << "): Error with the working directory. " << foldername << " already exists and is not a directory." << std::endl;
            exit(1);
        }
        return ret;
    }
    
    bool Manager::changeWorkingDirectory(const char* dir)
    {
        if (chdir(dir) != 0) {
            perror("Manager::changeWorkingDirectory: Error with chdir");
            return false;
        }
        else {
            resourceLibrary->setWorkingDirectory(std::string(dir));
        }
        return true;
    }
    
}