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
#include "ResourceLibrary.hpp"
#include "Settings.h"
#include <errno.h>
#include <cmath>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include "SocialManager.h"
#include "SocialManagerClient.h"
#include "RobotID.h"

// Manager should initialize the structures: Memories, value function, policy evolution and babbling
// It should also load the settings

using namespace boost::filesystem;

namespace MDB_Social {

    Manager::Manager(const char* _settingfile, int _argc, char* _argv[], std::string _ID, std::string _cwd)
    {
        localRobotID = _ID;
        initialized = false;
        cwd = _cwd;
        argc = _argc;
        argv = _argv;
        settingFile = std::string(_settingfile);

        settings = SettingsLibrary::getInstance(_ID);
        resourceLibrary = ResourceLibrary::getInstance(_ID);
        
        smc = NULL;
        socialMode = false;
        testIndividualNoValueFunction = false;
        testIndividualNoTrace = false;
        testGeneration = false;
        resourceLibrary->setWorkingDirectory(_cwd);
        
        traceMemoryMaximumSize = 0;
        deactivateValueFunction = false;
        deactivateBabbling = false;
        doNotLoadValueFunctionToMemory = false;
        
    }
    
    Manager::~Manager()
    {
        delete vf;
        delete policyGA;
        delete traceMemory;
        delete policyMemory;
        delete resourceLibrary->getGaussianGenerator();
        delete resourceLibrary->getUniformGenerator();
        delete resourceLibrary->getUniformIntegerGenerator();
        delete resourceLibrary;
        delete settings;
        if (smc)
            delete smc;
    }

    void Manager::initializeManager()
    {
        if (initialized)
            return;
//        setID(_ID);
        RobotID::setID(localRobotID);
        settings = RobotID::getSettings();
        resourceLibrary = RobotID::getResourceLibrary();
        std::cout << "Manager: robotid = " << RobotID::getID() << std::endl;
        
        RandomGenerator::InitializeRandomSeed();
        resourceLibrary->setUniformGenerator(UniformGenerator::getLocalInstance());
        resourceLibrary->setUniformIntegerGenerator(UniformIntegerGenerator::getLocalInstance());
        resourceLibrary->setGaussianGenerator(GaussianGenerator::getLocalInstance());
        resourceLibrary->getGaussianGenerator()->InitializeRandomSeed();
        resourceLibrary->getUniformGenerator()->InitializeRandomSeed();
        resourceLibrary->getUniformIntegerGenerator()->InitializeRandomSeed();
        
        
//        settings = SettingsLibrary::getInstance(ID);
        settings->setParameterSources(settingFile.c_str(), argc, argv);
//        resourceLibrary = ResourceLibrary::getInstance(robotid);

        traceMemory = new TraceMemory();
//        traceMemory->setID(RobotID::robotid);
        vfMemory = new ValueFunctionMemory();
//        vfMemory->setID(RobotID::robotid);
        policyMemory = new PolicyMemory();
//        policyMemory->setID(RobotID::robotid);
        resourceLibrary->setTraceMemory(traceMemory);
        resourceLibrary->setValueFunctionMemory(vfMemory);
        resourceLibrary->setPolicyMemory(policyMemory);
                
        vf = new ValueFunction();
//        vf->setID(RobotID::robotid);
        vf->registerParameters();

        policyGA = new Policy();
//        policyGA->setID(RobotID::robotid);
        policyGA->registerParameters();
        
        this->registerParameters();
        
        // Let's load the settings from the file and the arguments
        if (!settings->processAllParameters(settingFile.c_str(), argc, argv)) {
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
        initialized = true;
        
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
        settings->registerParameter<unsigned>("Trace.RewardBackpropagationStepRepeatSize", 1, "Number of repetitions of each step when propagating the reward.");
        settings->registerParameter<double>("Trace.RewardBackpropagationMinimumAbsoluteValue", 0.1, "Minimum value for a reward to be backpropagated.");
        
        settings->registerParameter<int>("Log.loggingFrequency", -1, "Number of updates between logging the memories. -1 logs only before exiting.");
        settings->registerParameter<bool>("Log.logValueFunction", false, "Activates the logging of the value functions");
        settings->registerParameter<bool>("Log.logPolicy", false, "Activates the logging of the policies");
        settings->registerParameter<bool>("Log.logTraces", false, "Activates the logging of the traces");
        settings->registerParameter<std::string>("Log.logFilenamePrefix", "log", "Prefix for the files generated to store the logs.");
        
        settings->registerParameter<bool>("General.testIndividual", false, "Test a previously trained individual.");
        settings->registerParameter<bool>("General.testGeneration", false, "Test all the individuals of a previously evolved generation.");
        settings->registerParameter<int>("General.Test.individual", -1, "Which individual to test. -1 designates the best individual.");
        settings->registerParameter<int>("General.Test.generation", 0, "Which generation to test.");
        settings->registerParameter<bool>("General.Test.noValueFunction", false, "Do not load the value function for the test");
        settings->registerParameter<bool>("General.Test.noTrace", false, "Do not load the traces for the test");
        settings->registerParameter<unsigned>("General.traceMemoryMaximumSize", 0, "Maximum size for the trace memory (0 = unlimited).");
        settings->registerParameter<bool>("General.deactivateValueFunction", false, "Do not use the value function if set to true.");
        settings->registerParameter<bool>("General.deactivateBabbling", false, "Do not use babbling if set to true.");
        settings->registerParameter<bool>("General.doNotLoadValueFunctionToMemory", false, "Do not load the value function from the memory file. It must then be handle by the value function itself.");
//        settings->registerParameter<std::string>("General.workingDirectory", std::string("."), "Set the working directory of the robot. In social mode, the default is the ID of the robot.");
        
        settings->registerParameter<int>("General.experimentCycles", 0, "Number of cycles of controller-value function optimization to run.");
    }

    void Manager::loadParameters()
    {
//        Settings* settings = Settings::getInstance();
        RewardBackpropagationStepSize = settings->value<unsigned>("Trace.RewardBackpropagationStepSize").second;
        RewardBackpropagationStepRepeatSize = settings->value<unsigned>("Trace.RewardBackpropagationStepRepeatSize").second;
        RewardBackpropagationMinimumAbsoluteValue = settings->value<double>("Trace.RewardBackpropagationMinimumAbsoluteValue").second;
        
        loggingFrequency = settings->value<int>("Log.loggingFrequency").second;
        logValueFunction = settings->value<bool>("Log.logValueFunction").second;
        logPolicy = settings->value<bool>("Log.logPolicy").second;
        logTraces = settings->value<bool>("Log.logTraces").second;
        logFilenamePrefix = settings->value<std::string>("Log.logFilenamePrefix").second;
        
        testGeneration = settings->value<bool>("General.testGeneration").second;
        testIndividual = settings->value<bool>("General.testIndividual").second;
        individualToTest = settings->value<int>("General.Test.individual").second;
        generationToTest = settings->value<int>("General.Test.generation").second;
        testIndividualNoValueFunction = settings->value<bool>("General.Test.noValueFunction").second;
        testIndividualNoTrace = settings->value<bool>("General.Test.noTrace").second;
        traceMemoryMaximumSize = settings->value<unsigned>("General.traceMemoryMaximumSize").second;
        deactivateValueFunction = settings->value<bool>("General.deactivateValueFunction").second;
        deactivateBabbling = settings->value<bool>("General.deactivateBabbling").second;
        experimentCycles = settings->value<int>("General.experimentCycles").second;
        doNotLoadValueFunctionToMemory = settings->value<bool>("General.doNotLoadValueFunctionToMemory").second;
//        workingDirectory = settings->value<std::string>("General.workingDirectory").second;
//        if (workingDirectory.back() == '/')
//            workingDirectory.erase(workingDirectory.size()-1);
//        if (!changeWorkingDirectory(workingDirectory.c_str()))
//            exit(1);
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
        initializeManager();
        runExperiment();
    }

    void Manager::runExperiment()
    {
        if (testIndividual) {  // We just test an individual
            std::cout << "Testing individual number " << individualToTest << " from cycle " << generationToTest << std::endl;
            runIndividualTest();
        }
        else if (testGeneration) {
            runGenerationTest();
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
            std::cerr << "Manager: runIndividualTest: Failed to load the memories for generation " << generationToTest << " ." << std::endl;
            exit(1);
        }
        
        // Select the genotype and launch the fitness
        GAFitness* fit = policyGA->getFitnessFunction();
        if (individualToTest < policyMemory->size()) {
            double fitval = fit->evaluateFitness((*policyMemory)[individualToTest], generationToTest, individualToTest, true);
            std::cout << "The fitness of the individual (index = " << individualToTest << "; evaluated fitness = " << (*policyMemory)[individualToTest].getFitness() << ") is " << fitval << std::endl;
        }
        else {
            std::cerr << "Manager: Error with the index of the individual to test. The memory contains only " << policyMemory->size() << " individuals." << std::endl;
            exit(1);
        }
        
    }

    void Manager::runGenerationTest()
    {
        if (!loadMemoriesFromFiles(generationToTest, !testIndividualNoTrace)) {   // No need for the traces
            std::cerr << "Manager: runGenerationTest: Failed to load the memories for generation " << generationToTest << " ." << std::endl;
            exit(1);
        }
        
        // Select the genotype and launch the fitness
        GAFitness* fit = policyGA->getFitnessFunction();
        unsigned index = 0;
        std::cout << "Testing generation " << generationToTest << " : ";
        std::cout.flush();
        for (PolicyMemory::iterator it = policyMemory->begin(); it != policyMemory->end(); ++it) {
            std::cout << index << " (fit=" << fit->evaluateFitness(*it, generationToTest, index, true) << ") ";
            std::cout.flush();
            index++;
        }
        
    }
    
    void Manager::processTraceMemory()
    {
        // Need to backpropagate the rewards.
        if (!traceMemory->size())
            return;
        
        traceMemory->trimMemory();
        
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
        
        if (!doNotLoadValueFunctionToMemory) {    // Yeah it's ugly, but I'm tired.
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
        std::cout << "****************** Manager::initializeLocalConnection : ID = " << resourceLibrary->getLocalID() << std::endl;
        resourceLibrary->setSocialManagerClient(smc);
    }
    
    void Manager::setSocialMode(bool mode)
    {
        socialMode = mode;
        if (socialMode) {
            createWorkingDirectory(resourceLibrary->getWorkingDirectory().c_str());
//            workingDirectory += std::string("/") + ;
//            resourceLibrary->setWorkingDirectory(workingDirectory);
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
                std::cerr << "Manager (robotid = " << RobotID::getID() << "): Error creating the working directory. Directory " << foldername << " cannot be created and does not exist." << std::endl;
                exit (1);
            }
        }
        ret = is_directory(p);
        if (!ret) {
            std::cerr << "Manager (robotid = " << RobotID::getID() << "): Error with the working directory. " << foldername << " already exists and is not a directory." << std::endl;
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