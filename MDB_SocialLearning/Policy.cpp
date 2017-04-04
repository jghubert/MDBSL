/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Policy.cpp
 * Author: Julien Hubert
 * 
 * Created on August 5, 2016, 3:35 PM
 */

#include <iostream>
#include "Policy.h"
#include "Settings.h"
#include "GALibrary.hpp"
#include "ResourceLibrary.hpp"

namespace MDB_Social {

    Policy::Policy() 
    {
        ga = NULL;
        babbling = new BabblingPolicy();
        babbling->setID(robotid);
        recommendBabbling = false;
    }

    Policy::~Policy() 
    {
        if (ga)
            delete ga;
        delete babbling;
    }

    
    void Policy::loadParameters()
    {
//        Settings* settings = Settings::getInstance();
        geneticAlgorithmType = settings->value<std::string>("policy.geneticAlgorithmType").second;
        
        babbling->loadParameters();
    }

    void Policy::registerParameters()
    {
//        Settings* settings = Settings::getInstance();
//        settings->registerParameter<std::string>("policy.geneticAlgorithmType", std::string("ECF"), "Type of genetic algorithm to use for evolving the policy.");
        if (settings->registerAndRetrieveParameter<std::string>(geneticAlgorithmType, "policy.geneticAlgorithmType", std::string("SimpleGeneticAlgorithm"), "Type of genetic algorithm to use for evolving the policy.")) {
            ga = GALibrary::getModel(geneticAlgorithmType);
            ga->setID(robotid);
            ga->registerParameters();  // Should be automatically loaded from this call.
            resourceLibrary->setGeneticAlgorithm(ga);
        }
        
        settings->registerCallbackFunction(this->callbackParameters, this);
        
        babbling->registerParameters();
    }
    
    bool Policy::initialise()
    {
        if (!geneticAlgorithmType.empty()) {
            if (!ga) {
                ga = GALibrary::getModel(geneticAlgorithmType);
                ga->registerParameters();  // Should be automatically loaded from this call.
                resourceLibrary->setGeneticAlgorithm(ga);
            }
            ga->initialise();
            ga->setExternalMemory(resourceLibrary->getPolicyMemory());
        }
        else
            return false;

        babbling->initialise();
        
        return true;
    }

    void Policy::callbackParameters(void* c)
    {
        Policy* inst = static_cast<Policy*>(c);
        inst->loadParameters();
    }

    void Policy::run()
    {
        if (recommendBabbling)
            babbling->run();
        else {
            ga->setWorkingDirectory(resourceLibrary->getWorkingDirectory().c_str());
            ga->run();
        }
    }
    
    void Policy::publish()
    {
        if (!recommendBabbling)
            ga->publish();
    }
    
    GAFitness* Policy::getFitnessFunction()
    {
        return ga->getFitnessFunction();
    }
    
    void Policy::setBabblingRecommendation(bool b)
    {
        recommendBabbling = b;
        ga->getFitnessFunction()->setBabblingRecommendation(b);
    }

    void Policy::setID(std::string& _id)
    {
        RobotID::setID(_id);
        if (ga)
            ga->setID(_id);
        babbling->setID(_id);
    }

    void Policy::socialPreProcessing()
    {
        ga->getFitnessFunction()->preprocessing();
    }

    void Policy::socialPostProcessing()
    {
        ga->getFitnessFunction()->postprocessing();
    }
    
    
}

