/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BabblingPolicy.cpp
 * Author: Julien Hubert
 * 
 * Created on October 10, 2016, 11:38 AM
 */

#include "BabblingPolicy.h"
#include "ResourceLibrary.hpp"
#include "Settings.h"

namespace MDB_Social {


    BabblingPolicy::BabblingPolicy() 
    {
        fitness = NULL;
        trialCount = 1;
    }

    BabblingPolicy::~BabblingPolicy() 
    {

    }

    void BabblingPolicy::initialise()
    {
        fitness = resourceLibrary->getGAFitness();
        
    }
    
    void BabblingPolicy::registerParameters()
    {
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<unsigned>("Babbling.trialCount", 1, "Babbling: Number of trials per babbling session.");
    }
    
    void BabblingPolicy::loadParameters()
    {
//        Settings* settings = Settings::getInstance();
        trialCount = settings->value<unsigned>("Babbling.trialCount").second;
    }
    
    
    bool BabblingPolicy::run()
    {
        if (!fitness) {
            std::cerr << "BabblingPolicy: ERROR : Fitness function not provided." << std::endl;
            exit(1);
        }
        
        unsigned ulim = std::numeric_limits<unsigned>::max();
        std::cout << "Launching babbling for " << trialCount << " trials." << std::endl;
        std::cout << "   Trial ";
        for (unsigned i=0; i<trialCount; ++i) {
            std::cout << i << " ";
            std::cout.flush();
            fitness->evaluateFitness(dummyGenotype, ulim, ulim, false);
        }
        std::cout <<" DONE" << std::endl;
        
        return true;
    }
    
    
    
    
}