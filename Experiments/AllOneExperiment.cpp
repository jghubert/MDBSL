/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   AllOneExperiment.cpp
 * Author: Julien Hubert
 *
 * Created on August 12, 2016, 6:10 PM
 */


#include "AllOneExperiment.h"
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/SocialManagerClient.h"
#include "RobotID.h"

namespace MDB_Social {

    AllOneExperiment::AllOneExperiment()
    {
        
    }
    
    AllOneExperiment::~AllOneExperiment()
    {
        
    }

    void AllOneExperiment::loadParameters()
    {
        
    }

    void AllOneExperiment::preprocessing() 
    {
        // Let's get the list of robots.
        ResourceLibraryData* resourceLibrary = RobotID::getResourceLibrary();
        SocialManagerClient* smclient = resourceLibrary->getSocialManagerClient();
//        SocialManagerClient* smclient = NULL;
//            std::cout << " p " << std::endl;
        if (smclient) {  // test if we are in a social environment

            smclient->synchronise();
            
        }        
    }
    
    double AllOneExperiment::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
        double fit = 0.0;
        unsigned gensize = individual.getSize();
        for (unsigned i=0; i<gensize; ++i) {
            fit += individual[i];
        }
        fit /= 1.0*gensize;
        
        return fit;
    }
    
    
    
}
