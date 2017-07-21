/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GaussianProcess.cpp
 * Author: julienhubert
 * 
 * Created on July 21, 2017, 3:46 PM
 */

#include "GaussianProcess.h"
#include "Settings.h"

namespace MDB_Social {

    GaussianProcess::GaussianProcess() 
    {
        gp = NULL;
    }

    GaussianProcess::~GaussianProcess() 
    {
        if (gp)
            delete gp;
    }
    
    void GaussianProcess::registerParameters(std::string prefix)
    {
        Settings* settings = RobotID::getSettings();
        settings->registerParameter<int>((prefix+".inputDimensionality").c_str(), 0, "GaussianProcess: Dimensionality of the inputs.");
        settings->registerParameter<std::string>((prefix+".covarianceFunction").c_str(), std::string("CovSum ( CovSEiso, CovNoise)"), "GaussianProcess: Dimensionality of the inputs.");
    }        

    void GaussianProcess::loadParameters(std::string prefix)
    {
        Settings* settings = RobotID::getSettings();
        inputDimensionality = settings->value<int>((prefix+".inputDimensionality").c_str()).second;
        covarianceFunction = settings->value<std::string>((prefix+".covarianceFunction").c_str()).second;
        
        gp = new libgp::GaussianProcess(inputDimensionality, covarianceFunction);
    }

    void GaussianProcess::initializeFromParameters()
    {
        
    }
    
    void GaussianProcess::train()
    {
        // So far no training required
    }

    void GaussianProcess::reset()
    {
        gp->clear_sampleset();
    }

    void GaussianProcess::publish()
    {
        
    }

    void GaussianProcess::setExternalMemory(Memory<NeuralNetworkData>* mem)
    {
        externalMemory = mem;
    }
    
    bool GaussianProcess::loadFromExternalMemory()
    {
        return false;
    }
    
    bool GaussianProcess::saveModelToFile(const char* filename)
    {
        if (gp) {
            gp->write(filename);            
        }
        return gp != NULL;
    }
    
    
}