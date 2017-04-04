/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_1DPhototaxis.cpp
 * Author: Julien Hubert
 * 
 * Created on October 5, 2016, 10:39 AM
 */

#include "FastSim_1DPhototaxis.h"
#include <vector>

namespace MDB_Social {

    FastSim_1DPhototaxis::FastSim_1DPhototaxis() 
    {
        std::vector<enum FeedforwardNN::ActivationFunction> activationFunctions(1, FeedforwardNN::SIGMOID);
        
        nbinputs = 1 + 1; // light sensor and 1 infrared sensor
        nboutputs = 2;
        controller = static_cast<FeedforwardNN*>(ModelLibrary::getModel("Feedforward"));
        layers.resize(2);
        layers[0] = nbinputs;
        layers[1] = nboutputs;
        controller->setup(layers, activationFunctions);
        
        controllerMinimumWeight = -10.0;
        controllerMaximumWeight = 10.0;
        
        babbling = static_cast<BabblingStandard*>(ModelLibrary::getModel("BabblingStandard"));
        std::vector<double> outmin(nboutputs, 0.2);
        std::vector<double> outmax(nboutputs, 1.0);
        babbling->setOutputMinMax(outmin,outmax);
        babbling->registerParameters("experiment");
        useOnlyBabbling = false;
        
        logRobotPos = false;
        sensorLog = false;
        
    }

    FastSim_1DPhototaxis::FastSim_1DPhototaxis(const FastSim_1DPhototaxis& orig) 
    {
        
    }

    FastSim_1DPhototaxis::~FastSim_1DPhototaxis() 
    {
        
    }

    void FastSim_!DPhototaxis::registerParameters()
    {
        std::cout << "FastSim_1DPhototaxis : registering the parameters...";
        std::cout.flush();
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<double>("experiment.controllerMinimumWeight", -10.0, "Minimum weight of the neural network.");
        settings->registerParameter<double>("experiment.controllerMaximumWeight", 10.0, "Maximum weight of the neural network.");
        settings->registerParameter<int>("experiment.trialCount", 0, "Number of trials for the experiment.");
        settings->registerParameter<int>("experiment.epochCount", 0, "Number of steps for one trial of the experiment.");
        settings->registerParameter<double>("experiment.timestep", 0.01, "Simulation step of the experiment in seconds.");
        settings->registerParameter<double>("experiment.rewardZoneDiameter", 10.0, "Diameter of the reward zone.");
        settings->registerParameter<bool>("experiment.useOnlyBabbling", false, "Use only the babbling controller.");
        settings->registerParameter<bool>("experiment.logRobotPosition", false, "Output the position of the robot in a log file.");
        settings->registerParameter<bool>("experiment.sensorLogFlag", false, "Log the sensors values during the experiment.");
        settings->registerParameter<unsigned>("experiment.hiddenNeurons", 10, "Number of hidden neurons for the controller.");
        std::cout << " DONE" << std::endl;
    }
    
    void FastSim_1DPhototaxis::loadParameters()
    {
        std::cout << "FastSim_1DPhototaxis: Loading parameters..." << std::endl;
//        Settings* settings = Settings::getInstance();
        try {
            controllerMinimumWeight = settings->value<double>("experiment.controllerMinimumWeight").second;
            controllerMaximumWeight = settings->value<double>("experiment.controllerMaximumWeight").second;
            trialCount = settings->value<int>("experiment.trialCount").second;
            epochCount = settings->value<int>("experiment.epochCount").second;
            timestep = settings->value<double>("experiment.timestep").second;
            rewardZoneDiameter = settings->value<double>("experiment.rewardZoneDiameter").second;
            useOnlyBabbling = settings->value<bool>("experiment.useOnlyBabbling").second;
            logRobotPos = settings->value<bool>("experiment.logRobotPosition").second;
            sensorLog = settings->value<bool>("experiment.sensorLogFlag").second;
            hiddenNeurons = settings->value<unsigned>("experiment.hiddenNeurons").second;
            babbling->loadParameters("experiment");
            world->initialize();
            std::cout << "FastSim_1DPhototaxis: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "FastSim_1DPhototaxis: Error loading the parameters: " << e.what() << std::endl;
            exit(1);
        }
    }

}
