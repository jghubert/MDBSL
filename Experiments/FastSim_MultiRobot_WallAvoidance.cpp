/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_MultiRobot_WallAvoidance.cpp
 * Author: Julien Hubert
 * 
 * Created on February 20, 2017, 5:30 PM
 */

#include "../MDB_SocialLearning/Settings.h"
#include "FastSim_MultiRobot_WallAvoidance.h"
#include <cstdlib>
#include <algorithm>
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/SocialManagerClient.h"
#include "../MDB_SocialLearning/PolicyMemory.hpp"

namespace MDB_Social {

    FastSim_MultiRobot_WallAvoidance::FastSim_MultiRobot_WallAvoidance(std::string id) 
    {
        std::cout << "FastSim_MultiRobot_WallAvoidance = " << id << std::endl;
        setID(id);
        registerParameters();
        
        world = static_cast<FastSimSimulator*>(this->getSimulator("FastSim"));
        world->setID(id);
        world->registerParameters();

        
        nbinputs = 1;
        nboutputs = 2;
        hiddenNeurons = 5;
        
        controllerMinimumWeight = -10.0;
        controllerMaximumWeight = 10.0;

        useOnlyBabbling = false;

        logRobotPos = false;
        sensorLog = false;
    }

    FastSim_MultiRobot_WallAvoidance::~FastSim_MultiRobot_WallAvoidance() 
    {
        delete world;
        delete controller;
    }

    void FastSim_MultiRobot_WallAvoidance::registerParameters()
    {
        std::cout << "FastSim_Phototaxis : registering the parameters...";
        std::cout.flush();
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<unsigned>("experiment.nbinputs", 1, "Number of inputs/sensors on the neural network.");
        settings->registerParameter<unsigned>("experiment.nboutputs", 2, "Number of outputs on the neural network.");
        settings->registerParameter<unsigned>("experiment.hiddenNeurons", 10, "Number of hidden neurons for the controller.");
        settings->registerParameter<double>("experiment.controllerMinimumWeight", 0.0, "Minimum weight of the neural network.");
        settings->registerParameter<double>("experiment.controllerMaximumWeight", 0.0, "Maximum weight of the neural network.");
        settings->registerParameter<int>("experiment.trialCount", 0, "Number of trials for the experiment.");
        settings->registerParameter<int>("experiment.epochCount", 0, "Number of steps for one trial of the experiment.");
        settings->registerParameter<double>("experiment.timestep", 0.01, "Simulation step of the experiment in seconds.");
//        settings->registerParameter<bool>("experiment.useOnlyBabbling", false, "Use only the babbling controller.");
//        settings->registerParameter<bool>("experiment.useOnlyTrueReward", false, "Use only true rewards, no estimation through the value function");
        settings->registerParameter<bool>("experiment.logRobotPosition", false, "Output the position of the robot in a log file.");
        settings->registerParameter<bool>("experiment.sensorLogFlag", false, "Log the sensors values during the experiment.");
        settings->registerParameter<double>("experiment.maxSpeed", 5, "Maximum speed of the robot.");
//        settings->registerParameter<bool>("experiment.valueFunctionTest", false, "Produce a map of the environment in terms of potential reward from the VF.");
        std::cout << " DONE" << std::endl;
    }
    
    void FastSim_MultiRobot_WallAvoidance::loadParameters()
    {
        std::cout << "FastSim_Phototaxis: Loading parameters..." << std::endl;
//        Settings* settings = Settings::getInstance();
        try {
            nbinputs = settings->value<unsigned>("experiment.nbinputs").second;
            nboutputs = settings->value<unsigned>("experiment.nboutputs").second;
            hiddenNeurons = settings->value<unsigned>("experiment.hiddenNeurons").second;
            controllerMinimumWeight = settings->value<double>("experiment.controllerMinimumWeight").second;
            controllerMaximumWeight = settings->value<double>("experiment.controllerMaximumWeight").second;
            trialCount = settings->value<int>("experiment.trialCount").second;
            epochCount = settings->value<int>("experiment.epochCount").second;
            timestep = settings->value<double>("experiment.timestep").second;
            useOnlyBabbling = settings->value<bool>("experiment.useOnlyBabbling").second;
            logRobotPos = settings->value<bool>("experiment.logRobotPosition").second;
            sensorLog = settings->value<bool>("experiment.sensorLogFlag").second;
            maxSpeed = settings->value<double>("experiment.maxSpeed").second;
//            useOnlyTrueReward = settings->value<bool>("experiment.useOnlyTrueReward").second;
//            valueFunctionTest = settings->value<bool>("experiment.valueFunctionTest").second;
//            babbling->loadParameters("experiment");
            world->initialize();
            std::cout << "FastSim_Phototaxis: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "FastSim_Phototaxis: Error loading the parameters: " << e.what() << std::endl;
            exit(1);
        }
        
        controller = static_cast<FeedforwardNN*>(ModelLibrary::getModel("Feedforward"));
        std::vector<enum FeedforwardNN::ActivationFunction> activationFunctions(1, FeedforwardNN::SIGMOID);
        if (hiddenNeurons) {
            layers.resize(3);
            layers[0] = nbinputs;
            layers[1] = hiddenNeurons;
            layers[2] = nboutputs;
        }
        else {
            layers.resize(2);
            layers[0] = nbinputs;
            layers[1] = nboutputs;
        }
        controller->setup(layers, activationFunctions);
        
    }

    void FastSim_MultiRobot_WallAvoidance::preprocessing() 
    {
        // Let's get the list of robots.
        SocialManagerClient* smclient = resourceLibrary->getSocialManagerClient();
//        SocialManagerClient* smclient = NULL;
//            std::cout << " p " << std::endl;
        if (smclient) {  // test if we are in a social environment
            
            smclient->synchronise();
            std::string other_robot = smclient->getRandomRobotID(getID());
            // Retrieve its genotypes
            PolicyMemory* pm = smclient->getPolicyMemory(other_robot);
            // Find the best policy and replace the worst current one
            unsigned best = 0;
            double bestFitness = (*pm)[best].getFitness();
            for (unsigned i = 1; i < pm->size(); ++i) {
                if ((*pm)[i].getFitness() > bestFitness) {
                    best = i;
                    bestFitness = (*pm)[best].getFitness();
                }
            }
            // We have the best. We need to compare the fitness to the worst individual in our population
            PolicyMemory* mypm = smclient->getPolicyMemory(getID());
            unsigned worst = 0;
            double worstFitness = (*mypm)[worst].getFitness();
            for (unsigned i = 1; i < mypm->size(); ++i) {
                if ((*mypm)[i].getFitness() < worstFitness) {
                    worst = i;
                    worstFitness = (*mypm)[worst].getFitness();
                }
            }
            if (bestFitness > worstFitness) {
//                std::cout << " Social learning: copying " << robotIds[other_robot] << ":" << best << " to " << getID() << ":" << worst << std::endl;
                resourceLibrary->getGeneticAlgorithm()->importGenotype(&(*pm)[best], worst);
            }
        }        
    }
    
    void FastSim_MultiRobot_WallAvoidance::postprocessing() 
    {
        
    }
    
    
    void FastSim_MultiRobot_WallAvoidance::installGenotype(Genotype& individual)
    {
        std::vector<FeedforwardNN::weight_t> weights(individual.getSize());
        
        controller->getWeights(weights);

        if (weights.size() != individual.getSize()) {
            std::cerr << "FastSim_Phototaxis: Size of the genotype (size = " << individual.getSize() << ") differs from the size of the network (size = " << weights.size() << ")." << std::endl;
            exit(1);
        }
        
        for (unsigned i=0; i<individual.getSize(); ++i) {
            weights[i].weight =  (controllerMaximumWeight - controllerMinimumWeight)*individual[i] + controllerMinimumWeight;
        }
        controller->setWeights(weights);
    }

    void FastSim_MultiRobot_WallAvoidance::logRobotPosition(unsigned trial, unsigned epoch)
    {
        if (logRobotPos) {
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double orient = world->getRobot()->get_pos().theta();
            robotLogPosFile << trial << " " << epoch << " " << x << " " << y << " " << orient << std::endl;
        }
        
    }
    
    void FastSim_MultiRobot_WallAvoidance::relocateRobot()
    {
        double w = world->getMapWidth();
        
        double robotRadius = world->getRobot()->get_radius();
        
        double x = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        double y = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        double orient = drand48() * M_2_PI;
        world->getRobot()->reinit();
        world->moveRobot(x, y, orient);
    }
    
    double FastSim_MultiRobot_WallAvoidance::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
        testIndividual = _testIndividual;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);

        installGenotype(individual);

        
        if (logRobotPos) {
            robotLogPosFile.open("robotPositions.log", std::ios_base::trunc);
        }
        
        if (sensorLog)
            sensorLogFile.open("sensors.log", std::ios_base::trunc);

        double gfitness = 0.0;
        
        for (unsigned trial = 0; trial < trialCount; ++trial) {
            controller->reset();
            relocateRobot();

            double lfitness = 0.0;
            bool crashed = false;
            double irmax;
            double V, deltaV;
            double left, right;
            if (testIndividual) {
                std::cout << "Trial " << trial << " :" << std::endl;
            }
            for (unsigned epoch = 0; epoch < epochCount && !crashed; ++epoch) {
                world->getLaserSensors(laserSensors);

                for (unsigned i=0; i<laserSensors.size(); ++i) {
                    nninputs[i] = 1.0-laserSensors[i];
                }
//                std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin());
                
                if (testIndividual) {
                    std::cout << "Inputs = ";
                    for (unsigned i=0; i<nbinputs; ++i)
                        std::cout << nninputs[i] << " ";
                    std::cout << std::endl;
                }

                controller->run(nninputs, nnoutput);
                left = timestep*(nnoutput[0]*2*maxSpeed-maxSpeed);
                right = timestep*(nnoutput[1]*2*maxSpeed-maxSpeed);
                world->updateRobot(left, right);
                world->step();

                logRobotPosition(trial, epoch);
                if (sensorLog) {
                    sensorLogFile << trial << " " << epoch << " ";
                    for (unsigned i=0; i<nninputs.size(); ++i)
                        sensorLogFile << nninputs[i] << " ";
                    sensorLogFile << nnoutput[0] << " " << nnoutput[1] << std::endl;
                }
                
                crashed = world->getRobot()->get_collision();
                irmax = nninputs[0];
                for (unsigned i=1; i<nninputs.size(); ++i) {
                    if (irmax < nninputs[i])
                        irmax = nninputs[i];
                }
                V = (left+right)/(maxSpeed*2);
                deltaV = std::abs(left-right)/(maxSpeed*2);
                lfitness += V*(1-sqrt(deltaV))*(1-irmax);

                if (testIndividual) {
                    std::cout << "  V = " << V << " ; deltaV = " << deltaV << " ; irmax = " << irmax << " ; crashed = " << crashed << " ; fitness = " <<  V*(1-sqrt(deltaV))*(1-irmax) << std::endl;
                }
                
            }
            if (testIndividual) {
                std::cout << "    Fitness for the trial = " << lfitness / (1.0*epochCount) << std::endl;
            }
            gfitness += lfitness / (1.0*epochCount);
            
        }            
        
        gfitness /= trialCount;

        if (testIndividual) {
            std::cout << "Fitness for the individual = " << gfitness << std::endl;
        }
        
        if (logRobotPos) {
            robotLogPosFile.close();
        }
        if (sensorLog)
            sensorLogFile.close();
        
        return gfitness;
    }

    
}
