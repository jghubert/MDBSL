/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Phototaxis_DistanceFit.cpp
 * Author: Julien Hubert
 * 
 * Created on August 22, 2016, 5:21 PM
 */

#include "../MDB_SocialLearning/Settings.h"
#include "FastSim_Phototaxis_DistanceFit.h"
#include <cstdlib>
#include <algorithm>
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/ValueFunction.h"

namespace MDB_Social {

    FastSim_Phototaxis_DistanceFit::FastSim_Phototaxis_DistanceFit() 
    {
        registerParameters();
        
        world = static_cast<FastSimSimulator*>(this->getSimulator("FastSim"));
        world->registerParameters();

        
        nbinputs = 1;
        nboutputs = 2;
        hiddenNeurons = 5;
        
        controllerMinimumWeight = -10.0;
        controllerMaximumWeight = 10.0;
                
        logRobotPos = false;
        sensorLog = false;
//        robot->add_light_sensor(fastsim::LightSensor(1,0.0f,100.0f));
        
//        light = boost::shared_ptr<fastsim::IlluminatedSwitch>(new fastsim::IlluminatedSwitch(1, 5.0f, 300.0f, 300.0f, true));
//        world->getMap()->add_illuminated_switch(light);
    }

    FastSim_Phototaxis_DistanceFit::FastSim_Phototaxis_DistanceFit(const FastSim_Phototaxis_DistanceFit& orig) 
    {
        
    }

    FastSim_Phototaxis_DistanceFit::~FastSim_Phototaxis_DistanceFit() 
    {
        delete world;
        delete controller;
    }

    void FastSim_Phototaxis_DistanceFit::registerParameters()
    {
        std::cout << "FastSim_Phototaxis_DistanceFit : registering the parameters...";
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
        settings->registerParameter<bool>("experiment.logRobotPosition", false, "Output the position of the robot in a log file.");
        settings->registerParameter<bool>("experiment.sensorLogFlag", false, "Log the sensors values during the experiment.");
        settings->registerParameter<double>("experiment.maxSpeed", 5, "Maximum speed of the robot.");
        std::cout << " DONE" << std::endl;
    }
    
    void FastSim_Phototaxis_DistanceFit::loadParameters()
    {
        std::cout << "FastSim_Phototaxis_DistanceFit: Loading parameters..." << std::endl;
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
            logRobotPos = settings->value<bool>("experiment.logRobotPosition").second;
            sensorLog = settings->value<bool>("experiment.sensorLogFlag").second;
            maxSpeed = settings->value<double>("experiment.maxSpeed").second;
            testIndividual = settings->value<bool>("General.testIndividual").second;
            world->initialize();
            std::cout << "FastSim_Phototaxis_DistanceFit: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "FastSim_Phototaxis_DistanceFit: Error loading the parameters: " << e.what() << std::endl;
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

    
    void FastSim_Phototaxis_DistanceFit::installGenotype(Genotype& individual)
    {
        std::vector<FeedforwardNN::weight_t> weights(individual.getSize());
        
        controller->getWeights(weights);
        if (weights.size() != individual.getSize()) {
            std::cerr << "FastSim_Phototaxis_DistanceFit: Size of the genotype (size = " << individual.getSize() << ") differs from the size of the network (size = " << weights.size() << ")." << std::endl;
            exit(1);
        }
        
        for (unsigned i=0; i<individual.getSize(); ++i) {
            weights[i].weight =  (controllerMaximumWeight - controllerMinimumWeight)*individual[i] + controllerMinimumWeight;
        }
        
        controller->setWeights(weights);
    }

    void FastSim_Phototaxis_DistanceFit::logRobotPosition(unsigned trial, unsigned epoch)
    {
        if (logRobotPos) {
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double orient = world->getRobot()->get_pos().theta();
            robotLogPosFile << trial << " " << epoch << " " << x << " " << y << " " << orient << std::endl;
        }
    }
    
    void FastSim_Phototaxis_DistanceFit::relocateRobot()
    {
        double w = world->getMapWidth();
        
        double robotRadius = world->getRobot()->get_radius();
        
        double x = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        double y = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        double orient = drand48() * M_2_PI;
        world->getRobot()->reinit();
        world->moveRobot(x, y, orient);
    }

    double FastSim_Phototaxis_DistanceFit::computeDistance()
    {
        // Compute the distance between the goal and the robot, and compute the reward accordingly.
        double x = world->getRobot()->get_pos().get_x();
        double y = world->getRobot()->get_pos().get_y();
        
        fastsim::Map::ill_sw_t ilswitch = world->getMap()->get_illuminated_switch_by_color(1);
        double ilx = ilswitch->get_x();
        double ily = ilswitch->get_y();
        double dist = sqrt(pow(x-ilx, 2.0)+ pow(y-ily, 2.0));
        
//        std::cout << "Phototaxis : dist = " << dist << std::endl;
        
        return dist;
        
    }
    
    double FastSim_Phototaxis_DistanceFit::evaluateFitness(Genotype& individual)
    {
        // In this experiment we evolve and test a policy. The policy uses the value function and the current state to decide what the next action should be.
        // This part only cares about the policy.
        // A reward will be given when the robot reaches a given distance from the light source.
        
        std::vector<double> lightSensors;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        double dist;
        
        if (logRobotPos) {
            robotLogPosFile.open("robotPositions.log", std::ios_base::trunc);
        }
        
        if (sensorLog)
            sensorLogFile.open("sensors.log", std::ios_base::trunc);

        installGenotype(individual);
//        controller->save("FannNetwork.log"); 
       
        double fitness = 0.0;
        for (unsigned trial = 0; trial < trialCount; ++trial) {
//            controller->reset();
            relocateRobot();
            
            double closest = computeDistance();
            double fartest = closest;
            
            unsigned epoch;
            bool collision = false;
            for (epoch = 0; epoch < epochCount && !collision; ++epoch) {
                world->getLightSensors(lightSensors);
                world->getLaserSensors(laserSensors);

                std::copy(lightSensors.begin(), lightSensors.end(), nninputs.begin());
                std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin()+lightSensors.size());
                nninputs[nbinputs-1] = 1.0;   // bias
                if (testIndividual) {
                    std::cout << "Inputs = ";
                    for (unsigned i=0; i<nbinputs; ++i)
                        std::cout << nninputs[i] << " ";
                    std::cout << std::endl;
                }
                
                // ACTIVATE ME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                nnoutput = controller->run(nninputs);
                
                // REMOVE ME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                nnoutput[0] = -maxSpeed*timestep;
//                nnoutput[1] = maxSpeed*timestep;
                
//                std::cout << "Speed : " << timestep*(nnoutput[0]*2*maxSpeed-maxSpeed) << " - " << timestep*(nnoutput[1]*2*maxSpeed-maxSpeed) << std::endl;
                world->updateRobot(timestep*(nnoutput[0]*2*maxSpeed-maxSpeed), timestep*(nnoutput[1]*2*maxSpeed-maxSpeed));
                world->step();

                logRobotPosition(trial, epoch);
                if (sensorLog) {
                    sensorLogFile << trial << " " << epoch << " ";
                    for (unsigned i=0; i<nninputs.size(); ++i)
                        sensorLogFile << nninputs[i] << " ";
                    sensorLogFile << nnoutput[0] << " " << nnoutput[1] << std::endl;
                }

                dist = computeDistance();
                
                if (dist > fartest) {
                    closest = dist;
                    fartest = dist;
                }
                else if (dist < closest)
                    closest = dist;
                                
                collision = world->getRobot()->get_collision();
                
                // need to compute the fitness now. Normally it should be the VF, but we don't have VF now...
            }
            
            fitness += (fartest-closest)/fartest + (1.0*epoch) / epochCount;
        }

//        std::cout << "Phototaxis: rewardTotal = " << rewardTotal << " ; fitness = " << (rewardTotal*1.0)/(1.0*trialCount*epochCount) << std::endl;

        if (logRobotPos) {
            robotLogPosFile.close();
        }
        if (sensorLog)
            sensorLogFile.close();
        
        return fitness / trialCount;
    }

}
