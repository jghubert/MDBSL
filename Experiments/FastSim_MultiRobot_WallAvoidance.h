/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_MultiRobot_WallAvoidance.h
 * Author: Julien Hubert
 *
 * Created on February 20, 2017, 5:30 PM
 */

#ifndef FASTSIM_MULTIROBOT_WALLAVOIDANCE_H
#define FASTSIM_MULTIROBOT_WALLAVOIDANCE_H

#include <fstream>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
#include "../MDB_SocialLearning/BabblingStandard.h"

namespace MDB_Social {

    class FastSim_MultiRobot_WallAvoidance: public GAFitness 
    {
    private:
        FastSimSimulator* world;
        FeedforwardNN* controller;
        unsigned nbinputs;
        unsigned nboutputs;
        unsigned hiddenNeurons;
        double controllerMinimumWeight;
        double controllerMaximumWeight;
        std::vector<unsigned> layers;
        bool useOnlyBabbling;
        bool useOnlyTrueReward;
        
        unsigned trialCount;
        unsigned epochCount;
        double timestep;
        
        double maxSpeed;
        
        bool testIndividual;
                
        bool logRobotPos;
        std::ofstream robotLogPosFile;
        bool sensorLog;
        std::ofstream sensorLogFile;
        
        void registerParameters();
        
        void installGenotype(Genotype& individual);
        
        void relocateRobot();
        
        void logRobotPosition(unsigned trial, unsigned epoch);
        
    public:
        FastSim_MultiRobot_WallAvoidance(std::string id="Default");
        virtual ~FastSim_MultiRobot_WallAvoidance();

        double evaluateFitness(Genotype& individual) override;

        void loadParameters() override;
        
        void preprocessing() override;    // Called before testing the current generation
        void postprocessing() override;   // Called after testing the current generation
        

    };

}
#endif /* FASTSIM_MULTIROBOT_WALLAVOIDANCE_H */

