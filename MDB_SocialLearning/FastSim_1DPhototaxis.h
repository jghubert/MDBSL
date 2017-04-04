/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_1DPhototaxis.h
 * Author: Julien Hubert
 *
 * Created on October 5, 2016, 10:39 AM
 */

#ifndef FASTSIM_1DPHOTOTAXIS_H
#define FASTSIM_1DPHOTOTAXIS_H

#include <fstream>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
#include "../MDB_SocialLearning/BabblingStandard.h"


namespace MDB_Social {

    class FastSim_1DPhototaxis: public GAFitness 
    {
    private:

        FeedforwardNN* controller;
        BabblingStandard* babbling;
        
        unsigned nbinputs;
        unsigned hiddenNeurons;
        unsigned nboutputs;
        double controllerMinimumWeight;
        double controllerMaximumWeight;
        std::vector<unsigned> layers;
        bool useOnlyBabbling;

        unsigned trialCount;
        unsigned epochCount;
        double timestep;
        
        double rewardZoneDiameter;
        
        bool logRobotPos;
        std::ofstream robotLogPosFile;
        bool sensorLog;
        std::ofstream sensorLogFile;
        
        void registerParameters();
        
        void installGenotype(Genotype& individual);
        
        void relocateRobot();

        bool computeReward();
        
        void logRobotPosition(unsigned trial, unsigned epoch);
        
    public:
        FastSim_1DPhototaxis();
        FastSim_1DPhototaxis(const FastSim_1DPhototaxis& orig);
        virtual ~FastSim_1DPhototaxis();

        double evaluateFitness(Genotype& individual) override;

        void loadParameters() override;
    };
}
#endif /* FASTSIM_1DPHOTOTAXIS_H */

