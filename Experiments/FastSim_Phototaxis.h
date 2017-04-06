/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Phototaxis.h
 * Author: Julien Hubert
 *
 * Created on August 22, 2016, 5:21 PM
 */

#ifndef FASTSIM_PHOTOTAXIS_H
#define FASTSIM_PHOTOTAXIS_H

#include <fstream>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
#include "../MDB_SocialLearning/BabblingStandard.h"

namespace MDB_Social {

    class FastSim_Phototaxis: public GAFitness 
    {
    private:
        // Tons of boost::shared_ptr because FastSim only understands this...
        FastSimSimulator* world;
//        boost::shared_ptr<fastsim::IlluminatedSwitch> light;
        FeedforwardNN* controller;
        BabblingStandard* babbling;
        
        unsigned nbinputs;
        unsigned nboutputs;
        unsigned hiddenNeurons;
        double controllerMinimumWeight;
        double controllerMaximumWeight;
        std::vector<unsigned> layers;
        bool useOnlyBabbling;
        bool useOnlyTrueReward;
        bool useOnlyRewardedStates;
        bool useRestrictedVFasFitness;
        double thresholdForVFasFitness;
        bool endTrialWhenOnReward;
        unsigned maxTimeOnReward;
        bool useTracesWhenLightVisible;
        bool useSeeTheLightInputs;
        bool showFastSimViewer;
        
        bool valueFunctionTest;
        
        unsigned trialCount;
        unsigned epochCount;
        double timestep;
        
        double maxSpeed;
        
        bool testIndividual;
        
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
        
        void testValueFunction();
        
    public:
        FastSim_Phototaxis(std::string id="Default");
        FastSim_Phototaxis(const FastSim_Phototaxis& orig);
        virtual ~FastSim_Phototaxis();

        double evaluateFitness(Genotype& individual) override;

        void loadParameters() override;
        
    };

}

#endif /* FASTSIM_PHOTOTAXIS_H */

