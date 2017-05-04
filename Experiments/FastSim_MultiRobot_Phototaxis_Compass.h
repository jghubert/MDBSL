/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_MultiRobot_Phototaxis_Compass.h
 * Author: Julien Hubert
 *
 * Created on August 22, 2016, 5:21 PM
 */

#ifndef FASTSIM_MULTIROBOT_PHOTOTAXIS_COMPASS_H
#define FASTSIM_MULTIROBOT_PHOTOTAXIS_COMPASS_H

#include <fstream>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
#include "../MDB_SocialLearning/BabblingStandard.h"
#ifdef USE_REV
class REV;
class REVInit;
#endif

namespace MDB_Social {

    class FastSim_MultiRobot_Phototaxis_Compass: public GAFitness 
    {
    private:
        typedef struct {
            double orientation;
            double distance;
        } compass_info_t;
        
        
        // Tons of boost::shared_ptr because FastSim only understands this...
        FastSimSimulator* world;
//        boost::shared_ptr<fastsim::IlluminatedSwitch> light;
        FeedforwardNN* controller;
        BabblingStandard* babbling;
#ifdef USE_REV
        REV* rev;
        REVInit* revinit;
#endif        

        
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
        bool showFastSimViewer;
        bool showREV;
        bool realtime;
        unsigned framerate;
        bool printInputsOutputs;

        bool compassTest;
        bool valueFunctionTest;
        bool fitnessComparisonTest;
        
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
        double computeDistance();
        
        void logRobotPosition(unsigned trial, unsigned epoch);
        
        void testValueFunction();
        
        compass_info_t computeCompass();
        void testCompass();
        
    public:
        FastSim_MultiRobot_Phototaxis_Compass(std::string id="Default");
        FastSim_MultiRobot_Phototaxis_Compass(const FastSim_MultiRobot_Phototaxis_Compass& orig);
        virtual ~FastSim_MultiRobot_Phototaxis_Compass();

        double evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual=false) override;

        void loadParameters() override;

        void preprocessing() override;    // Called before testing the current generation
        void postprocessing() override;   // Called after testing the current generation
        
    };

}

#endif /* FASTSIM_PHOTOTAXIS_H */

