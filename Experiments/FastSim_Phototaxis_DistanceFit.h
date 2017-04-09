/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Phototaxis_DistanceFit.h
 * Author: Julien Hubert
 *
 * Created on August 22, 2016, 5:21 PM
 */

#ifndef FASTSIM_PHOTOTAXIS_DISTANCE_FIT_H
#define FASTSIM_PHOTOTAXIS_DISTANCE_FIT_H

#include <fstream>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
//#include "../MDB_SocialLearning/BabblingStandard.h"
#ifdef USE_REV
class REV;
class REVInit;
#endif

namespace MDB_Social {

    class FastSim_Phototaxis_DistanceFit: public GAFitness 
    {
    private:
        // Tons of boost::shared_ptr because FastSim only understands this...
        FastSimSimulator* world;
//        boost::shared_ptr<fastsim::IlluminatedSwitch> light;
        FeedforwardNN* controller;
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
        double maxSpeed;

        unsigned trialCount;
        unsigned epochCount;
        double timestep;
        bool showREV;
        bool realtime;
        unsigned framerate;
        
        bool testIndividual;
        
        bool logRobotPos;
        std::ofstream robotLogPosFile;
        bool sensorLog;
        std::ofstream sensorLogFile;
        
        void registerParameters();
        
        void installGenotype(Genotype& individual);
        
        void relocateRobot();

        double computeDistance();
        
        void logRobotPosition(unsigned trial, unsigned epoch);
        
    public:
        FastSim_Phototaxis_DistanceFit(std::string id="Default");
        FastSim_Phototaxis_DistanceFit(const FastSim_Phototaxis_DistanceFit& orig);
        virtual ~FastSim_Phototaxis_DistanceFit();

        double evaluateFitness(Genotype& individual) override;

        void loadParameters() override;
        
    };

}

#endif /* FASTSIM_PHOTOTAXIS_H */

