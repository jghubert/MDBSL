/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Forage_Wall.h
 * Author: Julien Hubert
 *
 * Created on May 16, 2017, 14:10 PM
 */

#ifndef FASTSIM_FORAGE_WALL_H
#define FASTSIM_FORAGE_WALL_H

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

    class FastSim_Forage_Wall: public GAFitness
    {
    private:
        typedef struct {
            // J: do we need double orientation and distance for foraging?
            double orientation;
            double distance;
        } compass_info_t;
        
        typedef struct {
            // J: do we need double orientation and distance for foraging?
            double x;
            double y;
            double d;
            bool visible;
        } puck_location_t;
        
        
        // Tons of boost::shared_ptr because FastSim only understands this...
        FastSimSimulator* world;
//        boost::shared_ptr<fastsim::IlluminatedSwitch> light;
        FeedforwardNN* controller;
        BabblingStandard* babbling;
#ifdef USE_REV
        REV* rev;
        REVInit* revinit;
        
        unsigned puckREVIndexStart;
#endif        

        std::vector<puck_location_t> pucksList;
        
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
        unsigned REVsizeMultiplicator;
        bool realtime;
        unsigned framerate;
        bool printInputsOutputs;
        unsigned numberBalls;
        double diameterTarget;
        double diameterPuck;
    

        bool compassTest;
        bool valueFunctionTest;
//        bool fitnessComparisonTest;
        
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
        
        void relocateBalls();
        
        void relocateBall(unsigned p);

        bool computeReward(bool);
        
        bool checkCollisionTarget(double x, double y, double d);
        
        bool checkCollisionRobot(double x, double y, double d);
        
        unsigned checkCollisionAllPucks(double x, double y, double d);
        
        bool checkCollisionOnePuck(double x, double y, double d, unsigned p);
        
        bool checkCollisionOtherPucks(double x, double y, double d, unsigned p);
        
        double computeDistanceToPuck(unsigned p);
        std::pair<int, double> computeDistanceClosestBall();
        double computeDistanceTarget();
        
        void logRobotPosition(unsigned trial, unsigned epoch);
        
        void testValueFunction();
        
        //J: change?
        compass_info_t computeCompassTarget();
        compass_info_t computeCompassClosestPuck();
        
        void testCompass();
        
    public:
        FastSim_Forage_Wall(std::string id="Default");
        FastSim_Forage_Wall(const FastSim_Forage_Wall& orig);
        virtual ~FastSim_Forage_Wall();

        double evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual=false) override;

        void loadParameters() override;
        
    };

}

#endif /* FASTSIM_FORAGE_WALL_H */

