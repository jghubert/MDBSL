/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Flower_Poison.h
 * Author: Julien Hubert
 *
 * Created on May 21, 2017, 14:10 PM
 */

#ifndef FASTSIM_FLOWER_POISON_H
#define FASTSIM_FLOWER_POISON_H

#include <fstream>
#include <tuple>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
#include "../MDB_SocialLearning/BabblingStandard.h"
#ifdef USE_REV
#include "../RobotExperimentViewer/RobotExperimentViewer/revinit.h"
#include "../RobotExperimentViewer/RobotExperimentViewer/rev.h"
#endif

namespace MDB_Social {

    class FastSim_Flower_Poison: public GAFitness
    {
    private:
        typedef struct {
            // J: do we need double orientation and distance for foraging?
            double orientation;
            double distance;
        } compass_info_t;
        
        class FoodPoisonObject {
        public: 
            // J: do we need double orientation and distance for foraging?
            double x;
            double y;
            double d;
            bool visible;
            unsigned type;
            bool food;
            
            FoodPoisonObject() {
                x = 0.0;
                y = 0.0;
                d = 0.0;
                visible = false;
                type = 0;
                food = true;
            }
            
            FoodPoisonObject(double _x, double _y, double _d, bool _v, unsigned _type, bool _food)
            : x(_x), y(_y), d(_d), visible(_v), type(_type), food(_food)  {}

            bool operator !=(const FoodPoisonObject& a) const {
                return !(std::fabs(this->x - a.x) < 1e-6 && std::fabs(this->y - a.y) < 1e-6);
            }
        };

        class CollisionData {
        public:
            unsigned type;
            bool food;
            unsigned index;
            bool collision;
            
            bool operator !=(const CollisionData& a) const {
                return !(this->type == a.type && this->index == a.index);
            }
        };
        
        // Tons of boost::shared_ptr because FastSim only understands this...
        FastSimSimulator* world;
//        boost::shared_ptr<fastsim::IlluminatedSwitch> light;
        FeedforwardNN* controller;
        BabblingStandard* babbling;
#ifdef USE_REV
        REV* rev;
        REVInit* revinit;
        
        std::vector<unsigned> flowerREVIndexStarts;
        std::vector<unsigned> poisonREVIndexStarts;
        std::vector<REV::Color> flowerREVColors;
        std::vector<REV::Color> poisonREVColors;
#endif        
        typedef std::vector<std::vector<FoodPoisonObject> > ObjectMatrix;
        ObjectMatrix flowerList;
        ObjectMatrix poisonList;
        
        unsigned nbinputs;
        unsigned nboutputs;
        std::vector<unsigned> hiddenNeurons;
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
//        double diameterTarget;
        double diameterFlower;
        double diameterPoison;
        unsigned nbFlowerType;
        std::vector<unsigned> nbFlowerTypes;
//        unsigned nbFlowerPerType;
        unsigned nbPoisonType;
        std::vector<unsigned> nbPoisonTypes;
//        unsigned nbPoisonPerType;
        bool socialLearningControllers;
        bool socialLearningValueFunctions;
        bool socialLearningTraces;
    

        bool compassTest;
        bool valueFunctionTest;
        unsigned valueFunctionTestNbOrientations;
        bool attractionTest;
//        bool fitnessComparisonTest;
        
        unsigned trialCount;
        unsigned epochCount;
        double timestep;
        
        double maxSpeed;
        
        bool testIndividual;
        
//        double rewardZoneDiameter;
        
        bool logRobotPos;
        std::ofstream robotLogPosFile;
        bool sensorLog;
        std::ofstream sensorLogFile;
        bool logSocialLearning;
        std::ofstream socialLearningLogFile;
        
        void registerParameters();
        
        void installGenotype(Genotype& individual);
        
        void relocateRobot();
        
        void relocateFoodSources();
        
        void relocateFlower(unsigned type, unsigned p);
        void relocatePoison(unsigned type, unsigned p);

//        bool computeReward(bool);
        
//        bool checkCollisionTarget(double x, double y, double d);
        
        bool checkCollisionRobot(double x, double y, double d);
        
        CollisionData checkCollisionAllFoodSource(double x, double y, double d);
        
        bool checkCollisionOneFoodSource(double x, double y, double d, FoodPoisonObject& p);
        
        bool checkCollisionOtherFoodSource(double x, double y, double d, FoodPoisonObject& food);
        
        double computeDistanceToFoodSource(FoodPoisonObject& p);
        std::tuple<int, int, double> computeDistanceClosestFlower();
        std::tuple<int, int, double> computeDistanceClosestFlower(int type);
        std::tuple<int, int, double> computeDistanceClosestPoison();
        std::tuple<int, int, double> computeDistanceClosestPoison(int type);
//        double computeDistanceTarget();
        
        void logRobotPosition(unsigned trial, unsigned epoch);
        
        void testValueFunction();

        void runAttractionTest();
        
        //J: change?
//        compass_info_t computeCompassTarget();
        compass_info_t computeCompassClosestFlower(int rtype = -1);
        compass_info_t computeCompassClosestPoison(int rtype = -1);
        
//        void testCompass();
        
    public:
        FastSim_Flower_Poison();
        FastSim_Flower_Poison(const FastSim_Flower_Poison& orig);
        virtual ~FastSim_Flower_Poison();

        double evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual=false) override;

        void loadParameters() override;
        
        virtual void preprocessing() override;
        
    };

}

#endif /* FASTSIM_FLOWER_POISON_H */

