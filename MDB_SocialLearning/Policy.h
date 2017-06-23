/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Policy.h
 * Author: Julien Hubert
 *
 * Created on August 5, 2016, 3:35 PM
 */

#ifndef POLICY_H
#define POLICY_H

#include "RobotID.h"
#include "GeneticAlgorithm.hpp"
#include "BabblingPolicy.h"
#include <string>

namespace MDB_Social {
    class ResourceLibraryData;
    class Settings;

    class Policy {
    private:
        std::string geneticAlgorithmType;
        Settings* settings;
        ResourceLibraryData* resourceLibrary;

        GeneticAlgorithm* ga;
        BabblingPolicy* babbling;
        
        bool recommendBabbling;
        
        void loadParameters();
        
    public:
        Policy();
        virtual ~Policy();

        void registerParameters();
        
        bool initialise();

        static void callbackParameters(void*);
        
        bool run();
        
        void socialPreProcessing();
        void socialPostProcessing();
        
        void publish();

        GAFitness* getFitnessFunction();

        void setBabblingRecommendation(bool b);
        
//        virtual void setID(std::string& _id) override;

        
    };

}
#endif /* POLICY_H */

