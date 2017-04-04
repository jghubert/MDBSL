/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SocialManager.h
 * Author: Julien Hubert
 *
 * Created on November 10, 2016, 9:22 PM
 */

#ifndef SOCIALMANAGER_H
#define SOCIALMANAGER_H

#include "Manager.h"
#include <vector>
#include <unordered_map>
#include <string>
#include <boost/thread/barrier.hpp>
#include <boost/thread/thread.hpp>

namespace MDB_Social {

    class TraceMemory;
    class ValueFunctionMemory;
    class PolicyMemory;

    class SocialManager {
    private:
        unsigned nbrobots;
        std::vector<std::string> robotIDs;
        std::vector<std::string> robotConfigFiles;
//        std::vector<boost::thread*> robotThreads;
        boost::thread_group threadGroup;
        
        std::unordered_map<std::string, Manager*> robots;
        
        boost::barrier* syncbarrier; 

    public:
        SocialManager(const char* settingfile, int argc, char* argv[]);
        SocialManager(const SocialManager& orig);
        virtual ~SocialManager();

        void registerParameters();
        void loadParameters();
        
        void runExperiment();
        void initializeRobots();
        
        void synchronise();   // called by threads to synchronise

        std::vector<std::string> getRobotIDs() const;
        TraceMemory* getTraceMemory(std::string robotid);
        PolicyMemory* getPolicyMemory(std::string robotid);
        ValueFunctionMemory* getValueFunctionMemory(std::string robotid);
        
    };

}

#endif /* SOCIALMANAGER_H */

