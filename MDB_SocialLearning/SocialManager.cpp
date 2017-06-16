/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SocialManager.cpp
 * Author: Julien Hubert
 * 
 * Created on November 10, 2016, 9:22 PM
 */

#include "SocialManager.h"
#include "Settings.h"
#include "ValueFunctionMemory.hpp"
#include "PolicyMemory.hpp"
#include "TraceMemory.h"
#include "colormod.hpp"

namespace MDB_Social {

    SocialManager::SocialManager(const char* settingfile, int argc, char* argv[]) 
    {
        syncbarrier = NULL;
        
        Settings* settings = SettingsLibrary::getInstance("SocialManager");
        settings->setParameterSources(settingfile, argc, argv);

        this->registerParameters();
        
        // Let's load the settings from the file and the arguments
        if (!settings->processAllParameters(settingfile, argc, argv)) {
            std::cerr << "Manager:: Error loading the parameters." << std::endl;
            exit(1);
        }

        loadParameters();
        
    }

    SocialManager::SocialManager(const SocialManager& orig) 
    {
        
    }

    SocialManager::~SocialManager() 
    {
        if (syncbarrier)
            delete syncbarrier;
    }

    void SocialManager::registerParameters()
    {
        Settings* settings = SettingsLibrary::getInstance("SocialManager");
        if (!(settings->registerAndRetrieveParameter<unsigned>(nbrobots, "SocialManager.nbRobots", 0, "Number of robots to manage"))) {
            std::cerr << "SocialManager: Could not find SocialManager.nbRobots in the configuration file." << std::endl;
            exit(1);
        }
        
        syncbarrier = new boost::barrier(nbrobots);
        
        std::string rids;
        if (settings->registerAndRetrieveParameter<std::string>(rids, "SocialManager.robotIDs", "Default", "IDs of the robots separated by ;")) {
            std::cout << "SocialManager - robots to load: " << rids << std::endl;

            robotIDs.reserve(nbrobots);
//            robotThreads.resize(nbrobots, NULL);
            std::string tmp;
            for (size_t i=0; i<rids.size(); ++i) {
                if (rids[i] == ';') {
                    robotIDs.push_back(tmp);
                    tmp.clear();
                }
                else {
                    tmp.push_back(rids[i]);
                }
            }
            if (!tmp.empty())
                robotIDs.push_back(tmp);
            
            for (size_t i=0; i<robotIDs.size(); ++i) {
                settings->registerParameter<std::string>((robotIDs[i] + ".configFile").c_str(), robotIDs[i] + ".cfg", ("Config file of robot " + robotIDs[i]).c_str());
            }
        }
    }
    
    void SocialManager::loadParameters()
    {
        Settings* settings = SettingsLibrary::getInstance("SocialManager");
//        nbrobots = settings->value<unsigned>("SocialManager.nbRobots").second;
//        std::string rids = settings->value<std::string>("SocialManager.robotIDs").second;
        // parse the inputs

        robotConfigFiles.reserve(nbrobots);
        for (size_t i=0; i<nbrobots; ++i) {
            robotConfigFiles.push_back(settings->value<std::string>((robotIDs[i] + ".configFile").c_str()).second);
        }
    }

    void SocialManager::initializeRobots()
    {
        std::cout << Color::Modifier(Color::Code::FG_GREEN) << "Social Manager: Initialization of the robots... " << Color::Modifier(Color::Code::FG_DEFAULT) << std::endl;
        std::cout.flush();
        if (robots.size() > 0) {
            for (auto it = robots.begin(); it != robots.end(); ++it)
                delete it->second;
            robots.clear();
        }
        
        Manager *mg;
        for (unsigned i=0; i<nbrobots; ++i) {
            std::cout << Color::Modifier(Color::Code::FG_GREEN) << "             Loading robot " << robotIDs[i] << Color::Modifier(Color::Code::FG_DEFAULT) << std::endl;
            mg = new Manager(robotConfigFiles[i].c_str(), 0, NULL, robotIDs[i]);
            robots[robotIDs[i]] = mg;
            mg->initializeLocalConnection(this);
            mg->setSocialMode(true);
            
        }
        std::cout << Color::Modifier(Color::Code::FG_GREEN) << "Finished loading the robots" << Color::Modifier(Color::Code::FG_DEFAULT) << std::endl;
    }

    void SocialManager::runExperiment()
    {
        std::cout << "Social Manager: Running the experiments on " << nbrobots << " robots." << std::endl;
        for (auto it = robots.begin(); it != robots.end(); ++it) {
            threadGroup.create_thread(boost::ref(*(it->second)));
//            robotThreads[i] = new boost::thread(boost::ref((it->second)->runExperiment));
        }
        threadGroup.join_all();
    }

    std::vector<std::string> SocialManager::getRobotIDs() const
    {
        return robotIDs;
    }


    TraceMemory* SocialManager::getTraceMemory(std::string robotid)
    {
        auto it = robots.find(robotid);
        if (it != robots.end()) {
            return it->second->getTraceMemory();
        }
        else
            return NULL;
    }
    
    PolicyMemory* SocialManager::getPolicyMemory(std::string robotid)
    {
        auto it = robots.find(robotid);
        if (it != robots.end()) {
            return it->second->getPolicyMemory();
        }
        else
            return NULL;
        
    }
    
    ValueFunctionMemory* SocialManager::getValueFunctionMemory(std::string robotid)
    {
        auto it = robots.find(robotid);
        if (it != robots.end()) {
            return it->second->getVFMemory();
        }
        else
            return NULL;
        
    }

    void SocialManager::synchronise()
    {
        syncbarrier->wait();
    }

    boost::barrier* SocialManager::getBarrier() const
    {
        return syncbarrier;
    }
    
}
