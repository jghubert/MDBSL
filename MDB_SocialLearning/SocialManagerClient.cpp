/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SocialManagerClient.cpp
 * Author: julienhubert
 * 
 * Created on February 23, 2017, 3:27 PM
 */

#include "SocialManagerClient.h"
#include "SocialManager.h"
#include "ValueFunctionMemory.hpp"
#include "PolicyMemory.hpp"
#include "TraceMemory.h"
#include <iostream>
#include <cstdlib>

namespace MDB_Social {

    SocialManagerClient::SocialManagerClient() 
    {
        connectionType = connection_t::NONE;
        sm = NULL;
        ip = "127.0.0.1";
        port = 0;
        syncbarrier = NULL;
    }

    SocialManagerClient::SocialManagerClient(const SocialManagerClient& orig) {
    }

    SocialManagerClient::~SocialManagerClient() {
    }
    
    void SocialManagerClient::setupLocalConnection(SocialManager* smPtr)
    {
        connectionType = connection_t::LOCAL;
        sm = smPtr;
        syncbarrier = smPtr->getBarrier();
    }
    
    void SocialManagerClient::setupIPConnection(const char* _IP, unsigned _port)
    {
        connectionType = connection_t::IP;
        ip = _IP;
        port = _port;
    }

    std::vector<std::string> SocialManagerClient::getRobotIDs() const
    {
        switch(connectionType) {
            case LOCAL:
                return sm->getRobotIDs();
                break;
            case IP:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not yet implemented." << std::endl;
                exit(1);
                break;
            case NONE:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not defined." << std::endl;
                exit(1);
                break;
            default:
                break;
        }
        return std::vector<std::string>();
    }
    
    TraceMemory* SocialManagerClient::getTraceMemory(std::string robotid)
    {
        switch(connectionType) {
            case LOCAL:
                return sm->getTraceMemory(robotid);
                break;
            case IP:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not yet implemented." << std::endl;
                exit(1);
                break;
            case NONE:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not defined." << std::endl;
                exit(1);
                break;
            default:
                break;
        }
        return NULL;
    }
    
    PolicyMemory* SocialManagerClient::getPolicyMemory(std::string robotid)
    {
        switch(connectionType) {
            case LOCAL:
                return sm->getPolicyMemory(robotid);
                break;
            case IP:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not yet implemented." << std::endl;
                exit(1);
                break;
            case NONE:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not defined." << std::endl;
                exit(1);
                break;
            default:
                break;
        }
        return NULL;
    }
    
    ValueFunctionMemory* SocialManagerClient::getValueFunctionMemory(std::string robotid)
    {
        switch(connectionType) {
            case LOCAL:
                return sm->getValueFunctionMemory(robotid);
                break;
            case IP:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not yet implemented." << std::endl;
                exit(1);
                break;
            case NONE:
                std::cerr << "SocialManagerClient: Connection over IP to the social manager not defined." << std::endl;
                exit(1);
                break;
            default:
                break;
        }
        return NULL;
    }

    void SocialManagerClient::synchronise()
    {
        if (syncbarrier) {
            syncbarrier->wait();
        }
    }
    
    
    std::string SocialManagerClient::getRandomRobotID(std::string excludeID) const
    {
        std::vector<std::string> robotIds = this->getRobotIDs();
        // We choose one random robot
        unsigned other_robot = lrand48() % robotIds.size();
        if (!excludeID.empty()) {
            while (robotIds[other_robot] == excludeID)
                other_robot = lrand48() % robotIds.size();
        }
        return robotIds[other_robot];
    }
    
    
}
