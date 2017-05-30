/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SocialManagerClient.h
 * Author: julienhubert
 *
 * Created on February 23, 2017, 3:27 PM
 */

#ifndef SOCIALMANAGERCLIENT_H
#define SOCIALMANAGERCLIENT_H

#include <vector>
#include <string>

namespace MDB_Social {

    class SocialManager;
    class TraceMemory;
    class ValueFunctionMemory;
    class PolicyMemory;

    class SocialManagerClient {
    public:        
        enum connection_t {
            LOCAL=0,
            IP,
            NONE
        };

    private:
        connection_t connectionType;
        SocialManager* sm;
        std::string ip;
        unsigned port;

    public:
        SocialManagerClient();
        SocialManagerClient(const SocialManagerClient& orig);
        virtual ~SocialManagerClient();

        void setupLocalConnection(SocialManager* smPtr);
        void setupIPConnection(const char* _IP, unsigned _port);
        
        std::vector<std::string> getRobotIDs() const;
        std::string getRandomRobotID(std::string excludeID="") const;
        TraceMemory* getTraceMemory(std::string robotid);
        PolicyMemory* getPolicyMemory(std::string robotid);
        ValueFunctionMemory* getValueFunctionMemory(std::string robotid);
        
        void synchronise();
    };

}

#endif /* SOCIALMANAGERCLIENT_H */

