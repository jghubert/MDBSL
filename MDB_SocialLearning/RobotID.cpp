/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RobotID.cpp
 * Author: Julien Hubert
 * 
 * Created on November 24, 2016, 11:41 AM
 */

#include "RobotID.h"
#include "Settings.h"
#include "ResourceLibrary.hpp"

namespace MDB_Social {
    __thread RobotID* RobotID::instance = NULL;

    
    RobotID::RobotID() 
    {
        robotid = "Default";
        settings = SettingsLibrary::getInstance(robotid);
        resourceLibrary = ResourceLibrary::getInstance(robotid);
    }

    RobotID::RobotID(std::string id):
        robotid(id)
    {
        settings = SettingsLibrary::getInstance(robotid);
        resourceLibrary = ResourceLibrary::getInstance(robotid);
    }
    
    RobotID::~RobotID() 
    {
        
    }

    void RobotID::setID(std::string _id)
    {
        if (!instance) {
            instance = new RobotID(_id);
        }
        else {
            instance->robotid = _id;
            instance->settings = SettingsLibrary::getInstance(instance->robotid);
            instance->resourceLibrary = ResourceLibrary::getInstance(instance->robotid);
            
        }
    }
    
    std::string RobotID::getID()
    {
        if (instance)
            return instance->robotid;
        else
            return std::string();
    }

    Settings* RobotID::getSettings()
    {
        if (instance)
            return instance->settings;
        else
            return NULL;
    }
    
    ResourceLibraryData* RobotID::getResourceLibrary()
    {
        if (instance)
            return instance->resourceLibrary;
        else
            return NULL;
    }

}