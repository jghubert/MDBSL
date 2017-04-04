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

    RobotID::RobotID() 
    {
        robotid = "Default";
        
        settings = NULL;
        resourceLibrary = NULL;
    }

    RobotID::RobotID(std::string id)
    {
        robotid = id;
        
        settings = SettingsLibrary::getInstance(robotid);
        resourceLibrary = ResourceLibrary::getInstance(robotid);
    }
    
    
    RobotID::RobotID(const RobotID& orig) 
    {
        
    }

    RobotID::~RobotID() 
    {
        
    }

    void RobotID::setID(std::string& _id)
    {
        robotid = _id;
        settings = SettingsLibrary::getInstance(robotid);
        resourceLibrary = ResourceLibrary::getInstance(robotid);
    }
    
    std::string RobotID::getID() const
    {
        return robotid;
    }

}