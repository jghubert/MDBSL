/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RobotID.h
 * Author: Julien Hubert
 *
 * Created on November 24, 2016, 11:41 AM
 */

#ifndef ROBOTID_H
#define ROBOTID_H

#include <string>

namespace MDB_Social {

    class Settings;
    class ResourceLibraryData;
    
    class RobotID {
    protected:
         std::string robotid;
        // __thread
        Settings* settings;
        ResourceLibraryData* resourceLibrary;
        
        RobotID();
        RobotID(std::string id);
    public:
        __thread static RobotID* instance;

        ~RobotID();

        static void setID(std::string _id);
        static std::string getID();
        
        static Settings* getSettings();
        static ResourceLibraryData* getResourceLibrary();
    };

}
#endif /* ROBOTID_H */

