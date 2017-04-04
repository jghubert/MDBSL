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
        
        Settings* settings;
        ResourceLibraryData* resourceLibrary;

    public:
        RobotID();
        RobotID(std::string id);
        RobotID(const RobotID& orig);
        virtual ~RobotID();

        virtual void setID(std::string& _id);
        std::string getID() const;
    };

}
#endif /* ROBOTID_H */

