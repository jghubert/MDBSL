/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Simulator.hpp
 * Author: Julien Hubert
 *
 * Created on August 14, 2016, 11:31 AM
 */

#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "RobotID.h"
#include "ResourceLibrary.hpp"
#include "TraceMemory.h"

namespace MDB_Social {

    class Simulator: public RobotID
    {
    private:
        
        virtual void loadParameters() = 0;
    protected:
        TraceMemory* traceMemory;
    public:
        Simulator() {
        }

//        virtual ~Simulator();

        virtual void setID(std::string& _id) override {
            RobotID::setID(_id);
            traceMemory = resourceLibrary->getTraceMemory();
        }
        
        virtual void step() = 0;
        
        virtual void registerParameters() = 0;
        
        virtual void initialize() = 0;
    };
}

#endif /* SIMULATOR_HPP */

