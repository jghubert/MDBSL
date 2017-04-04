/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Model.hpp
 * Author: Julien Hubert
 *
 * Created on June 16, 2016, 2:49 PM
 */

#ifndef MODEL_HPP
#define MODEL_HPP

#include <string>
#include <vector>

#include "Memory.hpp"
#include "RobotID.h"

namespace MDB_Social {
    class Model: public RobotID {
    protected:
        std::string type;
        
        
    public:
        Model(std::string _type, std::string id="Default"):
        RobotID(id)
        {
            type = _type;
        }
        
        virtual ~Model() {}
        
        std::string getType() const {
            return type;
        }
        
        virtual void train() = 0;
        virtual void reset() = 0;
        virtual void publish() = 0;
        virtual void registerParameters(std::string prefix=std::string()) = 0;
        virtual void loadParameters(std::string prefix=std::string()) = 0;
        virtual void initializeFromParameters() = 0;
        
        virtual void setExternalMemory(Memory<NeuralNetworkData>* mem) {}
        virtual bool loadFromExternalMemory() {return false;}
        virtual bool saveModelToFile(const char* filename) {return false;}
    };
    
    
}

#endif /* MODEL_HPP */

