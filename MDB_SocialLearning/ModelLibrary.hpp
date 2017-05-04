/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ModelLibrary.hpp
 * Author: Julien Hubert
 *
 * Created on June 16, 2016, 3:32 PM
 */

#ifndef MODELLIBRARY_HPP
#define MODELLIBRARY_HPP

#include <string>
#include <iostream>
#include "Model.hpp"
#include "BabblingStandard.h"

//#define USE_FANN
#ifdef USE_FANN
#include "FeedforwardNN.h"
#endif


namespace MDB_Social {
    
    class ModelLibrary {
    private:
        static Model* getFeedforward(std::string id) {
#ifdef USE_FANN
            return new FeedforwardNN(id);
#else
            std::cerr << "Model Library Error: getFeedforward has no model to propose. Compile with the FANN option to use the FANN library." << std::endl;
            return NULL;
#endif            
        }
        
        static Model* getBabblingStandard(std::string id) {
            return new BabblingStandard(id);
        }
        
    public:
        static Model* getModel(std::string type, std::string id="Default") {
            if (type == "Feedforward") {
                return getFeedforward(id);
            }
            else if (type == "BabblingStandard") {
                return getBabblingStandard(id);
            }
            else
                return NULL;
        }
        
    };
    
}



#endif /* MODELLIBRARY_HPP */

