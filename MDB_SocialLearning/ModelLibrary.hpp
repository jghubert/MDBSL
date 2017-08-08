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

#ifdef USE_GAUSSIAN_PROCESS
#include "GaussianProcess.h"
#endif

//#define USE_FANN
#ifdef USE_FANN
#include "FeedforwardNN.h"
#endif


namespace MDB_Social {
    
    class ModelLibrary {
    private:
        static Model* getFeedforward() {
#ifdef USE_FANN
            return new FeedforwardNN();
#else
            std::cerr << "Model Library Error: getFeedforward has no model to propose. Compile with the FANN option to use the FANN library." << std::endl;
            return NULL;
#endif            
        }
        
        static Model* getGaussianProcess() {
#ifdef USE_GAUSSIAN_PROCESS
            return new GaussianProcess();
#else
            std::cerr << "Model Library Error: getGaussianProcess has no model to propose. Compile with the GAUSSIAN_PROCESS option to use the libgp library." << std::endl;
            return NULL;
#endif
        }
        
        static Model* getBabblingStandard() {
            return new BabblingStandard();
        }
        
    public:
        static Model* getModel(std::string type) {
            if (type == "Feedforward") {
                return getFeedforward();
            }
            else if (type == "BabblingStandard") {
                return getBabblingStandard();
            }
            else if (type == "GaussianProcess") {
                return getGaussianProcess();
            }
            else
                return NULL;
        }
        
    };
    
}



#endif /* MODELLIBRARY_HPP */

