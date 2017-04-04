/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MemoryLibrary.hpp
 * Author: jhubert
 *
 * Created on August 3, 2016, 1:33 PM
 */

#ifndef MEMORYLIBRARY_HPP
#define MEMORYLIBRARY_HPP

namespace MDB_Social {
    
    class MemoryLibrary {
    private:
        static Model* getValueFunctionMemory() {
#ifdef USE_FANN
            return new FeedforwardNN();
#else
            std::cerr << "Model Library Error: getFeedforward has no model to propose. Compile with the FANN option to use the FANN library." << std::endl;
            return NULL;
#endif            
        }
        
    public:
        static Model* getModel(std::string type) {
            if (type == "ValueFunction") {
                return getFeedforward();
            }
            else
                return NULL;
        }
        
    };
}

#endif /* MEMORYLIBRARY_HPP */

