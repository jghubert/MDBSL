/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GALibrary.hpp
 * Author: Julien Hubert
 *
 * Created on August 4, 2016, 2:56 PM
 */

#ifndef GALIBRARY_HPP
#define GALIBRARY_HPP
#include <iostream>
#include "GeneticAlgorithm.hpp"
#include "SimpleGeneticAlgorithm.h"

//#define USE_ECF
#ifdef USE_ECF
#include "ECF.h"
#endif


namespace MDB_Social {

    class GALibrary {
    private:
        static GeneticAlgorithm* getECF() {
#ifdef USE_ECF
            return new ECF;
#else
            std::cerr << "Genetic Algorithm Library Error: ECF was not compiled in. Compile with the ECF option to use the ECF library." << std::endl;
            return NULL;
#endif            
        }
        
        static GeneticAlgorithm* getSimpleGeneticAlgorithm() {
            return new SimpleGeneticAlgorithm();
        }
        
    public:
        static GeneticAlgorithm* getModel(std::string type) {
            if (type == "ECF") {
                return getECF();
            }
            else if (type == "SimpleGeneticAlgorithm")
                return getSimpleGeneticAlgorithm();
            else
                return NULL;
            
        }
    };
    
}

#endif /* GALIBRARY_HPP */

