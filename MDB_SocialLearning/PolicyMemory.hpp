/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   policy_memory.hpp
 * Author: Julien Hubert
 *
 * Created on June 14, 2016, 4:56 PM
 */

#ifndef POLICY_MEMORY_HPP
#define POLICY_MEMORY_HPP

#include "GeneticAlgorithm.hpp"
#include "Memory.hpp"

namespace MDB_Social {
//    template <class dtype>
    class PolicyMemory : public Memory<Genotype> {
        private:
            
        public:
            PolicyMemory() {}
            ~PolicyMemory() {}
            
            
        
    };

}

#endif /* POLICY_MEMORY_HPP */

