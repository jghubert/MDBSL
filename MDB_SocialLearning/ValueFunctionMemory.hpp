/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ValueFunctionMemory.hpp
 * Author: Julien Hubert
 *
 * Created on June 14, 2016, 4:56 PM
 */

#ifndef VALUE_FUNCTION_MEMORY_HPP
#define VALUE_FUNCTION_MEMORY_HPP

#include <fstream>
#include <string>
#include "Settings.h"
#include "Memory.hpp"

namespace MDB_Social {
    
    class ValueFunctionMemory : public Memory<NeuralNetworkData> {    //std::tuple<unsigned,unsigned,double> 
    private:
        
    public:
        ValueFunctionMemory();
        virtual ~ValueFunctionMemory();

                    
    };

}

#endif /* VALUE_FUNCTION_MEMORY_HPP */

