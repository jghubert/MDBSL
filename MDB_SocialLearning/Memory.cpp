/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <tuple>
#include <fstream>
#include "Memory.hpp"

#ifndef SEPARATOR
#define SEPARATOR " "
#endif

namespace MDB_Social 
{

    NeuralNetworkData::NeuralNetworkData()
    {
        from = 0;
        to = 0;
        weight = 0.0;
    }
    
    NeuralNetworkData::NeuralNetworkData(const NeuralNetworkData& v)
    {
        this->from = v.from;
        this->to = v.to;
        this->weight = v.weight;
    }

    NeuralNetworkData::~NeuralNetworkData()
    {
        
    }

    NeuralNetworkData& NeuralNetworkData::operator=(const NeuralNetworkData& v)
    {
        this->from = v.from;
        this->to = v.to;
        this->weight = v.weight;
        
        return *this;
    }
    
}