/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TraceMemory.cpp
 * Author: Julien Hubert
 * 
 * Created on August 4, 2016, 11:48 AM
 */

#include "TraceMemory.h"
#include <math.h>

namespace MDB_Social {

/*********************** TRACE ********************************************/    
    
    Trace::Trace()
    {
        true_reward = 0.0;
        expected_reward = 0.0;
        estimated_reward = 0.0;
        reliability = 0.0;
    }
    
    Trace::Trace(const Trace& t)
    {
        this->inputs = t.inputs;
        this->outputs = t.outputs;
        this->inputs_t1 = t.inputs_t1;
        this->true_reward = t.true_reward;
        this->expected_reward = t.expected_reward;
        this->estimated_reward = t.estimated_reward;
        this->reliability = t.reliability;
    }
    
    Trace::~Trace()
    {
        
    }

    Trace& Trace::operator=(Trace t)
    {
        this->inputs = t.inputs;
        this->outputs = t.outputs;
        this->inputs_t1 = t.inputs_t1;
        this->true_reward = t.true_reward;
        this->expected_reward = t.expected_reward;
        this->estimated_reward = t.estimated_reward;
        this->reliability = t.reliability;
        
        return *this;
    }
    
    double Trace::computeInputDistance(Trace& t, enum DistanceMeasure dm)
    {
        double ret = 1e6;
        switch (dm) {
            case EUCLIDIAN:
            {
                ret = 0.0;
                double a;
                for (size_t i=0; i<inputs.size(); ++i) {
                    a = inputs[i] - t.inputs[i];
                    ret += a * a;
//                    ret += pow(inputs[i] - t.inputs[i],2.0);
                }
                ret = sqrt(ret);
                break;
            }
            case EUCLIDIAN2:
            {
                ret = 0.0;
                double a;
                for (size_t i=0; i<inputs.size(); ++i) {
                    a = inputs[i] - t.inputs[i];
                    ret += a * a;
//                    ret += pow(inputs[i] - t.inputs[i],2.0);
                }
                break;
            }
            case COSINE:
            {
                double AB=0.0;
                double A=0.0, B=0.0;
                for (size_t i=0; i<inputs.size(); ++i) {
                    AB += inputs[i]*t.inputs[i];
                    A += inputs[i]*inputs[i];
                    B += t.inputs[i]*t.inputs[i];
                }
                ret = AB / (sqrt(A)*sqrt(B));
                break;
            }                
            case COSINE2:
            {
                double AB=0.0;
                double A=0.0, B=0.0;
                for (size_t i=0; i<inputs.size(); ++i) {
                    AB += inputs[i]*t.inputs[i];
                    A += inputs[i]*inputs[i];
                    B += t.inputs[i]*t.inputs[i];
                }
                ret = AB*AB / (A*B);
                break;
            }                
            default:
                break;
        }
        return ret;
    }
    
    
/************************** TRACEMEMORY *******************************/
    TraceMemory::TraceMemory() {
    }

    TraceMemory::~TraceMemory() {
    }

    double TraceMemory::computeShortestDistance(Trace& t, enum Trace::DistanceMeasure dm)
    {
        double best = -1e6;
        double dist;
        auto itend = end();
        for (auto it = begin(); it != itend; ++it) {
            dist = it->computeInputDistance(t, dm);
            if (best < dist)
                best = dist;
        }
        return best;
    }

    
}