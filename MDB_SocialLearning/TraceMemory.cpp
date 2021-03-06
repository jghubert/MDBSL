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
#include <boost/uuid/nil_generator.hpp>

namespace MDB_Social {

/*********************** TRACE ********************************************/    
    
    Trace::Trace()
    {
        true_reward = 0.0;
        expected_reward = 0.0;
        estimated_reward = 0.0;
        reliability = 0.0;
        usedForVFTraining = false;
        uuid = boost::uuids::nil_uuid();
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
        this->usedForVFTraining = t.usedForVFTraining;
        this->uuid = t.uuid;
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
        this->usedForVFTraining = t.usedForVFTraining;
        this->uuid = t.uuid;
        
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
        defaultUUID = boost::uuids::nil_uuid();
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

    void TraceMemory::setDefaultUUID(boost::uuids::uuid& _uuid)
    {
        defaultUUID = _uuid;
    }
    
    void TraceMemory::resetDefaultUUID()
    {
        defaultUUID = boost::uuids::nil_uuid();
    }
    
    void TraceMemory::push_front(Trace& d)
    {
        if (d.uuid.is_nil())
            d.uuid = defaultUUID;
        Memory<Trace>::push_front(d);
    }
    
    void TraceMemory::push_back(Trace& d)
    {
        if (d.uuid.is_nil())
            d.uuid = defaultUUID;
        Memory<Trace>::push_back(d);
    }
    
    std::vector<std::pair<unsigned, Memory<Trace>::iterator> > TraceMemory::countIndependentTraces()
    {
        std::vector<std::pair<unsigned, Memory<Trace>::iterator> > ret;
        unsigned count = 0;

        if (this->size() > 0) {
            TraceMemory::iterator it = this->begin();
            TraceMemory::iterator itend = end();
            TraceMemory::iterator itlast = it;
            
            boost::uuids::uuid currentid = it->uuid;
            count++;
            it++;
            
            for (; it != itend; ++it) {
                if (it->uuid != currentid) {
                    ret.push_back(std::make_pair(count, itlast));
                    count = 0;
                    currentid = it->uuid;
                }
                else
                    count++;
                itlast = it;
            }
            ret.push_back(std::make_pair(count, itend));
        }
        return ret;
    }
 
    
}