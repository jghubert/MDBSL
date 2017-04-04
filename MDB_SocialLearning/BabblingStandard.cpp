/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BabblingStandard.cpp
 * Author: Julien Hubert
 * 
 * Created on September 26, 2016, 3:24 PM
 */

#include <chrono>
#include "BabblingStandard.h"
#include "ResourceLibrary.hpp"
#include "Settings.h"


namespace MDB_Social {

    BabblingStandard::BabblingStandard(std::string id):
        Model("BabblingStandard", id)
    {
        traceMemory = resourceLibrary->getTraceMemory();
        
        noveltyTraceLength = 0;
        noveltyProposalsGenerated = 0;
        
        unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
        mersenne_generator.seed(seed1);
        
    }

    BabblingStandard::~BabblingStandard() 
    {
    
    }


    void BabblingStandard::train()
    {
        
    }

    void BabblingStandard::reset()
    {
        
    }
    
    void BabblingStandard::publish()
    {
        
    }

    bool BabblingStandard::loadFromExternalMemory()
    {
        return true;
    }
    
    void BabblingStandard::initializeFromParameters()
    {
        
    }

    
    void BabblingStandard::registerParameters(std::string prefix)
    {
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<int>((prefix+".noveltyTraceLength").c_str(), 1000, "BabblingStandard algorithm: Number of steps to consider in the trace.");
        settings->registerParameter<int>((prefix+".noveltyProposalsGenerated").c_str(), 10, "BabblingStandard algorithm: Number of proposals generated.");
        settings->registerParameter<double>((prefix+".noveltyCoefficientExponent").c_str(), 2.0, "BabblingStandard algorithm: Exponent to apply on the distance.");
        
    }

    void BabblingStandard::loadParameters(std::string prefix)
    {
//        Settings* settings = Settings::getInstance();
        noveltyTraceLength = settings->value<int>((prefix + ".noveltyTraceLength").c_str()).second;
        noveltyProposalsGenerated = settings->value<int>((prefix + ".noveltyProposalsGenerated").c_str()).second;
        noveltyCoefficientExponent = settings->value<double>((prefix + ".noveltyCoefficientExponent").c_str()).second;
    }

    void BabblingStandard::setExternalMemory(Memory<NeuralNetworkData>* mem)
    {
        
    }

#define BABBLING_STANDARD_TRACE_DISTANCE_EUCLIDIAN
    double BabblingStandard::getTracesDistance(const Trace& A, const Trace& B) const
    {
        double ret = 0.0;
#ifdef BABBLING_STANDARD_TRACE_DISTANCE_EUCLIDIAN
        for (size_t i=0; i<A.inputs.size(); ++i)
            ret += std::pow(A.inputs[i] - B.inputs[i], 2.0);
        
        for (size_t i=0; i<A.outputs.size(); ++i)
            ret += std::pow(A.outputs[i] - B.outputs[i], 2.0);

        return std::sqrt(ret);

#else
        return ret;
#endif
        
    }
    
    
    double BabblingStandard::computeNoveltyCoefficient(std::vector<double>& inputs, std::vector<double>& outputs) const
    {
        double coeff = 0.0;
        
        unsigned ntl = noveltyTraceLength > traceMemory->size() ? traceMemory->size() : noveltyTraceLength;

        Trace B;
        B.inputs = inputs;
        B.outputs = outputs;
        Trace A;
        
        TraceMemory::const_reverse_iterator it = traceMemory->rend();
        unsigned k=0;
        for (; k<ntl && it != traceMemory->rend(); ++k) {
            A = *it;
            coeff += pow(getTracesDistance(*it, B) , noveltyCoefficientExponent);
            it++;
        }
        coeff /= (1.0*k);
        
        return coeff;
    }    
    
    std::vector<double> BabblingStandard::run(std::vector<double>& inputs)
    {
        // We need to generate some actions to undertake.
        // We take the last action and mutate the outputs with a normal distribution
        // We do that k times (mutations of mutations), and then we compute the
        // distance with the last M steps. We then choose the most innovative.

        if (traceMemory->size() == 0) {
            unsigned nboutputs = outmin.size();
            std::vector<double> outputs(nboutputs);
            for (size_t i=0; i<nboutputs; ++i) {
                outputs[i] += normal_dist(mersenne_generator);
                if (outputs[i] > outmax[i])
                    outputs[i] += -outmax[i] + outmin[i];
                else if (outputs[i] < outmin[i])
                    outputs[i] += outmax[i] - outmin[i];
            }
            return outputs;
        }
        else {
            std::vector<std::vector<double> > outputs(noveltyProposalsGenerated);
            std::vector<double> novelty(noveltyProposalsGenerated);

            Trace last = traceMemory->back();
            unsigned nboutputs = last.outputs.size();


            // Generate action proposals
            for (int i=0; i<noveltyProposalsGenerated; ++i) {
    //            outputs[i].resize(nboutputs);

                // Compute a variation
                for (size_t j=0; j<nboutputs; ++j) {
                    last.outputs[j] += normal_dist(mersenne_generator);
                    if (last.outputs[j] > outmax[j])
                        last.outputs[j] = outmax[j];
//                        last.outputs[j] += -outmax[j] + outmin[j];
                    else if (last.outputs[j] < outmin[j])
                        last.outputs[j] = outmin[j];
//                        last.outputs[j] += outmax[j] - outmin[j];
                }                
                outputs[i] = last.outputs;

                novelty[i] = computeNoveltyCoefficient(inputs, outputs[i]);
            }

            double bestNovelty = novelty[0];
            int bestIndex = 0; 
            for (int i=1; i<noveltyProposalsGenerated; ++i) {
                if (bestNovelty < novelty[i]) {
                    bestIndex = i;
                    bestNovelty = novelty[i];
                }
            }

            return outputs[bestIndex];
        }
    }

    void BabblingStandard::setOutputMinMax(std::vector<double>& _outmin, std::vector<double>& _outmax)
    {
        outmin = _outmin;
        outmax = _outmax;
    }


}
