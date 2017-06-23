/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BabblingStandard.h
 * Author: Julien Hubert
 *
 * Created on September 26, 2016, 3:24 PM
 */

#ifndef BABBLINGSTANDARD_H
#define BABBLINGSTANDARD_H

#include <tuple>
#include <vector>
#include <random>
#include "Memory.hpp"
#include "TraceMemory.h"
#include "Model.hpp"


namespace MDB_Social {

    class Settings;
    class ResourceLibraryData; 
    
    class BabblingStandard: public Model {
    private:
        Settings* settings;
        ResourceLibraryData* resourceLibrary;
        TraceMemory* traceMemory;

        std::mt19937 mersenne_generator;
        std::normal_distribution<double> normal_dist;

        std::vector<double> outmin;
        std::vector<double> outmax;
        
        int noveltyTraceLength;
        int noveltyProposalsGenerated;
        double noveltyCoefficientExponent;
        
        double getTracesDistance(const Trace& A, const Trace& B) const;
        double computeNoveltyCoefficient(std::vector<double>& inputs, std::vector<double>& outputs) const;
        
    public:
        BabblingStandard();
        virtual ~BabblingStandard();

        
        virtual void train() override;
        virtual void reset() override;
        virtual void publish() override;
        virtual void registerParameters(std::string prefix=std::string()) override;
        virtual void loadParameters(std::string prefix=std::string()) override;
        virtual void initializeFromParameters() override;

        virtual void setExternalMemory(Memory<NeuralNetworkData>* mem) override;
        virtual bool loadFromExternalMemory() override;
        
        std::vector<double> run(std::vector<double>& inputs);
        void setOutputMinMax(std::vector<double>& _outmin, std::vector<double>& _outmax);
        
        
        
    };

}
#endif /* BABBLINGSTANDARD_H */

