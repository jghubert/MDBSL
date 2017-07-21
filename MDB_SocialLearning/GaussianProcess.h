/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GaussianProcess.h
 * Author: julienhubert
 *
 * Created on July 21, 2017, 3:46 PM
 */

#ifndef GAUSSIANPROCESS_H
#define GAUSSIANPROCESS_H

#include "Model.hpp"
#include "Memory.hpp"
#include <gp/gp.h>
#include <string>

namespace MDB_Social {

    class GaussianProcess : public Model
    {
    private:
        libgp::GaussianProcess* gp;

        int inputDimensionality;
        std::string covarianceFunction;
        
        Memory<weight_t>* externalMemory;
    public:
        GaussianProcess();
        virtual ~GaussianProcess();

        virtual void train() override;
        virtual void reset() override;
        virtual void publish() override;
        
        virtual void registerParameters(std::string prefix=std::string()) override;
        virtual void loadParameters(std::string prefix=std::string()) override;
        virtual void initializeFromParameters() override;

        virtual void setExternalMemory(Memory<NeuralNetworkData>* mem) override;
        virtual bool loadFromExternalMemory() override;
        virtual bool saveModelToFile(const char* filename) override;


    };
}
    
#endif /* GAUSSIANPROCESS_H */

