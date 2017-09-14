/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CaffeDeepNet.h
 * Author: Julien Hubert
 *
 * Created on August 11, 2017, 12:21 PM
 */

#ifndef CAFFEDEEPNET_H
#define CAFFEDEEPNET_H

#include "Model.hpp"
#include <caffe/caffe.hpp>
#include <string>
#include <memory>

namespace MDB_Social {
    class CaffeDeepNet : public Model
    {
    private:
        std::string modelConfigFilename;
        std::string solverConfigFilename;
        std::string trainingInputLayerName;
        std::string testingInputLayerName;
        std::string testingOutputLayerName;
        std::string publishToFilename;
        bool loadLearnedParametersFromFile;
        std::string learnedParametersFilename;
        bool useLSTM;
        std::vector<std::string> lstmClipBlobsNames; 
        caffe::SolverParameter solver_param;
        std::shared_ptr<caffe::Solver<double> > solver;
        std::shared_ptr<caffe::Net<double> > evalnet;
        
    public:
        CaffeDeepNet();
        virtual ~CaffeDeepNet();
        
        virtual void train() override;
        virtual void reset() override;
        virtual void publish() override;
        virtual void step(unsigned nbsteps = 1);

        virtual void trainSequentially(std::vector<std::vector<std::vector<double> > >& inputs, std::vector<std::vector<std::vector<double> > >& outputs);
        
        virtual void copySolverToTestingNetwork();
        
        virtual void registerParameters(std::string prefix=std::string()) override;
        virtual void loadParameters(std::string prefix=std::string()) override;
        virtual void initializeFromParameters() override;

        virtual void setExternalMemory(Memory<NeuralNetworkData>* mem) override;
        virtual bool loadFromExternalMemory() override;
        virtual bool saveModelToFile(const char* filename) override;

        void lstmClipBlobSet(caffe::Net<double>* net, std::string name, double value);
        
        void setTrainingSet(unsigned numData, std::vector<double>& inputs, std::vector<double>& outputs);
        std::vector<double> run(unsigned numData, std::vector<double>& inputs);
        void run(unsigned numData, std::vector<double>& inputs, std::vector<double>& outputs);
        
    };
    
    
}

#endif /* CAFFEDEEPNET_H */
