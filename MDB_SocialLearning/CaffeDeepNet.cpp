/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CaffeDeepNet.cpp
 * Author: julienhubert
 * 
 * Created on August 11, 2017, 12:21 PM
 */

#include "CaffeDeepNet.h"
#include "Settings.h"
#include "RobotID.h"
#include <caffe/layers/memory_data_layer.hpp>

namespace MDB_Social {

    CaffeDeepNet::CaffeDeepNet():
        Model("CaffeDeepNet")
    {
        
    }
    
    CaffeDeepNet::~CaffeDeepNet()
    {
        
    }

    void CaffeDeepNet::train()
    {
        solver->Solve();
        copySolverToTestingNetwork();
    }
    
    void CaffeDeepNet::step(unsigned nbsteps)
    {
        solver->Step(nbsteps);
    }

    void CaffeDeepNet::trainSequentially(std::vector<std::vector<std::vector<double> > >& inputs, std::vector<std::vector<std::vector<double> > >& outputs)
    {
        unsigned nbSequences = inputs.size();
        
        for (unsigned s=0; s<nbSequences; ++s) {
            size_t nbdata = inputs[s].size();
            // We consider there is a LSTM
            
            // Need to set the clip to 1
            for (size_t i=0; i<nbdata; ++i) {
                setTrainingSet(1, inputs[s][i], outputs[s][i]);
                if (i == 0)
                    for (unsigned j=0; j<lstmClipBlobsNames.size(); ++j)
                        lstmClipBlobSet(solver->net().get(), lstmClipBlobsNames[j], 1.0);
                    // Need to set the clip to 0
                else 
                    for (unsigned j=0; j<lstmClipBlobsNames.size(); ++j)
                        lstmClipBlobSet(solver->net().get(), lstmClipBlobsNames[j], 0.0);

                this->step(1);
            }
        }
        copySolverToTestingNetwork();
    }

    
    void CaffeDeepNet::copySolverToTestingNetwork()
    {
        caffe::NetParameter netparam;
        solver->net()->ToProto(&netparam);
        evalnet->CopyTrainedLayersFrom(netparam);
    }
    
    void CaffeDeepNet::reset()
    {
        solver->InitTrainNet();
    }
    
    void CaffeDeepNet::publish()
    {
        this->saveModelToFile(publishToFilename.c_str());
    }

    void CaffeDeepNet::registerParameters(std::string prefix)
    {
        Settings* settings = SettingsLibrary::getInstance(); //RobotID::getSettings();
        settings->registerParameter<std::string>( (prefix+".modelConfigFilename").c_str(), std::string("caffe.model.prototxt"), "Model description for the Caffe library.");
        settings->registerParameter<std::string>( (prefix+".solverConfigFilename").c_str(), std::string("caffe.solver.prototxt"), "Parameters of the Solver for the Caffe library.");
        settings->registerParameter<std::string>( (prefix+".trainingInputLayerName").c_str(), std::string("training_input"), "Name of the layer containing the input data during the training phase.");
        settings->registerParameter<std::string>( (prefix+".testingInputLayerName").c_str(), std::string("testing_input"), "Name of the layer containing the input data during the testing phase.");
        settings->registerParameter<std::string>( (prefix+".testingOutputLayerName").c_str(), std::string("testing_output"), "Name of the layer containing the output data during the testing phase.");
        settings->registerParameter<std::string>( (prefix+".publishToFilename").c_str(), std::string("caffe.published_model.dat"), "Filename where the network must be saved during publishing.");
        settings->registerParameter<bool>( (prefix+".loadLearnedParametersFromFile").c_str(), false, "Indicates if the learned parameters should be loaded from a file.");
        settings->registerParameter<std::string>( (prefix+".learnedParametersFilename").c_str(), std::string(), "Filename where the learned parameters of the network are contained.");
        settings->registerParameter<bool>( (prefix+".useLSTM").c_str(), false, "Activate the LSTM options.");
        settings->registerParameter<std::string>( (prefix+".lstmClipBlobsName").c_str(), std::string(), "List pf the clip blobs to reset between sequences when using a lstm");
    }
    
    void CaffeDeepNet::loadParameters(std::string prefix)
    {
        Settings* settings = SettingsLibrary::getInstance();
        modelConfigFilename = settings->value<std::string>( (prefix+".modelConfigFilename").c_str()).second;
        solverConfigFilename = settings->value<std::string>( (prefix+".solverConfigFilename").c_str()).second;
        trainingInputLayerName = settings->value<std::string>( (prefix+".trainingInputLayerName").c_str()).second;
        testingInputLayerName = settings->value<std::string>( (prefix+".testingInputLayerName").c_str()).second;
        testingOutputLayerName = settings->value<std::string>( (prefix+".testingOutputLayerName").c_str()).second;
        publishToFilename = settings->value<std::string>( (prefix+".publishToFilename").c_str()).second;
        loadLearnedParametersFromFile = settings->value<bool>( (prefix+".loadLearnedParametersFromFile").c_str()).second;
        learnedParametersFilename = settings->value<std::string>( (prefix+".learnedParametersFilename").c_str()).second;
        useLSTM = settings->value<bool>( (prefix+".useLSTM").c_str()).second;
        std::string lstmClipBlobsNameStr = settings->value<std::string>( (prefix+".lstmClipBlobsName").c_str()).second;

        if (!lstmClipBlobsNameStr.empty()){
            lstmClipBlobsNames.clear();
            unsigned index = 0, lastCut = 0;
            for (; index < lstmClipBlobsNameStr.size(); ++index) {
                if (lstmClipBlobsNameStr[index] == ';') {
                    std::string tmp = lstmClipBlobsNameStr.substr(lastCut, index-lastCut);
                    lstmClipBlobsNames.push_back(tmp);
                    lastCut = index+1;
                }
            }
            std::string tmp = lstmClipBlobsNameStr.substr(lastCut, index-lastCut);
            lstmClipBlobsNames.push_back(tmp);
        }

        
        caffe::ReadSolverParamsFromTextFileOrDie(solverConfigFilename.c_str(), &solver_param);
        solver.reset(caffe::SolverRegistry<double>::CreateSolver(solver_param));
        
        evalnet.reset(new caffe::Net<double>(modelConfigFilename.c_str(), caffe::TEST));
        
        if (loadLearnedParametersFromFile) {
            evalnet->CopyTrainedLayersFromHDF5(learnedParametersFilename);
        }
    }

    void CaffeDeepNet::lstmClipBlobSet(caffe::Net<double>* net, std::string name, double value)
    {
        caffe::Blob<double>* clip = net->blob_by_name(name).get();
        double* idata = clip->mutable_cpu_data();
        for (size_t i=0; i<clip->shape(0); ++i)
            idata[i] = value;
    }

    
    void CaffeDeepNet::initializeFromParameters()
    {
        
    }

    void CaffeDeepNet::setExternalMemory(Memory<NeuralNetworkData>* mem)
    {
        
    }
    
    bool CaffeDeepNet::loadFromExternalMemory()
    {
        return false;
    }
    
    bool CaffeDeepNet::saveModelToFile(const char* filename)
    {
        solver->net()->ToHDF5(filename);
        return true;
    }

    void CaffeDeepNet::setTrainingSet(unsigned numData, std::vector<double>& inputs, std::vector<double>& outputs)
    {
        // Need to format the data and upload them to the network.
//        std::cout << "CaffeDeepNet: setTrainingSet : input_blobs.size() = " << solver->net()->input_blobs().size() << std::endl;
        caffe::Blob<double>* input_data = solver->net()->input_blobs()[0];
        double* idata = input_data->mutable_cpu_data();
        for (size_t i=0; i<inputs.size(); ++i)
            idata[i] = inputs[i];

        caffe::Blob<double>* labels = solver->net()->input_blobs()[2];
        idata = labels->mutable_cpu_data();
        for (size_t i=0; i<outputs.size(); ++i)
            idata[i] = outputs[i];
        
        
//        caffe::MemoryDataLayer<double> *dataLayer_trainnet = (caffe::MemoryDataLayer<double> *) (solver->net()->layer_by_name(trainingInputLayerName).get());        
//        dataLayer_trainnet->set_batch_size(numData);
//        dataLayer_trainnet->Reset(inputs.data(), outputs.data(), numData);
    }
        
    std::vector<double> CaffeDeepNet::run(unsigned numData, std::vector<double>& inputs)
    {
        std::vector<double> ret;
        this->run(numData, inputs,ret);
        return ret;
    }
    
    void CaffeDeepNet::run(unsigned numData, std::vector<double>& inputs, std::vector<double>& outputs)
    {
//        std::cout << "CaffeDeepNet: input blobs count => evalnet =  " << evalnet->num_inputs () << " ; solver = " << solver->net()->num_inputs() << std::endl;
        caffe::Blob<double>* input_data = evalnet->input_blobs()[0];
        double* idata = input_data->mutable_cpu_data();
        for (size_t i=0; i<inputs.size(); ++i)
            idata[i] = inputs[i];

//        caffe::MemoryDataLayer<double> *dataLayer_testnet = (caffe::MemoryDataLayer<double> *) (evalnet->layer_by_name(testingInputLayerName).get());        
//        dataLayer_testnet->set_batch_size(numData);
//        dataLayer_testnet->Reset(inputs.data(), outputs.data(), numData);
        
        evalnet->Forward();

        boost::shared_ptr<caffe::Blob<double> > output_data = evalnet->blob_by_name(testingOutputLayerName);
        const double* begin = output_data->cpu_data();
//        const double* end = begin + output_data->shape(0);
        
//        caffe::Blob<double>* output_data = evalnet->output_blobs()[0];
//        const double* begin = output_data->cpu_data();
//        int outsize = output_data->shape(0);
//        if (outputs.size() != outsize)
//            outputs.resize(outsize, 0.0);
            
        for (int i=0; i<output_data->shape(0); ++i)
            outputs[i] = begin[i];
    }
    
}