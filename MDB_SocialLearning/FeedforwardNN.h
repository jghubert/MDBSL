/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FeedforwardNN.h
 * Author: Julien Hubert
 *
 * Created on June 16, 2016, 4:32 PM
 */

#ifndef FEEDFORWARDNN_H
#define FEEDFORWARDNN_H

#include "Model.hpp"
#include <doublefann.h>
#include <fann_cpp.h>
#include <vector>
#include "Memory.hpp"


namespace MDB_Social {

//    template<typename dtype = double>
    class FeedforwardNN : public Model 
    {
    public:
        typedef NeuralNetworkData weight_t;
        
    private:
        FANN::neural_net net;
        FANN::training_data trainingData;

        double minWeights;
        double maxWeights;

        int nbinputs;
        int nboutputs;
        std::vector<unsigned> layers;
        double weightMinimum;
        double weightMaximum;
        
        int maxTrainingEpoch;
        float desiredTrainingError;

        double learningRate;


        Memory<weight_t>* externalMemory;

    public:
        enum ActivationFunction {
            LINEAR = FANN::LINEAR,
            THRESHOLD = FANN::THRESHOLD,
            THRESHOLD_SYMETRIC = FANN::THRESHOLD_SYMMETRIC,
            SIGMOID = FANN::SIGMOID,
            SIGMOID_STEPWISE = FANN::SIGMOID_STEPWISE,
            SIGMOID_SYMMETRIC = FANN::SIGMOID_SYMMETRIC,
            SIGMOID_SYMMETRIC_STEPWISE = FANN::SIGMOID_SYMMETRIC_STEPWISE,
            GAUSSIAN = FANN::GAUSSIAN,
            GAUSSIAN_SYMETRIC = FANN::GAUSSIAN_SYMMETRIC,
            GAUSSIAN_STEPWISE = FANN::GAUSSIAN_STEPWISE,
            ELLIOT = FANN::ELLIOT,
            ELLIOT_SYMETRIC = FANN::ELLIOT_SYMMETRIC,
            LINEAR_PIECE = FANN::LINEAR_PIECE,
            LINER_PIECE_SYMETRIC = FANN::LINEAR_PIECE_SYMMETRIC,
            SIN_SYMETRIC = FANN::SIN_SYMMETRIC,
            COS_SYMETRIC = FANN::COS_SYMMETRIC
        };

        enum TrainingAlgorithm {
            INCREMENTAL = FANN::TRAIN_INCREMENTAL,
            BATCH = FANN::TRAIN_BATCH,
            QUICKPROP = FANN::TRAIN_QUICKPROP,
            RPROP = FANN::TRAIN_RPROP,
            SARPROP = FANN::TRAIN_SARPROP
        };

        enum ErrorFunction {
            LINEARFCT = FANN::ERRORFUNC_LINEAR,
            TANHFCT = FANN::ERRORFUNC_TANH
        };

        enum StopFunction {
            MSE = FANN::STOPFUNC_MSE,
            BIT = FANN::STOPFUNC_BIT
        };

        FeedforwardNN(std::string id="Default");
        ~FeedforwardNN();

        virtual void registerParameters(std::string prefix=std::string()) override;
        virtual void loadParameters(std::string prefix=std::string()) override;
        virtual void initializeFromParameters() override;
        
        void reset() override;
        void resetWeights();
        void setWeightsMinMax(double mi, double ma);

        bool setup(const std::vector<unsigned>& layers, std::vector<enum ActivationFunction>& activationFunctions);
        bool setWeights(std::vector<weight_t>& weights);
        void getWeights(std::vector<weight_t>& weights);
        void setActivationFunctionSteepness(std::vector<double>& steepness);
        void setActivationFunctionSteepness(double steepness);

        void addToTrainingSet(std::vector<double>& input, std::vector<double>& output);
        void setTrainingSet(unsigned numData, std::vector<double>& inputs, std::vector<double>& outputs);
        void resetTrainingSet();
        void train() override;
        void trainWithCascade(int max_neurons);
        std::vector<double> run(std::vector<double>& inputs);
        void run(std::vector<double>& inputs, std::vector<double>& outputs);
        void setTrainingAlgorithm(enum TrainingAlgorithm ta);
        void setTrainingStopFunction(enum StopFunction sf);
        void setTrainingErrorFunction(enum ErrorFunction ef);
        void setMaxTrainingEpoch(int epoch);
        void setDesiredError(float derror);
        void setLearningRate(float lrate);
        void setCascadeOutputChangeFraction(float cocf);

        std::vector<unsigned> getLayerArray();
        
        virtual void setExternalMemory(Memory<NeuralNetworkData>* mem) override;
        void publish() override;
        bool loadFromExternalMemory() override;
        bool save(const std::string& filename);
        bool saveModelToFile(const char* filename) override;
            
    };
    
}

#endif /* FEEDFORWARDNN_H */
