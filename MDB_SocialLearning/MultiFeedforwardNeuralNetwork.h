/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MultiFeedforwardNeuralNetwork.h
 * Author: julienhubert
 *
 * Created on May 1, 2017, 12:54 PM
 */

#include "Model.hpp"
#include "FeedforwardNN.h"
#include <list>

#ifndef MULTIFEEDFORWARDNEURALNETWORK_H
#define MULTIFEEDFORWARDNEURALNETWORK_H

namespace MDB_Social {

    class MultiFeedforwardNeuralNetwork: public Model
    {
    private:
        std::list<FeedforwardNN*> nnList;
        
    public:
        MultiFeedforwardNeuralNetwork();
        MultiFeedforwardNeuralNetwork(const MultiFeedforwardNeuralNetwork& orig);
        virtual ~MultiFeedforwardNeuralNetwork();

        void train();
        void reset();
        void publish();
        void registerParameters(std::string prefix=std::string());
        void loadParameters(std::string prefix=std::string());
        void initializeFromParameters();
        
    };
}
#endif /* MULTIFEEDFORWARDNEURALNETWORK_H */

