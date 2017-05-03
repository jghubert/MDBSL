/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   XOR_Experiment.cpp
 * Author: Julien Hubert
 * 
 * Created on October 14, 2016, 10:52 AM
 */

#include <iostream>
#include "XOR_Experiment.h"

namespace MDB_Social {

    XOR_Experiment::XOR_Experiment(std::string id) 
    {
        registerParameters();
        nbinputs = 2;
        nboutputs = 1;
        hiddenNeurons = 2;

        controllerMinimumWeight = -10.0;
        controllerMaximumWeight = 10.0;
        
        testIndividual = false;
    }

    XOR_Experiment::XOR_Experiment(const XOR_Experiment& orig) 
    {
    }

    XOR_Experiment::~XOR_Experiment() 
    {
    }

    void XOR_Experiment::registerParameters()
    {
        std::cout << "XOR_Experiment : registering the parameters...";
        std::cout.flush();
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<unsigned>("experiment.nbinputs", 1, "Number of inputs/sensors on the neural network.");
        settings->registerParameter<unsigned>("experiment.nboutputs", 2, "Number of outputs on the neural network.");
        settings->registerParameter<unsigned>("experiment.hiddenNeurons", 10, "Number of hidden neurons for the controller.");
        settings->registerParameter<double>("experiment.controllerMinimumWeight", 0.0, "Minimum weight of the neural network.");
        settings->registerParameter<double>("experiment.controllerMaximumWeight", 0.0, "Maximum weight of the neural network.");
        std::cout << " DONE" << std::endl;
    }
    
    void XOR_Experiment::loadParameters()
    {
        std::cout << "XOR_Experiment: Loading parameters..." << std::endl;
//        Settings* settings = Settings::getInstance();

        try {
            nbinputs = settings->value<unsigned>("experiment.nbinputs").second;
            nboutputs = settings->value<unsigned>("experiment.nboutputs").second;
            hiddenNeurons = settings->value<unsigned>("experiment.hiddenNeurons").second;
            controllerMinimumWeight = settings->value<double>("experiment.controllerMinimumWeight").second;
            controllerMaximumWeight = settings->value<double>("experiment.controllerMaximumWeight").second;
            std::cout << "XOR_Experiment: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "XOR_Experiment: Error loading the parameters: " << e.what() << std::endl;
            exit(1);
        }
        
        controller = static_cast<FeedforwardNN*>(ModelLibrary::getModel("Feedforward"));
        std::vector<enum FeedforwardNN::ActivationFunction> activationFunctions(1, FeedforwardNN::SIGMOID);
        if (hiddenNeurons) {
            layers.resize(3);
            layers[0] = nbinputs;
            layers[1] = hiddenNeurons;
            layers[2] = nboutputs;
        }
        else {
            layers.resize(2);
            layers[0] = nbinputs;
            layers[1] = nboutputs;
        }
        controller->setup(layers, activationFunctions);
        
    }

    void XOR_Experiment::installGenotype(Genotype& individual)
    {
        std::vector<FeedforwardNN::weight_t> weights(individual.getSize());
        
        controller->getWeights(weights);
        if (weights.size() != individual.getSize()) {
            std::cerr << "XOR_Experiment: Size of the genotype (size = " << individual.getSize() << ") differs from the size of the network (size = " << weights.size() << ")." << std::endl;
            exit(1);
        }
        
//        std::cout << "controllerMaximumWeight = " << controllerMaximumWeight << " ; controllerMinimumWeight = " << controllerMinimumWeight <<  " ; Weights = " << std::endl;
        for (unsigned i=0; i<individual.getSize(); ++i) {
            
            weights[i].weight =  (controllerMaximumWeight - controllerMinimumWeight)*individual[i] + controllerMinimumWeight;
//            std::cout << "  " << std::get<0>(weights[i]) << " to " << std::get<1>(weights[i]) << " = " << std::get<2>(weights[i]) << " ; gene = " << individual[i] << std::endl;
        }
        
        controller->setWeights(weights);
    }
    
    double XOR_Experiment::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
        testIndividual = _testIndividual;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        
        installGenotype(individual);
        
        double fitness = 0.0;
        
        nninputs[0] = 0.0;
        nninputs[1] = 0.0;
        nnoutput = controller->run(nninputs);
        fitness += std::abs(1.0 - nnoutput[0]);
        if (testIndividual)
            std::cout << "inputs = " << nninputs[0] << " " << nninputs[1] << " -> " << nnoutput[0] << std::endl;
        
        nninputs[0] = 1.0;
        nninputs[1] = 0.0;
        nnoutput = controller->run(nninputs);
        fitness += std::abs(nnoutput[0]);
        if (testIndividual)
            std::cout << "inputs = " << nninputs[0] << " " << nninputs[1] << " -> " << nnoutput[0] << std::endl;
        
        nninputs[0] = 0.0;
        nninputs[1] = 1.0;
        nnoutput = controller->run(nninputs);
        fitness += std::abs(nnoutput[0]);
        if (testIndividual)
            std::cout << "inputs = " << nninputs[0] << " " << nninputs[1] << " -> " << nnoutput[0] << std::endl;

        nninputs[0] = 1.0;
        nninputs[1] = 1.0;
        nnoutput = controller->run(nninputs);
        fitness += std::abs(1.0 - nnoutput[0]);
        if (testIndividual)
            std::cout << "inputs = " << nninputs[0] << " " << nninputs[1] << " -> " << nnoutput[0] << std::endl;

        fitness /= 4.0;
        return fitness;
    }

    
}
