/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   XOR_Experiment.h
 * Author: Julien Hubert
 *
 * Created on October 14, 2016, 10:52 AM
 */

#include <fstream>
#include <vector>
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"

#ifndef XOR_EXPERIMENT_H
#define XOR_EXPERIMENT_H

namespace MDB_Social {

    class XOR_Experiment: public GAFitness {
    private:
        FeedforwardNN* controller;

        unsigned nbinputs;
        unsigned nboutputs;
        unsigned hiddenNeurons;
        double controllerMinimumWeight;
        double controllerMaximumWeight;
        std::vector<unsigned> layers;
        bool testIndividual;

        void registerParameters();
        
        void installGenotype(Genotype& individual);

        
    public:
        XOR_Experiment();
        XOR_Experiment(const XOR_Experiment& orig);
        virtual ~XOR_Experiment();

        double evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual=false) override;

        void loadParameters() override;

        virtual void preprocessing() override;
    };
}

#endif /* XOR_EXPERIMENT_H */

