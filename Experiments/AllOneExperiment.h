/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   AllOneExperiment.h
 * Author: Julien Hubert
 *
 * Created on August 12, 2016, 6:10 PM
 */

#ifndef ALLONEEXPERIMENT_H
#define ALLONEEXPERIMENT_H

#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"

namespace MDB_Social {

    class AllOneExperiment: public GAFitness
    {
    private:
        
    public:
        AllOneExperiment();
        ~AllOneExperiment();
        
        double evaluateFitness(Genotype& individual) override;
        
        void loadParameters() override;
    };
    
}


#endif /* ALLONEEXPERIMENT_H */
