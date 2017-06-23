/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BabblingPolicy.h
 * Author: Julien Hubert
 *
 * Created on October 10, 2016, 11:38 AM
 */

#ifndef BABBLINGPOLICY_H
#define BABBLINGPOLICY_H

#include "GeneticAlgorithm.hpp"

namespace MDB_Social {

    class BabblingPolicy {
    private:
        GAFitness* fitness;
        Genotype dummyGenotype;

        unsigned trialCount;
        
    public:
        BabblingPolicy();
        virtual ~BabblingPolicy();

        virtual bool run();
        
        virtual void initialise();
        
        virtual void registerParameters();
        virtual void loadParameters();
    };

}

#endif /* BABBLINGPOLICY_H */

