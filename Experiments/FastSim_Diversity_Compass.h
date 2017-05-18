/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Diversity_Compass.h
 * Author: Evert Haasdijk
 *
 * Created on Wed May 17 11:31:35 CEST 2017
 */

#ifndef FASTSIM_DIVERSITY_COMPASS_H
#define FASTSIM_DIVERSITY_COMPASS_H

#include <fstream>
#include <map>
#include <vector>
#include "FastSim_Phototaxis_Compass.h"
#include "../MDB_SocialLearning/GeneticAlgorithm.hpp"
#include "../MDB_SocialLearning/SimulatorLibrary.hpp"
#include "../MDB_SocialLearning/ModelLibrary.hpp"
#include "../MDB_SocialLearning/BabblingStandard.h"

namespace MDB_Social {

    class FastSim_Diversity_Compass: public FastSim_Phototaxis_Compass 
    {
    public:        
        typedef std::vector<double> BehaviourDescription;

        FastSim_Diversity_Compass(std::string id="Default");
        FastSim_Diversity_Compass(const FastSim_Diversity_Compass& orig);
        virtual ~FastSim_Diversity_Compass();

        double evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual=false) override;

        /// Called just before the individuals are evaluated
        void prepareEvaluation(void) override { behaviours.clear(); };
        /// Any postprocessing after all individuals have been evaluated, but before sorting the population
        virtual void postProcessIndividual(Genotype& individual) override;

    private:
  //  	traceIterator startOfEvaluation;
  //  	traceIterator endOfEvaluation;

        typedef FastSim_Phototaxis_Compass Base;

        std::map<boost::uuids::uuid, BehaviourDescription> behaviours;

    };

}

#endif /* FASTSIM_DIVERSITY_H */

