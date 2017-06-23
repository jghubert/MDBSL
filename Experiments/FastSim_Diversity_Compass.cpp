/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Diversity_Compass.cpp
 * Author: Evert Haasdijk
 *
 * Created on Wed May 17 11:31:35 CEST 2017
 */

#include "../MDB_SocialLearning/Settings.h"
#include "FastSim_Diversity_Compass.h"
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>
#include <cassert>
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/ValueFunction.h"
#include "RobotID.h"

namespace MDB_Social {

// TODO: this is in large part a copy of FastSim_Phototaxis_Compass, should refactor to deduplicate code (e.g., common base class?)


    FastSim_Diversity_Compass::FastSim_Diversity_Compass() 
    {
    }

    FastSim_Diversity_Compass::FastSim_Diversity_Compass(const FastSim_Diversity_Compass& orig) 
    {
        
    }

    FastSim_Diversity_Compass::~FastSim_Diversity_Compass() 
    {
    }

//    void FastSim_Diversity_Compass::registerParameters()
//    {
//    }
//
//    void FastSim_Diversity_Compass::loadParameters()
//    {
//    	Base::loadParameters();
//        std::cout << "FastSim_Diversity_Compass: Loading parameters..." << std::endl;
////        Settings* settings = Settings::getInstance();
//        try {
//          std::cout << "FastSim_Diversity_Compass: Parameters loaded." << std::endl;
//        }
//        catch (std::exception e) {
//            std::cerr << "FastSim_Diversity_Compass: Error loading the parameters: " << e.what() << std::endl;
//            exit(1);
//        }
//    }

    struct SumTraces
	{
    	Trace operator()(const Trace &lhs, const Trace &rhs) const
    	{
//    	    assert(lhs.inputs.size() == rhs.inputs.size());
//    	    assert(lhs.outputs.size() == rhs.outputs.size());
//    	    assert(lhs.inputs_t1.size() == rhs.inputs_t1.size());

    		Trace retval(lhs);

    		for (std::size_t i = 0; i < lhs.inputs.size(); ++i)
    			retval.inputs[i] += rhs.inputs[i];
    		for (std::size_t i = 0; i < lhs.outputs.size(); ++i)
    			retval.outputs[i] += rhs.outputs[i];
    		for (std::size_t i = 0; i < lhs.inputs_t1.size(); ++i)
    			retval.inputs_t1[i] += rhs.inputs_t1[i];

    		return retval;
    	}
	};

    struct Divider{
        double rhs;

    	Divider(double rhs_) : rhs(rhs_) {}
    	double operator()(double lhs) const {return lhs / rhs;}
    };

    double FastSim_Diversity_Compass::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
    	// Call FastSim_Phototaxis_Compass evaluate as that performs exactly the actions we need
    	// But make sure no standard babbling is performed
    	// The actual diversity fitness is computed afterward in postprocessing because that
    	// requires a comparison of all individuals in the population
    	
    	// TODO: because the population is maintained in the GeneticAlgorithm, we use one population
    	// for babbling and for the actual task; we may need to use separate populations. But maybe
    	// increasing diversity is good for the task as well
    	
    	// must be sure that 'regular' babbling is disabled for now, reset after running the controller
    	bool oldReccomendBabbling(false);
    	std::swap(oldReccomendBabbling, recommendBabbling);
    	bool oldUseOnlyBabbling(false);
    	std::swap(oldUseOnlyBabbling, useOnlyBabbling);
    	
        // log start of trace for this individual
        ResourceLibrary* resourceLibrary = RobotID::getResourceLibrary();
        TraceMemory* tm = resourceLibrary->getTraceMemory();
        size_t traceOffset = tm->size();    // Won't work if the memory is full and new inputs erase older.

        double fitness = Base::evaluateFitness(individual, gen, ind, _testIndividual);

        // restore babbling mode
        recommendBabbling = oldReccomendBabbling;
        useOnlyBabbling = oldUseOnlyBabbling;

        TraceMemory::iterator startOfEvaluation = tm->begin() + traceOffset;

        if (useBabbling()) {
			// Store trace info for postprocessing and calculating diversity; we use average activation level for now
			// TODO: this should be encapsulated in a BehaviourDescription class to ease alternative implementations
			BehaviourDescription behaviour;

			Trace empty(*tm->begin());
			std::fill(empty.inputs.begin(), empty.inputs.end(), 0.0);
			std::fill(empty.outputs.begin(), empty.outputs.end(), 0.0);
			std::fill(empty.inputs_t1.begin(), empty.inputs_t1.end(), 0.0);

			Trace sumActivationLevels = std::accumulate(startOfEvaluation, tm->end(), empty, SumTraces());

			behaviour.reserve(sumActivationLevels.inputs.size() + sumActivationLevels.outputs.size() + sumActivationLevels.inputs_t1.size());
			Divider d (static_cast<double>( tm->end() - startOfEvaluation));
			std::transform(sumActivationLevels.inputs.begin(), sumActivationLevels.inputs.end(),
					std::back_inserter(behaviour),d);
			std::transform(sumActivationLevels.outputs.begin(), sumActivationLevels.outputs.end(),
					std::back_inserter(behaviour), d);
			std::transform(sumActivationLevels.inputs_t1.begin(), sumActivationLevels.inputs_t1.end(),
					std::back_inserter(behaviour), d);

			behaviours[individual.getUUID()] = behaviour;
        }

    	return fitness;
    }

    struct DistanceTo {
    	DistanceTo(const FastSim_Diversity_Compass::BehaviourDescription& behaviour_) :
    		behaviour(behaviour_)
    	{
        }
        
    	double operator()(const FastSim_Diversity_Compass::BehaviourDescription &rhs) const {
    		// TODO: implement speedy distance calc suggested by Julien
    		std::vector<double> tmp;
    		tmp.reserve(behaviour.size());

    		std::transform (behaviour.begin(), behaviour.end(), rhs.begin(), std::back_inserter(tmp),
    		    	[](double element1, double element2) {return pow((element1-element2),2);});

    		return  sqrt(std::accumulate(tmp.begin(), tmp.end(), 0));
    	}

    private:
    	const FastSim_Diversity_Compass::BehaviourDescription& behaviour;
    };

    /// Any postprocessing a population after all individuals have been evaluated, but before sorting the population
    void FastSim_Diversity_Compass::postProcessIndividual(Genotype& individual) {

    	if (useBabbling()) {

			// Compute average distance to all other BehaviourDescriptions for the individual's behaviour
			// and set that as fitness;

			double distance = 0.0;
			DistanceTo d(behaviours[individual.getUUID()]);

			for (std::map<boost::uuids::uuid, BehaviourDescription>::iterator i = behaviours.begin(); i != behaviours.end(); ++i) {
				if (i->first != individual.getUUID()) {
					distance += d(i->second);
				}
			}

			individual.setFitness((behaviours.size() > 1 ) ? distance / behaviours.size()-1 : 0.0);
    	}
    }
}
