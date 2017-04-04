/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ECF.cpp
 * Author: Julien Hubert
 * 
 * Created on August 5, 2016, 11:20 AM
 */

#include "ECF.h"
#include "Settings.h"
#include "colormod.hpp"

namespace MDB_Social {

/*********************** ECFGenotype******************************************/    
    ECFGenotype::ECFGenotype()
    {
        ecfgen = NULL;
    }
    
    ECFGenotype::~ECFGenotype()
    {
        
    }
    
    void ECFGenotype::setGenotype(FloatingPoint::FloatingPoint* p)
    {
        ecfgen = p;
    }
    
    size_t ECFGenotype::getSize() const
    {
        return ecfgen->realValue.size();
    }
    
    double ECFGenotype::operator[] (unsigned index)
    {
        return ecfgen->realValue[index];
    }
    
/*********************** ECFFitness ******************************************/    
    
    ECFFitness::ECFFitness()
    {
        gaFitness = NULL;
    }
    
    ECFFitness::ECFFitness(GAFitness* fit)
    {
        gaFitness = fit;
    }
    
    ECFFitness::~ECFFitness()
    {

    }

    FitnessP ECFFitness::evaluate(IndividualP individual)
    {
        // We need to convert the genotype to the format used in GAFitness
        // and provide the right format for the fitness
        
        FitnessP ret(new FitnessMax);
        
        FloatingPoint::FloatingPoint* ecfgen = (FloatingPoint::FloatingPoint*) individual->getGenotype().get();
        ecfGenotype.setGenotype(ecfgen);

        // The value function is available through the ResourceLibrary class.
        // No need to add it to the fitness function
        double fit = gaFitness->evaluateFitness(ecfGenotype);
        ret->setValue(fit);
        ecfGenotype.setFitness(fit);   // kind of useless as ecfGenotype is just a temporary container
        
        return ret;
    }


    
/**************************  ECF  *******************************************/
    
    ECF::ECF() 
    {
        ecfFitness = NULL;
        
        ECFState = StateP(new State);
        
    }

    ECF::~ECF() 
    {
        if (ecfFitness)
            delete ecfFitness;
    }

    
    bool ECF::run()
    {
        return ECFState->run();
    }

    void ECF::registerParameters()
    {
        GeneticAlgorithm::registerParameters();
        
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<std::string>("ga.ecf.configFilename", "NONE", "Filename of the config file for the ECF algorithm.");
        
        settings->registerCallbackFunction(callbackParameters, this);
    }

    void ECF::loadParameters()
    {
        GeneticAlgorithm::loadParameters();
        
//        Settings* settings = Settings::getInstance();
        ECFConfigFilename = settings->value<std::string>("ga.ecf.configFilename").second;
    }
    
    void ECF::callbackParameters(void* c)
    {
        ECF* inst = static_cast<ECF*>(c);
        inst->loadParameters();
    }
    
    void ECF::initialise()
    {
        GeneticAlgorithm::initialise();
        
        // We have the fitness function. We now need to convert it to an ECF fitness function
        ecfFitness = new ECFFitness(fitness);   // fitness comes from GeneticAlgorithm
        
        ECFState->setEvalOp(ecfFitness);
        
        if (ECFConfigFilename != "NONE") {
            char* argv[2];
            argv[0] = new char[1];
            argv[0][0] = 'a';
            argv[1] = new char[ECFConfigFilename.size()+1];
            sprintf(argv[1], "%s", ECFConfigFilename.c_str());
//            argv[1] = ECFConfigFilename.data();
            std::cout << "ECF: Loading config file " << ECFConfigFilename << std::endl;
            if (ECFState->initialize(2, argv)) {
                Color::Modifier cdefault(Color::FG_DEFAULT);
                Color::Modifier green(Color::FG_GREEN);
                std::cout << green <<  "ECF: Config file loaded." << cdefault << std::endl;
            }
            else {
                Color::Modifier cdefault(Color::FG_DEFAULT);
                Color::Modifier red(Color::FG_RED);
                std::cerr << red << "ECF: An error occurred during the initialization of the ECF framework." << cdefault << std::endl;
                exit(1);
            }
            
        }
        else {
            ECFState->initialize(0, (char**)NULL);
        }
    }
 
    void ECF::publish()
    {
        // Need to get all the genotypes and their associated fitness, create a Genotype to store it in.
        PopulationP pop = ECFState->getPopulation();
        DemeP individuals = pop->at(0);
        // Get rid of the old genotypes
        externalMemory->clear();
        
        for (size_t i=0; i<individuals->getSize(); ++i) {
            Genotype g;
            g = ((FloatingPoint::FloatingPoint*)(individuals->at(i)->getGenotype().get()))->realValue;
            if (individuals->at(i)->fitness)
                g.setFitness(individuals->at(i)->fitness->getValue());
            else
                g.setFitness(-1.0);
            externalMemory->push_back(g);
        }
    }
}
