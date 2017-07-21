/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <iostream>
#include <algorithm>
#include "GeneticAlgorithm.hpp"
#include "Settings.h"
#include "FitnessLibrary.hpp"
#include "SimulatorLibrary.hpp"
#include "TraceMemory.h"
#include "ResourceLibrary.hpp"
#include "RobotID.h"

namespace MDB_Social {

    Genotype& Genotype::operator= (std::vector<double>& V)
    {
        genes.resize(V.size(), 0.0);
        std::copy(V.begin(), V.end(), std::begin(genes));
        return *this;
    }

    Genotype& Genotype::operator= (Genotype& G)
    {
        genes = G.genes;
        fitness = G.fitness;
        id = G.id;
        return *this;
    }
    
//    std::ostream& operator<<(std::ostream &output, const Genotype& G)

    
    Simulator* GAFitness::getSimulator(std::string sim)
    {
        Simulator* ret = SimulatorLibrary::getSimulator(sim);
//        ret->setID(robotid);
        return ret;
    }

    
    GeneticAlgorithm::GeneticAlgorithm()
    {
        fitness = NULL;
        experiment = "NONE";
        
        settings = RobotID::getSettings();
        resourceLibrary = RobotID::getResourceLibrary();
    }
        
    GeneticAlgorithm::~GeneticAlgorithm()
    {
        if (fitness)
            delete fitness;
    }
    
    void GeneticAlgorithm::registerParameters()
    {
        std::cout << "GeneticAlgorithm : registerParameters." << std::endl;
//        Settings* settings = Settings::getInstance();
//        settings->registerParameter<std::string>("ga.experiment", std::string("NONE"), "Name of the experiment to run.");
        if (settings->registerAndRetrieveParameter<std::string>(experiment, "ga.experiment", std::string("NONE"), "Name of the experiment to run.")) {
            std::cout << "GeneticAlgorithm - Loading experiment " << experiment << std::endl;
            if (experiment != "NONE") {
                fitness = FitnessLibrary::getFitness(experiment);
                if (!fitness) {
                    std::cerr << "GeneticAlgorithm - Error: Experiment " << experiment << " unknown." << std::endl;
                    exit(1);
                }
                else {
                    resourceLibrary->setGAFitness(fitness);
                    std::cout << "GeneticAlgorithm - Loaded experiment " << experiment << std::endl;
                }
            }
            
        }
        else 
            std::cerr << "GeneticAlgorithm::registerParameters: Failed to find ga.experiment" << std::endl;
        
        settings->registerCallbackFunction(this->callbackParameters, this);
        
    }

    void GeneticAlgorithm::initialise()
    {
        if (experiment != "NONE" && !fitness) {
            fitness = FitnessLibrary::getFitness(experiment);
            if (!fitness) {
                std::cerr << "GeneticAlgorithm - Error: Experiment " << experiment << " unknown." << std::endl;
                exit(1);
            }
            else {
                resourceLibrary->setGAFitness(fitness);
            }
        }
        else if (fitness)
            fitness->loadParameters();
    }

    void GeneticAlgorithm::callbackParameters(void* c)
    {
        GeneticAlgorithm* inst = static_cast<GeneticAlgorithm*>(c);
        inst->loadParameters();
    }
    
    void GeneticAlgorithm::setFitness(GAFitness* g)
    {
        fitness = g;
    }

    void GeneticAlgorithm::loadParameters() 
    {
//        Settings* settings = Settings::getInstance();
        experiment = settings->value<std::string>("ga.experiment").second;
        
    }
    
    void GeneticAlgorithm::setExternalMemory(Memory<Genotype>* mem)
    {
        externalMemory = mem;
    }

    GAFitness* GeneticAlgorithm::getFitnessFunction()
    {
        return fitness;
    }
    
    void GeneticAlgorithm::importGenotype(Genotype* ind, int destination)
    {
        std::cerr << "GeneticAlgorithm: ERROR: importGenotype has not been implemented for this genetic algorithm." << std::endl;
        exit(1);
    }
    
   void GeneticAlgorithm::setWorkingDirectory(const char* cwd)
    {
        workingDirectory = std::string(cwd);
    }
   
    void GeneticAlgorithm::startRecordingTraces(Genotype* ind)
    {
        TraceMemory* tm = resourceLibrary->getTraceMemory();
        if (tm)
            tm->setDefaultUUID(ind->getUUID());
    }
    
    void GeneticAlgorithm::stopRecordingTraces()
    {
        TraceMemory* tm = resourceLibrary->getTraceMemory();
        if (tm)
            tm->resetDefaultUUID();
    }
   
}

