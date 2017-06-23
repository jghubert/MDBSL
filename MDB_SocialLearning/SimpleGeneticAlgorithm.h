/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SimpleGeneticAlgorithm.h
 * Author: Julien Hubert
 *
 * Created on October 6, 2016, 11:23 AM
 */

#ifndef SIMPLEGENETICALGORITHM_H
#define SIMPLEGENETICALGORITHM_H
#include <fstream>
#include <unordered_set>
#include "GeneticAlgorithm.hpp"
#include "RandomGenerators.h"

namespace MDB_Social {
    
    class SimpleGeneticAlgorithm;
    
    class SGAGenotype: public Genotype
    {
        friend class SimpleGeneticAlgorithm;
    protected:
        
    public:
        SGAGenotype();
        virtual ~SGAGenotype();
        
        void setSize(unsigned s);
        virtual size_t getSize() const override;
        virtual double& operator[] (unsigned index) override;
        void randomize();
        void reset();
        void mutate(double mproba=1.0, double mu=0.0, double std=0.0);
        void cross(SGAGenotype* g, unsigned npoints);
        
        SGAGenotype* clone();
        void copy(SGAGenotype* g);

        bool operator < (const SGAGenotype& b) const;
        bool operator > (const SGAGenotype& b) const;
        
    };
    
    class SimpleGeneticAlgorithm: public GeneticAlgorithm 
    {
    private:
        enum SelectionAlgorithm {
            TOURNAMENT = 0,
            ROULETTE_WHEEL = 1,
            UNKNOWN
        };
        
        UniformIntegerGenerator* uig;
        UniformGenerator* ug;

        
        std::vector<SGAGenotype*> population;
        std::vector<SGAGenotype*> newPopulation;
        
        unsigned populationSize;
        unsigned genomeSize;
        unsigned tournamentSize;
        unsigned crossoverSize;
        double geneMutationProbability;
        double mutationMean;
        double mutationStd;
        double reproductionProbability;
        unsigned maxGenerationCount;
        unsigned generationStep;
        SelectionAlgorithm selectionAlgorithm;
        unsigned elitism;
        
        //! Internal variable for Tournament Selection : Pool of remaining individuals
//        std::vector<unsigned> tourRandVect;
        std::unordered_multiset<unsigned> tourRandSet;
        //! Mating pool selected after Tournament Selection is applied
        std::vector<unsigned> tourMatingPool;
        //! Internal variable for Tournament Selection : Size of the pool of remaining individuals
        unsigned tourRandVectSize;
        //! Retain the last individual chosen as a parent for tournament selection
        unsigned tourLastChosen;
        
        unsigned currentGeneration;
        
        std::ofstream logfile;
        
        SelectionAlgorithm identifySelectionAlgorithm(const std::string& selection) const;
        void createPopulation();
                
        void mutatePopulation();
        void tournamentCreateMatingPool();
        void reproducePopulation();
        unsigned selectParent();
        void evaluatePopulation();
        static bool compareGenotypes(SGAGenotype* A, SGAGenotype* B);
        void sortPopulation();
        
        double meanPopulationFitness;
        double stdPopulationFitness;
        void computePopulationStatistics();
        void logStatistics();
        
        void refillTourRandSet();
        
    protected:
        void loadParameters() override;
        void resizePopulation(unsigned newsize);
        
    public:
        SimpleGeneticAlgorithm();
        virtual ~SimpleGeneticAlgorithm();

        bool run() override;

        void initialise() override;
        
        void registerParameters() override;

        void publish() override;
        
        void randomizePopulation();
        
        virtual void importGenotype(Genotype* ind, int destination) override;

        void reset();
                
    };

}
#endif /* SIMPLEGENETICALGORITHM_H */

