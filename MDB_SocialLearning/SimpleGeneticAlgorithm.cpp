/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SimpleGeneticAlgorithm.cpp
 * Author: Julien Hubert
 * 
 * Created on October 6, 2016, 11:23 AM
 */

#include "SimpleGeneticAlgorithm.h"
#include "Settings.h"
#include "RandomGenerators.h"
#include "ResourceLibrary.hpp"
#include <string>

namespace MDB_Social {

    SGAGenotype::SGAGenotype()
    {

    }
    
    SGAGenotype::~SGAGenotype()
    {
        
    }
        
    void SGAGenotype::setSize(unsigned s)
    {
        genes.resize(s);
    }
    
    size_t SGAGenotype::getSize() const
    {
        return genes.size();
    }

    double& SGAGenotype::operator[] (unsigned index)
    {
        return genes[index];
    }

    bool SGAGenotype::operator < (const SGAGenotype& b) const
    {
        return fitness < b.fitness;
    }
    
    bool SGAGenotype::operator > (const SGAGenotype& b) const
    {
        return fitness > b.fitness;
    }
    
    void SGAGenotype::randomize()
    {
        UniformGenerator* ug = RobotID::getResourceLibrary()->getUniformGenerator();
        for (size_t i=0; i < genes.size(); ++i)
            genes[i] = (*ug)();  // number within [0; 1]
    }
    
    void SGAGenotype::reset()
    {
        randomize();
        fitness = 0.0;
    }
    
    SGAGenotype* SGAGenotype::clone()
    {
        SGAGenotype* ret = new SGAGenotype();
        
        *ret = *this;
        return ret;
        
    }

    void SGAGenotype::copy(SGAGenotype* g)
    {
        *this = *g;
    }

    void SGAGenotype::mutate(double mproba, double mu, double std)
    {
        GaussianGenerator* gg = RobotID::getResourceLibrary()->getGaussianGenerator();
        UniformGenerator* ug = RobotID::getResourceLibrary()->getUniformGenerator();

        double tmp;
        for (size_t i=0; i<genes.size(); ++i) {
            if ((*ug)() < mproba) {
                tmp = genes[i] + mu+(*gg)()*std;
                if (tmp > 1.0)
                    tmp = 1.0;
                else if (tmp < 0.0)
                    tmp = 0.0;
                genes[i] = tmp;
            }
        }
        
//        delete gg;
//        delete ug;
    }

    void SGAGenotype::cross(SGAGenotype* g, unsigned npoints)
    {
        UniformIntegerGenerator* uig = RobotID::getResourceLibrary()->getUniformIntegerGenerator();
        
        // First generate the indexes of the crossovers
        std::vector<unsigned> cpoints(npoints+2,0);

        unsigned tmp;
        bool found = false;
        for (unsigned i=1; i <= npoints; ++i) {
            do {
                found = false;
                tmp = (*uig)() % genes.size();
                for (unsigned j=0; j<i && !found; ++j)
                    found = (tmp == cpoints[j]);
            }
            while (found);

            cpoints[i] = tmp;
        }
        cpoints[npoints+1] = genes.size();
        
        
        for (unsigned i=1; i < npoints; ++i)
            cpoints[i] = (*uig)();
        cpoints[npoints+1] = genes.size();
        std::sort(cpoints.begin(), cpoints.end());
        
        unsigned first = (*uig)()%2;  // 0 = g starts; 1 = this starts
        for (unsigned i=first; i<npoints+first; i+=2)
            std::copy(std::begin(g->genes)+cpoints[i], std::begin(g->genes)+cpoints[i+1], std::begin(this->genes)+cpoints[i]);
            
//        delete uig;
    }
    
    
    SimpleGeneticAlgorithm::SimpleGeneticAlgorithm()
    {
        elitism = 0;
        currentGeneration = 0;
        uig = resourceLibrary->getUniformIntegerGenerator();
        ug = resourceLibrary->getUniformGenerator();  
        generationStep = 1;
    }


    SimpleGeneticAlgorithm::~SimpleGeneticAlgorithm() 
    {
        for (unsigned i=0; i<populationSize; ++i) {
            delete population[i];
            population[i] = NULL;
            delete newPopulation[i];
            newPopulation[i] = NULL;
        }
        
        delete uig;
        delete ug;
    }
    
    bool SimpleGeneticAlgorithm::run()
    {
        if (!fitness) {
            std::cerr << "SimpleGeneticAlgorithm: ERROR : Fitness function not provided." << std::endl;
            exit(1);
        }

        std::string logname = workingDirectory + "/SimpleGeneticAlgorithm.log";
        logfile.open(logname, std::ios_base::app);
        if (!logfile.is_open()) {
            std::cerr << "SimpleGeneticAlgorithm: ERROR: Impossible to open the log file " << logname << std::endl;
            exit(1);
        }

        if (logPhylogeneticTree) {
            logname = workingDirectory + "/SimpleGeneticAlgorithm.PhylogeneticTree.log";
            phylogeneticFile.open(logname, std::ios_base::app);
            if (!phylogeneticFile.is_open()) {
                std::cerr << "SimpleGeneticAlgorithm: ERROR: Impossible to open the phylogenetic log file " << logname << std::endl;
                exit(1);
            }
        }

        
        std::cout << "Starting evolution for " << maxGenerationCount << " generations." << std::endl;
        std::cout << "    Current generations :";
        
        unsigned currentMaxGeneration = currentGeneration + maxGenerationCount;
        while (currentGeneration < currentMaxGeneration) {
            
            if (currentGeneration > 0)
                reproducePopulation();
                
            std::cout << currentGeneration << " ";
            std::cout.flush();
//            fitness->preprocessing(); // Moved that to Manager level
            evaluatePopulation();
//            fitness->postprocessing();
            
            sortPopulation();


//            std::cout << "Population fitness: ";
//            for (unsigned i=0; i<population.size(); ++i)
//                std::cout << population[i]->getFitness() << " ";
//            std::cout << std::endl;
            
            computePopulationStatistics();
            logStatistics();
            
            currentGeneration++;

//            if (currentGeneration < maxGenerationCount)
//                reproducePopulation();
            
        }
        std::cout << "DONE" << std::endl;
        
        logfile.close();
        if (logPhylogeneticTree)
            phylogeneticFile.close();
        
        return currentGeneration == maxGenerationCount;
    }

    void SimpleGeneticAlgorithm::evaluatePopulation()
    {
        for (unsigned i=0; i<populationSize; ++i) {
            Genotype* g = static_cast<Genotype*>(population[i]);
            startRecordingTraces(g);
            double fit = fitness->evaluateFitness(*g, currentGeneration, i, false);
            population[i]->setFitness(fit);
            stopRecordingTraces();
//            std::cout << "SimpleGeneticAlgorithm:: Fitness from population = " << population[i]->getFitness() << " ; fitness from evaluation = " << fit << std::endl;
        }
    }

    bool SimpleGeneticAlgorithm::compareGenotypes(SGAGenotype* A, SGAGenotype* B)
    {
        return A->getFitness() > B->getFitness();
    }

    
    void SimpleGeneticAlgorithm::sortPopulation()
    {
        std::sort(population.begin(), population.end(), compareGenotypes);
    }
    
    void SimpleGeneticAlgorithm::createPopulation()
    {
        population.resize(populationSize, NULL);
        newPopulation.resize(populationSize, NULL);
        for (unsigned i=0; i<populationSize; ++i) {
            population[i] = new SGAGenotype();
            population[i]->setSize(genomeSize);
            newPopulation[i] = new SGAGenotype();
            newPopulation[i]->setSize(genomeSize);
        }
        randomizePopulation();
    }
    

    void SimpleGeneticAlgorithm::initialise()
    {
        loadParameters();

        GeneticAlgorithm::initialise();

        // Tournament selection initialization
        tourRandVectSize = populationSize*2;
//        tourRandVect.resize(tourRandVectSize);
        tourMatingPool.resize(populationSize);
//        for (unsigned i=0; i<populationSize; ++i) 
//        {
//          tourRandVect[i*2] = i;
//          tourRandVect[i*2+1] = i;
//        }

        createPopulation();
        
        
    }
    
    void SimpleGeneticAlgorithm::randomizePopulation()
    {
        for (size_t i=0; i<populationSize; ++i) {
//            if (getID() == "RobotA") {
//                for (unsigned g=0; g< population[i]->getSize(); ++g)
//                    (*population[i])[g] = 0.1;
//            }
//            else
//                for (unsigned g=0; g< population[i]->getSize(); ++g)
//                    (*population[i])[g] = 0.7;
//        }
            population[i]->randomize();
        }
    }

    void SimpleGeneticAlgorithm::reset()
    {
        for (size_t i=0; i<populationSize; ++i)
            population[i]->reset();
        
        currentGeneration = 0;
    }
    
    void SimpleGeneticAlgorithm::mutatePopulation()
    {
        for (unsigned i=elitism; i<populationSize; ++i)
            population[i]->mutate(geneMutationProbability, mutationMean, mutationStd);
    }

    void SimpleGeneticAlgorithm::tournamentCreateMatingPool()
    {
//        unsigned ts = tourRandVectSize;
        unsigned p1,p2;
        unsigned index = 0;

        refillTourRandSet();
        std::unordered_multiset<unsigned>::iterator itp1, itp2;
        
        for (unsigned i=0;i<populationSize;++i)
        {
//            if (ts>2) 
            if (tourRandSet.size() >2) 
            {
//                p1 = (*uig)() % ts;
                p1 = (*uig)() % tourRandSet.size();
                itp1 = tourRandSet.begin();
                std::advance(itp1, p1);
                do {
                    p2 = (*uig)() % tourRandSet.size();
                    itp2 = tourRandSet.begin();
                    std::advance(itp2, p2);
//                  p2 = (*uig)() % ts;
                } while (*itp1 == *itp2);
//                } while (tourRandVect[p1] == tourRandVect[p2]);
            }
            else
            {
//                p1 = 0;
//                p2 = 1;
                itp1 = tourRandSet.begin();
                itp2 = tourRandSet.begin();
                itp2++;
            }

            if (population[*itp1]->fitness > population[*itp2]->fitness) {
                tourMatingPool[index++] = *itp1;
                tourRandSet.erase(itp1);
            }
            else {
                tourMatingPool[index++] = *itp2;
                tourRandSet.erase(itp2);
            }
//            if (population[tourRandVect[p1]]->fitness > population[tourRandVect[p2]]->fitness)
//                tourMatingPool[index++] = tourRandVect[p1];
//            else
//                tourMatingPool[index++] = tourRandVect[p2];

//            std::cout << "p1 = " << p1 << "; p2 = " << p2 << "; Added to the mating pool = " << tourMatingPool[index-1] << std::endl;

            
//            unsigned p1tmp = tourRandVect[p1];
//            unsigned p2tmp = tourRandVect[p2];
//            tourRandVect[p1] = tourRandVect[ts-1];
//            tourRandVect[p2] = tourRandVect[ts-2];
//            tourRandVect[ts-1] = p1tmp;
//            tourRandVect[ts-2] = p2tmp;
//            ts -= 2;

//        std::cout << "post swap : tourRandVect = ";
//        for (unsigned i=0; i<tourRandVect.size(); ++i)
//            std::cout << tourRandVect[i] << " ";
//        std::cout << std::endl;
//        std::cout << "post erase : tourRandSet = ";
//        for (auto it=tourRandSet.begin(); it != tourRandSet.end(); ++it)
//            std::cout << *it << " ";
//        std::cout << std::endl;

//        std::cout << "tourMatingPool = ";
//        for (unsigned i=0; i<tourMatingPool.size(); ++i)
//            std::cout << tourMatingPool[i] << " ";
//        std::cout << std::endl;
        }

    }
    
    unsigned SimpleGeneticAlgorithm::selectParent()
    {
        switch (selectionAlgorithm) {
            case TOURNAMENT:
                return tourMatingPool[(*uig)()%populationSize];
                break;
                
            case ROULETTE_WHEEL:

                break;
                
            case UNKNOWN:
            default:
                std::cerr << "SimpleGeneticAlgorithm: ERROR : Unknown selection algorithm." << std::endl;
                break;
        }
        return 0;
    }
    
    
    void SimpleGeneticAlgorithm::reproducePopulation()
    {
//        std::vector<SGAGenotype*> newPopulation(populationSize, NULL);
        
        for (unsigned i=0; i<elitism; ++i) {
            newPopulation[i]->copy(population[i]);
            newPopulation[i]->regenerateUUID();
            if (logPhylogeneticTree)
                phylogeneticFile << currentGeneration << " " << newPopulation[i]->getUUID() << std::endl;
        }
//            newPopulation[i] = population[i];
        
        switch (selectionAlgorithm) {
            case TOURNAMENT:
                tournamentCreateMatingPool();
                break;
                
            case ROULETTE_WHEEL:
                
                break;
                
            case UNKNOWN:
            default:
                std::cerr << "SimpleGeneticAlgorithm: ERROR : Unknown selection algorithm." << std::endl;
                break;
        }
        
        unsigned mother, father;
        for (unsigned i=elitism; i < populationSize; ++i) {
            mother = selectParent();
            
//            newPopulation[i] = population[mother]->clone();
            newPopulation[i]->copy(population[mother]);
            newPopulation[i]->regenerateUUID();

            if (logPhylogeneticTree)
                phylogeneticFile << currentGeneration << " " << newPopulation[i]->getUUID() << " " << population[mother]->getUUID();

            if ((*ug)() < reproductionProbability) {
                do
                    father = selectParent();
                while (father == mother);

                newPopulation[i]->cross(population[father], crossoverSize);
                if (logPhylogeneticTree)
                    phylogeneticFile << " " << population[father]->getUUID();
            }
            if (logPhylogeneticTree)
                phylogeneticFile << std::endl;
        }
        
        // delete the old individuals and replace them with the new ones
//        for (unsigned i=elitism; i < populationSize; ++i) {
//            delete population[i];
//            population[i] = newPopulation[i];
//        }
        population.swap(newPopulation);
        
        mutatePopulation();
    }
    
    void SimpleGeneticAlgorithm::registerParameters()
    {
        GeneticAlgorithm::registerParameters();
        
//        Settings* settings = Settings::getInstance();
        
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.generationCount", 0, "SimpleGeneticAlgorithm: Amount of generations for the experiment.");
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.generationStep", 1, "SimpleGeneticAlgorithm: Number of generations between publication of the genotypes.");
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.populationSize", 0, "SimpleGeneticAlgorithm: Size of the population.");
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.genomeSize", 0, "SimpleGeneticAlgorithm: Size of the genome.");
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.tournamentSize", 2, "SimpleGeneticAlgorithm: Size of the tournament used for tournament selection.");
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.crossoverSize", 0, "SimpleGeneticAlgorithm: Number of crossover points for the crossover operator.");
        settings->registerParameter<double>("SimpleGeneticAlgorithm.geneMutationProbability", 1.0, "SimpleGeneticAlgorithm: Probability that a gene will be mutated.");
        settings->registerParameter<double>("SimpleGeneticAlgorithm.mutationMean", 0.0, "SimpleGeneticAlgorithm: Mean of the Gaussian distribution used for mutation.");
        settings->registerParameter<double>("SimpleGeneticAlgorithm.mutationStd", 0.1, "SimpleGeneticAlgorithm: STD of the Gaussian distribution used for mutation.");
        settings->registerParameter<double>("SimpleGeneticAlgorithm.reproductionProbability", 1.0, "SimpleGeneticAlgorithm: Probability of using sexual reproduction.");
        settings->registerParameter<unsigned>("SimpleGeneticAlgorithm.elitism", 0, "SimpleGeneticAlgorithm: Number of individuals to copy to the next generation.");
        settings->registerParameter<std::string>("SimpleGeneticAlgorithm.selectionAlgorithm", std::string("tournament"), "SimpleGeneticAlgorithm: Algorithm for the selection (tournament, roulette_wheel.");
        settings->registerParameter<bool>("SimpleGeneticAlgorithm.logPhylogeneticTree", false, "SimpleGeneticAlgorithm: log the phylogenetic tree of the evolution.");
    }
    
    void SimpleGeneticAlgorithm::loadParameters()
    {
        GeneticAlgorithm::loadParameters();
        
//        Settings* settings = Settings::getInstance();

        maxGenerationCount = settings->value<unsigned>("SimpleGeneticAlgorithm.generationCount").second;
        generationStep = settings->value<unsigned>("SimpleGeneticAlgorithm.generationStep").second;
        populationSize = settings->value<unsigned>("SimpleGeneticAlgorithm.populationSize").second;
        genomeSize = settings->value<unsigned>("SimpleGeneticAlgorithm.genomeSize").second;
        tournamentSize = settings->value<unsigned>("SimpleGeneticAlgorithm.tournamentSize").second;
        crossoverSize = settings->value<unsigned>("SimpleGeneticAlgorithm.crossoverSize").second;
        geneMutationProbability = settings->value<double>("SimpleGeneticAlgorithm.geneMutationProbability").second;
        mutationMean = settings->value<double>("SimpleGeneticAlgorithm.mutationMean").second;
        mutationStd = settings->value<double>("SimpleGeneticAlgorithm.mutationStd").second;
        
        reproductionProbability = settings->value<double>("SimpleGeneticAlgorithm.reproductionProbability").second;
        elitism = settings->value<unsigned>("SimpleGeneticAlgorithm.elitism").second;
        std::string selection = settings->value<std::string>("SimpleGeneticAlgorithm.selectionAlgorithm").second;
        selectionAlgorithm = identifySelectionAlgorithm(selection);
        logPhylogeneticTree = settings->value<bool>("SimpleGeneticAlgorithm.logPhylogeneticTree").second;
    }

    SimpleGeneticAlgorithm::SelectionAlgorithm SimpleGeneticAlgorithm::identifySelectionAlgorithm(const std::string& selection) const
    {
        SelectionAlgorithm ret;
        if (selection == "tournament")
            ret = SelectionAlgorithm::TOURNAMENT;
        else if (selection == "roulette_wheel")
            ret = SelectionAlgorithm::ROULETTE_WHEEL;
        else {
            ret = SelectionAlgorithm::UNKNOWN;
            std::cerr << "SimpleGeneticAlgorithm: ERROR : Unknown selection algorithm: " << selection << std::endl;
            exit(1);
        }
        
        return ret;
    }

    
    void SimpleGeneticAlgorithm::publish()
    {
        externalMemory->clear();
        
        for (unsigned i=0; i<populationSize; ++i) {
            externalMemory->push_back(*population[i]);
        }
    }

    void SimpleGeneticAlgorithm::computePopulationStatistics()
    {
        meanPopulationFitness = 0.0;
        stdPopulationFitness = 0.0;
        
        for (unsigned i=0; i<populationSize; ++i)
            meanPopulationFitness += population[i]->getFitness();
        meanPopulationFitness /= 1.0*populationSize;
        
        for (unsigned i=0; i<populationSize; ++i)
            stdPopulationFitness += pow(population[i]->getFitness() - meanPopulationFitness, 2.0);
        stdPopulationFitness = sqrt(stdPopulationFitness / (1.0*(populationSize-1)));
    }

    void SimpleGeneticAlgorithm::logStatistics()
    {
        logfile << currentGeneration << " " << population[0]->getFitness() << " " << population[populationSize-1]->getFitness() 
                << " " << meanPopulationFitness << " " << stdPopulationFitness << std::endl;
    }
 
    
    void SimpleGeneticAlgorithm::resizePopulation(unsigned newsize)
    {
        std::cerr << "SimpleGeneticAlgorithm::resizePopulation not yet implemented." << std::endl;
        exit(2);
    }
    
    
    void SimpleGeneticAlgorithm::importGenotype(Genotype* ind, int destination)
    {
        if (destination < 0) {  // Append the new genotype, expanding the size of the population.
            SGAGenotype* tmp = new SGAGenotype();
            *tmp = *(SGAGenotype*)ind;
            population.push_back(tmp);
            resizePopulation(populationSize+1);
        }
        else {
            *population[destination] = *(SGAGenotype*)ind;
        }
    }
 
    void SimpleGeneticAlgorithm::refillTourRandSet()
    {
        tourRandSet.clear();
        for (unsigned i=0; i<populationSize; ++i) 
        {
            tourRandSet.insert(i);
            tourRandSet.insert(i);
        }
    }
       
}