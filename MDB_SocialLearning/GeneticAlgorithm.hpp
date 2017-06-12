/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GeneticAlgorithm.hpp
 * Author: Julien Hubert
 *
 * Created on August 4, 2016, 2:58 PM
 */

#ifndef GENETICALGORITHM_HPP
#define GENETICALGORITHM_HPP
#include <valarray>
#include <vector>
#include <string>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include "Memory.hpp"
#include "Settings.h"
#include "RobotID.h"

namespace MDB_Social {

    
    class Genotype {
    private:
    	const boost::uuids::uuid id;

    protected:
        std::valarray<double> genes;
        double fitness;
        
    public:
        Genotype() :
        	id(boost::uuids::random_generator()())
    		{fitness = 0.0;};
        virtual ~Genotype() {};
        
        virtual size_t getSize() const {return genes.size();};
        virtual double operator[] (unsigned index) {return genes[index];};
        virtual void setFitness(double _fit) {fitness = _fit;}
        virtual double getFitness() const {return fitness;}

        const boost::uuids::uuid& getID() const {return id;}
        
        Genotype& operator= (std::vector<double>& V);
        Genotype& operator= (Genotype& G);
        friend std::ostream& operator<<(std::ostream &output, const Genotype& G)
        {
            output << G.fitness << SEPARATOR << G.genes.size();
            for (size_t i=0; i<G.genes.size(); ++i)
                output << SEPARATOR << G.genes[i];
            return output;
        }
        
        friend std::istream& operator>>(std::istream& input, Genotype& G)
        {
            input >> G.fitness;
            if (!input.good())
                return input;

            size_t s;
            input >> s;
            if (input.good())
                G.genes.resize(s);
            else
                return input;
            
            unsigned index = 0;
            while (index < G.genes.size() && input.good())
                input >> G.genes[index++];
            
            return input;
        }
        

    };
    
    class Simulator;
    
    class GAFitness: public RobotID {
    protected:
        bool recommendBabbling;
        
    public:
        GAFitness(std::string id="Default") : RobotID(id) {recommendBabbling = false;}
        virtual ~GAFitness() {}

        virtual void preprocessing() {};    // Called before testing the current generation (SL specific, apparently)
        virtual void postprocessing() {};   // Called after testing the current generation
        
        virtual double evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool testing=false)=0;

        /// Called immediately before starting evaluations of the current generation
        virtual void prepareEvaluation(void) {};
        /// Any postprocessing after all individuals have been evaluated, but before sorting the population
        virtual void postProcessIndividual(Genotype& individual) {};

        virtual void loadParameters()=0;
        
        virtual void setBabblingRecommendation(bool b) {
            recommendBabbling = b;
        }
        
        Simulator* getSimulator(std::string sim);
    };
    
    class GeneticAlgorithm: public RobotID {
    protected:
        virtual void loadParameters();

        GAFitness* fitness;
        std::string experiment;
        std::string workingDirectory;
        
        Memory<Genotype>* externalMemory;
        
    public:
        GeneticAlgorithm();
        virtual ~GeneticAlgorithm();
        
        virtual bool run()=0;
        
        virtual void registerParameters();
        
        virtual void initialise();
        
        virtual void setFitness(GAFitness*);
        
        static void callbackParameters(void*);
        
        virtual void publish()=0;
        
        virtual void setExternalMemory(Memory<Genotype>* mem);
        
        virtual GAFitness* getFitnessFunction();

        virtual void setID(std::string& _id);
        
        virtual void importGenotype(Genotype* ind, int destination);

        virtual void setWorkingDirectory(const char* cwd);
    };
    
}


#endif /* GENETICALGORITHM_HPP */

