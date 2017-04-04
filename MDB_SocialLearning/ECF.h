/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ECF.h
 * Author: Julien Hubert
 *
 * Created on August 5, 2016, 11:20 AM
 */

#ifndef ECF_H
#define ECF_H

#include "GeneticAlgorithm.hpp"
#include <ecf/ECF.h>

namespace MDB_Social {

    class ECFGenotype: public Genotype
    {
    private:
        FloatingPoint::FloatingPoint* ecfgen;
        
    public:
        ECFGenotype();
        virtual ~ECFGenotype();
        
        void setGenotype(FloatingPoint::FloatingPoint* p);
        virtual size_t getSize() const override;
        virtual double operator[] (unsigned index) override;
        
    };
    
    class ECFFitness: public EvaluateOp {
    private:
        GAFitness* gaFitness;
        ECFGenotype ecfGenotype;

    public:
        ECFFitness();
        ECFFitness(GAFitness*);
        virtual ~ECFFitness();
        
        virtual FitnessP evaluate(IndividualP individual);
    };
    
    class ECF: public GeneticAlgorithm {
    private:
        std::string ECFConfigFilename;
        
        StateP ECFState;
        
        ECFFitness* ecfFitness;
        
    protected:
        virtual void loadParameters() override;
        
    public:
        ECF();
        virtual ~ECF();

        virtual bool run() override;

        virtual void registerParameters() override;
        
        void initialise() override;

        static void callbackParameters(void*);
        
        virtual void publish() override;

    };

}

#endif /* ECF_H */

