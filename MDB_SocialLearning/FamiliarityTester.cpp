/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FamiliarityTester.cpp
 * Author: Julien Hubert
 * 
 * Created on March 16, 2017, 3:49 PM
 */

#include "FamiliarityTester.h"

namespace MDB_Social {

    FamiliarityTester::FamiliarityTester(unsigned _maxPts, unsigned _dim, unsigned _nearestCount) 
        : maxPts(_maxPts), dim(_dim), nearestCount(_nearestCount)
    {
        queryPt.resize(dim,1);
        dataPts.resize(dim, maxPts);
        knnSearch = new mlpack::neighbor::AllkNN(mlpack::neighbor::GREEDY_SINGLE_TREE_MODE, 0.1);
        
        currentPts = 0;
    }

    FamiliarityTester::~FamiliarityTester() 
    {
        delete knnSearch;
    }

    void FamiliarityTester::setTrainingSet(TraceMemory* mem)
    {
        currentPts = 0;
        
        // Read the data.
        auto itend = mem->end();
        for (auto it = mem->begin(); currentPts < maxPts && it != itend; ++it) {
            it->usedForVFTraining = true;
            for (unsigned i=0; i<dim; ++i)
                dataPts(i,currentPts) = it->inputs[i];
            currentPts++;
        }
        
        knnSearch->Train(dataPts.cols(0,currentPts-1));
//        kdTree = new ANNkd_tree(dataPts, currentPts, dim);
    }

    void FamiliarityTester::setTrainingSet(TraceMemory* mem, std::vector<bool>& mask)
    {
        if (mask.size() == 0) {
            setTrainingSet(mem);
            return;
        }
        
        currentPts = 0;
        
        // Read the data.
        unsigned index = 0;
        auto itend = mem->end();
        for (auto it = mem->begin(); currentPts < maxPts && it != itend; ++it) {
            if (mask[index++]) {
                it->usedForVFTraining = true;
                for (unsigned i=0; i<dim; ++i)
                    dataPts(i,currentPts) = it->inputs[i];
                currentPts++;
            }
            else
                it->usedForVFTraining = false;
        }
        
        knnSearch->Train(dataPts.cols(0,currentPts-1));
    }

    
    double FamiliarityTester::getClosestNeighbour(Trace& trace)
    {
        for (unsigned i=0; i<dim; ++i)
            queryPt(i,0) = trace.inputs[i];
        
        knnSearch->Search(queryPt, 1, resultingNeighbors, resultingDistances);
//        kdTree->annkSearch(queryPt, nearestCount, nnIdx, dists);
        return resultingDistances(0,0);
//        return dists[0];
    }
    
}