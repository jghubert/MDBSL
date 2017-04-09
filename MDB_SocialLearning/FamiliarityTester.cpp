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
        queryPt = annAllocPt(dim); // allocate query point
        dataPts = annAllocPts(maxPts, dim); // allocate data points
        nnIdx = new ANNidx[nearestCount]; // allocate near neigh indices
        dists = new ANNdist[nearestCount]; // allocate near neighbor dists   
        kdTree = NULL;
        currentPts = 0;
    }

    FamiliarityTester::~FamiliarityTester() 
    {
        delete[] nnIdx;
        delete[] dists;
        delete kdTree;
        annClose();
    }

    void FamiliarityTester::setTrainingSet(TraceMemory* mem)
    {
        currentPts = 0;
        if (kdTree)
            delete kdTree;
        
        // Read the data.
        auto itend = mem->end();
        for (auto it = mem->begin(); currentPts < maxPts && it != itend; ++it) {
            it->usedForVFTraining = true;
            for (unsigned i=0; i<dim; ++i)
                dataPts[currentPts][i] = it->inputs[i];
            currentPts++;
        }
        
        kdTree = new ANNkd_tree(dataPts, currentPts, dim);
    }

    void FamiliarityTester::setTrainingSet(TraceMemory* mem, std::vector<bool>& mask)
    {
        if (mask.size() == 0) {
            setTrainingSet(mem);
            return;
        }
        
        currentPts = 0;
        if (kdTree)
            delete kdTree;
        
        // Read the data.
        unsigned index = 0;
        auto itend = mem->end();
        for (auto it = mem->begin(); currentPts < maxPts && it != itend; ++it) {
            if (mask[index++]) {
                it->usedForVFTraining = true;
                for (unsigned i=0; i<dim; ++i)
                    dataPts[currentPts][i] = it->inputs[i];
                currentPts++;
            }
            else
                it->usedForVFTraining = false;
        }
        
        kdTree = new ANNkd_tree(dataPts, currentPts, dim);
    }

    
    double FamiliarityTester::getClosestNeighbour(Trace& trace)
    {
        for (unsigned i=0; i<dim; ++i)
            queryPt[i] = trace.inputs[i];
        
        kdTree->annkSearch(queryPt, nearestCount, nnIdx, dists);
        return dists[0];
    }
    
}