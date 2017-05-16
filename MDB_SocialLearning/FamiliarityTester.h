/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FamiliarityTester.h
 * Author: Julien Hubert
 *
 * Created on March 16, 2017, 3:49 PM
 */

#ifndef FAMILIARITYTESTER_H
#define FAMILIARITYTESTER_H

//#include "ann/include/ANN/ANN.h"
#include <mlpack/methods/neighbor_search/neighbor_search.hpp>
#include "TraceMemory.h"
#include <vector>

namespace MDB_Social {

    class FamiliarityTester {
    private:
        arma::mat dataPts;
	arma::Mat<size_t> resultingNeighbors;
	arma::mat resultingDistances;
	arma::mat queryPt;
        mlpack::neighbor::AllkNN* knnSearch;

//        ANNpointArray dataPts; // data points
//        ANNpoint queryPt; // query point
//        ANNidxArray nnIdx; // near neighbor indices
//        ANNdistArray dists; // near neighbor distances
//        ANNkd_tree* kdTree; // search structure    
        
        unsigned maxPts;
        unsigned currentPts;
        unsigned dim;
        unsigned nearestCount;

    public:
        FamiliarityTester(unsigned _size, unsigned _dim, unsigned _nearestCount = 1);
        virtual ~FamiliarityTester();

        void setTrainingSet(TraceMemory* mem);
        void setTrainingSet(TraceMemory* mem, std::vector<bool>& mask);
        double getClosestNeighbour(Trace& trace);
    };
}

#endif /* FAMILIARITYTESTER_H */

