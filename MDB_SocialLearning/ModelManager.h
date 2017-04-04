/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ModelManager.h
 * Author: Julien Hubert
 *
 * Created on June 16, 2016, 2:47 PM
 */

#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "Model.hpp"

namespace MDB_Social {
    
    class ModelManager {
    private:
        Model* current_model;
        bool use_ga;
    public:
        ModelManager(bool _use_ga = false);        
        ~ModelManager();
        
        
    };
    
    
}

#endif /* MODELMANAGER_H */
