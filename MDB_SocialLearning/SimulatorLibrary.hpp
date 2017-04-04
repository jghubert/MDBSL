/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SimulatorLibrary.hpp
 * Author: Julien Hubert
 *
 * Created on August 14, 2016, 11:26 AM
 */

#ifndef SIMULATORLIBRARY_HPP
#define SIMULATORLIBRARY_HPP

#include <string>
#include "Simulator.hpp"
#include "FastSimSimulator.h"

namespace MDB_Social {

    class SimulatorLibrary {
    private:
        static FastSimSimulator* getFastSim() {
            return new FastSimSimulator();
        }

    public:
        static Simulator* getSimulator(std::string sim) {
            if (sim == "FastSim")
                return getFastSim();
            else
                return NULL;
        }

    };

}

#endif /* SIMULATORLIBRARY_HPP */

