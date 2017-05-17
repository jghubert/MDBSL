#ifndef FITNESSLIBRARY_HPP
#define FITNESSLIBRARY_HPP

#include <string>
#include "GeneticAlgorithm.hpp"
#include "../Experiments/FastSim_Phototaxis_Compass.h"
#include "../Experiments/FastSim_MultiRobot_Phototaxis_Compass.h"
#include "../Experiments/FastSim_Forage_Wall.h"

namespace MDB_Social {

	 class FitnessLibrary {
	 private:
		 static GAFitness* getFastSim_Phototaxis_Compass(std::string id) {
			 return new FastSim_Phototaxis_Compass(id);
		}
		 static GAFitness* getFastSim_MultiRobot_Phototaxis_Compass(std::string id) {
			 return new FastSim_MultiRobot_Phototaxis_Compass(id);
		}
		 static GAFitness* getFastSim_Forage_Wall(std::string id) {
			 return new FastSim_Forage_Wall(id);
		}

	 public:
		 static GAFitness* getFitness(std::string fit, std::string id="Default") {
			 if (fit == "FastSim_Phototaxis_Compass")
				 return getFastSim_Phototaxis_Compass(id);
			 else if (fit == "FastSim_MultiRobot_Phototaxis_Compass")
				 return getFastSim_MultiRobot_Phototaxis_Compass(id);
			 else if (fit == "FastSim_Forage_Wall")
				 return getFastSim_Forage_Wall(id);
			 else
				 return NULL;
		 }
	 };
}
#endif

