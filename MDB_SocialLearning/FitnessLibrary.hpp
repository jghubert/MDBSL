#ifndef FITNESSLIBRARY_HPP
#define FITNESSLIBRARY_HPP

#include <string>
#include "GeneticAlgorithm.hpp"
#include "../Experiments/FastSim_MultiRobot_WallAvoidance.h"
#include "../Experiments/FastSim_Phototaxis_Compass.h"
#include "../Experiments/FastSim_Diversity_Compass.h"

namespace MDB_Social {

	 class FitnessLibrary {
	 private:
		 static GAFitness* getFastSim_MultiRobot_WallAvoidance(std::string id) {
			 return new FastSim_MultiRobot_WallAvoidance(id);
		}
		 static GAFitness* getFastSim_Phototaxis_Compass(std::string id) {
			 return new FastSim_Phototaxis_Compass(id);
		}
		 static GAFitness* getFastSim_Diversity_Compass(std::string id) {
			 return new FastSim_Diversity_Compass(id);
		}

	 public:
		 static GAFitness* getFitness(std::string fit, std::string id="Default") {
			 if (fit == "FastSim_MultiRobot_WallAvoidance")
				 return getFastSim_MultiRobot_WallAvoidance(id);
			 else if (fit == "FastSim_Phototaxis_Compass")
				 return getFastSim_Phototaxis_Compass(id);
			 else if (fit == "FastSim_Diversity_Compass")
				 return getFastSim_Diversity_Compass(id);
			 else
				 return NULL;
		 }
	 };
}
#endif

