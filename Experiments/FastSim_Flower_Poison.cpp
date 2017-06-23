/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Flower_Poison.cpp
 * Author: Julien Hubert
 * 
 * Created on May 16, 2017, 14:10 PM
 */

#include "../MDB_SocialLearning/Settings.h"
#include "../MDB_SocialLearning/SocialManagerClient.h"
#include "FastSim_Flower_Poison.h"
#include <cstdlib>
#include <algorithm>
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/ValueFunction.h"
#include "RobotID.h"

namespace MDB_Social {

    FastSim_Flower_Poison::FastSim_Flower_Poison()
    {
        registerParameters();
        
        world = static_cast<FastSimSimulator*>(this->getSimulator("FastSim"));
        world->registerParameters();

        useOnlyRewardedStates = false;
        showFastSimViewer = false;
        showREV = false;
        realtime = false;
        framerate = 25;
        
        nbinputs = 1;
        nboutputs = 2;
//        hiddenNeurons = 5;
        
        attractionTest = false;
        
//        compassTest = false;
//        fitnessComparisonTest = false;
        
        controllerMinimumWeight = -10.0;
        controllerMaximumWeight = 10.0;
        
        babbling = static_cast<BabblingStandard*>(ModelLibrary::getModel("BabblingStandard"));
        std::vector<double> outmin(nboutputs, 0.2);
        std::vector<double> outmax(nboutputs, 1.0);
        babbling->setOutputMinMax(outmin,outmax);
        babbling->registerParameters("experiment");
        useOnlyBabbling = false;
        
        logRobotPos = false;
        sensorLog = false;
        logSocialLearning = false;
//        valueFunctionTest = false;
                
#ifdef USE_REV
        rev = NULL;
        revinit = NULL;
#endif
//        robot->add_light_sensor(fastsim::LightSensor(1,0.0f,100.0f));
        
//        light = boost::shared_ptr<fastsim::IlluminatedSwitch>(new fastsim::IlluminatedSwitch(1, 5.0f, 300.0f, 300.0f, true));
//        world->getMap()->add_illuminated_switch(light);
    }

    FastSim_Flower_Poison::FastSim_Flower_Poison(const FastSim_Flower_Poison& orig)
    {
        
    }

    FastSim_Flower_Poison::~FastSim_Flower_Poison()
    {
        delete world;
        delete controller;
        delete babbling;
        if (logSocialLearning)
            socialLearningLogFile.close();
        
#ifdef USE_REV
        delete revinit;
        revinit = NULL;
        rev = NULL;
#endif        
    }
    
    bool stringToVector(std::string& str, std::vector<unsigned>& ret)
    {
        bool ok = true;
        std::list<unsigned> tmp;
        unsigned number = 0;
        for (unsigned i=0; i < str.size(); ++i) {
            if (str[i] == ' ') {
                tmp.push_back(number);
                number = 0;
            }
            else if (str[i] <'0' || str[i] > '9')
                ok = false;
            else {
                number = number*10 + (str[i] - '0');
            }
        }
        tmp.push_back(number);
        ret.resize(tmp.size());
        
        std::copy(tmp.begin(), tmp.end(), ret.begin());
        
        return ok;
    }

    void FastSim_Flower_Poison::registerParameters()
    {
        std::cout << "FastSim_Flower_Poison : registering the parameters...";
        std::cout.flush();
        Settings* settings = RobotID::getSettings();
        settings->registerParameter<unsigned>("experiment.nbinputs", 1, "Number of inputs/sensors on the neural network.");
        settings->registerParameter<unsigned>("experiment.nboutputs", 2, "Number of outputs on the neural network.");
        settings->registerParameter<std::string>("experiment.hiddenNeurons", "10", "Number of hidden neurons for the controller.");
        settings->registerParameter<double>("experiment.controllerMinimumWeight", 0.0, "Minimum weight of the neural network.");
        settings->registerParameter<double>("experiment.controllerMaximumWeight", 0.0, "Maximum weight of the neural network.");
        settings->registerParameter<int>("experiment.trialCount", 0, "Number of trials for the experiment.");
        settings->registerParameter<int>("experiment.epochCount", 0, "Number of steps for one trial of the experiment.");
        settings->registerParameter<double>("experiment.timestep", 0.01, "Simulation step of the experiment in seconds.");
//        settings->registerParameter<double>("experiment.rewardZoneDiameter", 10.0, "Diameter of the reward zone.");
        settings->registerParameter<bool>("experiment.useOnlyBabbling", false, "Use only the babbling controller.");
        settings->registerParameter<bool>("experiment.useOnlyTrueReward", false, "Use only true rewards, no estimation through the value function");
        settings->registerParameter<bool>("experiment.logRobotPosition", false, "Output the position of the robot in a log file.");
        settings->registerParameter<bool>("experiment.sensorLogFlag", false, "Log the sensors values during the experiment.");
        settings->registerParameter<double>("experiment.maxSpeed", 5, "Maximum speed of the robot.");
        settings->registerParameter<bool>("experiment.valueFunctionTest", false, "Produce a map of the environment in terms of potential reward from the VF.");
        settings->registerParameter<unsigned>("experiment.valueFunctionTestNbOrientations", 10, "Number of orientations to test the value function on.");
        settings->registerParameter<bool>("experiment.useOnlyRewardedStates", false, "Keep only rewarded states in the traces to train the value function.");
        settings->registerParameter<bool>("experiment.useRestrictedVFasFitness", false, "Use VF as fitness only when the current trace has been encountered before.");
        settings->registerParameter<double>("experiment.thresholdForVFasFitness", 0.5, "Minimum distance between a trace and the traces in memory to use VF as fitness.");
        settings->registerParameter<bool>("experiment.endTrialWhenOnReward", false, "Limit the time allowed in the reward zone before ending the trial.");
        settings->registerParameter<unsigned>("experiment.maxTimeOnReward", 0, "Maximum time allowed in the reward zone before ending the trial.");
        settings->registerParameter<bool>("experiment.showFastSimViewer", false, "Show the fastsim viewer for the simulator.");
        settings->registerParameter<bool>("experiment.showREV", false, "Show REV viewer for the simulator.");
        settings->registerParameter<unsigned>("experiment.REVsizeMultiplicator", 2, "Increase the default size of the REV viewer by this factor");
        settings->registerParameter<bool>("experiment.realtime", false, "Play the experiment in realtime in the viewer.");
        settings->registerParameter<unsigned>("experiment.framerate", 25, "Framerate used to display the experiment in the viewer.");
        settings->registerParameter<bool>("experiment.compassTest", false, "Test the compass output by rotating the robot at different location and printing the readings.");
//        settings->registerParameter<bool>("experiment.fitnessComparisonTest", false, "Test the individual by using the learned and perfect fitness for comparison purposes.");
        settings->registerParameter<bool>("experiment.printInputsOutputs", false, "During testing, print the input and outputs of the neural network.");
//        settings->registerParameter<double>("experiment.diameterTarget", 1.0, "Diameter of the target zone");
        settings->registerParameter<double>("experiment.diameterFlower", 5.0, "Diameter of the flowers");
        settings->registerParameter<double>("experiment.diameterPoison", 5.0, "Diameter of the poisons");
        settings->registerParameter<unsigned>("experiment.nbFlowerType", 1, "Number of flower types available in the environment.");
//        settings->registerParameter<unsigned>("experiment.nbFlowerPerType", 1, "Number of flowers for each type.");
        settings->registerParameter<unsigned>("experiment.nbPoisonType", 1, "Number of poison types available in the environment.");
//        settings->registerParameter<unsigned>("experiment.nbPoisonPerType", 1, "Number of poison sources for each type.");
        settings->registerParameter<std::string>("experiment.nbFlowerTypes", std::string(""), "Types and number of flowers per type. Ex: 3 0 2");
        settings->registerParameter<std::string>("experiment.nbPoisonTypes", std::string(""), "Types and number of poisons per type. Ex: 3 0 2");
        settings->registerParameter<bool>("experiment.attractionTest", false, "Run attraction/repulsion test");
        settings->registerParameter<bool>("experiment.socialLearningControllers", false, "Social Learning: exchange controllers.");
        settings->registerParameter<bool>("experiment.socialLearningValueFunctions", false, "Social Learning: exchange value functions.");
        settings->registerParameter<bool>("experiment.socialLearningTraces", false, "Social Learning: exchange traces.");
        settings->registerParameter<bool>("experiment.logSocialLearning", false, "Log all social interactions in a file.");
        std::cout << " DONE" << std::endl;
        
    }
    
    void FastSim_Flower_Poison::loadParameters()
    {
        std::cout << "FastSim_Flower_Poison: Loading parameters..." << std::endl;
        Settings* settings = RobotID::getSettings();
        std::string stFlowerTypes;
        std::string stPoisonTypes;
        std::string hiddenNeuronsStr;
        try {
            nbinputs = settings->value<unsigned>("experiment.nbinputs").second;
            nboutputs = settings->value<unsigned>("experiment.nboutputs").second;
            hiddenNeuronsStr = settings->value<std::string>("experiment.hiddenNeurons").second;
            controllerMinimumWeight = settings->value<double>("experiment.controllerMinimumWeight").second;
            controllerMaximumWeight = settings->value<double>("experiment.controllerMaximumWeight").second;
            trialCount = settings->value<int>("experiment.trialCount").second;
            epochCount = settings->value<int>("experiment.epochCount").second;
            timestep = settings->value<double>("experiment.timestep").second;
//            rewardZoneDiameter = settings->value<double>("experiment.rewardZoneDiameter").second;
            useOnlyBabbling = settings->value<bool>("experiment.useOnlyBabbling").second;
            logRobotPos = settings->value<bool>("experiment.logRobotPosition").second;
            sensorLog = settings->value<bool>("experiment.sensorLogFlag").second;
            maxSpeed = settings->value<double>("experiment.maxSpeed").second;
//            testIndividual = settings->value<bool>("General.testIndividual").second;
            useOnlyTrueReward = settings->value<bool>("experiment.useOnlyTrueReward").second;
            valueFunctionTest = settings->value<bool>("experiment.valueFunctionTest").second;
            useOnlyRewardedStates = settings->value<bool>("experiment.useOnlyRewardedStates").second;
            useRestrictedVFasFitness = settings->value<bool>("experiment.useRestrictedVFasFitness").second;
            thresholdForVFasFitness = settings->value<double>("experiment.thresholdForVFasFitness").second;
            endTrialWhenOnReward = settings->value<bool>("experiment.endTrialWhenOnReward").second;
            maxTimeOnReward = settings->value<unsigned>("experiment.maxTimeOnReward").second;
            showFastSimViewer = settings->value<bool>("experiment.showFastSimViewer").second;
            showREV = settings->value<bool>("experiment.showREV").second;
            REVsizeMultiplicator = settings->value<unsigned>("experiment.REVsizeMultiplicator").second;
            realtime = settings->value<bool>("experiment.realtime").second;
            framerate = settings->value<unsigned>("experiment.framerate").second;
            compassTest = settings->value<bool>("experiment.compassTest").second;
//            fitnessComparisonTest = settings->value<bool>("experiment.fitnessComparisonTest").second;
            printInputsOutputs = settings->value<bool>("experiment.printInputsOutputs").second;
//            diameterTarget = settings->value<double>("experiment.diameterTarget").second;
            diameterFlower = settings->value<double>("experiment.diameterFlower").second;
            diameterPoison = settings->value<double>("experiment.diameterPoison").second;
            nbFlowerType = settings->value<unsigned>("experiment.nbFlowerType").second;
//            nbFlowerPerType = settings->value<unsigned>("experiment.nbFlowerPerType").second;
            nbPoisonType = settings->value<unsigned>("experiment.nbPoisonType").second;
//            nbPoisonPerType = settings->value<unsigned>("experiment.nbPoisonPerType").second;
            stFlowerTypes = settings->value<std::string>("experiment.nbFlowerTypes").second;
            stPoisonTypes = settings->value<std::string>("experiment.nbPoisonTypes").second;
            attractionTest = settings->value<bool>("experiment.attractionTest").second;
            valueFunctionTestNbOrientations = settings->value<unsigned>("experiment.valueFunctionTestNbOrientations").second;
            socialLearningControllers = settings->value<bool>("experiment.socialLearningControllers").second;
            socialLearningValueFunctions = settings->value<bool>("experiment.socialLearningValueFunctions").second;
            socialLearningTraces = settings->value<bool>("experiment.socialLearningTraces").second;
            logSocialLearning = settings->value<bool>("experiment.logSocialLearning").second;
            
            
            babbling->loadParameters("experiment");
            world->initialize();
            std::cout << "FastSim_Flower_Poison: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "FastSim_Flower_Poison: Error loading the parameters: " << e.what() << std::endl;
            exit(1);
        }
        
        // Parsing the flower and poison types strings.
        if ( !( stringToVector(stFlowerTypes, nbFlowerTypes) && stringToVector(stPoisonTypes, nbPoisonTypes) ) ) {
            std::cerr << "FastSim_Flower_Poison : ERROR with type description. A non numeral has been detected." << std::endl;
            exit(1);
        }
        nbFlowerType = nbFlowerTypes.size();
        nbPoisonType = nbPoisonTypes.size();

        flowerList.resize(nbFlowerType);
        for (unsigned i=0; i<nbFlowerType; ++i)
            flowerList[i].resize(nbFlowerTypes[i]);
        poisonList.resize(nbPoisonType);
        for (unsigned i=0; i<nbPoisonType; ++i)
            poisonList[i].resize(nbPoisonTypes[i]);

        if (nbinputs != ( 2*(nbFlowerType + nbPoisonType) + 8) ) {
            std::cerr << "FastSim_Flower_Poison: ERROR: the number of inputs should be " << (2*(nbFlowerType + nbPoisonType) + 8) << "." << std::endl;
            exit(1);
        }
        
        // Initialize the controller
        controller = static_cast<FeedforwardNN*>(ModelLibrary::getModel("Feedforward"));
        std::vector<enum FeedforwardNN::ActivationFunction> activationFunctions(1, FeedforwardNN::SIGMOID);
        if ( !(stringToVector(hiddenNeuronsStr, hiddenNeurons) ) ) {
            std::cerr << "FastSim_Flower_Poison : ERROR with hidden neuron description. A non numeral has been detected." << std::endl;
            exit(1);
        }
        layers.resize(2+hiddenNeurons.size());
        unsigned index = 0;
        layers[index++] = nbinputs;
        for (unsigned i=0; i<hiddenNeurons.size(); ++i)
            layers[index++] = hiddenNeurons[i];
        layers[index] = nboutputs;
        
//        if (hiddenNeurons.) {
//            layers.resize(3);
//            layers[0] = nbinputs;
//            layers[1] = hiddenNeurons;
//            layers[2] = nboutputs;
//        }
//        else {
//            layers.resize(2);
//            layers[0] = nbinputs;
//            layers[1] = nboutputs;
//        }
        controller->setup(layers, activationFunctions);
        
        if (showREV) {
#ifdef USE_REV
            
            double wwidth = world->getMapWidth();
            revinit = new REVInit();
            rev = revinit->getViewer();
            if (realtime)
                rev->setRealtime(true);
            rev->setSize(wwidth, wwidth);
            rev->setFramerate(framerate);

            // Draw the arena
            rev->setRobotRadius(world->getRobot()->get_radius());
//            rev->addZone(rewardZoneDiameter/2.0, wwidth/2.0, wwidth/2.0, REV::Color(127, 127, 127, 255)); // START
//            rev->addZone(diameterTarget/2.0, wwidth/2.0, wwidth/2.0, REV::Color(0, 0, 0, 255));
            

            // TODO: Different colors per type of flower/poison
            flowerREVColors.resize(nbFlowerType);
            poisonREVColors.resize(nbPoisonType);
            unsigned delta = std::floor(127.0 / nbFlowerType);
            for (unsigned t=0; t<nbFlowerType; ++t)
                flowerREVColors[t] = REV::Color(0, 127 + t*delta, 0, 255);
            delta = std::floor(127.0 / nbPoisonType);
            for (unsigned t=0; t<nbPoisonType; ++t)
                poisonREVColors[t] = REV::Color(127 + t*delta, 0, 0, 255);
            
            flowerREVIndexStarts.resize(nbFlowerType);
            poisonREVIndexStarts.resize(nbPoisonType);
            for (unsigned t=0; t<nbFlowerType; ++t) {
                if (nbFlowerTypes[t] > 0) {
                    flowerREVIndexStarts[t] = rev->addZone(diameterFlower / 2.0, wwidth/2.0, wwidth/2.0, flowerREVColors[t]);
                    for (unsigned i=1; i<nbFlowerTypes[t]; ++i)
                        rev->addZone(diameterFlower / 2.0, wwidth/2.0, wwidth/2.0, flowerREVColors[t]);
                }
            }
            
            for (unsigned t=0; t<nbPoisonType; ++t) {
                if (nbPoisonTypes[t] > 0) {
                    poisonREVIndexStarts[t] = rev->addZone(diameterPoison / 2.0, wwidth/2.0, wwidth/2.0, poisonREVColors[t]);
                    for (unsigned i=1; i<nbPoisonTypes[t]; ++i)
                        rev->addZone(diameterPoison / 2.0, wwidth/2.0, wwidth/2.0, poisonREVColors[t]);
                }
            }
            
#else
            std::cerr << "FastSim_Flower_Poison: the REV viewer is not compiled in." << std::endl;
#endif            
        }
        
    }

    void FastSim_Flower_Poison::preprocessing() 
    {
        // Let's get the list of robots.
        ResourceLibraryData* resourceLibrary = RobotID::getResourceLibrary();
        SocialManagerClient* smclient = resourceLibrary->getSocialManagerClient();
//        SocialManagerClient* smclient = NULL;
//            std::cout << " p " << std::endl;
        if (smclient) {  // test if we are in a social environment
            
            if (!socialLearningLogFile.is_open()) {
                std::string filename = resourceLibrary->getWorkingDirectory() + "/SocialLearning.log";
                socialLearningLogFile.open(filename.c_str(), std::ios_base::trunc);
                if (!socialLearningLogFile.is_open()) {
                    std::cerr << "FastSim_Flower_Poison: ERROR when opening SocialLearning.log." << std::endl;
                    exit(1);
                }
            }            
            smclient->synchronise();

            if (socialLearningControllers) {
                std::string other_robot = smclient->getRandomRobotID(RobotID::getID());
                // Retrieve its genotypes
                PolicyMemory* pm = smclient->getPolicyMemory(other_robot);
                // Find the best policy and replace the worst current one
                unsigned best = 0;
                double bestFitness = (*pm)[best].getFitness();
                for (unsigned i = 1; i < pm->size(); ++i) {
                    if ((*pm)[i].getFitness() > bestFitness) {
                        best = i;
                        bestFitness = (*pm)[best].getFitness();
                    }
                }
                // We have the best. We need to compare the fitness to the worst individual in our population
                PolicyMemory* mypm = smclient->getPolicyMemory(RobotID::getID());
                unsigned worst = 0;
                double worstFitness = (*mypm)[worst].getFitness();
                for (unsigned i = 1; i < mypm->size(); ++i) {
                    if ((*mypm)[i].getFitness() < worstFitness) {
                        worst = i;
                        worstFitness = (*mypm)[worst].getFitness();
                    }
                }
                if (bestFitness > worstFitness) {
    //                std::cout << " Social learning: copying " << robotIds[other_robot] << ":" << best << " to " << getID() << ":" << worst << std::endl;
                    resourceLibrary->getGeneticAlgorithm()->importGenotype(&(*pm)[best], worst);
                    
                    if (logSocialLearning) {
                        socialLearningLogFile << 0 << " " << RobotID::getID() << " " << other_robot << " " << best << " " << bestFitness << " " << worst << " " << worstFitness << std::endl; 
                    }
                }
            }
            
            if (socialLearningValueFunctions) {
                std::cout << "FastSim_Flower_Poison: Social learning on value functions not yet implemented." << std::endl;
                exit(1);
            }
            
            if (socialLearningTraces) {
                std::cout << "FastSim_Flower_Poison: Social learning on traces not yet implemented." << std::endl;
                exit(1);
            }
            
        }        
    }
    
    
    void FastSim_Flower_Poison::installGenotype(Genotype& individual)
    {
        std::vector<FeedforwardNN::weight_t> weights(individual.getSize());
        
        controller->getWeights(weights);

        if (weights.size() != individual.getSize()) {
            std::cerr << "FastSim_Flower_Poison: Size of the genotype (size = " << individual.getSize() << ") differs from the size of the network (size = " << weights.size() << ")." << std::endl;
            exit(1);
        }
        
        for (unsigned i=0; i<individual.getSize(); ++i) {
            weights[i].weight =  (controllerMaximumWeight - controllerMinimumWeight)*individual[i] + controllerMinimumWeight;
        }
        controller->setWeights(weights);
    }

    void FastSim_Flower_Poison::logRobotPosition(unsigned trial, unsigned epoch)
    {
        if (logRobotPos) {
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double orient = world->getRobot()->get_pos().theta();
            robotLogPosFile << trial << " " << epoch << " " << x << " " << y << " " << orient << std::endl;
        }
        
    }
    
    void FastSim_Flower_Poison::relocateRobot()
    {
        // Todo: not relocate robot on the target area
        double w = world->getMapWidth();
        
        double robotRadius = world->getRobot()->get_radius();
        
        double x;
        double y;
        
        do {
            x = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
            y = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        } while (checkCollisionAllFoodSource(x,y,robotRadius*2).collision);
            
        double orient = drand48() * M_2_PI;
        world->getRobot()->reinit();
        world->moveRobot(x, y, orient);
    }
    
    void FastSim_Flower_Poison::relocateFoodSources()
    {
        for (unsigned t=0; t<nbFlowerType; ++t)
            for (unsigned f=0; f<nbFlowerTypes[t]; ++f)
                relocateFlower(t,f);
        
        for (unsigned t=0; t<nbPoisonType; ++t)
            for (unsigned f=0; f<nbPoisonTypes[t]; ++f)
                relocatePoison(t,f);
    }
        
    void FastSim_Flower_Poison::relocateFlower(unsigned type, unsigned p)
    {
        double w = world->getMapWidth()*0.9;
        
        double flowerRadius = flowerList[type][p].d/2.0;
        
        double x;
        double y;
        do {
            x = drand48() * (w*0.94 - 2*flowerRadius)+flowerRadius*1.1;
            y = drand48() * (w*0.94 - 2*flowerRadius)+flowerRadius*1.1;
        } while (checkCollisionOtherFoodSource(x,y,flowerRadius*2, flowerList[type][p]) || checkCollisionRobot(x,y,flowerRadius*2) );
        
        // relocate
        flowerList[type][p].x = x;
        flowerList[type][p].y = y;
        flowerList[type][p].visible = true;
        flowerList[type][p].d = diameterFlower;
        flowerList[type][p].food = true;
        flowerList[type][p].type = type;
        
        if (showREV) {
#ifdef USE_REV
            rev->modifyZone(flowerREVIndexStarts[type]+p, flowerRadius, x, y, flowerREVColors[type]);
            rev->setZoneVisible(flowerREVIndexStarts[type]+p, true);

#endif        
        }
    }
    
    void FastSim_Flower_Poison::relocatePoison(unsigned type, unsigned p)
    {
        double w = world->getMapWidth()*0.9;
        
        double poisonRadius = poisonList[type][p].d/2.0;
        
        double x;
        double y;
        do {
            x = drand48() * (w*0.94 - 2*poisonRadius)+poisonRadius*1.1;
            y = drand48() * (w*0.94 - 2*poisonRadius)+poisonRadius*1.1;
        } while (checkCollisionOtherFoodSource(x,y,poisonRadius*2,poisonList[type][p]) || checkCollisionRobot(x,y,poisonRadius*2) );
        
        // relocate
        poisonList[type][p].x = x;
        poisonList[type][p].y = y;
        poisonList[type][p].visible = true;
        poisonList[type][p].d = diameterPoison;
        poisonList[type][p].food = false;
        poisonList[type][p].type = type;
        
        if (showREV) {
#ifdef USE_REV
            rev->modifyZone(poisonREVIndexStarts[type]+p, poisonRadius, x, y, poisonREVColors[type]);
            rev->setZoneVisible(poisonREVIndexStarts[type]+p, true);

#endif        
        }                
    }
    
    bool FastSim_Flower_Poison::checkCollisionRobot(double x, double y, double d)
    {
        bool collision = false;
        double robotX = world->getRobot()->get_pos().get_x();
        double robotY = world->getRobot()->get_pos().get_y();
        double robotRadius = world->getRobot()->get_radius();
        
        //check collision robot
        double dist = pow(x-robotX,2.0) + pow(y-robotY,2.0);
        if (dist < pow((d/2.0)+robotRadius,2.0)) {
            collision = true;
        }
                        
        return collision;
    }
    
    FastSim_Flower_Poison::CollisionData FastSim_Flower_Poison::checkCollisionAllFoodSource(double x, double y, double d)
    {
        CollisionData ret;
        ret.collision = false;
        
        //check collision per puck
        for (unsigned t=0; t<nbFlowerType && !ret.collision; ++t)
            for (unsigned p=0; p<nbFlowerTypes[t] && !ret.collision; ++p) {
                if(checkCollisionOneFoodSource(x,y,d,flowerList[t][p])) {
                    ret.index = p;
                    ret.type = t;
                    ret.food = true;
                    ret.collision = true;
                }
            }
        
        for (unsigned t=0; t<nbPoisonType && !ret.collision; ++t)
            for (unsigned p=0; p<nbPoisonTypes[t] && !ret.collision; ++p) {
                if(checkCollisionOneFoodSource(x,y,d,poisonList[t][p])) {
                    ret.index = p;
                    ret.type = t;
                    ret.food = false;
                    ret.collision = true;
                }
            }

        return ret;
    }
            
    bool FastSim_Flower_Poison::checkCollisionOtherFoodSource(double x, double y, double d, FoodPoisonObject& food) //double x, double y, double d, unsigned p)
    {
        bool collision = false;
                
        //check collision other pucks then given puck p
        for (unsigned t=0; t<nbFlowerType && !collision; ++t)
            for (unsigned p=0; p<nbFlowerTypes[t] && !collision; ++p) {
                if (flowerList[t][p] != food)
                    collision = checkCollisionOneFoodSource(x,y,d,flowerList[t][p]);
            }
        
        for (unsigned t=0; t<nbPoisonType && !collision; ++t)
            for (unsigned p=0; p<nbPoisonTypes[t] && !collision; ++p) {
                if (poisonList[t][p] != food)
                    collision = checkCollisionOneFoodSource(x,y,d,poisonList[t][p]);
            }

        return collision;
    }

    
    bool FastSim_Flower_Poison::checkCollisionOneFoodSource(double x, double y, double d, FoodPoisonObject& p)
    {
        // check collision with specific puck id
        double dist = pow(x-p.x, 2.0) + pow(y-p.y,2.0);;
        return (dist < pow(d/2.0+(p.d/2.0),2.0));
    }
                
    double FastSim_Flower_Poison::computeDistanceToFoodSource(FoodPoisonObject& p)
    {
        double x = world->getRobot()->get_pos().get_x();
        double y = world->getRobot()->get_pos().get_y();
        double dist = -1.0;
        if (p.visible)
            dist = sqrt(pow(x-p.x, 2.0) + pow(y-p.y, 2.0)) - p.d*0.5;

        return dist;
    }

    
    std::tuple<int, int, double> FastSim_Flower_Poison::computeDistanceClosestFlower()
    {
        double closestDist = 1e6;
        int tindex = -1;
        int cindex = -1;
        std::tuple<int, int, double> ret;
        
        double dist;
        for (unsigned t=0; t<nbFlowerType; ++t) {
            for (unsigned p=0; p<nbFlowerTypes[t]; ++p) {
                if (flowerList[t][p].visible) {
                    dist = computeDistanceToFoodSource(flowerList[t][p]);
                    if (dist < closestDist) {
                        tindex = t;
                        cindex = p;
                        closestDist = dist;
                    }
                }
            }
        }
        
        std::get<0>(ret) = tindex;
        std::get<1>(ret) = cindex;
        std::get<2>(ret) = closestDist;
        return ret;
    }

    std::tuple<int, int, double> FastSim_Flower_Poison::computeDistanceClosestFlower(int type)
    {
        double closestDist = 1e6;
        int tindex = -1;
        int cindex = -1;
        std::tuple<int, int, double> ret;
        
        double dist;
        for (unsigned p=0; p<nbFlowerTypes[type]; ++p) {
            if (flowerList[type][p].visible) {
                dist = computeDistanceToFoodSource(flowerList[type][p]);
                if (dist < closestDist) {
                    tindex = type;
                    cindex = p;
                    closestDist = dist;
                }
            }
        }
        
        std::get<0>(ret) = tindex;
        std::get<1>(ret) = cindex;
        std::get<2>(ret) = closestDist;
        return ret;
    }


    std::tuple<int, int, double> FastSim_Flower_Poison::computeDistanceClosestPoison()
    {
        double closestDist = 1e6;
        int tindex = -1;
        int cindex = -1;
        std::tuple<int, int, double> ret;
        
        double dist;
        for (unsigned t=0; t<nbPoisonType; ++t) {
            for (unsigned p=0; p<nbPoisonTypes[t]; ++p) {
                if (poisonList[t][p].visible) {
                    dist = computeDistanceToFoodSource(poisonList[t][p]);
                    if (dist < closestDist) {
                        tindex = t;
                        cindex = p;
                        closestDist = dist;
                    }
                }
            }
        }
        
        std::get<0>(ret) = tindex;
        std::get<1>(ret) = cindex;
        std::get<2>(ret) = closestDist;
        return ret;
    }
    
    std::tuple<int, int, double> FastSim_Flower_Poison::computeDistanceClosestPoison(int type)
    {
        double closestDist = 1e6;
        int tindex = -1;
        int cindex = -1;
        std::tuple<int, int, double> ret;
        
        double dist;
        for (unsigned p=0; p<nbPoisonTypes[type]; ++p) {
            if (poisonList[type][p].visible) {
                dist = computeDistanceToFoodSource(poisonList[type][p]);
                if (dist < closestDist) {
                    tindex = type;
                    cindex = p;
                    closestDist = dist;
                }
            }
        }
        
        std::get<0>(ret) = tindex;
        std::get<1>(ret) = cindex;
        std::get<2>(ret) = closestDist;
        return ret;
    }

    FastSim_Flower_Poison::compass_info_t FastSim_Flower_Poison::computeCompassClosestFlower(int rtype)
    {
        compass_info_t ret;
        ret.distance = -1.0;
        ret.orientation = -1.0;

        std::tuple<int, int, double> dcb;
        int type;
        if (rtype != -1) {
            dcb = computeDistanceClosestFlower(rtype);   // type, index, distance
            type = rtype;
        }
        else {
            dcb = computeDistanceClosestFlower();
            type = std::get<0>(dcb);
        }
        int index = std::get<1>(dcb);
        if (index != -1) {
            ret.distance = std::get<2>(dcb);
            
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double rorientation = world->getRobot()->get_pos().theta();
            
            ret.orientation = atan2((-y+flowerList[type][index].y),(flowerList[type][index].x-x)) - rorientation;
            if (ret.orientation < -M_PI)
                ret.orientation += 2*M_PI;
        }
        
        return ret;
    }

    FastSim_Flower_Poison::compass_info_t FastSim_Flower_Poison::computeCompassClosestPoison(int rtype)
    {
        compass_info_t ret;
        ret.distance = -1.0;
        ret.orientation = -1.0;

        std::tuple<int, int, double> dcb;
        int type;
        if (rtype != -1) {
            dcb = computeDistanceClosestPoison(rtype);   // type, index, distance
            type = rtype;
        }
        else {
            dcb = computeDistanceClosestPoison();   // type, index, distance
            type = std::get<0>(dcb);
        }
        int index = std::get<1>(dcb);
        if (index != -1) {
            ret.distance = std::get<2>(dcb);
            
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double rorientation = world->getRobot()->get_pos().theta();
            
            ret.orientation = atan2((-y+poisonList[type][index].y),(poisonList[type][index].x-x)) - rorientation;
            if (ret.orientation < -M_PI)
                ret.orientation += 2*M_PI;
        }
        
        return ret;
    }
    
    
    double FastSim_Flower_Poison::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
        // In this experiment we evolve and test a policy. The policy uses the value function and the current state to decide what the next action should be.
        // This part only cares about the policy.
        // A reward will be given when the robot reaches a given distance from the light source.

        testIndividual = _testIndividual;
        
        if (valueFunctionTest) {
            testValueFunction();
            return -1.0;
        }
        
        if (attractionTest) {
            installGenotype(individual);
            runAttractionTest();
            return -1.0;
        }
        
//        if (compassTest) {
//            testCompass();
//            return -1.0;
//        }
        ResourceLibraryData* resourceLibrary = RobotID::getResourceLibrary();
        std::string cwd = resourceLibrary->getWorkingDirectory();
        
        std::cout << "* ";
        std::cout.flush();
        
//        std::vector<double> lightSensors;
//        compass_info_t compassToTarget;
        compass_info_t compassToClosestFlower;
        compass_info_t compassToClosestPoison;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        double reward;
        double tVFF= thresholdForVFasFitness; // * thresholdForVFasFitness;
        
//        world->getLightSensors(lightSensors);   // Should be only one sensor
//        if (nbinputs != lightSensors.size()*(1+useSeeTheLightInputs)+8) {
//            std::cerr << "FastSim_Flower_Poison: Error: nbinputs should be " << 2*(1+useSeeTheLightInputs)+8 << std::endl;
//            exit(0);
//        }
        
        if (logRobotPos) {
            std::string tmp = cwd + "/robotPositions.log";
            robotLogPosFile.open(tmp.c_str(), std::ios_base::trunc);
        }
        
        if (sensorLog) {
            std::string tmp = cwd + "/sensors.log";
            sensorLogFile.open(tmp.c_str(), std::ios_base::trunc);
        }
        
        Trace trace;
        TraceMemory* tm = resourceLibrary->getTraceMemory();
        ValueFunction* vf = resourceLibrary->getValueFunction();
        trace.estimated_reward = 0.0;
        trace.expected_reward = 0.0;
        trace.reliability = 0.0;
        
        if (!useOnlyBabbling && !this->recommendBabbling)
            installGenotype(individual);

        double rewardTotal = 0.0;
        
        bool useValueFunction = !recommendBabbling && !useOnlyTrueReward;
        bool useBabbling = useOnlyBabbling || this->recommendBabbling;
        double lreward;
        unsigned epoch;
        unsigned index;
        
        if (showFastSimViewer) {
            if (!world->activateViewer(true))
                std::cout << Color::Modifier(Color::FG_RED) << "FastSim_Flower_Poison: ERROR: fastsim viewer not compiled in. Activate it using cmake -DUSE_FASTSIM_VIEWER=ON" << Color::Modifier(Color::FG_DEFAULT) << std::endl;
            else {
                std::cout << "FastSim_Flower_Poison: Viewer activated." << std::endl;
            }
        }

        double maxDistance = sqrt(2.0*pow(world->getMapWidth(),2.0));
        
        double robotDiameter = world->getRobot()->get_radius() * 2.0;
        
        std::vector<unsigned> flowerCounts(nbFlowerType, 0.0);
        std::vector<unsigned> poisonCounts(nbPoisonType, 0.0);
        for (unsigned trial = 0; trial < trialCount; ++trial) {
            controller->reset();
            relocateRobot();
            relocateFoodSources();
            
#ifdef USE_REV
            if (showREV) {
                double x = world->getRobot()->get_pos().get_x();
                double y = world->getRobot()->get_pos().get_y();
                double orient = world->getRobot()->get_pos().theta();
                rev->setRobotPosition(x, y, orient);
            }
#endif            
                        
            lreward = 0.0;
            for (epoch = 0; epoch < epochCount; ++epoch) {
                double robotX = world->getRobot()->get_pos().get_x();
                double robotY = world->getRobot()->get_pos().get_y();

                //                world->getLightSensors(lightSensors);   // Should be only one sensor
//                reward = computeReward(puckCarried != -1);
                reward = 0.0;

                CollisionData foodCaught = checkCollisionAllFoodSource(robotX, robotY, robotDiameter);
                if (foodCaught.collision) {
                    
                    if (foodCaught.food) {
                        flowerList[foodCaught.type][foodCaught.index].visible = false;
                        flowerCounts[foodCaught.type]++;
                        reward = 1.0;
                        relocateFlower(foodCaught.type, foodCaught.index);
                    }
                    else {
                        poisonList[foodCaught.type][foodCaught.index].visible = false;
                        poisonCounts[foodCaught.type]++;
                        reward = -1.0;
                        relocatePoison(foodCaught.type, foodCaught.index);
                    }
//#ifdef USE_REV
//                    if (showREV) {
//                        if (foodCaught.food)
//                            rev->setZoneVisible(flowerREVIndexStarts[foodCaught.type]+foodCaught.index, false);
//                        else
//                            rev->setZoneVisible(poisonREVIndexStarts[foodCaught.type]+foodCaught.index, false);
//                    }
//#endif        
                }


                world->getLaserSensors(laserSensors);
//                compassToTarget = computeCompassTarget();

                index = 0;
                for (unsigned f=0; f<nbFlowerType; ++f) {
                    compassToClosestFlower = computeCompassClosestFlower(f);
                    nninputs[index++] = compassToClosestFlower.orientation / M_PI;
                    nninputs[index++] = compassToClosestFlower.distance / maxDistance;
                }
                
                for (unsigned p=0; p<nbPoisonType; ++p) {
                    compassToClosestPoison = computeCompassClosestPoison(p);
                    nninputs[index++] = compassToClosestPoison.orientation / M_PI;
                    nninputs[index++] = compassToClosestPoison.distance / maxDistance;
                }
                
//                nninputs[index++] = compassToTarget.orientation / M_PI;
//                nninputs[index++] = compassToTarget.distance / maxDistance;
                std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin()+index);

                //                nninputs[nbinputs-1] = 1.0;   // bias
                if (testIndividual && printInputsOutputs) {
                    std::cout << "Inputs = ";
                    for (unsigned i=0; i<nbinputs; ++i)
                        std::cout << nninputs[i] << " ";
                    std::cout << std::endl;
                }
                
                
                if (useBabbling)
                    nnoutput = babbling->run(nninputs);
                else
                    controller->run(nninputs, nnoutput);
                
                world->updateRobot(timestep*(nnoutput[0]*2*maxSpeed-maxSpeed), timestep*(nnoutput[1]*2*maxSpeed-maxSpeed));
                world->step();

//                if (reward) {
//                    relocateBall(puckCarried);
//                    puckCarried = -1;
//                }
#ifdef USE_REV
                if (showREV) {
                    double orient = world->getRobot()->get_pos().theta();
                    rev->setRobotPosition(robotX, robotY, orient);
                    rev->step();
                    revinit->updateEventQueue();
                }
#endif
                logRobotPosition(trial, epoch);
                if (sensorLog) {
                    sensorLogFile << trial << " " << epoch << " ";
                    for (unsigned i=0; i<nninputs.size(); ++i)
                        sensorLogFile << nninputs[i] << " ";
                    sensorLogFile << nnoutput[0] << " " << nnoutput[1] << std::endl;
                }
                
                // I need to record the trace in the traceMemory
                trace.true_reward = reward;
                trace.inputs = nninputs;
                trace.outputs = nnoutput;

                trace.estimated_reward = 0.0;
                if (useValueFunction) {
//                    if (!useRestrictedVFasFitness || (useRestrictedVFasFitness && tm->computeShortestDistance(trace, Trace::EUCLIDIAN2) >= tVFF))
                    double fam = -1.0;
                    if (!useRestrictedVFasFitness || (useRestrictedVFasFitness && (fam=vf->computeFamiliarity(trace)) <= tVFF)) {
                        trace.estimated_reward = vf->estimateTrace(trace);
//                        std::cout << "trial:epoch " << trial << ":" << epoch << " -> familiarity = " << fam << std::endl;
                    }
                }

                if (!useOnlyRewardedStates || (useOnlyRewardedStates && (trace.true_reward > 1e-6 || trace.estimated_reward > 1e-6))) {
                    tm->push_back(trace);
                }
                
//                rewardTotal += std::max((double)reward, trace.estimated_reward);
                if (useValueFunction)
                    lreward += std::max((double)reward, trace.estimated_reward);
                else
                    lreward += (double)reward;
                // need to compute the fitness now. Normally it should be the VF, but we don't have VF now...
            }
            if (testIndividual) {
                std::cout << "Trial " << trial << " : total reward = " << lreward << " for " << epoch << " timesteps" << std::endl;
                std::cout << "    Flowers picked per type: ";
                for (unsigned t=0; t<nbFlowerType; ++t)
                    std::cout << t << " = " << flowerCounts[t] << "; ";
                std::cout << std::endl;
                std::cout << "    Poison picked per type: ";
                for (unsigned t=0; t<nbPoisonType; ++t)
                    std::cout << t << " = " << poisonCounts[t] << "; ";
                std::cout << std::endl;
                
            }
            rewardTotal += lreward;
            
        }

//        std::cout << "Phototaxis: rewardTotal = " << rewardTotal << " ; fitness = " << (rewardTotal*1.0)/(1.0*trialCount*epochCount) << std::endl;

        if (logRobotPos) {
            robotLogPosFile.close();
        }
        if (sensorLog)
            sensorLogFile.close();
                

//        if (getID() == "RobotA")
//            return 1.0;
//        else
//            return 2.0;
        return rewardTotal/(1.0*trialCount);
    }

    void FastSim_Flower_Poison::testValueFunction()
    {
        // This function tests the value function by placing the robot following a grid and measuring the response of the VF.
        // The robot will be tested with multiple orientations: one facing the light, one facing away from it.

        std::cout << "Testing the value function:" ;
        std::cout.flush();
        
        ResourceLibraryData* resourceLibrary = RobotID::getResourceLibrary();
        double robotRadius = world->getRobot()->get_radius();
        double w = world->getMapWidth() - 1.1*robotRadius;
        double deltax = 1.0;
        double deltay = 1.0;
        double orient = 0.0;

        compass_info_t compassToClosestFlower;
        compass_info_t compassToClosestPoison;

        if ( (*std::max_element(nbFlowerTypes.begin(),nbFlowerTypes.end()) > 1) || (*std::max_element(nbPoisonTypes.begin(),nbPoisonTypes.end()) > 1) ) {
            std::cerr << "Error: The number of flowers and poison should be set to 1." << std::endl;
            exit(1);
        }
        // Move the puck in a specific place
        double ww = world->getMapWidth();
        std::for_each(flowerList.begin(), flowerList.end(), [ww](std::vector<FoodPoisonObject>& a) {a[0].visible=true; a[0].x = ww*0.5; a[0].y = ww*0.75;});  // flower on TOP
        std::for_each(poisonList.begin(), poisonList.end(), [ww](std::vector<FoodPoisonObject>& a) {a[0].visible=true; a[0].x = ww*0.5; a[0].y = ww*0.25;});  // poison BELOW
        relocateFoodSources();

        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        double estimated_reward;
        ValueFunction* vf = resourceLibrary->getValueFunction();
//        vf->saveValueFunctionToFile("value_function.log");

//        boost::shared_ptr<fastsim::Map> map = world->getMap();
//        std::vector<fastsim::Map::ill_sw_t> lights = map->get_illuminated_switches();
//        double lightx = lights[0]->get_x();
//        double lighty = lights[0]->get_y();
        
        Trace trace;
        trace.estimated_reward = 0.0;
        trace.expected_reward = 0.0;
        trace.reliability = 0.0;
        
        unsigned nbsteps = (unsigned)std::ceil((w * w) / (deltax * deltay));
        unsigned stepMark = nbsteps / 100;
        unsigned mark = 0;
        unsigned index;
        
        std::ofstream outfile("valueFunctionTest.log", std::ios_base::trunc);
        
        double maxDistance = sqrt(2.0*pow(world->getMapWidth(),2.0));

        double orientDelta = 2.0 * M_PI / valueFunctionTestNbOrientations;
        
        for (double x = robotRadius*1.1; x < w ; x+=deltax) {
            for (double y = robotRadius*1.1; y < w; y+=deltay) {
                
                if ( std::fmod(x*y,stepMark) == 0  ) {
                    std::cout << " " << mark++ << "%";
                    std::cout.flush();
                }

                bool collision = checkCollisionAllFoodSource(x,y,robotRadius*2.0).collision;
//                orient = atan2(lighty-y,lightx-x);   // Looking toward it
                if (collision) {  // estimated_reward = -1
                    for (unsigned o=0; o<valueFunctionTestNbOrientations; ++o)
                        outfile << valueFunctionTestNbOrientations << " " << x << " " << y << " " << o * orientDelta << " -1" << std::endl;
                }
                else {
                    for (unsigned o=0; o<valueFunctionTestNbOrientations; ++o) {

                        orient = o * orientDelta;
                        
                        world->getRobot()->reinit();
                        world->moveRobot(x, y, orient);

                        compassToClosestFlower = computeCompassClosestFlower();
                        compassToClosestPoison = computeCompassClosestPoison();

                        index = 0;
                        for (unsigned f=0; f<nbFlowerType; ++f) {
                            compassToClosestFlower = computeCompassClosestFlower(f);
                            nninputs[index++] = compassToClosestFlower.orientation / M_PI;
                            nninputs[index++] = compassToClosestFlower.distance / maxDistance;
                        }

                        for (unsigned p=0; p<nbPoisonType; ++p) {
                            compassToClosestPoison = computeCompassClosestPoison(p);
                            nninputs[index++] = compassToClosestPoison.orientation / M_PI;
                            nninputs[index++] = compassToClosestPoison.distance / maxDistance;
                        }
                        std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin()+index);

                        world->updateRobot(0.0, 0.0);
                        world->step();


                        // I need to record the trace in the traceMemory
                        trace.true_reward = 0.0;
                        trace.inputs = nninputs;
                        trace.outputs = nnoutput;

                        estimated_reward = vf->estimateTrace(trace);

                        outfile << valueFunctionTestNbOrientations << " " << x << " " << y << " " << orient << " " << estimated_reward << std::endl;;

//                        orient += M_PI;   // Computing the opposite angle
//                        if (orient > M_2_PI)
//                            orient -= M_2_PI;
                    }
                }
            }
        }
        outfile.close();
    }

    void FastSim_Flower_Poison::runAttractionTest()
    {
        std::cout << "*** Running attraction test ***" << std::endl;
        
        unsigned epoch;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        compass_info_t compassToClosestFlower;
        compass_info_t compassToClosestPoison;
        double robotX, robotY;
        
        // Setup environment
        int flowerType = -1;
        int poisonType = -1;

        if ( (*std::max_element(nbFlowerTypes.begin(),nbFlowerTypes.end()) > 1) || (*std::max_element(nbPoisonTypes.begin(),nbPoisonTypes.end()) > 1) ) {
            std::cerr << "FastSim_FLower_Poison::runAttractionTest -> Error: The number of flowers and poison should be set to 1." << std::endl;
            exit(1);
        }
        
        std::ofstream outfile("attractionTest.log", std::ios_base::trunc);
        if (!outfile.is_open()) {
            std::cerr << "FastSim_Flower_Poison::runAttractionTest: ERROR: Impossible to open the file attractionTest.log" << std::endl;
            exit(1);
        }
        
        bool measureFlower = false;
        bool measurePoison = false;
        
        unsigned nbenv = nbPoisonType + nbFlowerType + nbPoisonType*nbFlowerType;
        double w = world->getMapWidth();
        double maxDistance = sqrt(2.0*pow(w,2.0));
        
        unsigned index;
        for (unsigned env = 0; env < nbenv; ++env) {
            std::for_each(flowerList.begin(), flowerList.end(), [w](std::vector<FoodPoisonObject>& a) {a[0].visible=false; a[0].x = w*0.5; a[0].y = w*0.75;});  // flower on TOP
            std::for_each(poisonList.begin(), poisonList.end(), [w](std::vector<FoodPoisonObject>& a) {a[0].visible=false; a[0].x = w*0.5; a[0].y = w*0.25;});  // poison BELOW

            flowerType++;
            if (flowerType >= nbFlowerType) {
                flowerType = -1;
                poisonType++;
            }
                        
            measureFlower = (flowerType != -1);
            measurePoison = (poisonType != -1);
            
            if (measureFlower && measurePoison) {  // Two food sources
                flowerList[flowerType][0].visible = true;
                poisonList[poisonType][0].visible = true;
            }
            else {    // One food source
                if (measureFlower)  // only flower
                    flowerList[flowerType][0].visible = true;
                else    // Only poison
                    poisonList[poisonType][0].visible = true;
            }
            
            std::cout << "Testing environment:: Flower = ";
            outfile << trialCount << " " << nbFlowerType << " " << nbPoisonType << " ";
            for (unsigned i = 0; i<nbFlowerType; ++i) {
                outfile << flowerList[i][0].visible << " ";
                std::cout << flowerList[i][0].visible << " ";
            }
            std::cout << " ; Poison = ";
            for (unsigned i = 0; i<nbPoisonType; ++i) {
                outfile << poisonList[i][0].visible << " ";
                std::cout << poisonList[i][0].visible << " ";                
             }
            std::cout << std::endl;

            double distDeltaPoison = 0.0;
            double distDeltaFlower = 0.0;
            for (unsigned trial = 0; trial < trialCount; ++trial) {
                controller->reset();
                
                // Put the robot back in the center of the arena with a random orientation
                world->getRobot()->reinit();
                world->moveRobot(w/2.0, w/2.0, drand48() * M_2_PI);
                
                double distBeginPoison;
                double distBeginFlower;
                double distEndPoison;
                double distEndFlower;
                
                if (measureFlower)
                    distBeginFlower = sqrt(pow(flowerList[flowerType][0].x - w/2.0, 2.0) + pow(flowerList[flowerType][0].y - w/2.0, 2.0));
                if (measurePoison)
                    distBeginPoison = sqrt(pow(poisonList[poisonType][0].x - w/2.0, 2.0) + pow(poisonList[poisonType][0].y - w/2.0, 2.0));
                
                for (epoch = 0; epoch < epochCount; ++epoch) {
                    world->getLaserSensors(laserSensors);
    //                compassToTarget = computeCompassTarget();

                    index = 0;
                    for (unsigned f=0; f<nbFlowerType; ++f) {
                        compassToClosestFlower = computeCompassClosestFlower(f);
                        nninputs[index++] = compassToClosestFlower.orientation / M_PI;
                        nninputs[index++] = compassToClosestFlower.distance / maxDistance;
                    }

                    for (unsigned p=0; p<nbPoisonType; ++p) {
                        compassToClosestPoison = computeCompassClosestPoison(p);
                        nninputs[index++] = compassToClosestPoison.orientation / M_PI;
                        nninputs[index++] = compassToClosestPoison.distance / maxDistance;
                    }

                    std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin()+index);

                    controller->run(nninputs, nnoutput);

                    world->updateRobot(timestep*(nnoutput[0]*2*maxSpeed-maxSpeed), timestep*(nnoutput[1]*2*maxSpeed-maxSpeed));
                    world->step();

                }

                robotX = world->getRobot()->get_pos().get_x();
                robotY = world->getRobot()->get_pos().get_y();
                
                // Measure distances to objects
                if (measureFlower) {
                    distEndFlower = sqrt(pow(flowerList[flowerType][0].x - robotX, 2.0) + pow(flowerList[flowerType][0].y - robotY, 2.0));
                    distDeltaFlower = distEndFlower - distBeginFlower;
                    outfile << distDeltaFlower << " ";
                }
                else
                    outfile << 0.0 << " ";
                
                if (measurePoison) {
                    distEndPoison = sqrt(pow(poisonList[poisonType][0].x - robotX, 2.0) + pow(poisonList[poisonType][0].y - robotY, 2.0));
                    distDeltaPoison = distEndPoison - distBeginPoison;
                    outfile << distDeltaPoison << " ";
                }
                else
                    outfile << 0.0 << " ";
            }
            // Compute the average of the distance for this environment
            // We take out the types for poison and flower
             
//            if (measureFlower) {
//                distDeltaFlower;
//                outfile << distDeltaFlower << " ";
//            }
//            else
//                outfile << 0.0 << " ";
//            if (measurePoison) {
//                distDeltaPoison /= (trialCount*1.0);
//                outfile << distDeltaPoison << " ";
//            }
//            else
//                outfile << 0.0 << " ";
            outfile << std::endl;
        }
        outfile.close();
    }
    
    
//    void FastSim_Flower_Poison::testCompass()
//    {
//
//        // Need to put the robot 
//
//        boost::shared_ptr<fastsim::Map> map = world->getMap();
//        double w = world->getMapWidth()/2.0;
//
//        double x;
//        double y;
//        double orient;
//
//        std::string labels[4] = {"TOP LEFT", "TOP RIGHT", "BOTTOM RIGHT", "BOTTOM LEFT"};
//        const double delta = M_PI / 36.0;
//        compass_info_t compassTarget;
//        compass_info_t compassPuck;
//        
//                
//        if (pucksList.size() != 1) {
//            std::cerr << "CompassTest: The number of pucks should be set to 1." << std::endl;
//            exit(1);
//        }
//            
//        // Move the puck in a specific place
//        pucksList[0].x = w;
//        pucksList[0].y = w*2.0 - 100.0;
//        pucksList[0].visible = true;
//        
//        for (unsigned p=0; p<4; ++p) {
//            std::cout << "**** Testing position " << labels[p] << std::endl;
//            switch (p) {
//                case 0:  // TOP LEFT
//                    x = 100.0;
//                    y = w * 2.0 - 100.0;
//                    break;
//                case 1:  // TOP RIGHT
//                    x = w * 2.0 - 100.0;
//                    y = w * 2.0 - 100.0;
//                    break;
//                case 2: // BOTTOM RIGHT
//                    x = w * 2.0 - 100.0;
//                    y = 100.0;
//                    break;
//                case 3: // BOTTOM LEFT
//                    x = 100.0;
//                    y = 100.0;
//                    break;
//                default:
//                    break;
//            }
//            orient = 0.0;
//            world->getRobot()->reinit();
//            world->moveRobot(x, y, orient);
//            
//            for (orient = 0.0; orient < 2.0*M_PI; orient += delta) {
//                world->moveRobot(x, y, orient);
//                world->updateRobot(0.0, 0.0);
//                world->step();
//                
//                compassTarget = computeCompassTarget();
//                compassPuck = computeCompassClosestPuck();
//                std::cout << "    robot: orient = " << orient << " ; compass2Target orient = " << compassTarget.orientation << " ; distance2Target = " << compassTarget.distance <<
//                        "; compass2Puck orient = " << compassPuck.orientation << " ; distance2Puck = " << compassPuck.distance<< std::endl;
//            }
//            std::cout << "---------------------------------------------------" << std::endl;
//            
//        }
//        
//    }

    
}
