/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_Forage_Wall.cpp
 * Author: Julien Hubert
 * 
 * Created on May 16, 2017, 14:10 PM
 */

#include "../MDB_SocialLearning/Settings.h"
#include "FastSim_Forage_Wall.h"
#include <cstdlib>
#include <algorithm>
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/ValueFunction.h"
#ifdef USE_REV
#include "../RobotExperimentViewer/RobotExperimentViewer/revinit.h"
#include "../RobotExperimentViewer/RobotExperimentViewer/rev.h"
#endif

namespace MDB_Social {

    FastSim_Forage_Wall::FastSim_Forage_Wall(std::string id)
    :GAFitness(id)
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
        hiddenNeurons = 5;
        
        compassTest = false;
        fitnessComparisonTest = false;
        
        controllerMinimumWeight = -10.0;
        controllerMaximumWeight = 10.0;
        
        babbling = static_cast<BabblingStandard*>(ModelLibrary::getModel("BabblingStandard", getID()));
        std::vector<double> outmin(nboutputs, 0.2);
        std::vector<double> outmax(nboutputs, 1.0);
        babbling->setOutputMinMax(outmin,outmax);
        babbling->registerParameters("experiment");
        useOnlyBabbling = false;
        
        logRobotPos = false;
        sensorLog = false;
        valueFunctionTest = false;
                
#ifdef USE_REV
        rev = NULL;
        revinit = NULL;
#endif
//        robot->add_light_sensor(fastsim::LightSensor(1,0.0f,100.0f));
        
//        light = boost::shared_ptr<fastsim::IlluminatedSwitch>(new fastsim::IlluminatedSwitch(1, 5.0f, 300.0f, 300.0f, true));
//        world->getMap()->add_illuminated_switch(light);
    }

    FastSim_Forage_Wall::FastSim_Forage_Wall(const FastSim_Forage_Wall& orig)
    {
        
    }

    FastSim_Forage_Wall::~FastSim_Forage_Wall()
    {
        delete world;
        delete controller;
        delete babbling;
#ifdef USE_REV
        delete revinit;
        revinit = NULL;
        rev = NULL;
#endif        
    }

    void FastSim_Forage_Wall::registerParameters()
    {
        std::cout << "FastSim_Forage_Wall : registering the parameters...";
        std::cout.flush();
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<unsigned>("experiment.nbinputs", 1, "Number of inputs/sensors on the neural network.");
        settings->registerParameter<unsigned>("experiment.nboutputs", 2, "Number of outputs on the neural network.");
        settings->registerParameter<unsigned>("experiment.hiddenNeurons", 10, "Number of hidden neurons for the controller.");
        settings->registerParameter<double>("experiment.controllerMinimumWeight", 0.0, "Minimum weight of the neural network.");
        settings->registerParameter<double>("experiment.controllerMaximumWeight", 0.0, "Maximum weight of the neural network.");
        settings->registerParameter<int>("experiment.trialCount", 0, "Number of trials for the experiment.");
        settings->registerParameter<int>("experiment.epochCount", 0, "Number of steps for one trial of the experiment.");
        settings->registerParameter<double>("experiment.timestep", 0.01, "Simulation step of the experiment in seconds.");
        settings->registerParameter<double>("experiment.rewardZoneDiameter", 10.0, "Diameter of the reward zone.");
        settings->registerParameter<bool>("experiment.useOnlyBabbling", false, "Use only the babbling controller.");
        settings->registerParameter<bool>("experiment.useOnlyTrueReward", false, "Use only true rewards, no estimation through the value function");
        settings->registerParameter<bool>("experiment.logRobotPosition", false, "Output the position of the robot in a log file.");
        settings->registerParameter<bool>("experiment.sensorLogFlag", false, "Log the sensors values during the experiment.");
        settings->registerParameter<double>("experiment.maxSpeed", 5, "Maximum speed of the robot.");
        settings->registerParameter<bool>("experiment.valueFunctionTest", false, "Produce a map of the environment in terms of potential reward from the VF.");
        settings->registerParameter<bool>("experiment.useOnlyRewardedStates", false, "Keep only rewarded states in the traces to train the value function.");
        settings->registerParameter<bool>("experiment.useRestrictedVFasFitness", false, "Use VF as fitness only when the current trace has been encountered before.");
        settings->registerParameter<double>("experiment.thresholdForVFasFitness", 0.5, "Minimum distance between a trace and the traces in memory to use VF as fitness.");
        settings->registerParameter<bool>("experiment.endTrialWhenOnReward", false, "Limit the time allowed in the reward zone before ending the trial.");
        settings->registerParameter<unsigned>("experiment.maxTimeOnReward", 0, "Maximum time allowed in the reward zone before ending the trial.");
        settings->registerParameter<bool>("experiment.showFastSimViewer", false, "Show the fastsim viewer for the simulator.");
        settings->registerParameter<bool>("experiment.showREV", false, "Show REV viewer for the simulator.");
        settings->registerParameter<bool>("experiment.realtime", false, "Play the experiment in realtime in the viewer.");
        settings->registerParameter<unsigned>("experiment.framerate", 25, "Framerate used to display the experiment in the viewer.");
        settings->registerParameter<bool>("experiment.compassTest", false, "Test the compass output by rotating the robot at different location and printing the readings.");
        settings->registerParameter<bool>("experiment.fitnessComparisonTest", false, "Test the individual by using the learned and perfect fitness for comparison purposes.");
        settings->registerParameter<bool>("experiment.printInputsOutputs", false, "During testing, print the input and outputs of the neural network.");
        settings->registerParameter<unsigned>("experiment.numberBalls", 1, "Number of balls in arena");
        settings->registerParameter<double>("experiment.diameterTarget", 1.0, "Diameter of the target zone");
        
        
        std::cout << " DONE" << std::endl;
        
    }
    
    void FastSim_Forage_Wall::loadParameters()
    {
        std::cout << "FastSim_Forage_Wall: Loading parameters..." << std::endl;
//        Settings* settings = Settings::getInstance();
        try {
            nbinputs = settings->value<unsigned>("experiment.nbinputs").second;
            nboutputs = settings->value<unsigned>("experiment.nboutputs").second;
            hiddenNeurons = settings->value<unsigned>("experiment.hiddenNeurons").second;
            controllerMinimumWeight = settings->value<double>("experiment.controllerMinimumWeight").second;
            controllerMaximumWeight = settings->value<double>("experiment.controllerMaximumWeight").second;
            trialCount = settings->value<int>("experiment.trialCount").second;
            epochCount = settings->value<int>("experiment.epochCount").second;
            timestep = settings->value<double>("experiment.timestep").second;
            rewardZoneDiameter = settings->value<double>("experiment.rewardZoneDiameter").second;
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
            realtime = settings->value<bool>("experiment.realtime").second;
            framerate = settings->value<unsigned>("experiment.framerate").second;
            compassTest = settings->value<bool>("experiment.compassTest").second;
            fitnessComparisonTest = settings->value<bool>("experiment.fitnessComparisonTest").second;
            printInputsOutputs = settings->value<bool>("experiment.printInputsOutputs").second;
            numberBalls = settings->value<unsigned>("experiment.numberBalls").second;
            diameterTarget = settings->value<double>("experiment.diameterTarget").second;
            
            babbling->loadParameters("experiment");
            world->initialize();
            std::cout << "FastSim_Forage_Wall: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "FastSim_Forage_Wall: Error loading the parameters: " << e.what() << std::endl;
            exit(1);
        }
        
        controller = static_cast<FeedforwardNN*>(ModelLibrary::getModel("Feedforward", getID()));
        std::vector<enum FeedforwardNN::ActivationFunction> activationFunctions(1, FeedforwardNN::SIGMOID);
        if (hiddenNeurons) {
            layers.resize(3);
            layers[0] = nbinputs;
            layers[1] = hiddenNeurons;
            layers[2] = nboutputs;
        }
        else {
            layers.resize(2);
            layers[0] = nbinputs;
            layers[1] = nboutputs;
        }
        controller->setup(layers, activationFunctions);
        
        pucksList.resize(numberBalls);
        
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
            rev->addZone(rewardZoneDiameter/2.0, wwidth/2.0, wwidth/2.0, REV::Color(127, 127, 127, 255)); // START
            rev->addZone(diameterTarget/2.0, wwidth/2.0, wwidth/2.0, REV::Color(0, 0, 0, 255));
            
#else
            std::cerr << "FastSim_Forage_Wall: the REV viewer is not compiled in." << std::endl;
#endif            
        }
        
    }

    
    void FastSim_Forage_Wall::installGenotype(Genotype& individual)
    {
        std::vector<FeedforwardNN::weight_t> weights(individual.getSize());
        
        controller->getWeights(weights);

        if (weights.size() != individual.getSize()) {
            std::cerr << "FastSim_Forage_Wall: Size of the genotype (size = " << individual.getSize() << ") differs from the size of the network (size = " << weights.size() << ")." << std::endl;
            exit(1);
        }
        
        for (unsigned i=0; i<individual.getSize(); ++i) {
            weights[i].weight =  (controllerMaximumWeight - controllerMinimumWeight)*individual[i] + controllerMinimumWeight;
        }
        controller->setWeights(weights);
    }

    void FastSim_Forage_Wall::logRobotPosition(unsigned trial, unsigned epoch)
    {
        if (logRobotPos) {
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double orient = world->getRobot()->get_pos().theta();
            robotLogPosFile << trial << " " << epoch << " " << x << " " << y << " " << orient << std::endl;
        }
        
    }
    
    void FastSim_Forage_Wall::relocateRobot()
    {
        // Todo: not relocate robot on the target area
        double w = world->getMapWidth();
        
        double robotRadius = world->getRobot()->get_radius();
        
        double x;
        double y;
        
        do {
        x = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        y = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        } while (checkCollisionAllPucks(x,y,robotRadius*2) ||  checkCollisionTarget(x,y,robotRadius*2));
            
        double orient = drand48() * M_2_PI;
        world->getRobot()->reinit();
        world->moveRobot(x, y, orient);
    }
    
    void FastSim_Forage_Wall::relocateBalls()
    {
        
        for (unsigned p=0; p<numberBalls ; ++p) {
            relocateBall(p);
        }
        
    }
        
    void FastSim_Forage_Wall::relocateBall(unsigned p)
    {
        double w = world->getMapWidth();
        
        double puckRadius = pucksList[p].d/2.0;
        
        double x;
        double y;
        do {
        x = drand48() * (w*0.94 - 2*puckRadius)+puckRadius*1.1;
        y = drand48() * (w*0.94 - 2*puckRadius)+puckRadius*1.1;
        } while (checkCollisionOtherPucks(x,y,puckRadius*2,p) ||  checkCollisionTarget(x,y,puckRadius*2));
        
        // relocate ball
        pucksList[p].x = x;
        pucksList[p].y = y;
        pucksList[p].visible = true;
                
    }
        
    
    bool FastSim_Forage_Wall::checkCollisionTarget(double x, double y, double d)
    {
        bool collision = false;
        double w = world->getMapWidth()/2.0;
            
        //check collision target. NOTE: only works if target is in squared center
        double dist = pow(x-w,2.0) + pow(y-w,2.0);
        if (dist < pow(d/2.0+(diameterTarget/2.0),2.0)) {
            collision = true;
        }
                        
        return collision;
    }
    
    bool FastSim_Forage_Wall::checkCollisionAllPucks(double x, double y, double d)
    {
        bool collision = false;
        
        //check collision per puck
        for (unsigned p=0; p<numberBalls  && !collision ; ++p) {
            collision = checkCollisionOnePuck(x,y,d,p);
        }
        
        return collision;
    }
            
    bool FastSim_Forage_Wall::checkCollisionOtherPucks(double x, double y, double d, unsigned p)
    {
        bool collision = false;
                
        //check collision other pucks then given puck p
        for (unsigned i=0; i<numberBalls  && !collision ; ++i) {
            if (!(i==p)){
                collision = checkCollisionOnePuck(x,y,d,i);
            }
        }
                
        return collision;
    }

    
    bool FastSim_Forage_Wall::checkCollisionOnePuck(double x, double y, double d, unsigned p)
    {
        // check collision with specific puck id
        bool collision = false;
        
        double dist = pow(x-pucksList[p].x, 2.0) + pow(y-pucksList[p].y,2.0);
        if (dist < pow(d/2.0+(pucksList[p].d/2.0),2.0))
                collision = true;
            
                            
        return collision;
    }
                

    bool FastSim_Forage_Wall::computeReward()
    {
        // Compute the distance between the goal and the robot, and compute the reward accordingly.
        double x = world->getRobot()->get_pos().get_x();
        double y = world->getRobot()->get_pos().get_y();
        
        fastsim::Map::ill_sw_t ilswitch = world->getMap()->get_illuminated_switch_by_color(1);
        double ilx = ilswitch->get_x();
        double ily = ilswitch->get_y();
        double dist = pow(x-ilx, 2.0)+ pow(y-ily, 2.0);
        
//        std::cout << "Phototaxis : dist = " << dist << std::endl;
        
        return dist <= rewardZoneDiameter*rewardZoneDiameter;
        
    }

    double FastSim_Forage_Wall::computeDistance()
    {
        // Compute the distance between the goal and the robot, and compute the reward accordingly.
        double x = world->getRobot()->get_pos().get_x();
        double y = world->getRobot()->get_pos().get_y();
        
        fastsim::Map::ill_sw_t ilswitch = world->getMap()->get_illuminated_switch_by_color(1);
        double ilx = ilswitch->get_x();
        double ily = ilswitch->get_y();
        double dist = sqrt(pow(x-ilx, 2.0)+ pow(y-ily, 2.0));
        
//        std::cout << "Phototaxis : dist = " << dist << std::endl;
        
        return dist;
        
    }
    
    FastSim_Forage_Wall::compass_info_t FastSim_Forage_Wall::computeCompass()
    {
        compass_info_t ret;
        double x = world->getRobot()->get_pos().get_x();
        double y = world->getRobot()->get_pos().get_y();
        double rorientation = world->getRobot()->get_pos().theta();
        
        boost::shared_ptr<fastsim::Map> map = world->getMap();
        std::vector<fastsim::Map::ill_sw_t> lights = map->get_illuminated_switches();
        double gx = lights[0]->get_x();
        double gy = lights[0]->get_y();
        double max_distance_squared = gx*gx+gy*gy;

        ret.orientation = atan2((-y+gy),(gx-x)) - rorientation;
        if (ret.orientation < -M_PI)
            ret.orientation += 2*M_PI;
//        if (ret.orientation < 0.0)
//            ret.orientation += 2*M_PI;

        double abx = x-gx;
        double aby = y-gy;
        ret.distance = (abx*abx + aby*aby) / max_distance_squared;

        return ret;
    }

    
    
    double FastSim_Forage_Wall::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
        // In this experiment we evolve and test a policy. The policy uses the value function and the current state to decide what the next action should be.
        // This part only cares about the policy.
        // A reward will be given when the robot reaches a given distance from the light source.
        
        testIndividual = _testIndividual;
        
        if (valueFunctionTest) {
            testValueFunction();
            return -1.0;
        }
        
        if (compassTest) {
            testCompass();
            return -1.0;
        }
        
        std::string cwd = resourceLibrary->getWorkingDirectory();
        
        std::cout << "* ";
        std::cout.flush();
        
//        std::vector<double> lightSensors;
        compass_info_t compass;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        bool reward;
        double tVFF= thresholdForVFasFitness; // * thresholdForVFasFitness;
        
//        world->getLightSensors(lightSensors);   // Should be only one sensor
//        if (nbinputs != lightSensors.size()*(1+useSeeTheLightInputs)+8) {
//            std::cerr << "FastSim_Forage_Wall: Error: nbinputs should be " << 2*(1+useSeeTheLightInputs)+8 << std::endl;
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
        unsigned onRewardZoneCounter;
        bool enoughTimeOnReward = false;
        double lreward;
        unsigned epoch;
        unsigned index;
        
        double closest;
        double fartest;
        double distFitness = 0.0;
        double dist;
        
        if (showFastSimViewer) {
            if (!world->activateViewer(true))
                std::cout << Color::Modifier(Color::FG_RED) << "FastSim_Forage_Wall: ERROR: fastsim viewer not compiled in. Activate it using cmake -DUSE_FASTSIM_VIEWER=ON" << Color::Modifier(Color::FG_DEFAULT) << std::endl;
            else {
                std::cout << "FastSim_Forage_Wall: Viewer activated." << std::endl;
            }
        }
        
        for (unsigned trial = 0; trial < trialCount; ++trial) {
            controller->reset();
            relocateRobot();
#ifdef USE_REV
            if (showREV) {
                double x = world->getRobot()->get_pos().get_x();
                double y = world->getRobot()->get_pos().get_y();
                double orient = world->getRobot()->get_pos().theta();
                rev->setRobotPosition(x, y, orient);
            }
#endif            
            
            if (fitnessComparisonTest) {
                closest = computeDistance();
                fartest = closest;
            }
            
            onRewardZoneCounter = 0;
            enoughTimeOnReward = false;
            lreward = 0.0;
            for (epoch = 0; epoch < epochCount && !enoughTimeOnReward; ++epoch) {
//                world->getLightSensors(lightSensors);   // Should be only one sensor
                reward = computeReward();
                if (reward > 1e-6)
                    onRewardZoneCounter++;
                else
                    onRewardZoneCounter = 0;
                enoughTimeOnReward = endTrialWhenOnReward && onRewardZoneCounter >= maxTimeOnReward;

                world->getLaserSensors(laserSensors);
                compass = computeCompass();
                
                index = 0;
                nninputs[index++] = compass.orientation;
                nninputs[index++] = compass.distance;
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

#ifdef USE_REV
                if (showREV) {
                    double x = world->getRobot()->get_pos().get_x();
                    double y = world->getRobot()->get_pos().get_y();
                    double orient = world->getRobot()->get_pos().theta();
                    rev->setRobotPosition(x, y, orient);
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
                
                if (fitnessComparisonTest) {
                    dist = computeDistance();

                    if (dist > fartest) {
                        closest = dist;
                        fartest = dist;
                    }
                    else if (dist < closest)
                        closest = dist;
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
                lreward += std::max((double)reward, trace.estimated_reward);
                // need to compute the fitness now. Normally it should be the VF, but we don't have VF now...
            }
            if (testIndividual) {
                std::cout << "Trial " << trial << " : total reward = " << lreward << " for " << epoch << " timesteps" << std::endl;
            }
            rewardTotal += lreward / epoch;
            
            if (fitnessComparisonTest)
                distFitness += (fartest-closest)/fartest; //+ (1.0*epoch) / epochCount;

        }

//        std::cout << "Phototaxis: rewardTotal = " << rewardTotal << " ; fitness = " << (rewardTotal*1.0)/(1.0*trialCount*epochCount) << std::endl;

        if (logRobotPos) {
            robotLogPosFile.close();
        }
        if (sensorLog)
            sensorLogFile.close();
        
        if (fitnessComparisonTest) {
            std::ofstream fcfile("fitnessComparisonTest.log", std::ios_base::app);
            if (!fcfile.is_open()) {
                std::cerr << "FastSim_Forage_Wall: Error opening the log file fitnessComparisonTest.log." << std::endl;
                exit(1);
            }
            fcfile << rewardTotal/(1.0*trialCount) << " " << distFitness / (1.0*trialCount) << std::endl;
            fcfile.close();
        }
        
        return rewardTotal/(1.0*trialCount);
    }

    void FastSim_Forage_Wall::testValueFunction()
    {
        // This function tests the value function by placing the robot following a grid and measuring the response of the VF.
        // The robot will be tested with multiple orientations: one facing the light, one facing away from it.

        std::cout << "Testing the value function:" ;
        std::cout.flush();
        
        double robotRadius = world->getRobot()->get_radius();
        double w = world->getMapWidth() - 1.1*robotRadius;
        double deltax = 1.0;
        double deltay = 1.0;
        double orient = 0.0;

        compass_info_t compass;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        double estimated_reward;
        ValueFunction* vf = resourceLibrary->getValueFunction();
        vf->saveValueFunctionToFile("value_function.log");

        boost::shared_ptr<fastsim::Map> map = world->getMap();
        std::vector<fastsim::Map::ill_sw_t> lights = map->get_illuminated_switches();
        double lightx = lights[0]->get_x();
        double lighty = lights[0]->get_y();
        
        Trace trace;
        trace.estimated_reward = 0.0;
        trace.expected_reward = 0.0;
        trace.reliability = 0.0;
        
        unsigned nbsteps = (unsigned)std::ceil((w * w) / (deltax * deltay));
        unsigned stepMark = nbsteps / 100;
        unsigned mark = 0;
        unsigned index;
        
        std::ofstream outfile("valueFunctionTest.log", std::ios_base::trunc);
        
        for (double x = robotRadius*1.1; x < w ; x+=deltax) {
            for (double y = robotRadius*1.1; y < w; y+=deltay) {
                
                if ( std::fmod(x*y,stepMark) == 0  ) {
                    std::cout << " " << mark++ << "%";
                    std::cout.flush();
                }

                orient = atan2(lighty-y,lightx-x);   // Looking toward it
                for (unsigned o=0; o<2; ++o) {

                    world->getRobot()->reinit();
                    world->moveRobot(x, y, orient);


                    world->getLaserSensors(laserSensors);
                    compass = computeCompass();

                    index = 0;
                    nninputs[index++] = compass.orientation;
                    nninputs[index++] = compass.distance;
                    std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin()+index);
//                    nninputs[nbinputs-1] = 1.0;   // bias


                    world->updateRobot(0.0, 0.0);
                    world->step();


                    // I need to record the trace in the traceMemory
                    trace.true_reward = 0.0;
                    trace.inputs = nninputs;
                    trace.outputs = nnoutput;

                    estimated_reward = vf->estimateTrace(trace);
                    
                    outfile << x << " " << y << " " << orient << " " << o << " " << estimated_reward << std::endl;;

                    orient += M_PI;   // Computing the opposite angle
                    if (orient > M_2_PI)
                        orient -= M_2_PI;
                }
            }
        }
        outfile.close();
    }

    void FastSim_Forage_Wall::testCompass()
    {

        // Need to put the robot 

        boost::shared_ptr<fastsim::Map> map = world->getMap();
        std::vector<fastsim::Map::ill_sw_t> lights = map->get_illuminated_switches();
        double gx = lights[0]->get_x();
        double gy = lights[0]->get_y();

        double x;
        double y;
        double orient;

        std::string labels[4] = {"TOP", "RIGHT", "BOTTOM", "LEFT"};
        const double delta = M_PI / 36.0;
        compass_info_t compass;
                
        for (unsigned p=0; p<4; ++p) {
            std::cout << "**** Testing position " << labels[p] << std::endl;
            switch (p) {
                case 0:  // TOP
                    x = gx;
                    y = gy/2.0;
                    break;
                case 1:  // RIGHT
                    x = gx * 1.5;
                    y = gy;
                    break;
                case 2: // BOTTOM
                    x = gx;
                    y = gy*1.5;
                    break;
                case 3: // LEFT
                    x = gx * 0.5;
                    y = gy;
                    break;
                default:
                    break;
            }
            orient = 0.0;
            world->getRobot()->reinit();
            world->moveRobot(x, y, orient);
            
            for (orient = 0.0; orient < 2.0*M_PI; orient += delta) {
                world->moveRobot(x, y, orient);
                world->updateRobot(0.0, 0.0);
                world->step();
                
                compass = computeCompass();
                std::cout << "    robot orient = " << orient << " ; compass orient = " << compass.orientation << " ; distance = " << compass.distance << std::endl;
            }
            std::cout << "---------------------------------------------------" << std::endl;
            
        }
        
    }

    
}
