/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSim_MultiRobot_Phototaxis.cpp
 * Author: Julien Hubert
 * 
 * Created on August 22, 2016, 5:21 PM
 */

#include "../MDB_SocialLearning/Settings.h"
#include "FastSim_MultiRobot_Phototaxis.h"
#include <cstdlib>
#include <algorithm>
#include "../MDB_SocialLearning/ResourceLibrary.hpp"
#include "../MDB_SocialLearning/ValueFunction.h"
#include "../MDB_SocialLearning/SocialManagerClient.h"
#ifdef USE_REV
#include "../RobotExperimentViewer/RobotExperimentViewer/revinit.h"
#include "../RobotExperimentViewer/RobotExperimentViewer/rev.h"
#endif

namespace MDB_Social {

    FastSim_MultiRobot_Phototaxis::FastSim_MultiRobot_Phototaxis() 
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
        
        useTracesWhenLightVisible = false;
        useSeeTheLightInputs = false;
        
#ifdef USE_REV
        rev = NULL;
        revinit = NULL;
#endif
//        robot->add_light_sensor(fastsim::LightSensor(1,0.0f,100.0f));
        
//        light = boost::shared_ptr<fastsim::IlluminatedSwitch>(new fastsim::IlluminatedSwitch(1, 5.0f, 300.0f, 300.0f, true));
//        world->getMap()->add_illuminated_switch(light);
    }

    FastSim_MultiRobot_Phototaxis::FastSim_MultiRobot_Phototaxis(const FastSim_MultiRobot_Phototaxis& orig) 
    {
        
    }

    FastSim_MultiRobot_Phototaxis::~FastSim_MultiRobot_Phototaxis() 
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

    void FastSim_MultiRobot_Phototaxis::registerParameters()
    {
        std::cout << "FastSim_MultiRobot_Phototaxis : registering the parameters...";
        std::cout.flush();
        Settings* settings = RobotID::getSettings();
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
        settings->registerParameter<bool>("experiment.useTracesWhenLightVisible", false, "Select only the traces when the light is visible to learn the value function.");
        settings->registerParameter<bool>("experiment.useSeeTheLightInputs", false, "Add an input to each light sensor to indicate is the light is in their field of view.");
        settings->registerParameter<bool>("experiment.showFastSimViewer", false, "Show the fastsim viewer for the simulator.");
        settings->registerParameter<bool>("experiment.showREV", false, "Show REV viewer for the simulator.");
        settings->registerParameter<bool>("experiment.realtime", false, "Play the experiment in realtime in the viewer.");
        settings->registerParameter<unsigned>("experiment.framerate", 25, "Framerate used to display the experiment in the viewer.");
        std::cout << " DONE" << std::endl;
        
    }
    
    void FastSim_MultiRobot_Phototaxis::loadParameters()
    {
        std::cout << "FastSim_MultiRobot_Phototaxis: Loading parameters..." << std::endl;
        Settings* settings = RobotID::getSettings();
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
            useOnlyTrueReward = settings->value<bool>("experiment.useOnlyTrueReward").second;
            valueFunctionTest = settings->value<bool>("experiment.valueFunctionTest").second;
            useOnlyRewardedStates = settings->value<bool>("experiment.useOnlyRewardedStates").second;
            useRestrictedVFasFitness = settings->value<bool>("experiment.useRestrictedVFasFitness").second;
            thresholdForVFasFitness = settings->value<double>("experiment.thresholdForVFasFitness").second;
            endTrialWhenOnReward = settings->value<bool>("experiment.endTrialWhenOnReward").second;
            maxTimeOnReward = settings->value<unsigned>("experiment.maxTimeOnReward").second;
            useTracesWhenLightVisible = settings->value<bool>("experiment.useTracesWhenLightVisible").second;
            useSeeTheLightInputs = settings->value<bool>("experiment.useSeeTheLightInputs").second;
            showFastSimViewer = settings->value<bool>("experiment.showFastSimViewer").second;
            showREV = settings->value<bool>("experiment.showREV").second;
            realtime = settings->value<bool>("experiment.realtime").second;
            framerate = settings->value<unsigned>("experiment.framerate").second;
            
            babbling->loadParameters("experiment");
            world->initialize();
            std::cout << "FastSim_MultiRobot_Phototaxis: Parameters loaded." << std::endl;
        }
        catch (std::exception e) {
            std::cerr << "FastSim_MultiRobot_Phototaxis: Error loading the parameters: " << e.what() << std::endl;
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
#else
            std::cerr << "FastSim_MultiRobot_Phototaxis: the REV viewer is not compiled in." << std::endl;
#endif            
        }
        
    }

    void FastSim_MultiRobot_Phototaxis::preprocessing() 
    {
        // Let's get the list of robots.
        SocialManagerClient* smclient = resourceLibrary->getSocialManagerClient();
//        SocialManagerClient* smclient = NULL;
//            std::cout << " p " << std::endl;
        if (smclient) {  // test if we are in a social environment
            
            smclient->synchronise();
            std::vector<std::string> robotIds = smclient->getRobotIDs();
            // We choose one random robot
            unsigned other_robot;
            do
                other_robot = lrand48() % robotIds.size();
            while (robotIds[other_robot] == RobotID::getID());
            // Retrieve its genotypes
            
            ValueFunction* vf = resourceLibrary->getValueFunction();
            vf->clearImportedValueFunction();
            ValueFunctionMemory* vfm;
            
            for (unsigned i=0; i<robotIds.size(); ++i) {
                vfm = smclient->getValueFunctionMemory(robotIds[i]);
                vf->addImportedValueFunction(vfm);
            }
            
            // Find the best policy and replace the worst current one
//            unsigned best = 0;
//            double bestFitness = (*pm)[best].getFitness();
//            for (unsigned i = 1; i < pm->size(); ++i) {
//                if ((*pm)[i].getFitness() > bestFitness) {
//                    best = i;
//                    bestFitness = (*pm)[best].getFitness();
//                }
//            }
//            // We have the best. We need to compare the fitness to the worst individual in our population
//            PolicyMemory* mypm = smclient->getPolicyMemory(getID());
//            unsigned worst = 0;
//            double worstFitness = (*mypm)[worst].getFitness();
//            for (unsigned i = 1; i < mypm->size(); ++i) {
//                if ((*mypm)[i].getFitness() < worstFitness) {
//                    worst = i;
//                    worstFitness = (*mypm)[worst].getFitness();
//                }
//            }
//            if (bestFitness > worstFitness) {
////                std::cout << " Social learning: copying " << robotIds[other_robot] << ":" << best << " to " << getID() << ":" << worst << std::endl;
//                resourceLibrary->getGeneticAlgorithm()->importGenotype(&(*pm)[best], worst);
//            }
        }        
    }
    
    void FastSim_MultiRobot_Phototaxis::postprocessing() 
    {
        
    }

    
    void FastSim_MultiRobot_Phototaxis::installGenotype(Genotype& individual)
    {
        std::vector<FeedforwardNN::weight_t> weights(individual.getSize());
        
        controller->getWeights(weights);

        if (weights.size() != individual.getSize()) {
            std::cerr << "FastSim_MultiRobot_Phototaxis: Size of the genotype (size = " << individual.getSize() << ") differs from the size of the network (size = " << weights.size() << ")." << std::endl;
            exit(1);
        }
        
        for (unsigned i=0; i<individual.getSize(); ++i) {
            weights[i].weight =  (controllerMaximumWeight - controllerMinimumWeight)*individual[i] + controllerMinimumWeight;
        }
        controller->setWeights(weights);
    }

    void FastSim_MultiRobot_Phototaxis::logRobotPosition(unsigned trial, unsigned epoch)
    {
        if (logRobotPos) {
            double x = world->getRobot()->get_pos().get_x();
            double y = world->getRobot()->get_pos().get_y();
            double orient = world->getRobot()->get_pos().theta();
            robotLogPosFile << trial << " " << epoch << " " << x << " " << y << " " << orient << std::endl;
        }
        
    }
    
    void FastSim_MultiRobot_Phototaxis::relocateRobot()
    {
        double w = world->getMapWidth();
        
        double robotRadius = world->getRobot()->get_radius();
        
        double x = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        double y = drand48() * (w*0.94 - 2*robotRadius)+robotRadius*1.1;
        double orient = drand48() * M_2_PI;
        world->getRobot()->reinit();
        world->moveRobot(x, y, orient);
    }

    bool FastSim_MultiRobot_Phototaxis::computeReward()
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
    
    double FastSim_MultiRobot_Phototaxis::evaluateFitness(Genotype& individual, unsigned gen, unsigned ind, bool _testIndividual)
    {
        testIndividual = _testIndividual;
        // In this experiment we evolve and test a policy. The policy uses the value function and the current state to decide what the next action should be.
        // This part only cares about the policy.
        // A reward will be given when the robot reaches a given distance from the light source.
        
        if (valueFunctionTest) {
            testValueFunction();
            return -1.0;
        }
        
        ResourceLibraryData* resourceLibrary = RobotID::getResourceLibrary();
        std::string cwd = resourceLibrary->getWorkingDirectory();
        
        std::cout << "* ";
        std::cout.flush();
        
        std::vector<double> lightSensors;
        std::vector<double> laserSensors;
        std::vector<double> nninputs(nbinputs);
        std::vector<double> nnoutput(nboutputs);
        bool reward;
        double tVFF= thresholdForVFasFitness; // * thresholdForVFasFitness;
        
        world->getLightSensors(lightSensors);   // Should be only one sensor
        if (nbinputs != lightSensors.size()*(1+useSeeTheLightInputs)+8) {
            std::cerr << "FastSim_Phototataxis: Error: nbinputs should be " << 2*(1+useSeeTheLightInputs)+8 << std::endl;
            exit(0);
        }
        
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
        
        if (showFastSimViewer) {
            if (!world->activateViewer(true))
                std::cout << Color::Modifier(Color::FG_RED) << "FastSim_MultiRobot_Phototaxis: ERROR: fastsim viewer not compiled in. Activate it using cmake -DUSE_FASTSIM_VIEWER=ON" << Color::Modifier(Color::FG_DEFAULT) << std::endl;
            else {
                std::cout << "FastSim_MultiRobot_Phototaxis: Viewer activated." << std::endl;
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
            onRewardZoneCounter = 0;
            enoughTimeOnReward = false;
            lreward = 0.0;
            for (epoch = 0; epoch < epochCount && !enoughTimeOnReward; ++epoch) {
                world->getLightSensors(lightSensors);   // Should be only one sensor
                reward = computeReward();
                if (reward > 1e-6)
                    onRewardZoneCounter++;
                else
                    onRewardZoneCounter = 0;
                enoughTimeOnReward = endTrialWhenOnReward && onRewardZoneCounter >= maxTimeOnReward;

                world->getLaserSensors(laserSensors);

                index = 0;
                nninputs[index++] = lightSensors[0];
                if (useSeeTheLightInputs)
                    nninputs[index++] = lightSensors[0] < 0.9;
                if (lightSensors.size() > 1) {
                    nninputs[index++] = lightSensors[1];
                    if (useSeeTheLightInputs)
                        nninputs[index++] = lightSensors[1] < 0.9;
                }
                std::copy(laserSensors.begin(), laserSensors.end(), nninputs.begin()+index);

                //                nninputs[nbinputs-1] = 1.0;   // bias
                if (testIndividual) {
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
                    if (!useTracesWhenLightVisible || (useTracesWhenLightVisible && (nninputs[0] < 0.8 || nninputs[1] < 0.8)))
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
        }

//        std::cout << "Phototaxis: rewardTotal = " << rewardTotal << " ; fitness = " << (rewardTotal*1.0)/(1.0*trialCount*epochCount) << std::endl;

        if (logRobotPos) {
            robotLogPosFile.close();
        }
        if (sensorLog)
            sensorLogFile.close();
        
        return rewardTotal/(1.0*trialCount);
    }

    void FastSim_MultiRobot_Phototaxis::testValueFunction()
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

        std::vector<double> lightSensors;
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


                    world->getLightSensors(lightSensors);   // Should be only one sensor

                    world->getLaserSensors(laserSensors);

                    index = 0;
                    nninputs[index++] = lightSensors[0];
                    if (useSeeTheLightInputs)
                        nninputs[index++] = lightSensors[0] < 0.9;
                    if (lightSensors.size() > 1) {
                        nninputs[index++] = lightSensors[1];
                        if (useSeeTheLightInputs)
                            nninputs[index++] = lightSensors[1] < 0.9;
                    }
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

    
}
