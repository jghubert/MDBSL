/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSimSimulator.cpp
 * Author: Julien Hubert
 * 
 * Created on August 14, 2016, 12:24 PM
 */

#include "FastSimSimulator.h"
#include "Settings.h"

namespace MDB_Social {

    FastSimSimulator::FastSimSimulator()
    {
        
    }

    FastSimSimulator::~FastSimSimulator() 
    {
        delete fsettings;
    }

    void FastSimSimulator::step()
    {
        simmap->update(robot->get_pos());
    }
    
    void FastSimSimulator::configureRobotAs(std::string name, boost::shared_ptr<fastsim::Robot> r)
    {
        if (name == "thymio") {
            
        }
        
        if (name == "epuck") {

        }
    }
    
    void FastSimSimulator::updateRobot(float v1, float v2)
    {
        robot->move(v1, v2, simmap);
    }

    void FastSimSimulator::registerParameters()
    {
        std::cout << "FastSimSimulator::registerParameters : ID = " << getID() << std::endl;
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<std::string>("simulator.fastsim.robotType", std::string("thymio"), "Indicate which robot to use in the simulator.");
        settings->registerParameter<std::string>("simulator.fastsim.mapfilename", std::string("NONE"), "Filename containing the map for the experiment.");
        settings->registerParameter<double>("simulator.fastsim.mapWidth", 100.0, "Size of the squared map.");
        settings->registerParameter<std::string>("simulator.fastsim.configFilename", "fastsim.cfg", "Name of the configuration file for the robot and environment.");
    }
    
    void FastSimSimulator::loadParameters()
    {
        std::cout << "FastSimSimulator::registerParameters : ID = " << getID() << std::endl;
        std::cout << "FastSimSimulator: loading parameters..." << std::endl;
//        Settings* settings = Settings::getInstance();
        try {
            mapFilename = settings->value<std::string>("simulator.fastsim.mapfilename").second;
            mapWidth = settings->value<double>("simulator.fastsim.mapWidth").second;
            robotType = settings->value<std::string>("simulator.fastsim.robotType").second;
            configFilename = settings->value<std::string>("simulator.fastsim.configFilename").second;
            std::cout << "FastSimSimulator: parameters loaded." << std::endl;
        } catch (std::exception e) {
            std::cerr << "FastSimSimulator: Error while loading the parameters: " << e.what() << std::endl;
            exit(1);
        }
    }

    boost::shared_ptr<fastsim::Robot> FastSimSimulator::getRobot() const
    {
        return robot;
    }

    boost::shared_ptr<fastsim::Map> FastSimSimulator::getMap() const
    {
        return simmap;
    }
    
    void FastSimSimulator::initialize()
    {
        loadParameters();
        
        try {
            std::cout << "FastSimSimulator: Loading config file " << configFilename << std::endl;
            fsettings = new fastsim::Settings(configFilename);
            std::cout << "FastSimSimulator: config file loaded." << std::endl;
            robot = fsettings->robot();
            simmap = fsettings->map();
        }
        catch(std::exception e) {
            std::cerr << "FastSimSimulator: Failed to open config file: " << e.what() << std::endl;
            exit(1);
        }
    }

    void FastSimSimulator::moveRobot(double x, double y, double orient)
    {
        robot->set_pos(fastsim::Posture(x, y, orient));
    }

    
    double FastSimSimulator::getMapWidth() const
    {
        return mapWidth;
    }

    void FastSimSimulator::getLightSensors(std::vector<double>& distances)
    {
        std::vector<fastsim::LightSensor> ls = robot->get_light_sensors();
        distances.resize(ls.size());
        
        double dist;
        for (unsigned i=0; i<ls.size(); ++i) {
            dist = ls[i].get_distance();
            if (dist < -0.5)  // test for dist == -1
                dist = mapWidth*M_SQRT2;   // maximum distance that can be measured in the world is the hypothenuse
            dist *= (0.95+drand48()*0.1);  // Adding some noise
            distances[i] = dist / (mapWidth*M_SQRT2*1.05);
        }
    }
    
    void FastSimSimulator::getLaserSensors(std::vector<double>& distances)
    {
        std::vector<fastsim::Laser> lasers = robot->get_lasers();
        distances.resize(lasers.size());
        
        for (unsigned i=0; i<lasers.size(); ++i) {
            distances[i] = lasers[i].get_dist();
            if (distances[i] < -0.5)
                distances[i] = lasers[i].get_range();
            distances[i] = distances[i]/(lasers[i].get_range());
        }
            
    }
    
    
    
}
