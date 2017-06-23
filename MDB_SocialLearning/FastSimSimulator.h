/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastSimSimulator.h
 * Author: Julien Hubert
 *
 * Created on August 14, 2016, 12:24 PM
 */

#ifndef FASTSIMSIMULATOR_H
#define FASTSIMSIMULATOR_H

#include "Simulator.hpp"
#include "fastsim/headers/fastsim.hpp"
#include <string>
#include <boost/smart_ptr.hpp>

namespace MDB_Social {

    class FastSimSimulator : public Simulator
    {
    private:
        void configureRobotAs(std::string name, boost::shared_ptr<fastsim::Robot> r);
        
        boost::shared_ptr<fastsim::Robot> robot;
        boost::shared_ptr<fastsim::Map> simmap;
        fastsim::Settings* fsettings;
        #ifdef USE_FASTSIM_VIEWER
        fastsim::Display *viewer;
        #endif
        bool viewerActivated;
        std::string mapFilename;
        std::string configFilename;
        double mapWidth;
        std::string robotType;
        
        virtual void loadParameters() override;
        
        public:
        FastSimSimulator();
        virtual ~FastSimSimulator();

        virtual void step() override;
        void updateRobot(float v1, float v2);

        virtual void registerParameters() override;
        
        virtual void initialize() override;
        
        boost::shared_ptr<fastsim::Robot> getRobot() const;
        boost::shared_ptr<fastsim::Map> getMap() const;

        void moveRobot(double x, double y, double orient);
        
        void getLightSensors(std::vector<double>& distances);
        void getLaserSensors(std::vector<double>& distances);
        
        double getMapWidth() const;
        
        bool activateViewer(bool a);
    };

}
#endif /* FASTSIMSIMULATOR_H */

