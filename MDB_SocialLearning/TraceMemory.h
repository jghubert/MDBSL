/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TraceMemory.h
 * Author: Julien Hubert
 *
 * Created on August 4, 2016, 11:48 AM
 */

#ifndef TRACEMEMORY_H
#define TRACEMEMORY_H

#include <vector>
#include <fstream>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>
#include "Memory.hpp"

namespace MDB_Social {

    class Trace {
    public:
        enum DistanceMeasure {
            EUCLIDIAN,
            EUCLIDIAN2,
            COSINE,
            COSINE2
        };
        
        Trace();
        Trace(const Trace& t);
        virtual ~Trace();
        
        Trace& operator=(Trace t);
        
        std::vector<double> inputs;   // vector of sensor values
        std::vector<double> outputs;    // set of actions (e.g. motor speeds)
        std::vector<double> inputs_t1;   // inputs after applying the action
        double estimated_reward;  // Reward from value function
        double expected_reward;  // rewards computed from the backpropagation process along the trace
        double true_reward;      // Reward received in the task
        double reliability;
        bool usedForVFTraining;
        boost::uuids::uuid uuid;
        
        double computeInputDistance(Trace& t, enum DistanceMeasure dm = EUCLIDIAN);
        
        friend std::ostream& operator<<(std::ostream& output, const Trace& T)
        {
            output << T.inputs.size();
            for (unsigned i=0; i<T.inputs.size(); ++i)
                output << SEPARATOR << T.inputs[i];
            output << SEPARATOR << T.inputs_t1.size();
            for (unsigned i=0; i<T.inputs_t1.size(); ++i)
                output << SEPARATOR << T.inputs_t1[i];
            output << SEPARATOR << T.outputs.size();
            for (unsigned i=0; i<T.outputs.size(); ++i)
                output << SEPARATOR << T.outputs[i];
            output << SEPARATOR << T.true_reward << SEPARATOR << T.expected_reward << SEPARATOR << T.estimated_reward << SEPARATOR << T.reliability
                    << SEPARATOR << T.usedForVFTraining << SEPARATOR << T.uuid;
            
//            std::cout << "Trace::operator<< : uuid = " << T.uuid << std::endl;
            
            return output;
        }
        
        friend std::istream& operator>>(std::istream& input, Trace& T)
        {
            size_t s;
            // inputs
            input >> s;
            if (input.good())
                T.inputs.resize(s);
            else
                return input;
            
            unsigned index = 0;
            while (index < s && input.good()) {
                input >> T.inputs[index++];
            }
            
            // inputs_t1
            if (input.good())
                input >> s;
            else
                return input;
            if (input.good())
                T.inputs_t1.resize(s);
            else
                return input;
            
            index = 0;
            while (index < s && input.good()) {
                input >> T.inputs_t1[index++];
            }
            
            // outputs
            if (input.good())
                input >> s;
            else
                return input;
            if (input.good())
                T.outputs.resize(s);
            else
                return input;
            
            index = 0;
            while (index < s && input.good()) {
                input >> T.outputs[index++];
            }
            
            input >> T.true_reward >> T.expected_reward >> T.estimated_reward >> T.reliability >> T.usedForVFTraining;

            if (input.peek() != '\n')
                input >> T.uuid;
            else {
                T.uuid = boost::uuids::nil_uuid();
            }
            
            return input;
        }
    };
    
    class TraceMemory: public Memory<Trace> {
    private:
        boost::uuids::uuid defaultUUID;
        
    public:
        TraceMemory();
        virtual ~TraceMemory();

        double computeShortestDistance(Trace& t, enum Trace::DistanceMeasure dm = Trace::DistanceMeasure::EUCLIDIAN);
        
        void setDefaultUUID(boost::uuids::uuid& _uuid);
        void resetDefaultUUID();
        
        virtual void push_front(Trace& d) override;
        virtual void push_back(Trace& d) override;
        
        
    };
}
#endif /* TRACEMEMORY_H */

