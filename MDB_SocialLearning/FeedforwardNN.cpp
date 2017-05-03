/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FeedforwardNN.cpp
 * Author: Julien Hubert
 * 
 * Created on June 16, 2016, 4:32 PM
 */

#include <boost/smart_ptr/shared_array.hpp>
#include "Settings.h"

#include "FeedforwardNN.h"

namespace MDB_Social {

    FeedforwardNN::FeedforwardNN(std::string id) :
        Model("FeedforwardNN", id)
    {
        minWeights = 0.0;
        maxWeights = 0.1;
        
        desiredTrainingError = 0.01;
        maxTrainingEpoch = 1000;
        
        externalMemory = NULL;
        
        nbinputs = 0;
        nboutputs = 0;
    }
    
    FeedforwardNN::~FeedforwardNN()
    {
        
    }

    bool FeedforwardNN::setup(const std::vector<unsigned>& layers, std::vector<enum FeedforwardNN::ActivationFunction>& activationFunctions)
    {
        bool ret = false;
        if (layers.size() >1) {
            if (layers[0]>0 && layers[1]>0) {
                ret = net.create_standard_array(layers.size(), layers.data());
                if (ret && activationFunctions.size() > 0) {
                    std::vector<enum ActivationFunction> af;
                    if (activationFunctions.size() == 1)
                        af.resize(layers.size(), activationFunctions[0]);
                    else
                        af = activationFunctions;
                    for (unsigned i=1; i < layers.size(); ++i)   // Starts at one
                        net.set_activation_function_layer((enum FANN::activation_function_enum)af[i], i);
                    reset();
                }
            }
            nbinputs = layers[0];
            nboutputs = layers.back();
        }
        return ret;
    }

    void FeedforwardNN::setActivationFunctionSteepness(std::vector<double>& steepness)
    {
        for (unsigned i=1; i < steepness.size(); ++i)
            net.set_activation_steepness_layer(steepness[i], i);
    }

    void FeedforwardNN::setActivationFunctionSteepness(double steepness)
    {
        for (unsigned i=1; i < net.get_num_layers(); ++i)
            net.set_activation_steepness_layer(steepness, i);
    }

    void FeedforwardNN::resetWeights()
    {
        net.randomize_weights(minWeights, maxWeights);
    }
    
    void FeedforwardNN::reset()
    {
        
    }
    
    void FeedforwardNN::setWeightsMinMax(double mi, double ma)
    {
        minWeights = mi;
        maxWeights = ma;
    }

    bool FeedforwardNN::setWeights(std::vector<weight_t>& weights)
    {
        if (weights.size() != net.get_total_connections())
            return false;
        
        size_t size = weights.size();
        for (unsigned i=0; i<size; ++i)
            net.set_weight(weights[i].from, weights[i].to, weights[i].weight);
        
        return true;
    }
    
    void FeedforwardNN::getWeights(std::vector<weight_t>& weights)
    {
        unsigned totcon = net.get_total_connections();
        weights.resize(totcon);
        
        FANN::connection* con = new FANN::connection[totcon];
        
        net.get_connection_array(con);
        for (unsigned i=0; i<totcon; ++i) {
            weights[i].from = con[i].from_neuron;
            weights[i].to = con[i].to_neuron;
            weights[i].weight = con[i].weight;
        }
        
        delete[] con;
    }


    std::vector<unsigned> FeedforwardNN::getLayerArray()
    {
        
        unsigned* layers = new unsigned[net.get_num_layers()];
        
        net.get_layer_array(layers);
        
        std::vector<unsigned> ret(net.get_num_layers());
        ret.assign(layers, layers+net.get_num_layers());
        
        delete[] layers;
        return ret;
    }

    
    void FeedforwardNN::addToTrainingSet(std::vector<double>& input, std::vector<double>& output)
    {
        if (trainingData.length_train_data() == 0) {
            double* in = input.data();
            double* out = output.data();
            trainingData.set_train_data(1, input.size(), &in, output.size(), &out);
        }
        else {
            FANN::training_data td;
            double* in = input.data();
            double* out = output.data();
            td.set_train_data(1, input.size(), &in, output.size(), &out);
            trainingData.merge_train_data(td);
        }
    }

    void FeedforwardNN::setTrainingSet(unsigned numData, std::vector<double>& inputs, std::vector<double>& outputs)
    {
        double* in = inputs.data();
        double* out = outputs.data();
        trainingData.set_train_data(numData, nbinputs, in, nboutputs, out);
    }
    
    void FeedforwardNN::resetTrainingSet()
    {
        trainingData.destroy_train();
        trainingData = FANN::training_data();
    }

    void FeedforwardNN::train()
    {
        if (maxTrainingEpoch == 1)
            net.train_epoch(trainingData);
        else
            net.train_on_data(trainingData, maxTrainingEpoch, 0, desiredTrainingError);
    }
    
    void FeedforwardNN::setMaxTrainingEpoch(int epoch)
    {
        maxTrainingEpoch = epoch;
    }
    
    void FeedforwardNN::setDesiredError(float derror)
    {
        desiredTrainingError = derror;
    }
    
    void FeedforwardNN::setTrainingAlgorithm(enum FeedforwardNN::TrainingAlgorithm ta)
    {
        net.set_training_algorithm((enum FANN::training_algorithm_enum)ta);
    }

    void FeedforwardNN::setTrainingStopFunction(enum FeedforwardNN::StopFunction sf)
    {
        net.set_train_stop_function((enum FANN::stop_function_enum)sf);
    }
    
    void FeedforwardNN::setTrainingErrorFunction(enum FeedforwardNN::ErrorFunction ef)
    {
        net.set_train_error_function((enum FANN::error_function_enum)ef);
    }
    
    void FeedforwardNN::setLearningRate(float lrate)
    {
        net.set_learning_rate(lrate);
    }

    void FeedforwardNN::trainWithCascade(int max_neurons)
    {
        net.cascadetrain_on_data(trainingData, max_neurons, 0, desiredTrainingError);
    }

    void FeedforwardNN::setCascadeOutputChangeFraction(float cocf)
    {
        net.set_cascade_output_change_fraction(cocf);
    }

    std::vector<double> FeedforwardNN::run(std::vector<double>& inputs)
    {
        if (inputs.size() != net.get_num_input()) {
            std::cerr << "FeedforwardNN::run : ERROR: data size differs from network size - input size: data = " 
                    << inputs.size() << " <-> net = " << net.get_num_input() << std::endl;
            exit(1);
        }

        double* out = net.run(inputs.data());
        std::vector<double> ret(net.get_num_output());
        for (unsigned i=0; i<ret.size(); ++i)
            ret[i] = out[i];
        
        return ret;
    }

    void FeedforwardNN::run(std::vector<double>& inputs, std::vector<double>& outputs)
    {
        if (inputs.size() != net.get_num_input()) {
            std::cerr << "FeedforwardNN::run : ERROR: data size differs from network size - input size: data = " 
                    << inputs.size() << " <-> net = " << net.get_num_input() << std::endl;
            exit(1);
        }
            
        double* out = net.run(inputs.data());
        if (outputs.size() != net.get_num_output())
            outputs.resize(net.get_num_output());
        for (unsigned i=0; i<outputs.size(); ++i)
            outputs[i] = out[i];
        
    }
    
    
    void FeedforwardNN::setExternalMemory(Memory<NeuralNetworkData>* mem)
    {
        externalMemory = mem;
    }
    
    void FeedforwardNN::publish()
    {
        if (!externalMemory)
            return;

        externalMemory->clear();
        
        unsigned nbcon = net.get_total_connections();
        FANN::connection* con= new FANN::connection[nbcon];
        net.get_connection_array(con);
        
        NeuralNetworkData nnd;
        for (unsigned i=0; i<nbcon; ++i) {
            nnd.from = con[i].from_neuron;
            nnd.to = con[i].to_neuron;
            nnd.weight = con[i].weight;
            externalMemory->push_back(nnd);
        }
        delete[] con;
    }

    bool FeedforwardNN::loadFromExternalMemory()
    {
        if (!externalMemory)
            return false;
        
        unsigned nbcon = net.get_total_connections();
        
        if (nbcon != externalMemory->size()) {
            std::cerr << "FeedforwardNN: loadFromExternalMemory: different size detected: net = " << nbcon << " ; externalMemory: " << externalMemory->size() << std::endl;
            return false;
        }
        
        FANN::connection* con= new FANN::connection[nbcon];
        net.get_connection_array(con);

        unsigned index = 0;
        for (Memory<weight_t>::const_iterator it = externalMemory->begin(); it != externalMemory->end(); ++it) {
            con[index].from_neuron = it->from;
            con[index].to_neuron = it->to;
            con[index].weight = it->weight;
            index++;
        }
        net.set_weight_array(con, nbcon);
        
        delete[] con;
        return true;
    }

    void FeedforwardNN::registerParameters(std::string prefix)
    {
//        Settings* settings = Settings::getInstance();
        settings->registerParameter<int>((prefix+".nbinputs").c_str(), 0, "FeedforwardNN: Number of inputs.");
        settings->registerParameter<int>((prefix+".nboutputs").c_str(), 0, "FeedforwardNN:  Number of outputs.");
        settings->registerParameter<std::string>((prefix+".hidden").c_str(), std::string("0"), "FeedforwardNN: Number of neurons in each hidden layer.");
        settings->registerParameter<double>((prefix+".learningRate").c_str(), 0.1, "FeedforwardNN: Learning rate"); 
        settings->registerParameter<int>((prefix+".maximumTrainingEpoch").c_str(), 1000, "FeedforwardNN: Maximum epochs for the training.");
        settings->registerParameter<double>((prefix+".desiredTrainingError").c_str(), 0.1, "FeedforwardNN: Desired training error.");
        settings->registerParameter<double>((prefix+".weightMinimum").c_str(), -1.0, "FeedforwardNN: Minimum value for the weights during random initialization.");
        settings->registerParameter<double>((prefix+".weightMaximum").c_str(), 1.0, "FeedforwardNN: Maximum value for the weights during random initialization.");
    }
    
    void FeedforwardNN::loadParameters(std::string prefix)
    {
//        Settings* settings = Settings::getInstance();
        layers.clear();
        nbinputs = settings->value<int>((prefix+".nbinputs").c_str()).second;
        nboutputs = settings->value<int>((prefix+".nboutputs").c_str()).second;
        std::string hiddenStr = settings->value<std::string>((prefix+".hidden").c_str()).second;
        // need to parse the hiddenStr
        if (!hiddenStr.empty() && hiddenStr != "0"){
            std::list<unsigned> layersLst;
            unsigned index = 0, lastCut = 0;
            for (; index < hiddenStr.size(); ++index) {
                if (hiddenStr[index] == ' ') {
                    std::string tmp = hiddenStr.substr(lastCut, index-lastCut);
                    layersLst.push_back(std::stoul(tmp));
                    lastCut = index+1;
                }
            }
            std::string tmp = hiddenStr.substr(lastCut, index-lastCut);
            layersLst.push_back(std::stoul(tmp));
            layers.assign(layersLst.begin(), layersLst.end());
        }
        layers.insert(layers.begin(), nbinputs);
        layers.push_back(nboutputs);
        
        learningRate = settings->value<double>((prefix+".learningRate").c_str()).second;
        maxTrainingEpoch = settings->value<int>((prefix+".maximumTrainingEpoch").c_str()).second;
        desiredTrainingError = settings->value<double>((prefix+".desiredTrainingError").c_str()).second;
        weightMinimum = settings->value<double>((prefix+".weightMinimum").c_str()).second;
        weightMaximum = settings->value<double>((prefix+".weightMaximum").c_str()).second;
        
    }

    void FeedforwardNN::initializeFromParameters()
    {
        // the parameters are 

        std::cout << "FeedforwardNN: initializing from parameters... (layers = ";
        for (unsigned i=0; i<layers.size(); ++i)
            std::cout << layers[i] << " ";
        std::cout << ")" << std::endl;
        std::vector<enum FeedforwardNN::ActivationFunction> af(1, FeedforwardNN::SIGMOID);
        this->setup(layers, af);
        this->setWeightsMinMax(weightMinimum, weightMaximum);
//        this->setTrainingAlgorithm(FeedforwardNN::BATCH);
        this->setTrainingAlgorithm(FeedforwardNN::RPROP);
        this->setTrainingErrorFunction(FeedforwardNN::TANHFCT);
        this->setTrainingStopFunction(FeedforwardNN::MSE);

        
        this->setLearningRate(learningRate);
        this->setMaxTrainingEpoch(maxTrainingEpoch);
        this->setDesiredError(desiredTrainingError);
    }

    bool FeedforwardNN::save(const std::string& filename)
    {
        return net.save(filename);
    }

    bool FeedforwardNN::saveModelToFile(const char* filename)
    {
        return save(filename);
    }

    
}

