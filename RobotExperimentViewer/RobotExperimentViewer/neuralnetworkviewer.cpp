#include "neuralnetworkviewer.h"

NeuralNetworkViewer::NeuralNetworkViewer(QWidget *parent) : QWidget(parent)
{
    
}

NeuralNetworkViewer::~NeuralNetworkViewer()
{
    
}

void NeuralNetworkViewer::setLayerActivations(unsigned layer, std::vector<double>& values)
{
    
}

void NeuralNetworkViewer::setNeuronActivation(unsigned layer, unsigned neuron, double value)
{
    
}

void NeuralNetworkViewer::setTopology(std::vector<unsigned>& _layers)
{
//    layers.resize(_layers.size());
//    neurons.resize(_layers.size());
//    neuronActivations.resize(_layers.size());
//    for (size_t i=0; i<_layers.size(); ++i) {
//        layers[i] = _layers[i];
//        neurons[i].resize(_layers[i]);
//        neuronActivations[i].resize(_layers[i]);
//        neuronActivations[i].fill(0.0);
//    }
    
//    updateTopology();
}

void NeuralNetworkViewer::updateTopology()
{
    
}

