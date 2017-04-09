#ifndef NEURALNETWORKVIEWER_H
#define NEURALNETWORKVIEWER_H

#include <QWidget>
#include <QVector>
#include <vector>

class NeuralNetworkViewer : public QWidget
{
    Q_OBJECT
public:
    explicit NeuralNetworkViewer(QWidget *parent = 0);
    ~NeuralNetworkViewer();
    
    void setLayerActivations(unsigned layer, std::vector<double>& values);
    void setNeuronActivation(unsigned layer, unsigned neuron, double value);
    void setTopology(std::vector<unsigned>& layers);
    
signals:
    
public slots:
    
private:
    QVector<unsigned> layers;
//    QVector<QVector2D> neurons;
    QVector<QVector<double> > neuronActivations;
    
    double neuronMinRadius;
    double neuronMaxRadius;
    
    void updateTopology();
    
};

#endif // NEURALNETWORKVIEWER_H
