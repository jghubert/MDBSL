#include "revinputviewer.h"
#include <QPainter>
#include <string>

REVInputViewer::REVInputViewer(unsigned _nbdata, QWidget *parent) : 
    nbdata(_nbdata), QWidget(parent)
{
    data.resize(nbdata);
}

REVInputViewer::~REVInputViewer()
{
    
}

void REVInputViewer::setData(std::vector<double>& d)
{
    for (unsigned i=0; i<nbdata; ++i)
        data[i].value = d[i];
}

void REVInputViewer::setParameters(std::vector<double>& mins, std::vector<double>& maxs, std::vector<std::string>& labels)
{
    for (unsigned i=0; i<nbdata; ++i) {
        data[i].min = mins[i];
        data[i].max = maxs[i];
        data[i].label = QString(labels[i].c_str());
    }
}


void REVInputViewer::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    
    
}


