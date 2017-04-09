#ifndef REVINPUTVIEWER_H
#define REVINPUTVIEWER_H

#include <QWidget>
#include <vector>


class REVInputViewer : public QWidget
{
    Q_OBJECT
public:
    class DisplayData
    {
    public:
        QString label;
        double min;
        double max;
        double value;
    };

    explicit REVInputViewer(unsigned _nbdata, QWidget *parent = 0);
    ~REVInputViewer();

    void setData(std::vector<double>& d);
    void setParameters(std::vector<double>& mins, std::vector<double>& maxs, std::vector<std::string>& labels);


signals:
    
public slots:
    
protected:
    virtual void paintEvent(QPaintEvent *);
    
private:
    unsigned nbdata;
    QVector<DisplayData> data;

    
};

#endif // REVINPUTVIEWER_H
