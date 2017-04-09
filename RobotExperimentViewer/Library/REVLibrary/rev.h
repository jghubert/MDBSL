#ifndef REV_H
#define REV_H

#include <QMainWindow>
#include <QColor>

class REV : public QMainWindow
{
    Q_OBJECT

public:
    class Color {
    public:
        unsigned r;
        unsigned g;
        unsigned b;
        unsigned a;

        Color()
            : r(0), g(0), b(0), a(0) {}

        Color(unsigned _r, unsigned _g, unsigned _b, unsigned _a)
            : r(_r), g(_g), b(_b), a(_a) {}
        
        QColor toColor() {
            return QColor(r,g,b,a);
        }
    };


    REV(QWidget *parent = 0);
    ~REV();

    void setRobotPosition(double x, double y, double orientation);
    void setRobotRadius(double r);
    int addZone(double left, double top, double width, double height, Color c);
    int addWall(double x1, double y1, double x2, double y2, Color c);

    void setSize(double _width, double _height);
    void setBackgroundColor(Color c);
    
    void setRealtime(bool rt);
    void setFramerate(unsigned fr);
    
    void step();

public slots:

protected:
    virtual void paintEvent(QPaintEvent *);

private:
    class Zone {
    public:
        double left;
        double top;
        double width;
        double height;
        Color color;

        Zone():
            left(0.0), top(0.0), width(0.0), height(0.0), color(Color()) {}

        Zone(double l, double t, double w, double h, Color c):
            left(l), top(t), width(w), height(h), color(c) {}
    };

    class Wall {
    public:
        double x1;
        double y1;
        double x2;
        double y2;
        Color color;

        Wall()
            : x1(0), y1(0), x2(0), y2(0), color(Color()) {}

        Wall(double _x1, double _y1, double _x2, double _y2, Color c)
            : x1(_x1), y1(_y1), x2(_x2), y2(_y2), color(c) {}

    };

    class Robot {
    public:
        double x;
        double y;
        double orientation;
        double radius;
        
        Robot() : x(0.0), y(0.0), orientation(0.0), radius(0.0) {}
    };

    QList<Zone*> zones;
    QList<Wall*> walls;
    Robot robot;

    double arenaWidth;
    double arenaHeight;
    QColor arenaBackgroundColor;
    
    bool realtime;
    unsigned framerate;
};

#endif // REV_H
