#ifndef REV_H
#define REV_H

#include <QMainWindow>
#include <QColor>

class QTimer;

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
    int addZone(double radius, double centerx, double centery, Color c);
    int addWall(double x1, double y1, double x2, double y2, Color c);
    int addLine(double x1, double y1, double orientation, double length, Color c);
    bool modifyZone(int index, double left, double top, double width, double height, Color c);
    bool modifyZone(int index, double radius, double centerx, double centery, Color c);
    bool modifyWall(int index, double x1, double y1, double x2, double y2, Color c);
    bool modifyLine(int index, double x1, double y1, double orientation, double length, Color c);
    bool setZoneColor(int index, Color c);
    bool setWallColor(int index, Color c);
    bool setZoneVisible(int index, bool v);
    
    
    void setSize(double _width, double _height);
    void setBackgroundColor(Color c);
    void setBackgroundImage(const char* imagefile);
    
    void setRealtime(bool rt);
    void setFramerate(unsigned fr);
    
    void setPause(bool p);
    
    void step();
    
    void reset();
    
    void saveRenderingToFile(const char* filename, unsigned width, unsigned height);
    
protected:
    virtual void paintEvent(QPaintEvent *);
    virtual void keyReleaseEvent(QKeyEvent *);
    
private:
    enum ZoneType{SQUARE, ROUND};
    
    bool onPause;
    
    class Zone {
    public:
        union {
            double left;
            double centerX;
        };
        union {
            double centerY;
            double top;
        };
        union {
            double width;
            double radius;
        };
        double height;
        Color color;
        bool visible;
        
        enum ZoneType type;
        
        Zone():
            left(0.0), top(0.0), width(0.0), height(0.0), color(Color()), visible(true), type(SQUARE) {}

        Zone(double l, double t, double w, double h, Color c, bool v=true):
            left(l), top(t), width(w), height(h), color(c), visible(v), type(SQUARE) {}
        Zone(double r, double cx, double cy, Color c, bool v=true):
            centerX(cx), centerY(cy), radius(r), color(c), visible(v), type(ROUND) {}
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
    
    class Line {
    public:
        double x0;
        double y0;
        double x1;
        double y1;
        Color color;
        
        Line() : x0(0.0), y0(0.0), x1(0.0), y1(0.0), color(Color()) {}
        Line(double _x0, double _y0, double _x1, double _y1, double _orient, Color c) : x0(_x0), y0(_y0), x1(_x1), y1(_y1), color(c) {}
        
    };

    QList<Zone*> zones;
    QList<Wall*> walls;
    QList<Line*> lines;
    Robot robot;

    double arenaWidth;
    double arenaHeight;
    QColor arenaBackgroundColor;
    
    bool realtime;
    unsigned framerate;
    
    void paint(QPainter& painter);
    
};

#endif // REV_H
