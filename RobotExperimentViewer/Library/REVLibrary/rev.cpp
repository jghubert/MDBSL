#include "rev.h"
#include <QPainter>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

// Left top is 0.0

REV::REV(QWidget *parent)
    : QMainWindow(parent)
{
    arenaWidth = 500;
    arenaHeight = 200;
    arenaBackgroundColor = QColor(255,255,255,255);
//    arenaBackgroundColor = QColor(255,0,0,255);
    
//    walls.append(new Wall(0.0, 0.0, 0.0, arenaHeight, Color(255,0,0,255)));  //Red - left
//    walls.append(new Wall(0.0, 0.0, arenaWidth, 0.0, Color(0,0,0,255)));     // Black - bottom
//    walls.append(new Wall(arenaWidth, 0.0, arenaWidth, arenaHeight, Color(0,255,0,255))); // Green - 
//    walls.append(new Wall(0.0, arenaHeight, arenaWidth, arenaHeight, Color(0,0,255,255)));  // Blue
    
//    zones.append(new Zone(25.0, 0.0, 50.0, arenaHeight/2.0, Color(125,125,125,255)));
    
//    setRobotPosition(200, 75, M_PI/2.0);
//    setRobotRadius(3.7);
    
    realtime = false;
    framerate = 25;
}

REV::~REV()
{
    QListIterator<Wall*> it(walls);
    Wall* w;
    while (it.hasNext()) {
        w = it.next();
        delete w;
    }
    
    QListIterator<Zone*> zt(zones);
    Zone* z;
    while (zt.hasNext()) {
        z = zt.next();
        delete z;
    }
}

void REV::setRobotPosition(double x, double y, double orientation)
{
    robot.x = x;
    robot.y = y;
    robot.orientation = orientation;
}

void REV::setRobotRadius(double r)
{
    robot.radius = r;
}

void REV::setRealtime(bool rt) {
    realtime = rt;
}

void REV::setFramerate(unsigned fr)
{
    framerate = fr;
}

void REV::step() 
{
    struct timeval start, end;
    if (realtime) {
        gettimeofday(&start, NULL);
        update();
        gettimeofday(&end, NULL);
        unsigned long elapsed = end.tv_usec - start.tv_usec;
        unsigned long tosleep = 1e6/(1.0*framerate) - elapsed;
        usleep(tosleep);
    }   
    else
        update();
}


int REV::addZone(double _left, double _top, double _width, double _height, Color c)
{
    int index = zones.size();

    zones.append(new Zone(_left, _top, _width, _height, c));

    return index;

}

int REV::addWall(double x1, double y1, double x2, double y2, Color c)
{
    int index = walls.size();
    
    walls.append(new Wall(x1, y1, x2, y2, c));
    
    return index;
}

void REV::setSize(double _width, double _height)
{
    this->arenaWidth = _width;
    this->arenaHeight = _height;
}

void REV::setBackgroundColor(Color c)
{
    arenaBackgroundColor = QColor(c.r, c.g, c.b, c.a);
}

void REV::paintEvent(QPaintEvent *event)
{
    QBrush brush;
    QPen pen;
    double marginx = 10;
    double marginy = 10;
    double dw = (this->width() - 2*marginx) * 1.0/ arenaWidth;
    double dh = (this->height() - 2*marginy) * 1.0/ arenaHeight;
    
    QPainter painter(this);
    // Background
    painter.setPen(arenaBackgroundColor);
    painter.setBrush(QBrush(arenaBackgroundColor));
    painter.drawRect(QRect(0,0,width(), height()));
    painter.setBackground(QBrush(arenaBackgroundColor));

    // Let's add the walls
    QListIterator<Wall*> it(walls);
    Wall* w;
    painter.save();
    while (it.hasNext()) {
        w = it.next();
        painter.setBrush(w->color.toColor());
        painter.setPen(w->color.toColor());
        painter.drawLine(QLineF(marginx+dw*w->x1, marginy+dh*w->y1, marginx+dw*w->x2, marginy+dh*w->y2));
    }
    painter.restore();
    
    // Let's add the zones
    QListIterator<Zone*> zt(zones);
    Zone* z;
    painter.save();
    while (zt.hasNext()) {
        z = zt.next();
        painter.setBrush(z->color.toColor());
        painter.setPen(z->color.toColor());
        painter.drawRect(QRectF(marginx+dw*z->left, marginy+dw*z->top, dw*z->width, dh*z->height));
    }
    painter.restore();
    
    // Let's add the robot
    painter.setBrush(Qt::NoBrush);
    painter.setPen(Qt::red);
    painter.drawEllipse(QPointF(marginx+dw*robot.x, marginy+dh*robot.y), dw*robot.radius, dh*robot.radius);
//    painter.save();
//    painter.rotate(robot.orientation);
    painter.drawLine(QLineF(marginx+dw*robot.x, marginy+dh*robot.y, marginx+dw*(robot.x+robot.radius*cos(robot.orientation)), marginy+dh*(robot.y+robot.radius*sin(robot.orientation))));
//    painter.restore();
    
}
