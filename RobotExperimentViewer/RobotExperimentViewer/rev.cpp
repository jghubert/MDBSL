#include "rev.h"
#include <QPainter>
#include <QKeyEvent>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <QTimer>

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
    
    onPause = false;
    
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
    
    QListIterator<Line*> lt(lines);
    Line* l;
    while (lt.hasNext()) {
        l = lt.next();
        delete l;
    }
}

void REV::reset()
{
    QListIterator<Wall*> it(walls);
    Wall* w;
    while (it.hasNext()) {
        w = it.next();
        delete w;
    }
    walls.clear();
    
    QListIterator<Zone*> zt(zones);
    Zone* z;
    while (zt.hasNext()) {
        z = zt.next();
        delete z;
    }
    zones.clear();
    
    QListIterator<Line*> lt(lines);
    Line* l;
    while (lt.hasNext()) {
        l = lt.next();
        delete l;
    }
    lines.clear();
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

void REV::setPause(bool p)
{
    onPause = p;
}


void REV::step() 
{
    struct timeval start, end;
    
    if (onPause) {
        do {
            usleep(1000);
        } while (onPause);
    }
    
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

int REV::addZone(double radius, double centerx, double centery, Color c)
{
    int index = zones.size();
    
    zones.append(new Zone(radius, centerx, centery, c));
    
    return index;
}


int REV::addWall(double x1, double y1, double x2, double y2, Color c)
{
    int index = walls.size();
    
    walls.append(new Wall(x1, y1, x2, y2, c));
    
    return index;
}

int REV::addLine(double x1, double y1, double orientation, double length, Color c)
{
    int index = lines.size();
    
    Line* l = new Line();
    l->x0 = x1;
    l->y0 = y1;
    l->x1 = length*cos(orientation) + x1;
    l->y1 = length*sin(orientation) + y1; 
    l->color = c;
    lines.append(l);
    
    return index;
}

bool REV::modifyZone(int index, double left, double top, double width, double height, Color c)
{
    if (index < zones.size()) {
        Zone* z = zones.at(index);
        z->left = left;
        z->top = top;
        z->width = width;
        z->height = height;
        z->color = c;
        return true;
    }
    else 
        return false;
}

bool REV::modifyZone(int index, double radius, double centerx, double centery, Color c)
{
    if (index < zones.size()) {
        Zone* z = zones.at(index);
        z->radius = radius;
        z->centerX = centerx;
        z->centerY = centery;
        z->color = c;
        return true;
    }
    else 
        return false;
}


bool REV::modifyWall(int index, double x1, double y1, double x2, double y2, Color c)
{
    if (index < walls.size()) {
        Wall* w = walls.at(index);
        w->x1 = x1;
        w->y1 = y1;
        w->x2 = x2;
        w->y2 = y2;
        w->color = c;
        return true;
    }
    else 
        return false;
}

bool REV::modifyLine(int index, double x1, double y1, double orientation, double length, Color c)
{
    if (index < lines.size()) {
        Line* l = lines.at(index);
        l->x0 = x1;
        l->y0 = y1;
        l->x1 = length*cos(orientation) + x1;
        l->y1 = length*sin(orientation) + y1; 
        l->color = c;
    }
}


bool REV::setZoneColor(int index, Color c)
{
    if (index < zones.size()) {
        zones.at(index)->color = c;
        return true;
    }
    else
        return false;
}

bool REV::setWallColor(int index, Color c)
{
    if (index < walls.size()) {
        walls.at(index)->color = c;
        return true;
    }
    else
        return false;
}

bool REV::setZoneVisible(int index, bool v)
{
    if (index < zones.size()) {
        zones.at(index)->visible = v;
        return true;
    }
    else
        return false;
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

void REV::setBackgroundImage(const char* imagefile)
{

}


void REV::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
//    painter.setWindow(this->window()->rect());
//    painter.setRenderHint(QPainter::Antialiasing);
    paint(painter);
    painter.end();
}

void REV::paint(QPainter& painter)
{
    QBrush brush;
    QPen pen;
    double marginx = 10;
    double marginy = 10;
//    double dw = (this->width() - 2*marginx) * 1.0/ arenaWidth;
//    double dh = (this->height() - 2*marginy) * 1.0/ arenaHeight;
    double dw = (painter.device()->width() - 2*marginx) * 1.0/ arenaWidth;
    double dh = (painter.device()->height() - 2*marginy) * 1.0/ arenaHeight;
    
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
        if (z->visible) {
            painter.setBrush(z->color.toColor());
            painter.setPen(z->color.toColor());
            if (z->type == SQUARE)
                painter.drawRect(QRectF(marginx+dw*z->left, marginy+dh*z->top, dw*z->width, dh*z->height));
            else if (z->type == ROUND)
                painter.drawEllipse(QPointF(marginx+dw*z->centerX, marginy+dh*z->centerY),dw*z->radius, dh*z->radius);
        }
    }
    painter.restore();
    
    // Let's add the robot
    painter.setBrush(Qt::NoBrush);
    painter.setPen(Qt::black);
    painter.drawEllipse(QPointF(marginx+dw*robot.x, marginy+dh*robot.y), dw*robot.radius, dh*robot.radius);
//    painter.save();
//    painter.rotate(robot.orientation);
    painter.drawLine(QLineF(marginx+dw*robot.x, marginy+dh*robot.y, marginx+dw*(robot.x-robot.radius*cos(robot.orientation)), marginy+dh*(robot.y-robot.radius*sin(robot.orientation))));
//    painter.restore();
    
    // Let's add the lines
    QListIterator<Line*> lt(lines);
    Line* l;
    painter.save();
    while (lt.hasNext()) {
        l = lt.next();
        painter.setBackground(l->color.toColor());
        painter.setPen(l->color.toColor());
        painter.drawLine(QLineF(marginx+dw*l->x0, marginy+dh*l->y0, marginx+dw*l->x1, marginy+dh*l->y1));
    }    
    painter.restore();
    
}

void REV::saveRenderingToFile(const char* filename, unsigned width, unsigned height)
{
    QImage img(QSize(width, height), QImage::Format_ARGB32);
    QPainter painter(&img);
    paint(painter);
    painter.end();
    img.save(QString(filename));
}


void REV::keyReleaseEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_P) {
        onPause = !onPause;
    }
}


