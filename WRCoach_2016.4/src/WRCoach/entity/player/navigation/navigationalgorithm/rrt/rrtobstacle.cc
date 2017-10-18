#include "rrtobstacle.hh"

RRTObstacle::RRTObstacle(Position pos, float radius) {
    _pos = pos;
    _radius = radius;
}

RRTObstacle::~RRTObstacle() { }

void RRTObstacle::setPos(Position &pos) { _pos = pos; }

void RRTObstacle::setPos(float x, float y) { _pos.setPosition(x, y, 0.0); }

void RRTObstacle::setRadius(float radius) { _radius = radius; }

Position RRTObstacle::getPos() { return _pos; }

float RRTObstacle::getRadius() { return _radius; }
