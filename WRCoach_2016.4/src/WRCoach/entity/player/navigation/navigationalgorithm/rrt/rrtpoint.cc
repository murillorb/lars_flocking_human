#include "rrtpoint.hh"

RRTPoint::RRTPoint()  {
    _pos.setUnknown();
    _previousPoint = NULL;

}

RRTPoint::RRTPoint(const Position &pos) {
    _pos = pos;
    _previousPoint = NULL;
}

RRTPoint::RRTPoint(const Position &pos, RRTPoint *previousPoint) {
    _pos = pos;
    _previousPoint = previousPoint;
}

RRTPoint::~RRTPoint() { }

void RRTPoint::setPos(Position pos) { _pos = pos; }

void RRTPoint::setPos(float x, float y) { _pos.setPosition(x, y, 0.0); }

void RRTPoint::setPreviousPoint(RRTPoint *previousPoint) { _previousPoint = previousPoint; }

Position RRTPoint::getPos() const { return _pos; }

RRTPoint* RRTPoint::getPreviousPoint() { return _previousPoint; }
