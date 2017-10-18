#ifndef RRTPOINT_H
#define RRTPOINT_H

#include <GEARSystem/Types/types.hh>
#include <WRCoach/utils/utils.hh>

class RRTPoint {
public:
    RRTPoint();
    RRTPoint(const Position &pos);
    RRTPoint(const Position &pos, RRTPoint *previousPoint);
    ~RRTPoint();

    void setPos(Position pos);
    void setPos(float x, float y);
    void setPreviousPoint(RRTPoint *previousPoint);

    Position getPos() const;
    RRTPoint* getPreviousPoint();

private:
    Position _pos;
    RRTPoint* _previousPoint;
};

#endif // RRTPOINT_H
