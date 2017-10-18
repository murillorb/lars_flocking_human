#ifndef RRTOBSTACLE_H
#define RRTOBSTACLE_H

#include <GEARSystem/Types/types.hh>
#include <WRCoach/utils/utils.hh>

class RRTObstacle {
public:
    RRTObstacle(Position pos, float radius);
    ~RRTObstacle();

    void setPos(Position &pos);
    void setPos(float x, float y);
    void setRadius(float radius);

    Position getPos();
    float getRadius();
private:
    Position _pos;
    float _radius;
};

#endif // RRTOBSTACLE_H
