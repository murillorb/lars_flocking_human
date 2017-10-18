/***
 * Warthog Robotics
 * University of Sao Paulo (USP) at Sao Carlos
 * http://www.warthog.sc.usp.br/
 *
 * This file is part of WRCoach project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***/

#ifndef WROPF_H_
#define WROPF_H_

#include <WRCoach/entity/player/navigation/navigationalgorithm/navigationalgorithm.hh>
#include <WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_opf.hh>

class OPF : public NavigationAlgorithm {
public:
    OPF();
    virtual ~OPF();

    void reset();
    void addBall(const Position &pos, const Velocity &vel);
    void addEnemyRobot(const Position &pos, const Velocity &vel);
    void addOwnRobot(const Position &pos, const Velocity &vel);
    void setGoal(const Position &pos, const Angle &angle);
    void setOrigin(const Position &pos, const Angle &ori, const Velocity &vel);

    Angle getDirection();
    float getDistance();
    QLinkedList<Position> getPath();
private:
    Pos position2pos(const Position &pos);
    int _getPath(Pos* path, int max);
private:
    OPFE *_opfe;
    QMutex _mOpfe;

    int _maxIterations;
    double _maxError;
    Pos _centerPos;

    float _width, _height;
    float _widthRes, _heightRes;

    QMutex mExtern;
    Pos _cpoPath[1000];
    int _cpoPathNumber;

    static int _count;
};

#endif // WROPF_H_
