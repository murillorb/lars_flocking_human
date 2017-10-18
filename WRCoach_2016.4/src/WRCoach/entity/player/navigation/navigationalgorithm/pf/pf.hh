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

#ifndef PF_HH
#define PF_HH

#include <WRCoach/entity/player/navigation/navigationalgorithm/navigationalgorithm.hh>
#include "vector.hh"

class PF : public NavigationAlgorithm {
public:
    PF();

    // Setters
    void reset();
    void addBall(const Position &pos, const Velocity &vel);
    void addOwnRobot(const Position &pos, const Velocity &vel);
    void addEnemyRobot(const Position &pos, const Velocity &vel);
	void addGoalArea(const Position &pos);

    // Run
    void run();

    // Results
    Angle getDirection() const;

private:
    // Inheritance clone implementation
    PF* clone() const;

    // Collision checking
    QList<std::pair<Position,bool> > _collisionPositions;
    float handleCollision(float resultantAngle);
    bool isOnCollisionRoute(float resultantAngle);

    // Internal addRobot
    void addRobot(const Position &pos, const Velocity &vel);

    // Forces
    void addRepulsiveForce(const Vector &v, float k);
    void addAttractiveForce(const Vector &v);
    void addForce(const Vector &v);

    // Distance function
    Vector applyDistanceFunction(Vector v, float k);

    // Auxiliary functions
    static Vector getVector(const Position &v1, const Position &v2);
    void addRepulsivePosition(const Position &pos, float k);
    void addCollisionPosition(const Position &pos, bool isBall = false);

    // Repulsive obstacles
    QList<std::pair<Position,float> > _repulsive;

    // Results
    Vector _resultantForce;
    float _resultantAngle;
};

#endif // PF_HH
