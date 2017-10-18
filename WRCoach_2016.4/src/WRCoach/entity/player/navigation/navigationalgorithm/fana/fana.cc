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

#include "fana.hh"
#include <iostream>

#pragma GCC diagnostic ignored "-Wunused-parameter"

#define OBSTACLE_RADIUS 0.30f

FANA::FANA() {

}

FANA::~FANA() {

}

void FANA::reset() {
    _obstacles.clear();
}

void FANA::addBall(const Position &pos, const Velocity &vel) {
    addObstacle(pos);
}

void FANA::addEnemyRobot(const Position &pos, const Velocity &vel) {
    addObstacle(pos);
}

void FANA::run() {
    // Goal angle
    float angGoal = WR::Utils::getAngle(originPos(), goalPos());
    WR::Utils::angleLimitZeroTwoPi(&angGoal);

    // Remove obstacles outside destination radius
    QList<Obstacle> obstacles(_obstacles);
    const float destinationRadius = WR::Utils::distance(originPos(), goalPos()) + OBSTACLE_RADIUS;
    for(int i=obstacles.size()-1; i>=0; i--) {
        Obstacle obst = obstacles[i];
        float distObst = WR::Utils::distance(originPos(), obst.position());
        if(distObst >= destinationRadius)
            obstacles.removeAt(i);
    }

    // Calc obstacles angle obstruction
    for(int i=0; i<obstacles.size(); i++)
        obstacles[i].calcAnglesFrom(originPos(), OBSTACLE_RADIUS);

    // Run free angles algorithm
    const QList<FreeAngles::Interval> freeAngles = FreeAngles::getFreeAngles(0, GEARSystem::Angle::twoPi, obstacles, true);

    // Check goal angle interval
    FreeAngles::Interval goalInterval;
    bool goalIntervalFound = false;
    for(int i=0; i<freeAngles.size(); i++) {
        FreeAngles::Interval interval = freeAngles.at(i);

        float tmpAngGoal = angGoal;

        // Exception case
        if(interval.angInitial()>interval.angFinal()) {
            if(angGoal>interval.angInitial())
                tmpAngGoal -= GEARSystem::Angle::twoPi;
            interval = FreeAngles::Interval(interval.angInitial()-GEARSystem::Angle::twoPi, interval.angFinal(), interval.obstructed());
        }

        // Check
        if(interval.angInitial()<=tmpAngGoal && tmpAngGoal<=interval.angFinal()) {
            goalInterval = interval;
            goalIntervalFound = true;
            break;
        }
    }

    // Check if found an interval
    if(goalIntervalFound==false) {
        std::cout << "[ERROR] FANA::run() not finding interval of goal! (should never reach here)\n";
        _resultantAngle.setUnknown();
        return;
    }

    // Check if interval is obstructed
    if(goalInterval.obstructed()) {
        // Calculate to go right or left
        float difI = fabs(WR::Utils::angleDiff(angGoal, goalInterval.angInitial()));
        float difF = fabs(WR::Utils::angleDiff(angGoal, goalInterval.angFinal()));
        float result = difI<=difF? goalInterval.angInitial() : goalInterval.angFinal();
        _resultantAngle.setValue(result);

    } else {
        _resultantAngle.setValue(angGoal);
    }

}

void FANA::addOwnRobot(const Position &pos, const Velocity &vel) {
    addObstacle(pos);
}

void FANA::addGoalArea(const Position &pos) {
    addObstacle(pos);
}

void FANA::addObstacle(const Position &pos) {
    Obstacle obst;
    obst.position() = pos;

    _obstacles.push_back(obst);
}

Angle FANA::getDirection() const {
    return _resultantAngle;
}

FANA *FANA::clone() const {
    return new FANA(*this);
}
