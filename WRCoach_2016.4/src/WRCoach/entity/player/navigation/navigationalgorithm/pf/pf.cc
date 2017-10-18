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

#include "pf.hh"
#include <iostream>
#include <WRCoach/const/pfconstants.hh>
#include <WRCoach/utils/freeangles/freeangles.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

PF::PF() {
    reset();
}

void PF::reset() {
    _resultantForce.setVector(0.0, 0.0);
    _resultantAngle = 0.0;
    _collisionPositions.clear();
    _repulsive.clear();
}

void PF::run() {
    // Reset resultant force
    _resultantForce.setVector(0.0, 0.0);

    // Add attractive force
    addAttractiveForce(getVector(originPos(), goalPos()));

    // Add repulsive forces
    for(int i=0; i<_repulsive.size(); i++)
        addRepulsiveForce(getVector(_repulsive.at(i).first, originPos()), _repulsive.at(i).second);

    // Calc resultant angle based on resultant force
    float resultantAngle = atan2(_resultantForce.y(), _resultantForce.x());

    // Collision detection; fixes bug with aligned obstacles in PF
    if(PFConstants::collisionCheckingEnabled()) {

        // Check collision
        if(isOnCollisionRoute(resultantAngle))
            resultantAngle = handleCollision(resultantAngle);
    }

    // Store result
    _resultantAngle = resultantAngle;
}

void PF::addBall(const Position &pos, const Velocity &vel) {
    // Add repulsive position
    addRepulsivePosition(pos, PFConstants::constBallRepulsive());
    addCollisionPosition(pos, true);
}

void PF::addGoalArea(const Position &pos) {
    // Add repulsive position
    addRepulsivePosition(pos, PFConstants::constAreaRepulsive());
    addCollisionPosition(pos);
}

void PF::addOwnRobot(const Position &pos, const Velocity &vel) {
    // Add robot
    addRobot(pos, vel);
}

void PF::addEnemyRobot(const Position &pos, const Velocity &vel) {
    // Add robot
    addRobot(pos, vel);
}

void PF::addRobot(const Position &pos, const Velocity &vel) {
    // Add repulsive position
    addRepulsivePosition(pos, PFConstants::constRepulsive());
    addCollisionPosition(pos);
}

void PF::addRepulsiveForce(const Vector &v, float k) {
    // Add force
    addForce(applyDistanceFunction(v, k));
}

void PF::addAttractiveForce(const Vector &v) {
    // Add force
    addForce(v.getUnitary());
}

void PF::addForce(const Vector &v) {
    // Sum in resultant force
    _resultantForce += v;
}

Vector PF::applyDistanceFunction(Vector v, float k) {
    const float d = v.getModule();
    const float xshift = PFConstants::functionXshift();
    const float yshift = PFConstants::functionYshift();
    const float factor = PFConstants::functionFactor();

    return (v.getUnitary() * k)/pow((d-xshift),factor)+yshift;
}

Vector PF::getVector(const Position &v1, const Position &v2) {
    return Vector(v2.x()-v1.x(), v2.y()-v1.y());
}

void PF::addRepulsivePosition(const Position &pos, float k) {
    std::pair<Position,float> repulsive = std::make_pair(pos, k);
    _repulsive.append(repulsive);
}

void PF::addCollisionPosition(const Position &pos, bool isBall) {
    _collisionPositions.push_back((std::make_pair(pos, isBall)));
}

Angle PF::getDirection() const {
    return Angle(true, _resultantAngle);
}

PF* PF::clone() const {
    return new PF(*this);
}

bool PF::isOnCollisionRoute(float resultantAngle) {
    // Check if colliding
    for(int i=0; i<_collisionPositions.size(); i++) {
        const Position posObst = _collisionPositions.at(i).first;
        const bool posObstBall = _collisionPositions.at(i).second;
        const float distObst = WR::Utils::distance(originPos(), posObst);
        const float angleObst = WR::Utils::getAngle(originPos(), posObst);
        const float collisionRadius = (posObstBall? PFConstants::ballCollisionRadius() : PFConstants::robotCollisionRadius());
        const float fix = atan2(collisionRadius, distObst);

        // Check collision distance
        if(distObst > PFConstants::collisionCheckingRadius())
            continue;

        // Check if goal is nearest than obstacle
        float distGoal = WR::Utils::distance(originPos(), goalPos());
        if(distGoal <= 0.95*distObst) // applied factor to avoid small distance collisions
            continue;

        // Check if its a collision route
        float dif = WR::Utils::angleDiff(angleObst, resultantAngle);
        if(fabs(dif) <= 2*fabs(fix))
            return true;
    }

    return false;
}

float PF::handleCollision(float resultantAngle) {

    // Put result. angle in known position
    WR::Utils::angleLimitZeroTwoPi(&resultantAngle);

    // Generate FreeAngles obstacle list
    QList<Obstacle> obstacles;
    for(int i=0; i<_collisionPositions.size(); i++) {
        const Position posObst = _collisionPositions.at(i).first;
        const bool isObstBall = _collisionPositions.at(i).second;
        const float collisionRadius = (isObstBall? PFConstants::ballCollisionRadius() : PFConstants::robotCollisionRadius());

        // Create obstacle
        Obstacle obst;
        obst.position() = posObst;
        obst.calcAnglesFrom(originPos(), collisionRadius);

        // Add to list
        obstacles.push_back(obst);
    }

    // Run Free Angles
    const QList<FreeAngles::Interval> fa = FreeAngles::getFreeAngles(0, GEARSystem::Angle::twoPi, obstacles, true);

    // Get current resultant angle interval
    FreeAngles::Interval resultantInterval;
    bool resultantIntervalFound = false;
    for(int i=0; i<fa.size(); i++) {
        FreeAngles::Interval interval = fa.at(i);

        float tmpResultAng = resultantAngle;

        // Exception case
        if(interval.angInitial()>interval.angFinal()) {
            if(tmpResultAng>interval.angInitial())
                tmpResultAng -= GEARSystem::Angle::twoPi;
            interval = FreeAngles::Interval(interval.angInitial()-GEARSystem::Angle::twoPi, interval.angFinal(), interval.obstructed());
        }

        // Check
        if(interval.angInitial()<=tmpResultAng && tmpResultAng<=interval.angFinal()) {
            resultantInterval = interval;
            resultantIntervalFound = true;
            break;
        }
    }

    // Check if found an interval
    if(resultantIntervalFound==false) {
        std::cout << "[ERROR] PF::handleCollision() not finding interval of resultant angle! (should never reach here)\n";
        return resultantAngle;
    }

    // Check if interval is obstructed
    if(resultantInterval.obstructed()) {
        // Calculate to go right or left
        float difI = fabs(WR::Utils::angleDiff(resultantAngle, resultantInterval.angInitial()));
        float difF = fabs(WR::Utils::angleDiff(resultantAngle, resultantInterval.angFinal()));
        float result = difI<=difF? resultantInterval.angInitial() : resultantInterval.angFinal();

        return result;
    }

    return resultantAngle;
}
