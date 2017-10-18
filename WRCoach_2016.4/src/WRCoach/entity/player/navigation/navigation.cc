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

#include "navigation.hh"
#include <WRCoach/entity/locations.hh>
#include <WRCoach/entity/player/player.hh>
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>

Navigation::Navigation(Player *player, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg) {
    _player = player;
    _navAlg = navAlg;
    _linCtrAlg = linCtrAlg;
    _angCtrAlg = angCtrAlg;

    _ourTeam = _player->team();
    _opTeam = _ourTeam->opTeam();
    _loc = _player->team()->loc();

    if(_navAlg!=NULL)
        _navAlg->initialize(_loc);
}

Navigation::~Navigation() {
    if(_navAlg!=NULL)
        delete _navAlg;
}

void Navigation::setGoal(const Position &destination, const Angle &orientation, bool avoidTeammates, bool avoidOpponents, bool avoidBall, bool avoidOurGoalArea, bool avoidTheirGoalArea) {
    // Reset algorithm
    _navAlg->reset();

    // Set origin position and orientation
    _navAlg->setOrigin(_player->position(), _player->orientation(), _player->velocity());

    // Sets goal position and orientation
    _navAlg->setGoal(destination, orientation);

    // Add ball as repulsive
    if(avoidBall)
        _navAlg->addBall(_loc->ball(), _loc->ballVelocity());

    // Add teammates as repulsive
    if(avoidTeammates) {
        QList<Player*> avPlayers = _ourTeam->avPlayers().values();
        for(int i=0; i<avPlayers.size(); i++) {
            Player *teammate = avPlayers.at(i);

            // Check if calling player
            if(teammate->playerId()==_player->playerId())
                continue;

            // Add robot as repulsive
            _navAlg->addOwnRobot(teammate->position(), teammate->velocity());
        }
    }

    // Adds opponents as repulsive
    if(avoidOpponents) {
        QList<Player*> avOpPlayers = _opTeam->avPlayers().values();
        for(int i=0; i<avOpPlayers.size(); i++) {
            Player *opponent = avOpPlayers.at(i);

            // Add robot as repulsive
            _navAlg->addEnemyRobot(opponent->position(), opponent->velocity());
        }
    }

    //Adds goal area as repulsive
    if(avoidOurGoalArea || avoidTheirGoalArea) {
        // Check restritions
        if(_loc->isOutsideField(_player->position()))
            return;
        if(_loc->isInsideOurArea(_player->position()))
            return;
        if(_loc->isInsideTheirArea(_player->position()))
            return;

        // Our goal area
        if(avoidOurGoalArea) {
            // Add our goal area
            Position far = WR::Utils::projectPointAtSegment(_loc->ourGoalLeftMidPost(), _loc->ourGoalRightMidPost(), _player->position());
            Position repulsion = WR::Utils::threePoints(far, _player->position(), _loc->fieldDefenseRadius(), 0.0f);
            _navAlg->addGoalArea(repulsion);
        }

        // Their goal area
        if(avoidTheirGoalArea) {
            // Add their goal area
            Position far = WR::Utils::projectPointAtSegment(_loc->theirGoalLeftMidPost(), _loc->theirGoalRightMidPost(), _player->position());
            Position repulsion = WR::Utils::threePoints(far, _player->position(), _loc->fieldDefenseRadius(), 0.0f);
            _navAlg->addGoalArea(repulsion);
        }
    }

}

Angle Navigation::getDirection() const {
    // Execute algorithm
    _navAlg->runNavigationAlgorithm();

    return _navAlg->getDirection();
}

float Navigation::getDistance() const {
    return _navAlg->getDistance();
}

QLinkedList<Position> Navigation::getPath() const {
    return _navAlg->getPath();
}

void Navigation::reset() {
    _linCtrAlg->reset();
    _angCtrAlg->reset();
}

void Navigation::setLinearPIDParameters(double kp, double ki, double kd, double iLimit) {
    ControlAlgorithmParameters params;
    params.setPIDParameters(kp, ki, kd, iLimit);
    params.setTo(_linCtrAlg);
}

void Navigation::setAngularPIDParameters(double kp, double ki, double kd, double ilimit) {
    ControlAlgorithmParameters params;
    params.setPIDParameters(kp, ki, kd, ilimit);
    params.setTo(_angCtrAlg);
}

void Navigation::setLinearDiscretePIDParameters(double kp, double ki, double kd) {
    ControlAlgorithmParameters params;
    params.setDiscretePIDParameters(kp, ki, kd);
    params.setTo(_linCtrAlg);
}

void Navigation::setAngularDiscretePIDParameters(double kp, double ki, double kd) {
    ControlAlgorithmParameters params;
    params.setDiscretePIDParameters(kp, ki, kd);
    params.setTo(_angCtrAlg);
}

void Navigation::setLinearCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed) {
    ControlAlgorithmParameters params;
    params.setCascadePIDParameters(dkp, dkd, skp, ski, skd, siLimit, maxSpeed);
    params.setTo(_linCtrAlg);
}

void Navigation::setAngularCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed) {
    ControlAlgorithmParameters params;
    params.setCascadePIDParameters(dkp, dkd, skp, ski, skd, siLimit, maxSpeed);
    params.setTo(_angCtrAlg);
}

double Navigation::getLinearSpeed(float distError, Velocity velocity) {
    // Iterate control algorithm
    float linearSpeed = _linCtrAlg->iterate(distError, velocity);

    // Limit to max linear speed
    WR::Utils::limitValue(&linearSpeed, -_maxLSpeed, _maxLSpeed);

    return linearSpeed;
}

double Navigation::getAngularSpeed(float angularError, Velocity velocity) {
    float angularSpeed = 0.0;

    // Fix high angle
    if(angularError > GEARSystem::Angle::pi)
        angularError -= 2*GEARSystem::Angle::pi;
    if(angularError < -GEARSystem::Angle::pi)
        angularError += 2*GEARSystem::Angle::pi;

    // Iterate control algorithm
    angularSpeed = _angCtrAlg->iterate(angularError, velocity);

    // Limit to max angular speed
    WR::Utils::limitValue(&angularSpeed, -_maxASpeed, _maxASpeed);

    return angularSpeed;
}
