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

#include "sslplayer.hh"
#include <WRCoach/utils/utils.hh>
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>
#include <WRCoach/const/globalconstants.hh>

SSLPlayer::SSLPlayer(World *world, WRTeam *team, quint8 playerId, Controller *ctr, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg, Behavior *defaultBehavior, SSLReferee *ref)
: Player(world, team, playerId, ctr, navAlg, linCtrAlg, angCtrAlg, defaultBehavior, ref) {

}

SSLPlayer::~SSLPlayer() {

}

void SSLPlayer::reset() {
    /// Linear control algorithm
//    setLinearPIDParameters(9.0, 0.0, 0.75, 0.0);
    setLinearPIDParameters(4.0, 0.0, 0.3, 0.0); // grSim
    setLinearDiscretePIDParameters(0.0, 0.0, 0.0);
    setLinearCascadePIDParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, maxLSpeed());
	setMaxLSpeed(2.25);
    setMaxLAcceleration(1.75);

    /// Angular control algorithm
//    setAngularPIDParameters(4.5, 0.15, 0.65, 20.0);
    setAngularPIDParameters(5.0, 0.15, 0.65, 20.0); // grSim
    setAngularDiscretePIDParameters(0.0, 0.0, 0.0);
    setAngularCascadePIDParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, maxASpeed());
    setMaxASpeed(GEARSystem::Angle::toRadians(220));

    /// Errors
    setLError(0.015);
    setAError(Angle::toRadians(4.0));
}

float SSLPlayer::robotRadius() const {
    return 0.09;
}

float SSLPlayer::robotLength() const {
    return 2.0*robotRadius();
}

bool SSLPlayer::cameraSpeedCheckEnabled() const {
    return false;
}

float SSLPlayer::cameraSpeedCheckFactor() const {
    return 0.75;
}

bool SSLPlayer::curveLSpeedReductionEnabled() const {
    return false;
}

std::pair<Angle,float> SSLPlayer::getNavDirectionDistance(const Position &destination, const Angle &positionToLook, bool avoidTeammates, bool avoidOpponents, bool avoidBall, bool avoidOurGoalArea, bool avoidTheirGoalArea) {
    std::pair<Angle,float> movement = Player::getNavDirectionDistance(destination, positionToLook, avoidTeammates, avoidOpponents, avoidBall, avoidOurGoalArea, avoidTheirGoalArea);
	movement.first.setValue(movement.first.value() - orientation().value() + GEARSystem::Angle::pi/2);
    return movement;
}

void SSLPlayer::_goToLookTo(const Position &destination, const Position &positionToLook, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea) {
	Angle angleToLook = this->angleTo(positionToLook.isUnknown()?destination:positionToLook);

//	if(isAtPosition(destination) == false) {
		// Gets direction and distance to destination
        std::pair<Angle,float> movement = getNavDirectionDistance(destination, (positionToLook.isUnknown()?orientation():angleToLook), avoidTeammates, avoidOpponents, avoidBall, avoidOurGoalArea, avoidTheirGoalArea);
        Angle direction = movement.first;
        float distance = movement.second;

        // Calc speed
        double linearSpeed = getLinearSpeed(distance);

		double angularSpeed = getAngularSpeed(angleToLook);

        // GoTo
        moveAndRotate(linearSpeed*cos(direction.value()), linearSpeed*sin(direction.value()), angularSpeed);

//	} else if(positionToLook.isUnknown()==false && isLookingTo(positionToLook)==false) {
//		// Calc angular speed
//		float angularSpeed = getAngularSpeed(angleToLook);

//		// GoTo
//		moveAndRotate(0.0, 0.0, angularSpeed);
//	} else
//		idle();
}

Position SSLPlayer::limitFieldDimensions(Position destination) {
	const Locations *loc = team()->loc();

	// X min
    if(destination.x() < loc->fieldMinX()-robotRadius())
        destination.setPosition(loc->fieldMinX()-robotRadius(), destination.y(), 0.0);
	// X max
    if(destination.x() > loc->fieldMaxX()+robotRadius())
        destination.setPosition(loc->fieldMaxX()+robotRadius(), destination.y(), 0.0);
	// Y min
    if(destination.y() < loc->fieldMinY()-robotRadius())
        destination.setPosition(destination.x(), loc->fieldMinY()-robotRadius(), 0.0);
	// Y max
    if(destination.y() > loc->fieldMaxY()+robotRadius())
        destination.setPosition(destination.x(), loc->fieldMaxY()+robotRadius(), 0.0);

	return destination;
}
