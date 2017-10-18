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

#include "vssplayer.hh"
#include <WRCoach/utils/utils.hh>
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>
#include <WRCoach/const/globalconstants.hh>
#include <WRCoach/entity/player/playerutils.hh>
#include <WRCoach/entity/player/playerbus.hh>
#include <WRCoach/entity/player/playeraccess.hh>

VSSPlayer::VSSPlayer(World *world, WRTeam *team, quint8 playerId, Controller *ctr, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg, Behavior *defaultBehavior, SSLReferee *ref)
 : Player(world, team, playerId, ctr, navAlg, linCtrAlg, angCtrAlg, defaultBehavior, ref) {
    _reversed = false;
    _avoidingGoalWalls = false;
}

VSSPlayer::~VSSPlayer() {

}

void VSSPlayer::reset() {
    /// Linear control algorithm
    setLinearPIDParameters(4.0, 0.0, 0.1, 0.0);
    setLinearCascadePIDParameters(0.5, 0.0, 0.001, 0.06, 0.0, 0.5, maxLSpeed());
    setMaxLSpeed(1.10);
    setMaxLAcceleration(0.9);

    /// Angular control algorithm
    setAngularPIDParameters(6.0, 0.0, 0.7, 0.0);
    setAngularCascadePIDParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, maxASpeed());
    setMaxASpeed(GEARSystem::Angle::toRadians(550));

    /// Errors
    setLError(0.015);
    setAError(Angle::toRadians(5));

    /// Robot reversion (VSS only)
    setReversionAngle(GEARSystem::Angle::toRadians(120));

    /// Robot anti-stuck (VSS only)
    setRobotStuckEnabled(true);
    setFieldWallStuckEnabled(true);
    setGoalWallsStuckEnabled(true);
}

float VSSPlayer::robotRadius() const {
    return 0.056;
}

float VSSPlayer::robotLength() const {
    return 0.08;
}

Angle VSSPlayer::orientation() const {
	if(_reversed) {
		return Angle(true, Player::orientation().value()+GEARSystem::Angle::pi);
	} else
		return Player::orientation();
}

bool VSSPlayer::isLookingTo(const Position &pos) {
    return VSSPlayer::isLookingTo(pos, aError());
}

bool VSSPlayer::isLookingTo(const Position &pos, float error) {
	Angle angle = this->angleTo(pos);
    processRobotReversion(angle);

    return Player::isLookingTo(pos, error);
}

void VSSPlayer::_goToLookTo(const Position &destination, const Position &positionToLook, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea) {
	Angle angleToLook = this->angleTo(positionToLook.isUnknown()?destination:positionToLook);

    if(isAtPosition(destination) == false) {
        // Gets direction and distance to destination
        std::pair<Angle,float> movement = getNavDirectionDistance(destination, (positionToLook.isUnknown()?orientation():angleToLook), avoidTeammates, avoidOpponents, avoidBall, avoidOurGoalArea, avoidTheirGoalArea);
        Angle direction = movement.first;
        float distance = movement.second;

        // Process robot reversion check
        processRobotReversion(direction);

        // Process anti-stuck checks
        processAntiStuck();

        // Calc speed
        double linearSpeed = getLinearSpeed(distance);
        double angularSpeed = getAngularSpeed(direction);

        // Move and rotate
        moveAndRotate(0.0, (_reversed?-1:1)*linearSpeed, angularSpeed);

    } else if(positionToLook.isUnknown()==false && isLookingTo(positionToLook)==false) {
        // Process the robot reversion
        processRobotReversion(angleToLook);

        // Calc angular speed
        float angularSpeed = getAngularSpeed(angleToLook);

        // GoTo
        moveAndRotate(0.0, 0.0, angularSpeed);

    } else {
        // At destination, and looking at positionToLook
        idle();

    }
}

Position VSSPlayer::limitFieldDimensions(Position destination) {
    // X min
    if(destination.x() < loc()->fieldMinX()+robotRadius())
        destination.setPosition(loc()->fieldMinX()+robotRadius(), destination.y(), 0.0);
    // X max
    if(destination.x() > loc()->fieldMaxX()-robotRadius())
        destination.setPosition(loc()->fieldMaxX()-robotRadius(), destination.y(), 0.0);
    // Y min
    if(destination.y() < loc()->fieldMinY()+robotRadius())
        destination.setPosition(destination.x(), loc()->fieldMinY()+robotRadius(), 0.0);
	// Y max
    if(destination.y() > loc()->fieldMaxY()-robotRadius())
        destination.setPosition(destination.x(), loc()->fieldMaxY()-robotRadius(), 0.0);

    return destination;
}

bool VSSPlayer::cameraSpeedCheckEnabled() const {
    return true;
}

float VSSPlayer::cameraSpeedCheckFactor() const {
    return 0.5;
}

bool VSSPlayer::curveLSpeedReductionEnabled() const {
    return true;
}

void VSSPlayer::processRobotReversion(const Angle &direction) {
    float diff = fabs(WR::Utils::angleDiff(direction, orientation()));
    if(diff>_reversionAngle) {
        if(_avoidingGoalWalls==false)
            revertRobot();
    }
}

void VSSPlayer::revertRobot() {
    _reversed = !_reversed;
}

void VSSPlayer::processAntiStuck() {
    // Check field wall stuck
    if(isFieldWallStuck())
        revertRobot();

    // Check robot stuck
    if(isRobotStuck())
        revertRobot();

    // Check goal walls stuck
    if(isGoalWallsStuck()) {
        revertRobot();
        _avoidingGoalWalls = true;
    }

    // Check clear flag for avoiding goal walls
    if(fabs(position().x()) <= 0.95*team()->loc()->fieldMaxX())
        _avoidingGoalWalls = false;
}

bool VSSPlayer::isFieldWallStuck() {
    // Check if is enabled
    if(_fieldWallStuckEnabled==false)
        return false;

    const Position posPlayer = position();
    const float stuckX = fabs(team()->loc()->ourFieldTopCorner().x()) - 1.2*robotRadius();
    const float stuckY = fabs(team()->loc()->ourFieldTopCorner().y()) - 1.2*robotRadius();

    // Inside field
    if(fabs(posPlayer.y())<=stuckY && fabs(posPlayer.x())<=stuckX)
        return false;

    // Upper wall
    if(posPlayer.y()>stuckY && fabs(WR::Utils::angleDiff(orientation(), Angle::pi/2)) < 2*aError())
        return true;

    // Bottom wall
    if(posPlayer.y()<-stuckY && fabs(WR::Utils::angleDiff(orientation(), 3*GEARSystem::Angle::pi/2)) < 2*aError())
        return true;

    // Right wall
    if(posPlayer.x()>stuckX && fabs(WR::Utils::angleDiff(orientation(), 0.0)) < 2*aError())
        return true;

    // Left wall
    if(posPlayer.x()<-stuckX && fabs(WR::Utils::angleDiff(orientation(), GEARSystem::Angle::pi)) < 2*aError())
        return true;

    // Upper right diagonal wall
    if(posPlayer.x()>stuckX && posPlayer.y()>stuckY && fabs(WR::Utils::angleDiff(orientation(), GEARSystem::Angle::pi/4)) < 2*aError()) {
//        std::cout << "diagonal upper right\n";
        return true;
    }

    // Upper left diagonal wall
    if(posPlayer.x()<-stuckX && posPlayer.y()>stuckY && fabs(WR::Utils::angleDiff(orientation(), 3*GEARSystem::Angle::pi/4)) < 2*aError()) {
//        std::cout << "diagonal upper left\n";
        return true;
    }

    // Bottom left diagonal wall
    if(posPlayer.x()<-stuckX && posPlayer.y()<-stuckY && fabs(WR::Utils::angleDiff(orientation(), 5*GEARSystem::Angle::pi/4)) < 2*aError()) {
//        std::cout << "diagonal bottom left\n";
        return true;
    }

    // Bottom right diagonal wall
    if(posPlayer.x()>stuckX && posPlayer.y()<-stuckY && fabs(WR::Utils::angleDiff(orientation(), 7*GEARSystem::Angle::pi/4)) < 2*aError()) {
//        std::cout << "diagonal bottom right\n";
        return true;
    }

    return false;
}

bool VSSPlayer::isRobotStuck() {
    // Check if is enabled
    if(_robotStuckEnabled==false)
        return false;

    // Get nearest players (obstacles)
    QList<kNN::element> obstacles = utils()->getKNN(5);

    // Run on obstacles
    for(int i=0; i<obstacles.size(); i++) {
        kNN::element obstacle = obstacles.at(i);

        // Check distance
        if(obstacle.value <= 1.1*robotLength()) {
            Position obstaclePos = (obstacle.team==teamId()? PlayerBus::ourPlayer(obstacle.id) : PlayerBus::theirPlayer(obstacle.id))->position();

            // Check orientation
			float obstacleAngle = this->angleTo(obstaclePos).value();
            float diff = WR::Utils::angleDiff(orientation(), obstacleAngle);
            if(fabs(diff) <= 8.0*aError())
                return true;

        } else {
            return false;
        }
    }

    return false;
}

bool VSSPlayer::isGoalWallsStuck() {
    // Check if is enabled
    if(_goalWallsStuckEnabled==false)
        return false;

    const Position posPlayer = position();
    const float stuckX = team()->loc()->fieldMaxX() - 0.8*robotRadius();
    const float stuckMinY = fabs(team()->loc()->ourGoalRightPost().y()) - 1.2*robotRadius();
    const float stuckMaxY = fabs(team()->loc()->ourGoalRightPost().y()) + 1.2*robotRadius();

    // Inner walls check
    // Position-orientation check
    if(fabs(posPlayer.x())>stuckX) {
        float diff = 0.0;

        // Upper post
        diff = WR::Utils::angleDiff(orientation(), GEARSystem::Angle::pi/2);
        if(posPlayer.y()>stuckMinY && posPlayer.y()<stuckMaxY && fabs(diff) < 4*aError())
            return true;

        // Bottom post
        diff = WR::Utils::angleDiff(orientation(), 3*GEARSystem::Angle::pi/2);
        if(posPlayer.y()<-stuckMinY && posPlayer.y()>-stuckMaxY && fabs(diff) < 4*aError())
            return true;
    }

    // Post check
    // Radius looking-to check
    float distRightPost = WR::Utils::distance(position(), team()->loc()->ourGoalRightPost());
    float distLeftPost = WR::Utils::distance(position(), team()->loc()->ourGoalLeftPost());
    // Upper post
    if(distRightPost<1.2*robotRadius() && isLookingTo(team()->loc()->ourGoalRightPost()))
        return true;
    // Bottom post
    if(distLeftPost<1.2*robotRadius() && isLookingTo(team()->loc()->ourGoalLeftPost()))
        return true;

    // Outside field goal line walls
    const float goalLineStuckX = fabs(team()->loc()->ourFieldTopCorner().x()) - 1.2*robotRadius();
    // Right wall
    if(posPlayer.x()>goalLineStuckX && fabs(WR::Utils::angleDiff(orientation(), 0.0)) < 2*aError())
        return true;
    // Left wall
    if(posPlayer.x()<-goalLineStuckX && fabs(WR::Utils::angleDiff(orientation(), GEARSystem::Angle::pi)) < 2*aError())
        return true;

    return false;
}
