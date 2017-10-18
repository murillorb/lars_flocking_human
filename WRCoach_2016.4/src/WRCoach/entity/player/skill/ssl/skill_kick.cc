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

#include "skill_kick.hh"
#include <WRCoach/entity/player/skill/wrskill.hh>

#define DIST_BEHINDBALL 0.16f
#define MINDISTBALL_AVOIDROBOTS 0.4f
#define SWITCHSTATE_MINDISTBALL 0.35f

QString Skill_Kick::name() {
	return "Skill_Kick";
}

Skill_Kick::Skill_Kick() {
    _posToKick.setUnknown();
    setKickPower(GlobalConstants::maxKickPower());
    avoidRobots(true);

    _state = STATE_POS;
}

void Skill_Kick::run() {
    if(_posToKick.isUnknown())
        std::cout << "[WARNING]" << name().toStdString() << ": position to kick not set!\n";

    // Local parameters
    Position desiredPos;
    bool avoidBall = false, avoidOpp = false, avoidTeam = false;

    const Position behindBall = WR::Utils::threePoints(loc()->ball(), _posToKick, DIST_BEHINDBALL, GEARSystem::Angle::pi);

    switch(_state) {
        default:
        case STATE_POS: {

            // Desired position
            desiredPos = behindBall;

            // PID hacking
            desiredPos = WR::Utils::threePoints(desiredPos, player()->position(), 0.03f, GEARSystem::Angle::pi);

            // Avoids
            avoidBall = utils()->isBehindBall(desiredPos);
            if(player()->distBall() > MINDISTBALL_AVOIDROBOTS) {
                avoidTeam = _avoidTeammates;
                avoidOpp = _avoidOpponents;
            } else {
                avoidTeam = false;
                avoidOpp = false;
            }

            // Switch state condition: is at behindBall
            if(player()->isNearbyPosition(behindBall, 1.5*player()->lError()))
                _state = STATE_KICK;

        } break;
        case STATE_KICK: {
            desiredPos = loc()->ball();
            avoidBall = false;
            avoidTeam = false;
            avoidOpp = false;

            // Switch state condition: distance to ball
            if(player()->distBall() > SWITCHSTATE_MINDISTBALL)
                _state = STATE_POS;

            // Switch state condition: looking to ball
            if(utils()->isBallInFront() == false)
                _state = STATE_POS;
        } break;
    }

    // goToLookTo
    Position lookPosition = WR::Utils::threePoints(_posToKick, loc()->ball(), GlobalConstants::highDistance(), GEARSystem::Angle::pi);
    player()->goToLookTo(desiredPos, lookPosition, avoidBall, avoidTeam, avoidOpp);

    // Enable kick
    if(player()->distBall()<GlobalConstants::minBallDistToKick() && player()->isLookingTo(_posToKick))
        player()->kick(_kickPower);
}
