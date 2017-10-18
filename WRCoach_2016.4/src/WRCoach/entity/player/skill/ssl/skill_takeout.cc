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

#include "skill_takeout.hh"
#include <WRCoach/entity/player/skill/wrskill.hh>

QString Skill_Takeout::name() {
    return "Skill_Takeout";
}

Skill_Takeout::Skill_Takeout() {
    _posToAvoid.setUnknown();
    setKickPower(GlobalConstants::maxKickPower());

    _state = STATE_POS;
}

void Skill_Takeout::run() {
    // Generate position if not set
	if(_posToAvoid.isUnknown())
        _posToAvoid = WR::Utils::threePoints(player()->position(), loc()->ball(), 0.5, GEARSystem::Angle::pi);

    // Calc necessary positions
    Position behindBall = WR::Utils::threePoints(loc()->ball(), _posToAvoid, 0.16f, 0.0f);

    // Calc desiredPos
    Position desiredPos;
    bool avoidTeam=true, avoidOp=true, avoidBall=true;

    switch(_state) {
        case STATE_POS: {
            desiredPos = behindBall;

            // PID hacking
            desiredPos = WR::Utils::threePoints(desiredPos, player()->position(), 0.03f, GEARSystem::Angle::pi);

            // Avoids
            avoidBall = utils()->isBehindBall(desiredPos);
            avoidTeam = true;
            avoidOp = false;

            // Switch state condition: is at behind ball
            if(player()->isNearbyPosition(behindBall, 1.5*player()->lError()))
                _state = STATE_KICK;

        } break;
        case STATE_KICK: {
            desiredPos = loc()->ball();

            // Avoids
            avoidBall = false;
            avoidTeam = false;
            avoidOp = false;

            // Switch state condition: distance to ball
            if(player()->distBall() > 0.3f)
                _state = STATE_POS;

            // Switch state condition: looking to ball
            if(utils()->isBallInFront()==false)
                _state = STATE_POS;

        } break;
    }

    // goToLookTo
    player()->goToLookTo(desiredPos, loc()->ball(), avoidBall, avoidTeam, avoidOp);

    // Enable kick
    if(player()->distBall()<GlobalConstants::minBallDistToKick())
        player()->kick(_kickPower);
}
