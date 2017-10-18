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

#include "skill_pushball.hh"
#include <WRCoach/entity/player/skill/wrskill.hh>

#define BALLPREVISION_MINVELOCITY 0.02f
#define BALLPREVISION_VELOCITY_FACTOR 3.0f
#define BALLPREVISION_FACTOR_LIMIT 0.15f

QString Skill_PushBall::name() {
	return "Skill_PushBall";
}

Skill_PushBall::Skill_PushBall() {
    _destination.setUnknown();
    setMaxPushDistance(GlobalConstants::maxPushDistance());
    avoidRobots(true);

    _state = STATE_POS;

    _currPos.setUnknown();
    _lastPos.setUnknown();
    _distPushed = 0.0f;
}

void Skill_PushBall::run() {
    if(_destination.isUnknown())
        std::cout << "[WARNING]" << name().toStdString() << ": destination not set!\n";

    // Calc behind ball
    Position behindBall = WR::Utils::threePoints(loc()->ball(), _destination, 0.16f, GEARSystem::Angle::pi);

    // Ball prevision
    if(loc()->ballVelocity().abs() > BALLPREVISION_MINVELOCITY) {
        // Calc unitary vector of velocity
        const Position velUni(true, loc()->ballVelocity().x()/loc()->ballVelocity().abs(), loc()->ballVelocity().y()/loc()->ballVelocity().abs(), 0.0);

        // Calc velocity factor
        float factor = BALLPREVISION_VELOCITY_FACTOR*loc()->ballVelocity().abs();
        WR::Utils::limitValue(&factor, 0.0f, BALLPREVISION_FACTOR_LIMIT);

        // Calc projected position
        const Position delta(true, factor*velUni.x(), factor*velUni.y(), 0.0);
        Position projectedPos(true, behindBall.x()+delta.x(), behindBall.y()+delta.y(), 0.0);

        // Set projected position only if player is behind ball (ie., passed ball line)
        if(utils()->isBehindBall(projectedPos))
            behindBall = projectedPos;
    }

    // Local parameters
    Position desiredPos;
    bool avoidBall, avoidOpp, avoidTeam;

    // State machine
    switch(_state) {
        default:
		case STATE_POS: {
            // Reset push distance control
            _currPos.setUnknown();

            // Desired position
            desiredPos = behindBall;

            // PID hacking
            desiredPos = WR::Utils::threePoints(desiredPos, player()->position(), 0.05f, GEARSystem::Angle::pi);

            // Avoid ball?
            float angDesired = WR::Utils::getAngle(loc()->ball(), desiredPos);
            float angPlayer = WR::Utils::getAngle(loc()->ball(), player()->position());
            float diff = WR::Utils::angleDiff(angDesired, angPlayer);
            avoidBall = (fabs(diff) > GEARSystem::Angle::toRadians(30));

            // Avoid robots?
            if(player()->distBall() > 0.40f) {
                avoidTeam = _avoidTeammates;
                avoidOpp = _avoidOpponents;
            } else {
                avoidTeam = false;
                avoidOpp = false;
            }

            // Switch state condition: is at behindBall
            if(player()->isNearbyPosition(behindBall, 2*player()->lError()))
                _state = STATE_PUSH;
        } break;

        case STATE_PUSH: {

            // Desired position
            desiredPos = WR::Utils::threePoints(loc()->ball(), player()->position(), 0.30, GEARSystem::Angle::pi);

            // Avoids
            avoidBall = false;
            avoidTeam = false;
            avoidOpp = false;

            // Push distance control
            if(_currPos.isUnknown()) {
                _distPushed = 0.0f;
                _currPos = loc()->ball();
            }
            _lastPos = _currPos;
            _currPos = loc()->ball();
            _distPushed += WR::Utils::distance(_lastPos, _currPos);

            // Switch state condition: distance to ball
            if(player()->distBall() > 0.35f) {
                _state = STATE_POS;
            }

            // Switch state condition: looking to ball
            if(utils()->isBallInFront() == false) {
                _state = STATE_POS;
            }

            // Max dist pushed condition
            if(_distPushed >= _maxPushDistance) {
                player()->idle();

                // Reset push distance control
                if(player()->distBall() > 0.10f)
                    _currPos.setUnknown();
            }
        } break;

	}

    // goToLookTo
    Position lookPos = WR::Utils::threePoints(_destination, loc()->ball(), GlobalConstants::highDistance(), GEARSystem::Angle::pi);
    player()->goToLookTo(desiredPos, lookPos, avoidBall, avoidTeam, avoidOpp);
}
