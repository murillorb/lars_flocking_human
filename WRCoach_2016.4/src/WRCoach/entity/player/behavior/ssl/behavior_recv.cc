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

#include "behavior_recv.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_Recv::name() {
    return "Behavior_Recv";
}

Behavior_Recv::Behavior_Recv() {
    setQuadrant(Quadrants::ALL);
    setFollowAttacker(true);
    setAttacker(-1, 0.5f);
    setRadius(2.5f, 4.5f);

    _sk_goTo = NULL;

    _state = STATE_POS;
}

void Behavior_Recv::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_Recv::run() {
    if(_attackerId==-1)
        std::cout << "[WARNING]" << name().toStdString() << ", attacker id not set!\n";

    // Check if attacker is available
    if(PlayerBus::ourPlayerAvailable(_attackerId)==false) {
        std::cout << "[WARNING] Attacker ID not available!\n";
        return;
    }

    Position desiredPos = player()->position();
    Position lookPos = loc()->ball();
    bool avoidRobots = true;

    // Switch state
    switch(_state) {
        default:
        case STATE_POS: {
            Position posAttackerNext;
            float radius = 0.0f;

            avoidRobots = true;

            if(_followAttacker) {
                posAttackerNext = PlayerBus::ourPlayer(_attackerId)->nextPosition();
                float distAttackerGoal = WR::Utils::distance(posAttackerNext, loc()->theirGoal());
                radius = distAttackerGoal+_incDistAttacker;
                WR::Utils::limitValue(&radius, _minRadius, _maxRadius);
            }

            // Get clear goal position
            Position clearGoalPos = utils()->getClearGoalAndAttackerPosition(_quadrant, _attackerId, _minRadius, _maxRadius);
            if(clearGoalPos.isUnknown()==false) {
                desiredPos = clearGoalPos;
             } else {
                if(_followAttacker)
                    desiredPos = utils()->getGoalPosition(_quadrant, radius);
                else
                    desiredPos = utils()->getGoalPosition(_quadrant, (_minRadius+_maxRadius)/2.0f);
            }

            // Avoid attacker
            float distAttacker = WR::Utils::distance(desiredPos, posAttackerNext);
            if(distAttacker < 1.5f)
                desiredPos = WR::Utils::threePoints(desiredPos, posAttackerNext, 1.5f, GEARSystem::Angle::pi);
        } break;

        case STATE_WAIT: {
            avoidRobots = false;

            // Stay in position
            desiredPos = player()->position();

            // Switch state condition: attacker dist to ball
            if(PlayerBus::ourPlayer(_attackerId)->distBall() > 0.4f)
                _state = STATE_POS;

            // Switch state condition: dist to ball
            if(player()->distBall() < 0.3f)
                _state = STATE_POS;

        } break;

        case STATE_INTERCEPT: {
            avoidRobots = false;

            // Get intercept position
            desiredPos = utils()->getBallInterception();

            // Advance slowly to ball
            desiredPos = WR::Utils::threePoints(desiredPos, loc()->ball(), 0.05f, GEARSystem::Angle::pi);

            // Switch state condition: ball received
            if(player()->distBall()<0.30f && loc()->ballVelocity().abs()<0.5f) {
                _state = STATE_POS;
//                std::cout << "ballReceived, dist: " << player()->distBall() << ", ballVel: " << loc()->ballVelocity().abs() << "!\n";
                emit ballReceived(player()->playerId());
            }

            // Switch state condition: ball received failure -> ball not coming
            if(utils()->isBallComing(0.25f, 0.75f)==false) {
                _state = STATE_POS;
//                std::cout << "ballNotReceived!\n";
                emit ballNotReceived(player()->playerId());
            }

        } break;

    }

    // Skill GoTo
    _sk_goTo->setDestination(desiredPos);
    _sk_goTo->setPositionToLook(lookPos);
    _sk_goTo->avoidRobots(avoidRobots);
    _sk_goTo->avoidBall(false);
	_sk_goTo->avoidGoalArea(true);

}

void Behavior_Recv::attackerAboutToKick(quint8 recvID) {
    if(player()->playerId()==recvID) {
//        std::cout << "attackerAboutToKick, STATE_WAIT!\n";
        _state = STATE_WAIT;
    }
}

void Behavior_Recv::attackerKicked(quint8 recvID) {
    if(player()->playerId()==recvID) {
//        std::cout << "recv attackerKicked TO ME (#" << (int)player()->playerId() << "), STATE_INTERCEPT!\n";
        _state = STATE_INTERCEPT;
    }
}
