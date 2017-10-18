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

#include "behavior_attack.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define RECV_MIN_DIST_TO_PASS 0.75f

QString Behavior_Attack::name() {
    return "Behavior_Attack";
}

Behavior_Attack::Behavior_Attack() {
    setMinimumKickDistance(GlobalConstants::highDistance());
    setEnableClearKick(true);
    setEnablePass(true);
    setForcePass(false);

    _sk_goTo = NULL;
    _sk_pushBall = NULL;

    _kickPower = GlobalConstants::maxKickPower();
    _recvId = -1;

    _state = STATE_WAIT;
}

void Behavior_Attack::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
    usesSkill(_sk_pushBall = new Skill_PushBall());

    setInitialSkill(_sk_goTo);

    addTransition(SKT_ATTACK, _sk_goTo, _sk_pushBall);
    addTransition(SKT_WAIT, _sk_pushBall, _sk_goTo);
}

void Behavior_Attack::run() {

    // Check if cannot kick ball
    if(player()->canKickBall()==false || loc()->isInsideTheirArea(loc()->ball(), 0.95f) ||
    loc()->isOutsideField(loc()->ball(), 1.05f) || loc()->isInsideOurArea(loc()->ball(), 2.0))
        _state = STATE_WAIT;

    // Switch state
    switch(_state) {
		default:
        case STATE_WAIT: {
            enableTransition(SKT_WAIT);

            // Calc waiting pos
            Position waitingPos;
            if(loc()->isInsideTheirArea(loc()->ball())) {
                Position far = WR::Utils::projectPointAtSegment(loc()->theirGoalRightMidPost(), loc()->theirGoalLeftMidPost(), player()->position());

                float waitDist = loc()->fieldDefenseRadius()+1.5*player()->robotRadius();
                if(loc()->isOutsideField(loc()->ball(), 0.975f))
                    waitingPos = WR::Utils::threePoints(loc()->theirGoal(), loc()->fieldCenter(), waitDist, 0.0);
                else
                    waitingPos = WR::Utils::threePoints(far, loc()->ball(), waitDist, 0.0f);

            } else if(loc()->isInsideOurArea(loc()->ball(), 1.5)) {
                waitingPos = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), 2.0f, GEARSystem::Angle::pi);
            } else {
                waitingPos = WR::Utils::threePoints(loc()->ball(), loc()->theirGoal(), 0.6f, GEARSystem::Angle::pi);
            }

            // Configure goTo
            Position lookPos = WR::Utils::threePoints(loc()->ball(), waitingPos, GlobalConstants::highDistance(), GEARSystem::Angle::pi);
            _sk_goTo->setDestination(waitingPos);
            _sk_goTo->setPositionToLook(lookPos);
            _sk_goTo->avoidBall(utils()->isBehindBall(waitingPos));
            _sk_goTo->avoidTeammates(true);
            _sk_goTo->avoidOpponents(true);
			_sk_goTo->avoidGoalArea(true);

            // Switch state condition
            if(player()->canKickBall() && loc()->isInsideTheirArea(loc()->ball())==false)
                _state = STATE_PUSH;

        } break;

		case STATE_PUSH: {
            enableTransition(SKT_ATTACK);

            _recvId = -1;
            _posToKick = loc()->theirGoal();
            _kickPower = 8.0f;

            // Pos to kick
            if(_clearKickEnabled) {

                // Clear kick pos
                Position clearKick = utils()->getClearKickPosition();
                if(clearKick.isUnknown()==false && _forcePass==false) {
                    _posToKick = clearKick;

                } else {
                    // Pass
                    if(_passEnabled || _forcePass) {

                        _mutex.lock();

                        // Select a receiver to pass
                        _recvId = getBestRecv();

                        // Check if has a recv to pass
                        if(_recvId!=-1) {

                            // Get Recv position
                            Position posRecv = PlayerBus::ourPlayer(_recvId)->position();
                            if(posRecv.isUnknown()==false) {
                                _posToKick = posRecv;

                                // Calc kick power
                                const float dist = WR::Utils::distance(player()->position(), _posToKick);
                                //const float T = 2.0f; // seconds
                                const float T = 0.7f; // seconds

                                _kickPower = dist/T;
                                WR::Utils::limitValue(&_kickPower, 1.0f, 8.0f);
                            }
                        }

                        _mutex.unlock();
                    }

                }

            }

            // Set position
            _sk_pushBall->setDestination(_posToKick);

            // Emit signal to inform its about to kick
            Position behindBall = WR::Utils::threePoints(loc()->ball(), _posToKick, 0.20, GEARSystem::Angle::pi);
            if(_recvId!=-1 && player()->distBall()<0.3f && utils()->isBehindBall(behindBall)==false)
                emit aboutToKick(_recvId);

            // Change state condition: kick
            float distToPosToKick = player()->distanceTo(_posToKick);
            if(distToPosToKick<_maxDistToKick && player()->distBall()<0.30f && utils()->isBallInFront()) { ///Checar primeira condição
                _timer.start();
                _state = STATE_KICK;
            }

        } break;

		case STATE_KICK: {
            enableTransition(SKT_ATTACK);

            // Reupdate recv position to avoid recv movement interference
            if(_recvId!=-1) {
                if(PlayerBus::ourPlayerAvailable(_recvId))
                    _posToKick = PlayerBus::ourPlayer(_recvId)->position();
            }

            // Enable kick
            if(player()->isLookingTo(_posToKick, 1.5*player()->aError()))
                player()->kick(_kickPower);

            // Change state condition: foot check and distance to ball
            if(!utils()->isBallInFront() || player()->distBall()>0.40f) {
                // Ball kicked monitoring
                if(_recvId!=-1)
                    emit ballKicked(_recvId);

                _state = STATE_PUSH;
            }

            // Change state condition: time
            _timer.stop();
            if(_timer.timesec()>=2.5f)
                _state = STATE_PUSH;

        } break;

    }

}

qint8 Behavior_Attack::getBestRecv() {
    // Check recvs list
    if(_recvs.empty())
        return -1;

    for(int i=_recvs.size()-1; i>=0; i--) {
        quint8 recv = _recvs.at(i);
        if(PlayerBus::ourPlayerAvailable(recv)==false)
            _recvs.removeAt(i);
    }

    // Order by distance to goal (bubble-sort)
    if(_recvs.size()>1) {
        for(int i=0; i<_recvs.size()-1; i++) {
            for(int j=i+1; j<_recvs.size(); j++) {
                quint8 recv1 = _recvs.at(i);
                float recv1dist = PlayerBus::ourPlayer(recv1)->distTheirGoal();

                quint8 recv2 = _recvs.at(j);
                float recv2dist = PlayerBus::ourPlayer(recv2)->distTheirGoal();

                if(recv2dist <= recv1dist)
                    _recvs.swap(i, j);
            }
        }
    }

    // Get best recv
    qint8 bestRecvId=-1;
    for(int i=0; i<_recvs.size(); i++) {
        const quint8 recvId = _recvs.at(i);

        // Get recv position
        const Position posRecv = PlayerBus::ourPlayer(recvId)->position();

        // Remove if too close
        float dist = player()->distanceTo(posRecv);
        if(_forcePass==false && dist<RECV_MIN_DIST_TO_PASS)
            continue;

        // Remove if too close to our goal
        if(_forcePass==false && PlayerBus::ourPlayer(recvId)->distOurGoal() < loc()->fieldMaxX()/2.0f)
            continue;

        // If has clear path
        if(utils()->hasClearPathToTeammate(recvId)) {
            bestRecvId = recvId;
            break;
        }
    }

    return bestRecvId;
}
