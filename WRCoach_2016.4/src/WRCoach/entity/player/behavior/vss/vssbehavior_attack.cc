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

#include "vssbehavior_attack.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define BALLINAREA_FACTOR_STOPPED 1.15
#define BALLINAREA_FACTOR_MOVING 1.50
#define BALLNEARWALL_DISTANCE 0.10
#define STATEWALL_DISTBALL 0.06

QString VSSBehavior_Attack::name() {
	return "VSSBehavior_Attack";
}

VSSBehavior_Attack::VSSBehavior_Attack() {
	_sk_pushBall = NULL;
    _sk_goTo = NULL;

    _subState = WALL_POS;
    _canGoToBall = false;

    this->setAvoidDefenseArea(true);
}

void VSSBehavior_Attack::configure() {
    usesSkill(_sk_pushBall = new VSSSkill_PushBall());
    usesSkill(_sk_goTo = new Skill_GoTo());
	usesSkill(_sk_spin = new Skill_Spin());

    setInitialSkill(_sk_goTo);

    addTransition(SKT_GOTO, _sk_pushBall, _sk_goTo);
    addTransition(SKT_GOTO, _sk_spin, _sk_goTo);
    addTransition(SKT_PUSH, _sk_goTo, _sk_pushBall);
    addTransition(SKT_PUSH, _sk_spin, _sk_pushBall);
    addTransition(SKT_SPIN, _sk_goTo, _sk_spin);
    addTransition(SKT_SPIN, _sk_pushBall, _sk_spin);
}

void VSSBehavior_Attack::run() {

    // Set Attacker specific Linear PID calibration
    player()->setLinearPIDParameters(6.0, 0.0, 0.2, 0.0);
    player()->setMaxLSpeed(1.0);
    player()->setMaxLAcceleration(0.7);

    Walls::Wall wall;

    // Switch state conditions
    if(_avoidDefenseArea && loc()->isInsideOurArea(loc()->ball(), BALLINAREA_FACTOR_STOPPED))
        _state = STATE_WAITAREA;
    else if(_avoidDefenseArea && loc()->isInsideOurArea(loc()->ball(), BALLINAREA_FACTOR_MOVING) && loc()->ballVelocity().abs() >= 0.04)
        _state = STATE_WAITAREA;
	else if(loc()->isNearWall(loc()->ball(), BALLNEARWALL_DISTANCE, &wall))
		_state = STATE_WALL;
    else
        _state = STATE_ATTACK;

    // Switch state machine
    switch(_state) {
        default:
        case STATE_ATTACK: {
            enableTransition(SKT_PUSH);

            // Reset wall state
            _canGoToBall = false;

            // Calc free goal position
            Position clearKickPos = utils()->getClearKickPosition();
            if(clearKickPos.isUnknown())
                clearKickPos = loc()->theirGoal();

            // Configure push ball
            _sk_pushBall->setDestination(clearKickPos);

        } break;

		case STATE_WAITAREA: {
            enableTransition(SKT_GOTO);

            // Reset wall state
            _canGoToBall = false;

            // Calc wait positions
            const float waitPosX = loc()->fieldMaxX()*3/4;
            int signalWaitY = loc()->ball().y()>0? 1 : -1;

            const float waitPosY = signalWaitY*loc()->fieldMaxY()*2/3;
            float signalX = (loc()->ourSide().isRight()? 1 : -1);

            Position waitPosUpper(true, signalX*waitPosX, waitPosY, 0.0);
            Position waitPosBottom(true, signalX*waitPosX, waitPosY, 0.0);

            // Calc desired position based on ball
            Position waitPos = (loc()->ball().y()>=0? waitPosBottom : waitPosUpper);

            // Configure goTo
            _sk_goTo->setDestination(waitPos);
            _sk_goTo->setPositionToLook(loc()->ball());
            _sk_goTo->avoidRobots(true);
            _sk_goTo->avoidBall(false);
            _sk_goTo->limitFieldDimensions(true);
            _sk_goTo->avoidOurGoalArea(true);

        } break;

		case STATE_WALL: {
			bool clockwise = false;

            // Sub state machine
            switch(_subState) {
                default:
                case WALL_POS: {
                    enableTransition(SKT_GOTO);

                    bool avoidBall = false;
                    Position desiredPos;

                    if(_canGoToBall) {
                        avoidBall = false;

                        // Calc position to spin
                        desiredPos = loc()->ball();

                        // Switch state condition: near ball
                        if(player()->distBall() <= 1.25*player()->robotRadius()) {
                            _subState = WALL_SPIN;
                            _spinTimer.start();
                        }

                        // Return state condition: far from ball
                        if(!Walls::isCorner(wall) && player()->distBall() > 0.20 )
                            _canGoToBall = false;

                    } else {
                        avoidBall = true;

                        // Calc desired position
                        if(wall == Walls::UPPER) {
                            float signal = loc()->ourSide().isRight()? 1 : -1;
                            desiredPos = WR::Utils::threePoints(loc()->ball(),  loc()->theirGoal(), 0.10, signal*GEARSystem::Angle::pi/2.0);

                        } else if(wall == Walls::BOTTOM) {
                            float signal = loc()->ourSide().isRight()? -1 : 1;
                            desiredPos = WR::Utils::threePoints(loc()->ball(),  loc()->theirGoal(), 0.10, signal*GEARSystem::Angle::pi/2.0);

                        } else if(wall == Walls::RIGHT) {
                            if(loc()->ourSide().isRight()) {
                                float delta = (loc()->ball().y()<0.0)? GEARSystem::Angle::pi/4.0 : -GEARSystem::Angle::pi/4.0;
                                desiredPos = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), 0.10, delta);
                            } else {
                                float delta = (loc()->ball().y()<0.0)? GEARSystem::Angle::pi*3/4 : -GEARSystem::Angle::pi*3/4;
                                desiredPos = WR::Utils::threePoints(loc()->ball(), loc()->theirGoal(), 0.10, delta);
                            }
                        } else if(wall == Walls::LEFT) {
                            if(loc()->ourSide().isLeft()) {
                                float delta = (loc()->ball().y()<0.0)? -GEARSystem::Angle::pi/4.0:GEARSystem::Angle::pi/4.0;
                                desiredPos = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), 0.10, delta);
                            } else {
                                float delta = (loc()->ball().y()<0.0)? -GEARSystem::Angle::pi*3/4:GEARSystem::Angle::pi*3/4;
                                desiredPos = WR::Utils::threePoints(loc()->ball(), loc()->theirGoal(), 0.10, delta);
                            }
                        } else {
                            // Switch state condition: ball in corner
                            _canGoToBall = true;
                        }

                        // Switch state condition: near position
                        if(player()->isNearbyPosition(desiredPos, 2*player()->lError()))
                            _canGoToBall = true;
                    }

                    // Configure goTo
                    _sk_goTo->setDestination(desiredPos);
                    _sk_goTo->avoidRobots(player()->distanceTo(loc()->ball())>0.30);
                    _sk_goTo->avoidBall(avoidBall);
                    _sk_goTo->limitFieldDimensions(true);
                    _sk_goTo->avoidOurGoalArea(false);

                } break;
                case WALL_SPIN: {
                    enableTransition(SKT_SPIN);

                    // Reset wall state
                    _canGoToBall = false;

                    // Calc rotation
                    if(loc()->ball().y() >= 0) {
                        if(loc()->ourSide().isRight())
                            clockwise = false;
                        else
                            clockwise = true;
                    } else {
                        if(loc()->ourSide().isRight())
                            clockwise = true;
                        else
                            clockwise = false;
                    }

                    // Configure spin
                    _sk_spin->setClockwise(clockwise);
                    _sk_spin->setFast(true);

                    // Return state condition: far from ball
                    _spinTimer.stop();
                    if(player()->distBall() >= 1.5f*player()->robotRadius() || _spinTimer.timesec() >= 1.0)
                        _subState = WALL_POS;

                } break;
            }

        } break;
    }
}
