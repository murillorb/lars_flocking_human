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

#include "behavior_gk.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define ATTACKER_MINBALLDIST 0.4f
#define GOALPOSTS_ERROR 0.05f
#define INTERCEPT_MINBALLVELOCITY 0.2f

QString Behavior_GK::name() {
    return "Behavior_GK";
}

Behavior_GK::Behavior_GK() {
    _sk_goTo = NULL;
    _sk_intBall = NULL;
    _sk_takeout = NULL;

    setRadius(0.5);
    setTakeoutEnabled(true);
    setTakeoutFactor(1.5);
    useAttackerOri(true);
}

void Behavior_GK::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
    usesSkill(_sk_intBall = new Skill_InterceptBall());
    usesSkill(_sk_takeout = new Skill_Takeout());
    setInitialSkill(_sk_goTo);
    addTransition(SKT_TAKEOUT, _sk_goTo, _sk_takeout);
    addTransition(SKT_TAKEOUT, _sk_intBall, _sk_takeout);
    addTransition(SKT_POS, _sk_takeout, _sk_goTo);
    addTransition(SKT_POS, _sk_intBall, _sk_goTo);
    addTransition(SKT_INTERCEPT, _sk_takeout, _sk_intBall);
    addTransition(SKT_INTERCEPT, _sk_goTo, _sk_intBall);
}

void Behavior_GK::run() {
    // State POS
    Position desiredPosition = getAttackerInterceptPosition();
    if(loc()->ourSide().isRight() && desiredPosition.x() > loc()->ourGoal().x()-GOALPOSTS_ERROR)       // Ensures that the robot
        desiredPosition.setPosition(loc()->ourGoal().x()-GOALPOSTS_ERROR, desiredPosition.y(), 0.0);   // doesnt cover
    else if(loc()->ourSide().isLeft() && desiredPosition.x() < loc()->ourGoal().x()+GOALPOSTS_ERROR)   // unnecessary angles
        desiredPosition.setPosition(loc()->ourGoal().x()+GOALPOSTS_ERROR, desiredPosition.y(), 0.0);
    _sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(loc()->ball());
	_sk_goTo->avoidAll(false);

    // State INTERCEPT
    _sk_intBall->setAdvance(true);

    // State TAKEOUT
    _sk_takeout->setPositionToAvoid(loc()->ourGoal());

    // Transitions
    if(player()->distBall()>_radius && utils()->isBallComingToGoal(INTERCEPT_MINBALLVELOCITY)) {
        enableTransition(SKT_INTERCEPT);
    } else if(_takeoutEnabled) {
        if(loc()->isInsideOurArea(loc()->ball(), _takeoutFactor)) {
            enableTransition(SKT_TAKEOUT);
        } else if(loc()->isInsideOurArea(loc()->ball(), 1.10*_takeoutFactor)==false) { // hysteresis (25%) applied on state exit to avoid oscillation
            enableTransition(SKT_POS);
        }
    } else {
        enableTransition(SKT_POS);
    }

    // Enable kick
    if(player()->canKickBall() && player()->distBall() < GlobalConstants::minBallDistToKick())
        player()->kick();
}

Position Behavior_GK::getAttackerInterceptPosition() {
    // Defense position
    Position interceptPosition = WR::Utils::threePoints(loc()->ourGoal(), loc()->ball(), _radius, 0.0);;
    if(_useAttackerOri==false)
        return interceptPosition;

    float goal_left = loc()->ourGoalLeftPost().y();
    float goal_right = loc()->ourGoalRightPost().y();

    // Calc ball impact based on attacker ori and check if its going to the goal
    Position posImpact = calcAttackerBallImpact();
    if(loc()->ourSide().isRight()) {
        goal_left += GOALPOSTS_ERROR;
        goal_right -= GOALPOSTS_ERROR;
        if(goal_left >= posImpact.y() && posImpact.y() >= goal_right)
            interceptPosition = WR::Utils::threePoints(posImpact, loc()->ball(), _radius, 0.0);
    } else {
        goal_left -= GOALPOSTS_ERROR;
        goal_right += GOALPOSTS_ERROR;
        if(goal_right >= posImpact.y() && posImpact.y() >= goal_left)
            interceptPosition = WR::Utils::threePoints(posImpact, loc()->ball(), _radius, 0.0);
    }

    return interceptPosition;
}

Position Behavior_GK::calcAttackerBallImpact() {
    // Get attacker info
    QList<kNN::element> knn = utils()->getOpponentKNN(1, loc()->ball());
    if(knn.empty())
        return loc()->ourGoal();

    const quint8 attackerId = knn.first().id;
    const Angle oriAttacker = PlayerBus::theirPlayer(attackerId)->orientation();
    const float distAttackerBall = PlayerBus::theirPlayer(attackerId)->distBall();

    // Check if attacker is sufficiently close to kick
    if(distAttackerBall>ATTACKER_MINBALLDIST)
        return loc()->ourGoal();

    // Check if attacker is really looking to ball, so he can kick
    if(PlayerBus::theirPlayer(attackerId)->utils()->isBallInFront()==false)
        return loc()->ourGoal();

    // Get impact position
    Position posImpact = utils()->getBallImpactPosition(oriAttacker);

    // Check if path is clear
    QList<quint8> exceptions;
    exceptions.push_back(player()->playerId());
    if(PlayerBus::theirPlayer(attackerId)->utils()->hasAnyPathTo(posImpact, exceptions)==false)
        return loc()->ourGoal();

    return posImpact;
}
