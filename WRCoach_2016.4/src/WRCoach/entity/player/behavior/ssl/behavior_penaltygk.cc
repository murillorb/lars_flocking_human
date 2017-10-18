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

#include "behavior_penaltygk.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define GK_DISTGOALLINE 0.07

QString Behavior_PenaltyGK::name() {
    return "Behavior_PenaltyGK";
}

Behavior_PenaltyGK::Behavior_PenaltyGK() {
    _sk_goTo = NULL;
}

void Behavior_PenaltyGK::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_PenaltyGK::run() {
    // Get attacker intercept position
    Position interceptPos = getAttackerInterceptPosition();

    // Project intercept position on goal line (GK can't run away from line)
    float signal = loc()->ourSide().isRight()? -1 : 1;
    Position desiredPosition = Position(true, loc()->ourGoal().x()+(signal*GK_DISTGOALLINE), interceptPos.y(), 0.0);
    Position lookPosition = Position(true, 0.0, player()->position().y(), 0.0);

    // GoTo
    _sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(lookPosition);
	_sk_goTo->avoidAll(false);

    // Enable kick
    if(player()->distBall() < GlobalConstants::minBallDistToKick())
        player()->kick();
}

Position Behavior_PenaltyGK::getAttackerInterceptPosition() {
    // Get attacker info
    QList<kNN::element> knn = utils()->getOpponentKNN(1, loc()->ball());
    if(knn.empty())
        return loc()->ourGoal();
    const quint8 attackerId = knn.first().id;
    const Position posAttacker = PlayerBus::theirPlayer(attackerId)->position();
    const Angle oriAttacker = PlayerBus::theirPlayer(attackerId)->orientation();

    // Check if exists an attacker
    if(posAttacker.isUnknown())
        return loc()->ourGoal();

    // Check attacker distance to ball
    const float distAttackerBall = WR::Utils::distance(posAttacker, loc()->ball());
    if(distAttackerBall > 0.40f)
        return loc()->ourGoal();

    // Defense position
    Position interceptPosition = loc()->ourGoal();
    float goalLeftPost = loc()->ourGoalLeftPost().y();
    float goalRightPost = loc()->ourGoalRightPost().y();

    // Calc ball impact based on attacker ori and check if its going to the goal
    Position posImpact = utils()->getBallImpactPosition(oriAttacker);
    if(loc()->ourSide().isRight()) {
        goalLeftPost += 0.09f;
        goalRightPost -= 0.09f;
        if(posImpact.isUnknown()) {
            float ori = oriAttacker.value();
            if(ori > (5.0f/4.0f)*GEARSystem::Angle::pi)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalRightPost(), loc()->ball(), 0.20f, 0.0f);
            else if(ori < (3.0f/4.0f)*GEARSystem::Angle::pi)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalLeftPost(), loc()->ball(), 0.20f, 0.0f);
            else
                interceptPosition = loc()->ourGoal();
        } else if(goalLeftPost >= posImpact.y() && posImpact.y() >= goalRightPost) {
            interceptPosition = WR::Utils::threePoints(posImpact, loc()->ball(), 0.20f, 0.0f);
        } else {
            if(posImpact.y() >= goalLeftPost)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalLeftPost(), loc()->ball(), 0.20f, 0.0f);
            if(posImpact.y() <= goalRightPost)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalRightPost(), loc()->ball(), 0.20f, 0.0f);
        }
    } else {
        goalLeftPost -= 0.09f;
        goalRightPost += 0.09f;
        if(posImpact.isUnknown()) {
            float ori = oriAttacker.value();
            if(ori < (7.0f/4.0f)*GEARSystem::Angle::pi && ori > GEARSystem::Angle::pi)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalLeftPost(), loc()->ball(), 0.20f, 0.0f);
            else if(ori > (1.0f/4.0f)*GEARSystem::Angle::pi && ori < GEARSystem::Angle::pi)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalRightPost(), loc()->ball(), 0.20f, 0.0f);
            else
                interceptPosition = loc()->ourGoal();
        } else if(goalRightPost >= posImpact.y() && posImpact.y() >= goalLeftPost) {
            interceptPosition = WR::Utils::threePoints(posImpact, loc()->ball(), 0.2, 0.0);
        } else {
            if(posImpact.y() >= goalRightPost)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalRightPost(), loc()->ball(), 0.2, 0.0);
            if(posImpact.y() <= goalLeftPost)
                interceptPosition = WR::Utils::threePoints(loc()->ourGoalLeftPost(), loc()->ball(), 0.2, 0.0);
        }
    }

    return interceptPosition;
}
