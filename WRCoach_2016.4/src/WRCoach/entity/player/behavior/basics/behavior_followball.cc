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

#include "behavior_followball.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_FollowBall::name() {
    return "Behavior_FollowBall";
}

Behavior_FollowBall::Behavior_FollowBall() {
    setBallDistance(0.6);

	_sk_goTo = NULL;
}

void Behavior_FollowBall::configure() {
    // Uses
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_FollowBall::run() {
    // Position
	Position desiredPosition = WR::Utils::threePoints(loc()->ball(), player()->position(), _distBall, 0.0);
	_sk_goTo->setDestination(desiredPosition);
//	_sk_goTo->setDestination(loc()->ball()); // UVF test
	_sk_goTo->setPositionToLook(loc()->ball());
//	_sk_goTo->setPositionToLook(loc()->theirGoal()); // UVF test
	_sk_goTo->avoidBall(false);
	_sk_goTo->avoidTeammates(true);
	_sk_goTo->avoidOpponents(true);
	_sk_goTo->avoidGoalArea(true);
}
