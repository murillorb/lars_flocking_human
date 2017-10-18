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

#include "behavior_followentity.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_FollowEntity::name() {
    return "Behavior_FollowEntity";
}

Behavior_FollowEntity::Behavior_FollowEntity() {
    setEntityDistance(0.6);

	_sk_goTo = NULL;
}

void Behavior_FollowEntity::configure() {
    // Uses
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_FollowEntity::run() {
    // Position
//    Position desiredPosition = WR::Utils::threePoints(loc()->ball(), player()->position(), _entityDistance, 0.0);
    Position desiredPosition = WR::Utils::threePoints(PlayerBus::theirPlayer(0)->position(), player()->position(), _entityDistance, 0.0);
    _sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(loc()->ball());
    _sk_goTo->avoidBall(false);
    _sk_goTo->avoidRobots(false);
    _sk_goTo->avoidGoalArea(false);
    _sk_goTo->limitFieldDimensions(true);
}
