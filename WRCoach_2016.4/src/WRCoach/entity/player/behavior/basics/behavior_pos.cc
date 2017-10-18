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

#include "behavior_pos.hh"
#include <float.h>
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_Pos::name() {
    return "Behavior_Pos";
}

Behavior_Pos::Behavior_Pos() {
    _destination.setUnknown();
    _posToLook.setUnknown();
    avoidAll(false);
    avoidGoalArea(true);
    limitFieldDimensions(true);

    _isPositioned = false;

    _sk_goTo = NULL;
}

void Behavior_Pos::configure() {
    // Skills
    usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_Pos::run() {
    Position desiredPosition = _destination;
    Position desiredPositionToLook = _posToLook;

    // Check if the params are not set
    if(desiredPosition.isUnknown())
        desiredPosition = player()->position();
    if(desiredPositionToLook.isUnknown())
        desiredPositionToLook = loc()->theirGoal();

    // Position to look
	// Projects a position very far away, so the robot keeps looking at it while moving
    Position posToLook = WR::Utils::threePoints(desiredPosition, desiredPositionToLook, GlobalConstants::highDistance(), 0.0);

    _sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(posToLook);
	_sk_goTo->avoidBall(_avoidBall ? utils()->isBehindBall(desiredPosition) : false);
    _sk_goTo->avoidTeammates(_avoidTeammates);
    _sk_goTo->avoidOpponents(_avoidOpponents);
	_sk_goTo->avoidGoalArea(_avoidGoalArea);

    // Check is positioned
    _isPositioned = player()->isAtPosition(desiredPosition);
}

bool Behavior_Pos::isPositioned() const {
	return _isPositioned;
}
