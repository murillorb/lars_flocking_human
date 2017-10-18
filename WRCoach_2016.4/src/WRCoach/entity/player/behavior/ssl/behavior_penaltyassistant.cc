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

#include "behavior_penaltyassistant.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_PenaltyAssistant::name() {
    return "Behavior_PenaltyKickAssistant";
}

Behavior_PenaltyAssistant::Behavior_PenaltyAssistant() {
    _sk_goTo = NULL;

    setAngle(GEARSystem::Angle::toRadians(30));
    setOurPenalty(true);
    setRadius(2.5);
    setSide(Sides::UNDEFINED);
}

void Behavior_PenaltyAssistant::configure() {
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_PenaltyAssistant::run() {
    float angle = 0.0;
	if(_side.isRight())
        angle = _angle;
	else if(_side.isLeft())
        angle = -_angle;

	// Position
    Position desiredPosition;
    Position posToLook;
    if(_ourPenalty) {
		desiredPosition = WR::Utils::threePoints(loc()->theirGoal(), loc()->theirPenaltyMark(), _radius, angle);
		posToLook = loc()->theirGoal();
	} else {
		desiredPosition = WR::Utils::threePoints(loc()->ourGoal(), loc()->ourPenaltyMark(), _radius, angle);
		posToLook = loc()->ourGoal();
	}

	// Configure skill
	_sk_goTo->setDestination(desiredPosition);
	_sk_goTo->setPositionToLook(posToLook);
	_sk_goTo->avoidAll(true);
	_sk_goTo->avoidGoalArea(true);
}
