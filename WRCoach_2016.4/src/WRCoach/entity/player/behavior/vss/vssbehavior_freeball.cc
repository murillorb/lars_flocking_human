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

#include "vssbehavior_freeball.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString VSSBehavior_FreeBall::name() {
	return "Behavior_FreeBall";
}

VSSBehavior_FreeBall::VSSBehavior_FreeBall() {
	_sk_goTo = NULL;
}

void VSSBehavior_FreeBall::configure() {
	// Uses
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void VSSBehavior_FreeBall::run() {
    // Set FreeBall specific Linear PID calibration
    player()->setLinearPIDParameters(2.5, 0.0, 0.2, 0.0);
    player()->setMaxLSpeed(0.7);
    player()->setMaxLAcceleration(0.8);

    if(loc()->isInsideOurField(loc()->ball())) {
        player()->setLinearPIDParameters(5, 0.0, 0, 0.0);
        player()->setMaxLSpeed(0.7);
        player()->setMaxLAcceleration(1.8);
    }

    // GoTo
    Position desiredPos;
    float signal = loc()->ball().x()*loc()->ball().y();
    if(loc()->ourSide().isRight()) {
        if(signal >= 0) {
            desiredPos = WR::Utils::threePoints(loc()->ball(), player()->position(), 1.0, 1.04*GEARSystem::Angle::pi);
        } else {
            desiredPos = WR::Utils::threePoints(loc()->ball(), player()->position(), 1.0, 0.96*GEARSystem::Angle::pi);
        }
    } else {
        if(signal >= 0) {
            desiredPos = WR::Utils::threePoints(loc()->ball(), player()->position(), 1.0, 1.04*GEARSystem::Angle::pi);
        } else {
            desiredPos = WR::Utils::threePoints(loc()->ball(), player()->position(), 1.0, 0.96*GEARSystem::Angle::pi);
        }
    }
    _sk_goTo->setDestination(desiredPos);
    _sk_goTo->avoidAll(false);
    _sk_goTo->avoidGoalArea(loc()->isInsideTheirField(loc()->ball()));
}

