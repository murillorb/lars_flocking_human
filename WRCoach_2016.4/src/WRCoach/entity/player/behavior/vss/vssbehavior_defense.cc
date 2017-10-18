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

#include "vssbehavior_defense.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString VSSBehavior_Defense::name() {
	return "VSSBehavior_Defense";
}

VSSBehavior_Defense::VSSBehavior_Defense() {
	_sk_goTo = NULL;
}

void VSSBehavior_Defense::configure() {
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void VSSBehavior_Defense::run() {

    // Calc position to defense
	float posX = loc()->fieldMaxX()/2.0;
    float posY = loc()->ball().y();
	if(loc()->ourSide().isRight()) {
        if(loc()->ball().x() > -0.20)
            posY = -loc()->ball().y();
	} else {
        if(loc()->ball().x() < 0.20)
			posY = -loc()->ball().y();    
	}
    WR::Utils::limitValue(&posY, loc()->fieldMinY() + 0.20, loc()->fieldMaxY() - 0.20);
    Position desiredPosition(true, (loc()->ourSide().isRight()? posX : -posX), posY, 0.0);

    // Configure goTo
	_sk_goTo->setDestination(desiredPosition);
	_sk_goTo->avoidBall(false);
	_sk_goTo->avoidOpponents(false);
	_sk_goTo->avoidTeammates(true);
	_sk_goTo->avoidGoalArea(true);
}
