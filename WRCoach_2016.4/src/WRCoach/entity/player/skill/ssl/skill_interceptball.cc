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

#include "skill_interceptball.hh"
#include <WRCoach/entity/player/skill/wrskill.hh>

QString Skill_InterceptBall::name() {
    return "Skill_InterceptBall";
}

Skill_InterceptBall::Skill_InterceptBall() {
    setAdvance(false);
    _positionToLook.setUnknown();
}

void Skill_InterceptBall::run() {

    // Get intercept position
    Position desiredPos = utils()->getBallInterception();

    // Move slowly to ball
    if(_advance)
        desiredPos = WR::Utils::threePoints(desiredPos, loc()->ball(), 0.10, 0.0);

    // goToLookTo
    Position lookPos = (_positionToLook.isUnknown()? loc()->ball() : _positionToLook);
    player()->goToLookTo(desiredPos, lookPos, false, true, false);
}
