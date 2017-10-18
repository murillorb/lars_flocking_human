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

#include "vssbehavior_gka.hh"
#include <WRCoach/utils/line/line.hh>
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define ROBOT_ERROR 1.5
#define DIST_X 0.1

QString VSSBehavior_GKA::name() {
    return "VSSBehavior_GKA";
}

VSSBehavior_GKA::VSSBehavior_GKA(){
    _sk_goTo = NULL;
}

void VSSBehavior_GKA::configure(){
    usesSkill(_sk_goTo = new Skill_GoTo());
    setInitialSkill(_sk_goTo);
}

void VSSBehavior_GKA::run() {
    // Set GKA specific Linear PID calibration
    player()->setLinearPIDParameters(4.0, 0.0, 0.2, 0.0);
    player()->setMaxLSpeed(0.8);
    player()->setMaxLAcceleration(1.15);

    // Calc posX
    int signal = loc()->ourSide().isRight()? 1 : -1;
    float posX = signal*(loc()->fieldMaxX()-(DIST_X + loc()->fieldDefenseLength()+ROBOT_ERROR*player()->robotLength()/2));

    // Calc posY
    float posY = loc()->ball().y();
    if(fabs(posY) >= fabs(loc()->ourGoalRightPost().y())) {
        int signal = loc()->ball().y()>=0? 1 : -1;
        posY = signal*fabs(loc()->ourGoalRightPost().y());
    }

    // Get interception
    Position desiredPos(true, posX, posY, 0.0);
    _sk_goTo->setDestination(desiredPos);

    Position movDotUp(true, posX, loc()->fieldMaxX(), 0.0);
    _sk_goTo->setPositionToLook(movDotUp);
    _sk_goTo->avoidRobots(false);
    _sk_goTo->avoidBall(false);
    _sk_goTo->limitFieldDimensions(true);
    _sk_goTo->avoidOurGoalArea(false);
}
