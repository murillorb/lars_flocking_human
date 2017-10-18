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

#include "vssbehavior_attacka.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define PROXIMITY_Y 0.66;
#define DIST_X 0.35;

QString VSSBehavior_AttackA::name() {
    return "VSSBehavior_AttackA";
}

VSSBehavior_AttackA::VSSBehavior_AttackA(){
    _sk_goTo = NULL;
}

void VSSBehavior_AttackA::configure(){
    usesSkill(_sk_goTo = new Skill_GoTo());
}

void VSSBehavior_AttackA::run(){
    // Check if attacker is available
    if(PlayerBus::ourPlayerAvailable(_attackerId)==false) {
        std::cout << "[WARNING] Attacker ID not available!\n";
        return;
    }

    // Calc poxX
    int signalX = loc()->ourSide().isRight() > 0 ? 1 : -1;
    float posX = loc()->ball().x() + signalX*DIST_X;

    // Calc posY
    int signalY = PlayerBus::ourPlayer(_attackerId)->position().y() > 0? -1 : 1;
    float posY = PlayerBus::ourPlayer(_attackerId)->position().y() + signalY*loc()->fieldMaxY()*PROXIMITY_Y;

    // goTo
    Position desiredPos(true, posX, posY, 0.0);
    _sk_goTo->setDestination(desiredPos);
    _sk_goTo->avoidRobots(true);
    _sk_goTo->avoidBall(true);
    _sk_goTo->limitFieldDimensions(true);
    _sk_goTo->avoidOurGoalArea(true);
}
