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

#include "behavior_penaltykick.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_PenaltyKick::name() {
    return "Behavior_PenaltyKick";
}

Behavior_PenaltyKick::Behavior_PenaltyKick() {
    _sk_goTo = NULL;

    _posToKick.setUnknown();

    _state = STATE_POS;
}

void Behavior_PenaltyKick::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_PenaltyKick::run() {

    // Set max speed when on penalty
    player()->setMaxLSpeed(0.5);

    Position posGoTo;
    Position posLookTo;

    switch(_state) {
        case STATE_POS: {
            posGoTo = WR::Utils::threePoints(loc()->ball(), loc()->theirGoal(), 0.12, GEARSystem::Angle::pi);
            posLookTo = WR::Utils::threePoints(loc()->ball(), loc()->theirGoal(), GlobalConstants::highDistance(), 0.0);

            // Calc pos to kick (aleatory)
            const Position posRight = loc()->theirGoalRightPost();
            const Position posLeft = loc()->theirGoalLeftPost();

            srand(clock());
            const int randSide = rand()%2;
            _posToKick = randSide? posRight : posLeft;

            // Switch state condition: positioned and can kick
            if(player()->isNearbyPosition(posGoTo, 2*player()->lError()) && utils()->isBallInFront() && player()->canKickBall())
                _state = STATE_KICK;
        } break;
        case STATE_KICK: {
            posGoTo = loc()->theirGoal();
            posLookTo = _posToKick;

            // Enable kick
            player()->kick();

            // Switch state condition: back to pos
            if(player()->distBall() > 0.5f || player()->canKickBall()==false)
                _state = STATE_POS;
        }
    }

    _sk_goTo->setDestination(posGoTo);
    _sk_goTo->setPositionToLook(posLookTo);
    _sk_goTo->avoidBall(utils()->isBehindBall(posGoTo) && _state==STATE_POS);
    _sk_goTo->avoidOpponents(_state==STATE_POS);
    _sk_goTo->avoidTeammates(_state==STATE_POS);
}
