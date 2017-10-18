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

#include "behavior_markball.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_MarkBall::name() {
	return "Behavior_MarkBall";
}

Behavior_MarkBall::Behavior_MarkBall() {
    setDistToMark(0.6f);
    setAdvanceToBall(false);
    setCanGoToOpTeamFieldSide(true);

    _state = STATE_POS;
    _sk_goTo = NULL;
}

void Behavior_MarkBall::configure() {
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_MarkBall::run() {
    bool avoidRobots = false;
    bool avoidBall = false;
    Position desiredPosition;
    Position lookPosition = loc()->ball();

    // Switch state machine
	switch(_state) {
        default:
        case STATE_POS: {
            desiredPosition = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), _distToMark, 0.0f);
            lookPosition = WR::Utils::threePoints(loc()->ourGoal(), loc()->ball(), GlobalConstants::highDistance(), 0.0f);
            avoidBall = utils()->isBehindBall(desiredPosition);
            avoidRobots = true;

            // Change state
            if(_advanceToBall && player()->isNearbyPosition(desiredPosition, 2.0f*player()->lError()))
                _state = STATE_GOTOBALL;
        } break;

        case STATE_GOTOBALL: {
			desiredPosition = loc()->ball();
            lookPosition = loc()->ball();
            avoidBall = avoidRobots = false;

            // Change state
            if(player()->distBall() > (_distToMark + 0.10f))
                _state = STATE_POS;
        } break;
	}

    // Defense area exception
    if(loc()->isInsideOurArea(desiredPosition)) {
        desiredPosition = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), _distToMark, GEARSystem::Angle::pi);
        lookPosition = loc()->ball();
        avoidBall = avoidRobots = true;
    }

    // Op. team field side exception
    if(_canGoToOpTeamFieldSide == false) {
        if((loc()->ourSide().isRight() && desiredPosition.x()<0.60f) || (loc()->ourSide().isLeft() && desiredPosition.x()>-0.60f))
            desiredPosition = WR::Utils::threePoints(loc()->ourGoal(), loc()->ball(), loc()->fieldMaxX()-0.60f, 0.0f);
    }

    // GoTo
	_sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(lookPosition);
    _sk_goTo->avoidRobots(avoidRobots);
    _sk_goTo->avoidBall(avoidBall);
	_sk_goTo->avoidGoalArea(true);
}
