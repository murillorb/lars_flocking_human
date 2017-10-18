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

#include "behavior_markballa.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_MarkBallA::name() {
    return "Behavior_MarkBallA";
}

Behavior_MarkBallA::Behavior_MarkBallA() {
    setDistToMark();
    setD();
    setSide();
    setCanGoToOpTeamFieldSide();

    _sk_goTo = NULL;
}

void Behavior_MarkBallA::configure() {
	usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_MarkBallA::run() {
    // Position
    Position desiredPosition = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), _distToMark, 0.0f);
    Position lookPosition = WR::Utils::threePoints(loc()->ourGoal(), loc()->ball(), 50.0f, 0.0f);

    // Op. team field side exception
    if(_canGoToOpTeamFieldSide == false) {
        if((loc()->ourSide().isRight() && desiredPosition.x()<0.60f) || (loc()->ourSide().isLeft() && desiredPosition.x()>-0.60f))
            desiredPosition = WR::Utils::threePoints(loc()->ourGoal(), loc()->ball(), loc()->fieldMaxX()-0.60f, 0.0f);
    }

    // Angle
    const Position projected(true, loc()->ourGoal().x(), loc()->ball().y(), 0.0f);
    float bx = WR::Utils::distance(loc()->ball(), projected);
    float by = loc()->ball().y();
    float angle = atan2(by, bx);

    // Position variation
    if(_side.isCenter())
        _d = 0.0f;
    float dx = _d*sin(angle);
    float dy = _d*cos(angle);
    if(loc()->ourSide().isLeft())
        dy = -dy;
    if(_side.isUndefined()) {
        if((loc()->ourSide().isRight() && loc()->ball().y()<=0) || (loc()->ourSide().isLeft() && loc()->ball().y()>0)) {
            dx = -dx;
            dy = -dy;
        }
    } else {
        if(_side.isLeft()) {
            dx = -dx;
            dy = -dy;
        }
    }

    // Adjust position
    desiredPosition.setPosition(desiredPosition.x()+dx, desiredPosition.y()+dy, 0.0);

    /// [WARNING] O tratamento abaixo não define uma nova posição que esteja necessáriamente fora da área.
    // Defense area exception
    if(loc()->isInsideOurArea(desiredPosition)) {
        desiredPosition = WR::Utils::threePoints(loc()->ball(), loc()->ourGoal(), _distToMark, GEARSystem::Angle::pi/2);
        lookPosition = loc()->ball();
    }

	_sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(lookPosition);
    _sk_goTo->avoidBall(utils()->isBehindBall(desiredPosition));
    _sk_goTo->avoidTeammates(true);
    _sk_goTo->avoidOpponents(true);
	_sk_goTo->avoidGoalArea(true);
}
