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

#include "behavior_gka.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

#define INTERCEPT_MINBALLDIST 0.5f
#define INTERCEPT_MINBALLVELOCITY 0.2f

QString Behavior_GKA::name() {
    return "Behavior_GKA";
}

Behavior_GKA::Behavior_GKA() {
    setRadius(1.35f);
    setD(0.20f);
    setTakeoutEnabled(true);
    setTakeoutFactor(2.0f);
    setSide(Sides::UNDEFINED);

    _sk_goTo = NULL;
    _sk_intBall = NULL;
    _sk_kickout = NULL;
}

void Behavior_GKA::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
    usesSkill(_sk_intBall = new Skill_InterceptBall());
    usesSkill(_sk_kickout = new Skill_Kick());
    setInitialSkill(_sk_goTo);
    addTransition(SKT_KICKOUT, _sk_goTo, _sk_kickout);
    addTransition(SKT_KICKOUT, _sk_intBall, _sk_kickout);
    addTransition(SKT_POS, _sk_kickout, _sk_goTo);
    addTransition(SKT_POS, _sk_intBall, _sk_goTo);
    addTransition(SKT_INTERCEPT, _sk_goTo, _sk_intBall);
    addTransition(SKT_INTERCEPT, _sk_kickout, _sk_intBall);
}

void Behavior_GKA::run() {

    // State POS
    // Position
    Position far = WR::Utils::projectPointAtSegment(loc()->ourGoalRightMidPost(), loc()->ourGoalLeftMidPost(), loc()->ball());
    Position desiredPosition = WR::Utils::threePoints(far, loc()->ball(), _radius, 0.0f);

    // Angle
    const Position projected(true, loc()->ourGoal().x(), loc()->ball().y(), 0.0);
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
        if((loc()->ourSide().isRight() && loc()->ball().y()>=player()->lError()) || (loc()->ourSide().isLeft() && loc()->ball().y()<-player()->lError())) {
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

    // Ensures that the robot doesnt cover unnecessary angles
	if(_side.isUndefined()) {
		if(loc()->ourSide().isRight() && desiredPosition.x() > loc()->ourGoal().x()-0.3)
			desiredPosition.setPosition(loc()->ourGoal().x()-0.3, desiredPosition.y(), 0.0);
		else if(loc()->ourSide().isLeft() && desiredPosition.x() < loc()->ourGoal().x()+0.3)
			desiredPosition.setPosition(loc()->ourGoal().x()+0.3, desiredPosition.y(), 0.0);
	} else {
		if(loc()->ourSide().isRight()) {
			if(loc()->ball().y()>=0) {
				if(_side.isRight()) {
					if(desiredPosition.x() > loc()->ourGoal().x()-0.2)
						desiredPosition.setPosition(loc()->ourGoal().x()-0.2, desiredPosition.y()+0.1, 0.0);
				} else {
					if(desiredPosition.x() > loc()->ourGoal().x()-0.45)
						desiredPosition.setPosition(loc()->ourGoal().x()-0.45, desiredPosition.y(), 0.0);
				}
			} else {
				if(_side.isRight()) {
					if(desiredPosition.x() > loc()->ourGoal().x()-0.45)
						desiredPosition.setPosition(loc()->ourGoal().x()-0.45, desiredPosition.y(), 0.0);
				} else {
					if(desiredPosition.x() > loc()->ourGoal().x()-0.2)
						desiredPosition.setPosition(loc()->ourGoal().x()-0.2, desiredPosition.y()-0.1, 0.0);
				}
			}
		} else {
			if(loc()->ball().y()>=0) {
				if(_side.isRight()) {
					if(desiredPosition.x() < loc()->ourGoal().x()+0.45)
						desiredPosition.setPosition(loc()->ourGoal().x()+0.45, desiredPosition.y(), 0.0);
				} else {
					if(desiredPosition.x() < loc()->ourGoal().x()+0.2)
						desiredPosition.setPosition(loc()->ourGoal().x()+0.2, desiredPosition.y()+0.1, 0.0);
				}
			} else {
				if(_side.isRight()) {
					if(desiredPosition.x() < loc()->ourGoal().x()+0.2)
						desiredPosition.setPosition(loc()->ourGoal().x()+0.2, desiredPosition.y()-0.1, 0.0);
				} else {
					if(desiredPosition.x() < loc()->ourGoal().x()+0.45)
						desiredPosition.setPosition(loc()->ourGoal().x()+0.45, desiredPosition.y(), 0.0);
				}
			}
		}
	}


    // Position to look
    Position posToLook = WR::Utils::threePoints(loc()->ourGoal(), loc()->ball(), GlobalConstants::highDistance(), 0.0);

    // Configure skill
    _sk_goTo->setDestination(desiredPosition);
    _sk_goTo->setPositionToLook(posToLook);
    _sk_goTo->avoidAll(player()->distanceTo(desiredPosition) > _radius);
    _sk_goTo->avoidGoalArea(true);


    // State INTERCEPT
    _sk_intBall->setAdvance(true);
    _sk_intBall->setPositionToLook(loc()->theirGoal());


    // State KICKOUT
    _sk_kickout->setPosToKick(loc()->theirGoal());


    // Transitions
    if(player()->distBall()>INTERCEPT_MINBALLDIST && utils()->isBallComingToGoal(INTERCEPT_MINBALLVELOCITY)) {
        enableTransition(SKT_INTERCEPT);
    } else if(_takeoutEnabled) {
        if(loc()->isInsideOurArea(loc()->ball(), _takeoutFactor)) {
            enableTransition(SKT_KICKOUT);
        } else if(loc()->isInsideOurArea(loc()->ball(), 1.10*_takeoutFactor)==false) { // hysteresis (10cm) applied on state exit to avoid oscillation
            enableTransition(SKT_POS);
        }
    } else {
        enableTransition(SKT_POS);
    }

}
