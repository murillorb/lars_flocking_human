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

#include "vssbehavior_gk.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString VSSBehavior_GK::name() {
	return "VSSBehavior_GK";
}

VSSBehavior_GK::VSSBehavior_GK() {
    // Default parameters
    setDistGoal();

    // Initialization
	_sk_goTo = NULL;
    _sk_takeout = NULL;

	_state = STATE_GOTO;
}

void VSSBehavior_GK::configure() {
	usesSkill(_sk_goTo = new Skill_GoTo());
    usesSkill(_sk_takeout = new VSSSkill_Takeout());

	setInitialSkill(_sk_goTo);

    addTransition(SKT_GOTO, _sk_takeout, _sk_goTo);
    addTransition(SKT_TAKEOUT, _sk_goTo, _sk_takeout);
}

void VSSBehavior_GK::run() {
	// Set GK specific Linear PID calibration
    player()->setLinearPIDParameters(4.0, 0.0, 0.2, 0.0);
    player()->setMaxLSpeed(0.8);
    player()->setMaxLAcceleration(1.15);

    // Disable wall stuck check
    player()->setFieldWallStuckEnabled(false);

    // Switch state
	switch(_state) {
        default:
		case STATE_GOTO: {
            enableTransition(SKT_GOTO);

			// Re-enable robot stuck out of takeout
			player()->setRobotStuckEnabled(true);

            // Calc desired position
            Position desiredPos;
            if(utils()->isBallComingToGoal(BALL_MINVELOCITY, 1.75)) { // intercept ball
                // Calc ball impact pos, considering robot squared size
                float signal = loc()->ourSide().isRight()? -1 : 1;
                desiredPos = utils()->getBallImpactPosition(loc()->ballVelocity().arg(), loc()->ourGoal().x()+signal*(_distGoal+2*player()->robotRadius()));
                desiredPos.setPosition(desiredPos.x()-signal*player()->robotRadius(), desiredPos.y(), 0.0);

				// Limit to goal posts
				if(fabs(desiredPos.y()) > fabs(loc()->ourGoalRightPost().y())) {
					float signal = desiredPos.y()/fabs(desiredPos.y());
					desiredPos.setPosition(desiredPos.x(), signal*fabs(loc()->ourGoalRightPost().y()), 0.0);
				}

            } else { // just positioning
                desiredPos = getFollowBallPos();
            }

            Position lookPos(true, desiredPos.x(), loc()->fieldMaxY(), 0.0);

            // GOTO
            _sk_goTo->setDestination(desiredPos);
            _sk_goTo->setPositionToLook(lookPos);
            _sk_goTo->avoidRobots(player()->distOurGoal() > loc()->fieldDefenseWidth()/2);
            _sk_goTo->avoidBall(utils()->isBehindBall(loc()->ourGoal()));
            _sk_goTo->limitFieldDimensions(false);

            // Switch state condition: ball in our area => short takeout
            if(loc()->isInsideOurArea(loc()->ball(), TAKEOUT_FACTOR_IN) && loc()->isInsideOurArea(player()->position(), TAKEOUT_FACTOR_IN))
                _state = STATE_SHORTTAKEOUT;

		} break;

        case STATE_SHORTTAKEOUT: {
            enableTransition(SKT_TAKEOUT);

			// Disable robot stuck at takeout
			player()->setRobotStuckEnabled(false);

            // Switch state condition: ball out of our area
            if(loc()->isInsideOurArea(loc()->ball(), TAKEOUT_FACTOR_OUT)==false) // hysteresis applied to avoid oscilation
                _state = STATE_GOTO;

		} break;

	}
}

Position VSSBehavior_GK::getFollowBallPos() {
	// X
    float signal = loc()->ourSide().isRight()? -1 : 1;
    float posX = loc()->ourGoal().x() + signal*(_distGoal+player()->robotRadius());

	// Y
	float posY;
	if(loc()->isInsideTheirField(loc()->ball()))
		posY = 0.0;
    else if((loc()->ourSide().isRight() && loc()->ball().x()<=loc()->fieldMaxX()*3/4) || (loc()->ourSide().isLeft() && loc()->ball().x()>=loc()->fieldMinX()*3/4)) {
		posY = (fabs(loc()->ourGoalRightPost().y())/fabs(loc()->fieldMaxY()))*loc()->ball().y();
	} else {
		if(loc()->ball().y() > fabs(loc()->ourGoalRightPost().y()))
            posY = fabs(loc()->ourGoalRightPost().y()) - player()->robotRadius()/2;
        else if(loc()->ball().y() < -fabs(loc()->ourGoalRightPost().y()))
            posY = -fabs(loc()->ourGoalRightPost().y()) + player()->robotRadius()/2;
		else
			posY = loc()->ball().y();
	}

	return Position(true, posX, posY, 0.0);
}
