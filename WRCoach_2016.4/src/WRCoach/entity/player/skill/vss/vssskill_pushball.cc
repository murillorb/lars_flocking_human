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

#include "vssskill_pushball.hh"
#include <WRCoach/entity/player/skill/wrskill.hh>

QString VSSSkill_PushBall::name() {
	return "VSSSkill_PushBall";
}

VSSSkill_PushBall::VSSSkill_PushBall() {
    _destination.setUnknown();
    setDistBall(0.08);

    _state = STATE_POS;
}

void VSSSkill_PushBall::run() {
    if(_destination.isUnknown()) {
        std::cout << "[WARNING] " << name().toStdString() << ", destination is unknown!\n";
        player()->idle();
        return;
    }

    bool avoidRobots = (player()->distBall() > 0.30);

    switch(_state) {
        default:
        case STATE_POS: {

            // Get ball velocity and unitary vector
            Velocity velBall = loc()->ballVelocity();
            Velocity velBallUni(true, (velBall.abs()==0.0? 0.0 : velBall.x()/velBall.abs()), (velBall.abs()==0.0? 0.0 : velBall.y()/velBall.abs()));

            // Calc const to multiply velocity unitary vector
            float k = velBall.abs()>0.015? velBall.abs() : 0.0;
            WR::Utils::limitValue(&k, 0.0, 0.15);

            // Calc projected position
            Position desiredPos = WR::Utils::threePoints(loc()->ball(), loc()->theirGoal(), _distBall, GEARSystem::Angle::pi);

            float factorX = 1;
            float factorY = 1;

            if(fabs(velBall.y())>0.10) {
                factorX = 1.3;
                factorY = 1.3;
            }

            if(loc()->ourSide().isRight() && velBall.x()<0)
                factorX = 0;
            if(loc()->ourSide().isLeft() && velBall.x()>0)
                factorX = 0;


            desiredPos.setPosition(desiredPos.x()+factorX*k*velBallUni.x(), desiredPos.y()+factorY*k*velBallUni.y(), 0.0);

            // goTo
            player()->goTo(desiredPos, utils()->isBehindBall(desiredPos), avoidRobots, avoidRobots);

            // Switch state condition: positioned to push
            float distLine = WR::Utils::distanceToLine(desiredPos, loc()->theirGoal(), player()->position());
            if(distLine < 1.10*player()->lError() && utils()->isBehindBall(_destination)) {
                player()->idle();
                _state = STATE_WAIT;
                _resetTimer.start();
            }

        } break;

        case STATE_WAIT: {

            // Look to
            if(player()->isLookingTo(_destination)==false)
                player()->lookTo(_destination);

            // Switch state condition: ball in path
            float distLine = WR::Utils::distanceToLine(player()->position(), loc()->theirGoal(), loc()->ball());
            if(distLine < 0.05) {
                player()->idle();
                _state = STATE_PUSH;
                return;
            }

            // Switch state condition: timer
            _resetTimer.stop();
            if(_resetTimer.timesec()>=0.75)
                _state = STATE_PUSH;

        } break;

        case STATE_PUSH: {
            // Set Attacker linear PID calibration

            player()->setLinearPIDParameters(5, 0.0, 0.15, 0.0);
            player()->setMaxLAcceleration(0.8);
            player()->setMaxLSpeed(1.20);

            // Go to
            player()->goTo(_destination, false, false, false, false);

            // Reset state condition: position to push ball
            float distLine = WR::Utils::distanceToLine(loc()->ball(), _destination, player()->position());
            if(utils()->isBehindBall(_destination)==false || distLine >= 1.5f*player()->robotRadius()) {
                player()->idle();
                _state = STATE_POS;
            }

        } break;

    }

}

