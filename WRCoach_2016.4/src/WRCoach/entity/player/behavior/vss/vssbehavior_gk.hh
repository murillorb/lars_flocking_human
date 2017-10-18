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

#ifndef VSSBEHAVIOR_GK_H
#define VSSBEHAVIOR_GK_H

#include <WRCoach/entity/player/behavior/behavior.hh>

#define TAKEOUT_FACTOR_IN 1.15
#define TAKEOUT_FACTOR_OUT 1.30
#define BALL_MINVELOCITY 0.05

class VSSBehavior_GK: public Behavior {
private:
	// Parameters
    float _distGoal;

	// Skills
	Skill_GoTo *_sk_goTo;
    VSSSkill_Takeout *_sk_takeout;

    // Skills Transitions
    enum{SKT_GOTO, SKT_TAKEOUT};

	// State
	quint8 _state;
    enum {STATE_GOTO, STATE_SHORTTAKEOUT, STATE_LONGTAKEOUT};

    // Internal
    Position getFollowBallPos();
    bool canPerformLongTakeout();

    void configure();
    void run();
public:
    VSSBehavior_GK();
    QString name();

    void setDistGoal(float distGoal = 0.01) { _distGoal = distGoal; }
};

#endif // VSSBEHAVIOR_GK_H
