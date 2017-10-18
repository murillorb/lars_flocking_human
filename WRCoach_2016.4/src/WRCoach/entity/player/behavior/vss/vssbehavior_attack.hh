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

#ifndef VSSBEHAVIOR_ATTACK_HH
#define VSSBEHAVIOR_ATTACK_HH

#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/utils/wrtimer/wrtimer.hh>


class VSSBehavior_Attack: public Behavior {
private:
	// Flags
	bool _avoidDefenseArea;

	// Skills
	VSSSkill_PushBall *_sk_pushBall;
    Skill_GoTo *_sk_goTo;
	Skill_Spin *_sk_spin;

    // Skills transition
    enum {SKT_GOTO, SKT_PUSH, SKT_SPIN};

    // State machine
    int _state;
    enum {STATE_ATTACK, STATE_WAITAREA, STATE_WALL};

    // Sub state machine
    int _subState;
    enum {WALL_POS, WALL_SPIN};

	// Internal
    bool _canGoToBall;
    WRTimer _spinTimer;

	void configure();
	void run();
public:
	VSSBehavior_Attack();
	QString name();

    void setAvoidDefenseArea(bool avoid) { _avoidDefenseArea = avoid; }
};

#endif // VSSBEHAVIOR_ATTACK_HH

