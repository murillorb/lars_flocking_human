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

#ifndef VSSSKILL_PUSHBALL_HH
#define VSSSKILL_PUSHBALL_HH

#include <WRCoach/entity/player/skill/skill.hh>
#include <WRCoach/utils/wrtimer/wrtimer.hh>

class VSSSkill_PushBall : public Skill {
private:
	// Parameters
    float _distBall;
    Position _destination;

    // State machine
    int _state;
    enum {STATE_POS, STATE_WAIT, STATE_PUSH};

    // Internal
    WRTimer _resetTimer;

	void run();
public:
    VSSSkill_PushBall();
    QString name();

    void setDistBall(float distBall) { _distBall = distBall; }
    void setDestination(const Position &destination) { _destination = destination; }
};

#endif // VSSSKILL_PUSHBALL_HH

