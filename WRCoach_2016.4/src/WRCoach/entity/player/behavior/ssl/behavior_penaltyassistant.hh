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

#ifndef BEHAVIOR_PENALTYASSISTANT_HH
#define BEHAVIOR_PENALTYASSISTANT_HH

#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/utils/fieldside/fieldside.hh>

class Behavior_PenaltyAssistant : public Behavior {
private:
	// Parameters
    FieldSide _side;
    bool _ourPenalty;
    float _radius;
	float _angle;

	// Skills
	Skill_GoTo *_sk_goTo;

	void configure();
	void run();
public:
	Behavior_PenaltyAssistant();
	QString name();

    void setSide(FieldSide side) { _side = side; }
    void setOurPenalty(bool ourPenalty) { _ourPenalty = ourPenalty; }
    void setRadius(float radius) { _radius = radius; }
    void setAngle(float angle) { _angle = angle; }
};

#endif // BEHAVIOR_PENALTYASSISTANT_HH

