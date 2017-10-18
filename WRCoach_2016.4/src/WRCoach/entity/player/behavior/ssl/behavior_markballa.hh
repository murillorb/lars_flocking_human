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

#ifndef BEHAVIOR_MARKBALLA_HH
#define BEHAVIOR_MARKBALLA_HH

#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/utils/fieldside/fieldside.hh>

class Behavior_MarkBallA: public Behavior {
private:
    // Parameters
	float _distToMark;
    float _d;
    FieldSide _side;
    bool _canGoToOpTeamFieldSide;

	// Skills
	Skill_GoTo *_sk_goTo;

	void configure();
	void run();
public:
    Behavior_MarkBallA();
	QString name();

    void setD(float d = 0.25f) { _d = d; }
    void setDistToMark(float distToMark = 0.90f) { _distToMark = distToMark; }
    void setSide(FieldSide side = Sides::UNDEFINED) { _side = side; }
    void setCanGoToOpTeamFieldSide(bool canGoToOpTeamFieldSide = true) { _canGoToOpTeamFieldSide = canGoToOpTeamFieldSide; }
};

#endif // BEHAVIOR_MARKBALLA_HH
