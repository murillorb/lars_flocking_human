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

#ifndef BEHAVIOR_MARKPLAYER_HH
#define BEHAVIOR_MARKPLAYER_HH

#include <WRCoach/entity/player/behavior/behavior.hh>

class Behavior_MarkPlayer: public Behavior {
private:
    // Parameters
    qint8 _toMarkID;
    float _distToMark;
    bool _canGoToOpTeamFieldSide;
    bool _markBetweenBall;

    // Skills
    Skill_GoTo *_sk_goTo;

    void configure();
    void run();
public:
    Behavior_MarkPlayer();
    QString name();

    void setPlayerToMarkID(quint8 toMarkID) { _toMarkID = toMarkID; }
    void setDistToMark(float distToMark = 0.30f) { _distToMark = distToMark; }
    void setCanGoToOpTeamFieldSide(bool canGoToOpTeamFieldSide = true) { _canGoToOpTeamFieldSide = canGoToOpTeamFieldSide; }
    void setMarkBetweenBall(bool markBetweenBall = false) { _markBetweenBall = markBetweenBall; }
};

#endif // BEHAVIOR_MARKPLAYER_HH
