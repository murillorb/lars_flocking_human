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

#ifndef BEHAVIOR_RECVA_H
#define BEHAVIOR_RECVA_H

#include <WRCoach/entity/player/behavior/behavior.hh>

class Behavior_FollowPlayer: public Behavior {
private:
    // Parameters
    float _radius;
    qint8 _playerID;
    qint8 _playerTeam;

    // Skills
    Skill_GoTo *_sk_goTo;

    void configure();
    void run();
public:
    Behavior_FollowPlayer();
    QString name();

    void setRadius(float radius) { _radius = radius; }
    void setPlayerID(qint8 playerID) { _playerID = playerID; }
    void setPlayerTeam(qint8 playerTeam) { _playerTeam = playerTeam; }
};

#endif // BEHAVIOR_ATTACK_RECVA_H
