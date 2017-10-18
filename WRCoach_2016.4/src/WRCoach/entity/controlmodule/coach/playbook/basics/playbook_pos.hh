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

#ifndef PLAYBOOK_POS_HH
#define PLAYBOOK_POS_HH

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>

class Playbook_Pos : public Playbook {
private:
    // Parameters
    Position _position;
    float _dist;
    bool _ourSide; // Informs if we want to be positioned at our or their field side

    // Behaviors
    QList<Behavior_Pos*> _bh_pos;

    void configure(int numPlayers);
    void run(int numPlayers);
public:
    Playbook_Pos();
    QString name();
    int maxNumPlayer();

    void setPosition(const Position &position) { _position = position; }
    void setDist(float dist) { _dist = dist; }
    void setOurSide(bool ourSide) { _ourSide = ourSide; }
};

#endif // PLAYBOOK_FOLLOWBALL_H
