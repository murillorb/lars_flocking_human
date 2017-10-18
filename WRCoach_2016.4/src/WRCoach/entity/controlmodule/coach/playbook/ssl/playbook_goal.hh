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

#ifndef PLAYBOOK_GOAL_HH
#define PLAYBOOK_GOAL_HH

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>

class Playbook_Goal : public Playbook {
private:
    // Parameters
    qint8 _gk;

    // Behaviors
    Behavior_GK *_bh_gk;
    Behavior_GKA *_bh_gka_uni;
    Behavior_GKA *_bh_gka_multiR, *_bh_gka_multiL, *_bh_gka_multiC;

    // State machine
    quint8 _state;
    enum {STATE_POS, STATE_TAKEOUT};

    // Internal
    quint8 _gka_uni, _gka_multiR, _gka_multiL, _gka_multiC;

    void configure(int numPlayers);
    void run(int numPlayers);
public:
    Playbook_Goal();
    QString name();
    int maxNumPlayer();

    void setGK(qint8 gk) { _gk = gk; }
};

#endif // PLAYBOOK_GOAL_HH
