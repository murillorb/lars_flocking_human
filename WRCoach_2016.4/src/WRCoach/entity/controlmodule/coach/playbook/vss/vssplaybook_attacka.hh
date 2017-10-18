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

#ifndef VSSPLAYBOOK_ATTACKA_H
#define VSSPLAYBOOK_ATTACKA_H

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>

class VSSPlaybook_AttackA : public Playbook {
private:
    // Parameters
    bool _attackerAvoidDefenseArea;

    // Behaviors
    VSSBehavior_Attack *_bh_attack;
    VSSBehavior_AttackA *_bh_attackA;
    VSSBehavior_Defense *_bh_defense;

    // Hysteresis
    int _state;
    enum {STATE_DEFENSE, STATE_ATTACKA};

    void configure(int numPlayers);
    void run(int numPlayers);
public:
    VSSPlaybook_AttackA();
    QString name();

    int maxNumPlayer();

    void setAttackerAvoidDefenseArea(bool avoid) { _attackerAvoidDefenseArea = avoid; }
};

#endif // VSSPLAYBOOK_ATTACKA_H
