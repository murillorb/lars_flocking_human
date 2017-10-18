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

#include "vssplaybook_attacka.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSPlaybook_AttackA::name() {
    return "VSSPlaybook_AttackA";
}

VSSPlaybook_AttackA::VSSPlaybook_AttackA()  {
    setAttackerAvoidDefenseArea(true);

    _bh_attack = NULL;
    _bh_attackA = NULL;
    _bh_defense = NULL;

    _state = STATE_DEFENSE;
}

int VSSPlaybook_AttackA::maxNumPlayer() {
    return 2;
}

void VSSPlaybook_AttackA::configure(int numPlayers) {
    usesBehavior(_bh_attackA = new VSSBehavior_AttackA());
    usesBehavior(_bh_attack = new VSSBehavior_Attack());
    usesBehavior(_bh_defense = new VSSBehavior_Defense());
}

void VSSPlaybook_AttackA::run(int numPlayers) {
    if(numPlayers == 1) {
        // Attacker
        quint8 attacker = dist()->getPlayer();
        setPlayerBehavior(attacker, _bh_attack);
        _bh_attack->setAvoidDefenseArea(_attackerAvoidDefenseArea);

    } else {
        quint8 firstPlayer = dist()->getOneKNN(loc()->ball());
        quint8 secondPlayer = dist()->getOneKNN(loc()->ball());

        // State machine
        switch(_state) {
            default:
            case STATE_DEFENSE: {
                // Attacker
                setPlayerBehavior(firstPlayer, _bh_attack);
                _bh_attack->setAvoidDefenseArea(_attackerAvoidDefenseArea);

                // Defense
                setPlayerBehavior(secondPlayer, _bh_defense);

                // Switch state condition: ball in attack field
                if((loc()->ourSide().isRight() && loc()->ball().x()<=-0.05) || (loc()->ourSide().isLeft() && loc()->ball().x()>=0.05))
                    _state = STATE_ATTACKA;

            } break;

            case STATE_ATTACKA: {
                bool ballComingToAttacker = PlayerBus::ourPlayer(firstPlayer)->utils()->isBallComing(0.10, 0.3);
                bool ballComingToAttackerA = PlayerBus::ourPlayer(secondPlayer)->utils()->isBallComing(0.10, 0.3);
                bool attackerAgoodPos = PlayerBus::ourPlayer(secondPlayer)->utils()->isBehindBall(loc()->theirGoal());
                if(ballComingToAttackerA && !ballComingToAttacker && attackerAgoodPos) {
                    // AttackerA
                    setPlayerBehavior(firstPlayer, _bh_attackA);

                    // Attacker
                    setPlayerBehavior(secondPlayer, _bh_attack);

                    _bh_attackA->setAttackerId(secondPlayer);
                } else {
                    // Attacker
                    setPlayerBehavior(firstPlayer, _bh_attack);

                    // AttackerA
                    setPlayerBehavior(secondPlayer, _bh_attackA);

                    _bh_attackA->setAttackerId(firstPlayer);
                }

                // Switch state condition: ball returned to defense field
                // 8 cm + 35 cm in X axis to put the robot in a smooth transition to GKA
                if((loc()->ourSide().isRight() && loc()->ball().x()>=0.10) || (loc()->ourSide().isLeft() && loc()->ball().x()<=-0.10))
                    _state = STATE_DEFENSE;
            }
        }

    }
}
