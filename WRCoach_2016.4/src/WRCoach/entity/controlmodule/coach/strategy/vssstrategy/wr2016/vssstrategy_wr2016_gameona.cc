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

#include "vssstrategy_wr2016_gameona.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSStrategy_WR2016_GameOnA::name() {
    return "VSSStrategy_WR2016_GameOnA";
}

VSSStrategy_WR2016_GameOnA::VSSStrategy_WR2016_GameOnA() {
    _pb_attackA = NULL;
    _pb_defense = NULL;
    _pb_goal = NULL;

}


void VSSStrategy_WR2016_GameOnA::configure(int numOurPlayers) {
    usesPlaybook(_pb_attackA = new VSSPlaybook_AttackA());
    usesPlaybook(_pb_goal = new VSSPlaybook_Goal());
    usesPlaybook(_pb_defense = new VSSPlaybook_Defense());
}

void VSSStrategy_WR2016_GameOnA::run(int numOurPlayers) {
    // Solo (GK-attacker)
    if(numOurPlayers == 1) {
        quint8 uniPlayer = dist()->getPlayer();
        Position posUniPlayer = PlayerBus::ourPlayer(uniPlayer)->position();

        if(loc()->isInsideOurArea(posUniPlayer) && loc()->isInsideOurArea(loc()->ball())) {
            _pb_goal->addPlayer(uniPlayer);
        } else {
            _pb_attackA->addPlayer(uniPlayer);
            _pb_attackA->setAttackerAvoidDefenseArea(false);
        }

    } else if(numOurPlayers >= 2) {
        // GK
        quint8 gk = dist()->getOneKNN(loc()->ourGoal());
        _pb_goal->addPlayer(gk);

        // Attacker
        quint8 attacker = dist()->getOneKNN(loc()->ball());
        _pb_attackA->addPlayer(attacker);
        _pb_attackA->setAttackerAvoidDefenseArea(true);

        // AttackerA or Defense
        quint8 thirdPlayer = dist()->getOneKNN(loc()->ball());
        _pb_attackA->addPlayer(thirdPlayer);

    }
}
