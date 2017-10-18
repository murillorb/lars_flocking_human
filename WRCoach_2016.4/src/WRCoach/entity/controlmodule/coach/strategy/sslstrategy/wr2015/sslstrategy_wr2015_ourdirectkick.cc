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

#include "sslstrategy_wr2015_ourdirectkick.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_OurDirectKick::name() {
	return "SSLStrategy_WR2015_OurDirectKick";
}

SSLStrategy_WR2015_OurDirectKick::SSLStrategy_WR2015_OurDirectKick() {
	_pb_atk = NULL;
	_pb_goal = NULL;
    _pb_defense = NULL;
}

void SSLStrategy_WR2015_OurDirectKick::configure(int numOurPlayers) {
    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_atk = new Playbook_Attack());
    usesPlaybook(_pb_defense = new Playbook_Defense());
}

void SSLStrategy_WR2015_OurDirectKick::run(int numOurPlayers) {
    // ADD PLAYERS
    const quint8 gk = dist()->getGK();

    // Attack (1, Kicker)
    quint8 kicker = (numOurPlayers==1)? gk : dist()->getKicker(loc()->ball());
    _pb_atk->setKicker(kicker);
    _pb_atk->addPlayer(kicker);

    // Goal (1, GK)
    if(numOurPlayers >= 2) {
        _pb_goal->setGK(gk);
        _pb_goal->addPlayer(gk);
    }

    // Attack (1, Recv)
    quint8 recv = dist()->getOneKNN(loc()->theirGoal());
    _pb_atk->addPlayer(recv);

    // Defense (1, MB)
    quint8 mb = dist()->getOneKNN(loc()->ball());
    _pb_defense->addPlayer(mb);
    _pb_defense->setAdvanceToBall(false);

    // Attack (1, Recv)
    quint8 recv2 = dist()->getOneKNN(loc()->theirGoal());
    _pb_atk->addPlayer(recv2);

    // Goal (1, GKA)
    quint8 gka = dist()->getOneKNN(loc()->ourGoal());
    _pb_goal->addPlayer(gka);

}
