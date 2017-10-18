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

#include "sslstrategy_test_gameon.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_Test_GameOn::name() {
    return "SSLStrategy_Test_GameOn";
}

SSLStrategy_Test_GameOn::SSLStrategy_Test_GameOn() {
//    _pb_goal = NULL;
    _pb_attack = NULL;
//	_pb_pos = NULL;
//	_pb_fb = NULL;
//	_pb_def = NULL;
}

void SSLStrategy_Test_GameOn::configure(int numOurPlayers) {
    usesPlaybook(_pb_attack = new Playbook_Attack());
//	usesPlaybook(_pb_pos = new Playbook_Pos());
//	usesPlaybook(_pb_fb = new Playbook_FollowBall());
//    usesPlaybook(_pb_goal = new Playbook_Goal());
}

void SSLStrategy_Test_GameOn::run(int numOurPlayers) {
//    quint8 gk = getGK();
//    _pb_goal->setGK(gk);
//    _pb_goal->addPlayer(gk);
//    _pb_goal->addPlayers(getAllPlayers());

    quint8 kicker = dist()->getOneKNN(loc()->ball());
    _pb_attack->addPlayer(kicker);
    _pb_attack->setKicker(kicker);
    _pb_attack->addPlayers(dist()->getAllPlayers());
    _pb_attack->setForcePass(false);
//	_pb_def->addPlayer(getOneKNN(loc()->ball()));
}
