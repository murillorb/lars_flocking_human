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

#include "sslstrategy_wr2015_theirpenalty.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_TheirPenalty::name() {
    return "SSLStrategy_WR2015_TheirPenalty";
}

SSLStrategy_WR2015_TheirPenalty::SSLStrategy_WR2015_TheirPenalty() {
    _pb_theirPenalty = NULL;
    _pb_pos = NULL;
}

void SSLStrategy_WR2015_TheirPenalty::configure(int numOurPlayers) {
    usesPlaybook(_pb_theirPenalty = new Playbook_TheirPenalty());
    usesPlaybook(_pb_pos = new Playbook_Pos());
}

void SSLStrategy_WR2015_TheirPenalty::run(int numOurPlayers) {
    // GK (1)
    quint8 gk = dist()->getGK();
    _pb_theirPenalty->addPlayer(gk);

    // Rest (5)
    QList<quint8> assistants = dist()->getAllPlayers();
    _pb_pos->addPlayers(assistants);
    _pb_pos->setPosition(loc()->ball());
    _pb_pos->setDist(1.2f);
    _pb_pos->setOurSide(false);
}

