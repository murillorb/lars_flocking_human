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

#include "sslstrategy_wr2015_ourpenalty.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_OurPenalty::name() {
    return "SSLStrategy_WR2015_OurPenalty";
}

SSLStrategy_WR2015_OurPenalty::SSLStrategy_WR2015_OurPenalty() {
	_pb_goal = NULL;
	_pb_ourpenalty = NULL;
}

void SSLStrategy_WR2015_OurPenalty::configure(int numOurPlayers) {
	usesPlaybook(_pb_ourpenalty = new Playbook_OurPenalty());
    usesPlaybook(_pb_goal = new Playbook_Goal());
}

void SSLStrategy_WR2015_OurPenalty::run(int numOurPlayers) {
    quint8 gk = dist()->getGK();

    // Kicker (1)
    quint8 kicker = numOurPlayers==1? gk : dist()->getKicker(loc()->ball());
    _pb_ourpenalty->addPlayer(kicker);

    // GK (1)
    if(numOurPlayers >= 2) {
        _pb_goal->setGK(gk);
        _pb_goal->addPlayer(gk);
    }

    // Penalty assistants (2)
    QList<quint8> assistants = dist()->getKNN(2, loc()->theirGoal());
    _pb_ourpenalty->addPlayers(assistants);

    // GKAs (2)
    QList<quint8> gkas = dist()->getKNN(2, loc()->ourGoal());
    _pb_goal->addPlayers(gkas);
}
