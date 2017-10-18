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

#include "playbook_theirpenalty.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString Playbook_TheirPenalty::name() {
	return "Playbook_TheirPenalty";
}

Playbook_TheirPenalty::Playbook_TheirPenalty() {
    _bh_penaltyGK = NULL;
}

int Playbook_TheirPenalty::maxNumPlayer() {
    return 1;
}

void Playbook_TheirPenalty::configure(int numPlayers) {
    usesBehavior(_bh_penaltyGK = new Behavior_PenaltyGK());
}

void Playbook_TheirPenalty::run(int numPlayers) {
    // Penalty GK
    setPlayerBehavior(dist()->getPlayer(), _bh_penaltyGK);
}

