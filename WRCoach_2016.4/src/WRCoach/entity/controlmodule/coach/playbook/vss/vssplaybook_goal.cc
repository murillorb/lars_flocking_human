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

#include "vssplaybook_goal.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSPlaybook_Goal::name() {
    return "VSSPlaybook_Goal";
}

VSSPlaybook_Goal::VSSPlaybook_Goal() {
	_bh_gk = NULL;
    _bh_gka = NULL;
}

int VSSPlaybook_Goal::maxNumPlayer() {
    return 2;
}

void VSSPlaybook_Goal::configure(int numPlayers) {
    usesBehavior(_bh_gk = new VSSBehavior_GK());
    usesBehavior(_bh_gka = new VSSBehavior_GKA());
}

void VSSPlaybook_Goal::run(int numPlayers) {
    // GK (1)
    quint8 gk = dist()->getOneKNN(loc()->ourGoal());
    setPlayerBehavior(gk, _bh_gk);

    // GKA (1)
    quint8 gka = dist()->getOneKNN(loc()->ourGoal());
    setPlayerBehavior(gka, _bh_gka);
}
