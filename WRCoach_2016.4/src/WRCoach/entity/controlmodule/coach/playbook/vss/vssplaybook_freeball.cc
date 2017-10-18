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

#include "vssplaybook_freeball.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSPlaybook_FreeBall::name() {
	return "VSSPlaybook_FreeBall";
}

VSSPlaybook_FreeBall::VSSPlaybook_FreeBall() {
    _bh_freeBall = NULL;
}

int VSSPlaybook_FreeBall::maxNumPlayer() {
    return 1;
}

void VSSPlaybook_FreeBall::configure(int numPlayers) {
    usesBehavior(_bh_freeBall = new VSSBehavior_FreeBall());
}

void VSSPlaybook_FreeBall::run(int numPlayers) {
    // FreeBall player
    quint8 player = dist()->getPlayer();
    setPlayerBehavior(player, _bh_freeBall);
}
