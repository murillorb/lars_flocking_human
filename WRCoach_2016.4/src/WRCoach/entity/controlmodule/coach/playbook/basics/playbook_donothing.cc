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

#include "playbook_donothing.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

QString Playbook_DoNothing::name() {
	return "Playbook_DoNothing";
}

Playbook_DoNothing::Playbook_DoNothing() {

}

int Playbook_DoNothing::maxNumPlayer() {
    return INT_MAX;
}

void Playbook_DoNothing::configure(int numPlayers) {
    _bh_doNothing.clear();
    for(int i=0; i<numPlayers; i++) {
        Behavior_DoNothing *bh_doNothing = new Behavior_DoNothing();
        usesBehavior(bh_doNothing);
        _bh_doNothing.push_back(bh_doNothing);
    }
}

void Playbook_DoNothing::run(int numPlayers) {
    for(int i=0; i<numPlayers; i++)
        setPlayerBehavior(dist()->getPlayer(), _bh_doNothing.at(i));
}
