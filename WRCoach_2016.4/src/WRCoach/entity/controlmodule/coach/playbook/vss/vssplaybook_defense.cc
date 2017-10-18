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

#include "vssplaybook_defense.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSPlaybook_Defense::name() {
	return "VSSPlaybook_Defense";
}

VSSPlaybook_Defense::VSSPlaybook_Defense() {
	_bh_defense = NULL;
	_bh_mark = NULL;
}

int VSSPlaybook_Defense::maxNumPlayer() {
    return 1;
}

void VSSPlaybook_Defense::configure(int numPlayers) {
	usesBehavior(_bh_defense = new VSSBehavior_Defense());
	usesBehavior(_bh_mark = new VSSBehavior_Mark());
}

void VSSPlaybook_Defense::run(int numPlayers) {
	// Set behavior
    qint8 markId = -1;
    /// Descomentar para ativar o marcador.
    QList<kNN::element> knn = utils()->getOpponentKNN(3, loc()->ball());
    for(int i=0; i<knn.size(); i++) {
        quint8 opId = knn.at(i).id;

        // Check distance to ball
        if(PlayerBus::theirPlayer(opId)->distBall() < 0.30)
            continue;

        // Check if is GK
        if(loc()->isInsideTheirArea(PlayerBus::theirPlayer(opId)->position()))
            continue;

        // Select
        markId = opId;
        break;
    }

    // Check marker or defender
    if(markId == -1) { // no players to mark => defender
        quint8 defender = dist()->getPlayer();
		setPlayerBehavior(defender, _bh_defense);
    } else { // players to mark => marker
        quint8 marker = dist()->getPlayer();
		setPlayerBehavior(marker, _bh_mark);
		_bh_mark->setMarkId(markId);
	}

}
