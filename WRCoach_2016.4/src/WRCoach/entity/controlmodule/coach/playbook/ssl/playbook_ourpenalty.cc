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

#include "playbook_ourpenalty.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

QString Playbook_OurPenalty::name() {
	return "Playbook_OurPenalty";
}

Playbook_OurPenalty::Playbook_OurPenalty() {
	_bh_pk = NULL;
    _bh_paR = NULL;
    _bh_paL = NULL;
}

int Playbook_OurPenalty::maxNumPlayer() {
    return 3;
}

void Playbook_OurPenalty::configure(int numPlayers) {
    // Penalty kicker
	usesBehavior(_bh_pk = new Behavior_PenaltyKick());

    // Assistant R
	if(numPlayers >= 2) {
        usesBehavior(_bh_paR = new Behavior_PenaltyAssistant());
        _bh_paR->setOurPenalty(true);
        _bh_paR->setSide(Sides::RIGHT);
	}

    // Assistant L
    if(numPlayers >= 3) {
        usesBehavior(_bh_paL = new Behavior_PenaltyAssistant());
        _bh_paL->setOurPenalty(true);
        _bh_paL->setSide(Sides::LEFT);
	}
}

void Playbook_OurPenalty::run(int numPlayers) {

    // Penalty kicker (1)
    quint8 pk = dist()->getOneKNN(loc()->theirGoal());
    setPlayerBehavior(pk, _bh_pk);

    // Assistant Uni (1)
    if(numPlayers==2) {
        quint8 assistant = dist()->getPlayer();
        setPlayerBehavior(assistant, _bh_paR);
    }

    // Assistants Duo (or 2)
    if(numPlayers==3) {
        QList<quint8> knn = dist()->getKNNandY(2, loc()->theirGoal());
		if(loc()->ourSide().isRight()) {
            setPlayerBehavior(knn.takeFirst(), _bh_paL);
            setPlayerBehavior(knn.takeFirst(), _bh_paR);
		} else {
            setPlayerBehavior(knn.takeFirst(), _bh_paR);
            setPlayerBehavior(knn.takeFirst(), _bh_paL);
		}
	}
}
