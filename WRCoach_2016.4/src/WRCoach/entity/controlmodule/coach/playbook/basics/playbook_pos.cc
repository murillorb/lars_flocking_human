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

#include "playbook_pos.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

QString Playbook_Pos::name() {
    return "Playbook_Pos";
}

Playbook_Pos::Playbook_Pos() {
    _position.setUnknown();
    setDist(0.6);
    setOurSide(true);
}

int Playbook_Pos::maxNumPlayer() {
    return 6;
}

void Playbook_Pos::configure(int numPlayers) {
    _bh_pos.clear();
    for(int i=0; i<numPlayers; i++) {
        Behavior_Pos *bh_pos = new Behavior_Pos();
        usesBehavior(bh_pos);
        _bh_pos.push_back(bh_pos);
    }
}

void Playbook_Pos::run(int numPlayers) {
    if(_position.isUnknown())
        std::cout << "[WARNING] Playbook_Pos: _position is unknown!\n";

    const Position projected(true, (_ourSide)?loc()->ourGoal().x():loc()->theirGoal().x(), _position.y(), 0.0);

    // Invert position or not
	// Will only invert if positioning side is the right field side
    bool invert = false;
    if(_ourSide && loc()->ourSide().isLeft())
        invert = true;
    if(_ourSide==false && loc()->ourSide().isRight())
        invert = true;

    // Positions
	// L1 => front line
	// L2 => intermediate line
	// L3 => back line
	const Position posL1_1 = WR::Utils::threePoints(_position, projected, _dist, GEARSystem::Angle::toRadians(invert ? 30 : -30));
    const Position posL1_2 = WR::Utils::threePoints(_position, projected, _dist, 0.0);
	const Position posL1_3 = WR::Utils::threePoints(_position, projected, _dist, GEARSystem::Angle::toRadians(invert ? -30 : 30));
	const Position posL2_1 = WR::Utils::threePoints(_position, projected, 1.5*_dist, -GEARSystem::Angle::toRadians(invert ? -15 : 15));
	const Position posL2_2 = WR::Utils::threePoints(_position, projected, 1.5*_dist, -GEARSystem::Angle::toRadians(invert ? 15 : -15));
    const Position posL3_1 = WR::Utils::threePoints(_position, projected, 1.75*_dist, 0.0);

    // Sort positions
    QList<Position> pos;
    switch(numPlayers) {
        case 1: {
            pos.push_back(posL1_2);
        } break;
        case 2: {
            pos.push_back(posL1_1);
            pos.push_back(posL1_3);
        } break;
        case 3: {
            pos.push_back(posL1_1);
            pos.push_back(posL1_2);
            pos.push_back(posL1_3);
        } break;
        case 4: {
            pos.push_back(posL1_1);
            pos.push_back(posL1_2);
            pos.push_back(posL1_3);
            pos.push_back(posL3_1);
        } break;
        case 5: {
            pos.push_back(posL1_1);
            pos.push_back(posL1_2);
            pos.push_back(posL1_3);
            pos.push_back(posL2_1);
            pos.push_back(posL2_2);
        } break;
        case 6: {
            pos.push_back(posL1_1);
            pos.push_back(posL1_2);
            pos.push_back(posL1_3);
            pos.push_back(posL2_1);
            pos.push_back(posL2_2);
            pos.push_back(posL3_1);
        } break;
    }

    // Sort players
    QList<quint8> player;
    switch(numPlayers) {
        case 1: {
            player.push_back(dist()->getPlayer());
        } break;
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:{
            QList<quint8> knn = dist()->getKNNandY(3, _position);
            while(knn.empty()==false)
                player.push_back(knn.takeFirst());

            knn = dist()->getKNNandY(2, _position);
            while(knn.empty()==false)
                player.push_back(knn.takeFirst());

            player.push_back(dist()->getPlayer());
        } break;
    }

    // Assign behaviors
    for(int i=0; i<numPlayers; i++) {
        Behavior_Pos *bh_pos = _bh_pos.at(i);
        setPlayerBehavior(player.at(i), bh_pos);

        bh_pos->setPosition(pos.takeFirst());
        bh_pos->setPositionToLook(_position);
		bh_pos->avoidAll(true);
    }
}
