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

#include "playbook_followball.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

QString Playbook_FollowBall::name() {
    return "Playbook_FollowBall";
}

Playbook_FollowBall::Playbook_FollowBall() {
    setBallDistance(0.6);
}

int Playbook_FollowBall::maxNumPlayer() {
    return INT_MAX;
}

void Playbook_FollowBall::configure(int numPlayers) {
    _bh_fb.clear();
    for(quint8 i=0; i<numPlayers; i++) {
        Behavior_FollowBall *bh_fb = new Behavior_FollowBall();
        usesBehavior(bh_fb);
        _bh_fb.push_back(bh_fb);
    }
}

void Playbook_FollowBall::run(int numPlayers) {
    for(int i=0; i<numPlayers; i++) {
        // Set behaviors
        Behavior_FollowBall *bh_fb = _bh_fb.at(i);
        setPlayerBehavior(dist()->getPlayer(), bh_fb);

        // Set ball distance to follow ball
        bh_fb->setBallDistance(_distBall);

    }
}
