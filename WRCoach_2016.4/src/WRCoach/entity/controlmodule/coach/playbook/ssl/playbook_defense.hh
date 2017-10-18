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

#ifndef PLAYBOOK_DEFENSE_H
#define PLAYBOOK_DEFENSE_H

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>

class Playbook_Defense : public Playbook {
private:
    // Parameters
    bool _canGoToOpTeamFieldSide;
    bool _advanceToBall;
    bool _markBetweenBall;
    float _markDistance;
    float _disableMarkBall;

    // Behaviors
    Behavior_MarkBall *_bh_markBall;
    QList<Behavior_MarkBallA*> _bh_markBallA;
    QList<Behavior_MarkPlayer*> _bh_markPlayer;

    // Internal
    bool isMarkable(quint8 opPlayer);

    void configure(int numPlayers);
    void run(int numPlayers);
public:
    Playbook_Defense();
    QString name();

    int maxNumPlayer();

    void setCanGoToOpTeamFieldSide(bool canGoToAttackArea = true) { _canGoToOpTeamFieldSide = canGoToAttackArea; }
    void setAdvanceToBall(bool advanceToBall = false) { _advanceToBall = advanceToBall; }
    void setMarkBetweenBall(bool markBetweenBall = false) { _markBetweenBall = markBetweenBall; }
    void setMarkDistance(float distance = 0.60f) { _markDistance = distance; }
    void setDisableMarkBall(bool disable = false) { _disableMarkBall = disable; }
};

#endif // PLAYBOOK_DEFENSE_H
