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

#include "playbook_goal.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#define GK_RADIUS 0.40
#define GK_TAKEOUT_FACTOR 1.5

#define GKA_RADIUS 1.15
#define GKA_TAKEOUT_FACTOR 2.0
#define GKA_TAKEOUT_RADIUS_EXTENDED 1.10*GKA_TAKEOUT_FACTOR

#define GKA_BALLDIST_REDUCE_D 2.5
#define GKA_D 0.17
#define GKA_D_REDUCED 0.12
#define GKA_D_EXTENDED 2.0*GKA_D

QString Playbook_Goal::name() {
    return "Playbook_Goal";
}

Playbook_Goal::Playbook_Goal() {
    setGK(-1);

    _bh_gk = NULL;
    _bh_gka_uni = NULL;
    _bh_gka_multiR = NULL;
    _bh_gka_multiL = NULL;
    _bh_gka_multiC = NULL;

    _state = STATE_POS;
    _gka_uni = 0;
    _gka_multiR = 0;
    _gka_multiL = 0;
    _gka_multiC = 0;
}

int Playbook_Goal::maxNumPlayer() {
    return 4;
}

void Playbook_Goal::configure(int numPlayers) {

    // GK
    usesBehavior(_bh_gk = new Behavior_GK());

    // GK + GKA
    if(numPlayers==2) {
        usesBehavior(_bh_gka_uni = new Behavior_GKA());
        _bh_gka_uni->setSide(Sides::UNDEFINED);
    }

    // GK + 2 GKA
    if(numPlayers>=3) {
        usesBehavior(_bh_gka_multiR = new Behavior_GKA());
        _bh_gka_multiR->setSide(Sides::RIGHT);
        usesBehavior(_bh_gka_multiL = new Behavior_GKA());
        _bh_gka_multiL->setSide(Sides::LEFT);
    }

    // GK + 3 GKA
    if(numPlayers>=4) {
        usesBehavior(_bh_gka_multiC = new Behavior_GKA());
        _bh_gka_multiC->setSide(Sides::CENTER);
    }
}

void Playbook_Goal::run(int numPlayers) {
    if(_gk == -1) {
        std::cout << "[WARNING] Playbook_Goal: GK not set, returning!\n";
        return;
    }

    // Set behaviors
    // GK (1)
    setPlayerBehavior(_gk, _bh_gk);
    dist()->removePlayer(_gk);


    // GKA Uni (1)
    if(numPlayers==2) {
        _gka_uni = dist()->getOneKNN(loc()->ourGoal());
        setPlayerBehavior(_gka_uni, _bh_gka_uni);
    }

    // GKA Multi (2)
    if(numPlayers==3) {
        QList<quint8> knn = dist()->getKNNandY(2, loc()->ourGoal());
        if(loc()->ourSide().isRight()) {
            _gka_multiL = knn.takeFirst();
            _gka_multiR = knn.takeFirst();
        } else {
            _gka_multiR = knn.takeFirst();
            _gka_multiL = knn.takeFirst();
        }

        // GKAs
        setPlayerBehavior(_gka_multiL, _bh_gka_multiL);
        setPlayerBehavior(_gka_multiR, _bh_gka_multiR);
    }

    // GKA Multi (3)
    if(numPlayers>=4) {
        QList<quint8> knn = dist()->getKNNandY(3, loc()->ourGoal());
        if(loc()->ourSide().isRight()) {
            _gka_multiL = knn.takeFirst();
            _gka_multiC = knn.takeFirst();
            _gka_multiR = knn.takeFirst();
        } else {
            _gka_multiR = knn.takeFirst();
            _gka_multiC = knn.takeFirst();
            _gka_multiL = knn.takeFirst();
        }

        // GKAs
        setPlayerBehavior(_gka_multiL, _bh_gka_multiL);
        setPlayerBehavior(_gka_multiR, _bh_gka_multiR);
        setPlayerBehavior(_gka_multiC, _bh_gka_multiC);
    }

    // Configure behaviors
    switch(_state) {
        default:
        case STATE_POS: {
            // GK
            _bh_gk->setRadius(GK_RADIUS);
            _bh_gk->setTakeoutEnabled(false);
            _bh_gk->setTakeoutFactor(GK_TAKEOUT_FACTOR);

            // GKA Uni
            if(numPlayers==2) {
                _bh_gka_uni->setRadius(GKA_RADIUS);
                _bh_gka_uni->setD(loc()->distBallOurGoal()<GKA_BALLDIST_REDUCE_D? GKA_D_REDUCED : GKA_D);
                _bh_gka_uni->setTakeoutEnabled(false);
                _bh_gka_uni->setTakeoutFactor(GKA_TAKEOUT_FACTOR);
            }

            // GKA Multi (2)
            if(numPlayers==3) {
                // GKA_L
                _bh_gka_multiL->setRadius(GKA_RADIUS);
                _bh_gka_multiL->setD(loc()->distBallOurGoal()<GKA_BALLDIST_REDUCE_D? GKA_D_REDUCED : GKA_D);
                _bh_gka_multiL->setTakeoutEnabled(false);
                _bh_gka_multiL->setTakeoutFactor(GKA_TAKEOUT_FACTOR);

                // GKA_R
                _bh_gka_multiR->setRadius(GKA_RADIUS);
                _bh_gka_multiR->setD(loc()->distBallOurGoal()<GKA_BALLDIST_REDUCE_D? GKA_D_REDUCED : GKA_D);
                _bh_gka_multiR->setTakeoutEnabled(false);
                _bh_gka_multiR->setTakeoutFactor(GKA_TAKEOUT_FACTOR);
            }

            // GKA Multi (3)
            if(numPlayers>=4) {
                // GKA_L
                _bh_gka_multiL->setRadius(GKA_RADIUS);
                _bh_gka_multiL->setD(GKA_D+0.10f);
                _bh_gka_multiL->setTakeoutEnabled(false);
                _bh_gka_multiL->setTakeoutFactor(GKA_TAKEOUT_FACTOR);

                // GKA_R
                _bh_gka_multiR->setRadius(GKA_RADIUS);
                _bh_gka_multiR->setD(GKA_D+0.10f);
                _bh_gka_multiR->setTakeoutEnabled(false);
                _bh_gka_multiR->setTakeoutFactor(GKA_TAKEOUT_FACTOR);

                // GKA_C
                _bh_gka_multiC->setRadius(GKA_RADIUS);
                _bh_gka_multiC->setTakeoutEnabled(false);
                _bh_gka_multiC->setTakeoutFactor(GKA_TAKEOUT_FACTOR);
            }

            // Switch state condition
            if(canKickBall() && loc()->distBallOurGoal() <= GKA_TAKEOUT_FACTOR)
                _state = STATE_TAKEOUT;

        } break;

        case STATE_TAKEOUT: {
            // GK Solo
            if(numPlayers==1)
                _bh_gk->setTakeoutEnabled(true);

            // GKA Uni
            if(numPlayers==2) {
                bool takeoutGK = loc()->isInsideOurArea(loc()->ball(), GK_TAKEOUT_FACTOR);
                _bh_gk->setTakeoutEnabled(takeoutGK);
                _bh_gka_uni->setTakeoutEnabled(!takeoutGK);
                _bh_gka_uni->setD(takeoutGK? GKA_D_EXTENDED : GKA_D);
            }

            // GKA Multi (2)
            if(numPlayers==3) {
                bool takeoutGK = loc()->isInsideOurArea(loc()->ball(), GK_TAKEOUT_FACTOR);
                bool takeoutGKA_R = false;
                bool takeoutGKA_L = false;
                if(takeoutGK==false) {
                    float distGKA_R = PlayerBus::ourPlayer(_gka_multiR)->distBall();
                    float distGKA_L = PlayerBus::ourPlayer(_gka_multiL)->distBall();
                    if(distGKA_L<distGKA_R)
                        takeoutGKA_L = true;
                    else
                        takeoutGKA_R = true;
                }

                // GK
                _bh_gk->setTakeoutEnabled(takeoutGK);

                // GKA R
                _bh_gka_multiR->setTakeoutEnabled(takeoutGKA_R);
                if(takeoutGK)
                    _bh_gka_multiR->setD(takeoutGK? GKA_D_EXTENDED : GKA_D);

                // GKA L
                _bh_gka_multiL->setTakeoutEnabled(takeoutGKA_L);
                if(takeoutGK)
                    _bh_gka_multiL->setD(takeoutGK? GKA_D_EXTENDED : GKA_D);
            }

            // GKA Multi (3)
            if(numPlayers>=4) {
                bool takeoutGK = loc()->isInsideOurArea(loc()->ball(), GK_TAKEOUT_FACTOR);
                bool takeoutGKA_R = false;
                bool takeoutGKA_L = false;
                bool takeoutGKA_C = false;
                if(takeoutGK==false) {
                    float distGKA_R = PlayerBus::ourPlayer(_gka_multiR)->distBall();
                    float distGKA_L = PlayerBus::ourPlayer(_gka_multiL)->distBall();
                    float distGKA_C = PlayerBus::ourPlayer(_gka_multiC)->distBall();
                    if(distGKA_L<distGKA_R && distGKA_L<distGKA_C)
                        takeoutGKA_L = true;
                    if(distGKA_R<distGKA_L && distGKA_R<distGKA_C)
                        takeoutGKA_R = true;
                    if(distGKA_C<distGKA_R && distGKA_C<distGKA_L)
                        takeoutGKA_C = true;
                }

                // GK
                _bh_gk->setTakeoutEnabled(takeoutGK);

                // GKA R
                _bh_gka_multiR->setTakeoutEnabled(takeoutGKA_R);
                if(takeoutGK)
                    _bh_gka_multiR->setD(takeoutGK? GKA_D_EXTENDED : GKA_D);

                // GKA L
                _bh_gka_multiL->setTakeoutEnabled(takeoutGKA_L);
                if(takeoutGK)
                    _bh_gka_multiL->setD(takeoutGK? GKA_D_EXTENDED : GKA_D);

                // GKA C
                _bh_gka_multiC->setTakeoutEnabled(takeoutGKA_C);
                if(takeoutGK) {
                    _bh_gka_multiC->setRadius(1.20*GKA_RADIUS);
                    _bh_gka_multiC->setD(takeoutGK? GKA_D_EXTENDED : GKA_D);
                }
            }

            // Switch state condition
            if(WR::Utils::distance(loc()->ball(), loc()->ourGoal()) > GKA_TAKEOUT_RADIUS_EXTENDED) // hysteresis applied
                _state = STATE_POS;

        } break;
    }
}
