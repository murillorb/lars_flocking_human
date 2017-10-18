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

#include "playbook_ourkickoff.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString Playbook_OurKickOff::name() {
    return "Playbook_OurKickOff";
}

Playbook_OurKickOff::Playbook_OurKickOff() {
    _kicker = -1;

	_bh_kickerPos = NULL;
    _bh_kickerAtk = NULL;
}

int Playbook_OurKickOff::maxNumPlayer() {
    return 4;
}

void Playbook_OurKickOff::configure(int numPlayers) {
    // Kicker
    usesBehavior(_bh_kickerPos = new Behavior_Pos());
    usesBehavior(_bh_kickerAtk = new Behavior_Attack());

    // Assistants
    _assistantPos.setUnknown();
    _bh_assistsPos.clear();
    for(int i=0; i<numPlayers-1; i++) {
        Behavior_Pos *bh_pos = new Behavior_Pos();
        usesBehavior(bh_pos);
        _bh_assistsPos.push_back(bh_pos);
    }
}

void Playbook_OurKickOff::run(int numPlayers) {
    if(_kicker == -1)
        std::cout << "[WARNING]" << name().toStdString() << ": kickerId not set!\n";

    // Kicker
    dist()->removePlayer(_kicker);
    const Position kickerPos = WR::Utils::threePoints(loc()->fieldCenter(), loc()->ourGoal(), 0.20f, 0.0);
    _bh_kickerPos->setPosition(kickerPos);
    _bh_kickerPos->avoidAll(true);

    // Assistants
    const float signalX = loc()->ourSide().isRight()? 1 : -1;
    const Position posUp = Position(true, signalX*0.2, 1.75, 0.0);
    const Position posCenter = Position(true, signalX*0.6, 0.0, 0.0);
    const Position posBot = Position(true, signalX*0.2, -1.75, 0.0);
    QList<quint8> assistants = dist()->getAllY();

    // Add recvAssistants to attacker, and configure attacker
    _bh_kickerAtk->clearRecvList();
    for(int i=0; i<assistants.size(); i++)
        _bh_kickerAtk->addRecv(assistants.at(i));
    _bh_kickerAtk->setEnableClearKick(true);
    _bh_kickerAtk->setEnablePass(true);

    // Set assistants
    bool assistsPositioned = false;
    switch(assistants.size()) {
        default:
        case 0: {
            assistsPositioned = true;
        } break;
        case 1: {
            // Random up/bot
            if(_assistantPos.isUnknown()) {
                srand(clock());
                const int randSide = rand()%2;
                _assistantPos = randSide? posUp : posBot;
            }

            // Set
            Behavior_Pos *bh_pos = _bh_assistsPos.first();
            setPlayerBehavior(assistants.first(), bh_pos);
            bh_pos->setPosition(_assistantPos);
            bh_pos->setPositionToLook(loc()->fieldCenter());

            assistsPositioned = bh_pos->isPositioned();
        } break;
        case 2: {
            // Set bot
            Behavior_Pos *bh_pos1 = _bh_assistsPos.at(0);
            setPlayerBehavior(assistants.at(0), bh_pos1);
            bh_pos1->setPosition(posBot);
            bh_pos1->setPositionToLook(loc()->fieldCenter());

            // Set up
            Behavior_Pos *bh_pos2 = _bh_assistsPos.at(1);
            setPlayerBehavior(assistants.at(1), bh_pos2);
            bh_pos2->setPosition(posUp);
            bh_pos2->setPositionToLook(loc()->fieldCenter());

            assistsPositioned = bh_pos1->isPositioned() && bh_pos2->isPositioned();
        } break;
        case 3: {
            // Set bot
            Behavior_Pos *bh_pos1 = _bh_assistsPos.at(0);
            setPlayerBehavior(assistants.at(0), bh_pos1);
            bh_pos1->setPosition(posBot);
            bh_pos1->setPositionToLook(loc()->fieldCenter());

            // Set center
            Behavior_Pos *bh_pos2 = _bh_assistsPos.at(1);
            setPlayerBehavior(assistants.at(1), bh_pos2);
            bh_pos2->setPosition(posCenter);
            bh_pos2->setPositionToLook(loc()->fieldCenter());

            // Set up
            Behavior_Pos *bh_pos3 = _bh_assistsPos.at(2);
            setPlayerBehavior(assistants.at(2), bh_pos3);
            bh_pos3->setPosition(posUp);
            bh_pos3->setPositionToLook(loc()->fieldCenter());

            assistsPositioned = bh_pos1->isPositioned() && bh_pos2->isPositioned() && bh_pos3->isPositioned();
        } break;
    }

    // Set kicker
    if(canKickBall() && assistsPositioned)
        setPlayerBehavior(_kicker, _bh_kickerAtk);
    else
        setPlayerBehavior(_kicker, _bh_kickerPos);
}
