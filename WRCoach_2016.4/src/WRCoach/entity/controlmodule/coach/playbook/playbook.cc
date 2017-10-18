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

#include "playbook.hh"
#include <WRCoach/entity/referee/sslreferee/sslreferee.hh>
#include <WRCoach/entity/controlmodule/coach/playersdistribution.hh>
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>
#include <WRCoach/entity/player/player.hh>
#include <WRCoach/entity/player/behavior/behavior.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

Playbook::Playbook() {
    _initialized = false;
    _lastNumPlayers = 0;
    _configureEnabled = false;
    _dist = NULL;
}

Playbook::~Playbook() {
    setCurrBehaviorsToOld();
    clearOldBehaviors();
    if(_dist!=NULL)
        delete _dist;
}

Locations* Playbook::loc() const {
    return _ourTeam->loc();
}

void Playbook::initialize(WRTeam *ourTeam, WRTeam *opTeam, CoachUtils *utils, qint8 *kickerId, SSLGameInfo::RefProcessedState *lastState, SSLReferee *ref) {
    _ourTeam = ourTeam;
    _opTeam = opTeam;
    _utils = utils;
    _ref = ref;

    _dist = new PlayersDistribution(_ourTeam, kickerId, lastState, ref);

    _initialized = true;
}

void Playbook::runPlaybook(QString strategyState) {

    // Check if StrategyState set players to this playbook
    if(_players.size()==0) {
        //std::cout << "[WARNING] " << strategyState.toStdString() << "->" << name().toStdString() << " has no players set!\n";
        return;
    }

    // Run configure(numPlayers) if num of players in this playbook changed
    if(_lastNumPlayers != _players.size()) {
        _lastNumPlayers = _players.size();
        setCurrBehaviorsToOld();
        _configureEnabled = true;
        configure(_lastNumPlayers);
        _configureEnabled = false;
    }


    // Ensure assignment table has only assignments of players that are in this playbook
    QList<quint8> ids = _assignmentTable.keys();
    for(int i=0; i<ids.size(); i++) {
        quint8 id = ids.at(i);
        if(_players.contains(id)==false)
            _assignmentTable.remove(id);
    }
    for(int i=0; i<_players.size(); i++) {
        quint8 id = _players.at(i);
        if(_assignmentTable.contains(id)==false)
            _assignmentTable.insert(id, NULL);
    }

    // Check excessive players on playbook, and print warning
    if(_players.size() > maxNumPlayer()) {
        std::cout << "[WARNING] " << strategyState.toStdString() << "->" << name().toStdString() << " has excessive players (num=" << _players.size() << ", max=" << maxNumPlayer() << ").\n";
    }

    // Run playbook (implemented by child)
    // This will update assignment table with current desired player-behavior pairs
    run(_lastNumPlayers);

    // Ensure players on this playbook has behaviors assigned to them
    updatePlayersBehaviors();
}

void Playbook::updatePlayersBehaviors() {
    QHash<quint8,Behavior*>::const_iterator it;
    for(it=_assignmentTable.constBegin(); it!=_assignmentTable.constEnd(); it++) {
        quint8 id = it.key();
        Behavior *behavior = it.value();

        // Check invalid id
        if(id==DIST_INVALID_ID)
            continue;

        // Get list of available players
        QHash<quint8,Player*> avPlayers = _ourTeam->avPlayers();
        Player *player = avPlayers.value(id);

        // Effectivelly set behavior to player
        player->setBehavior(behavior);

        // Warning to inform that some player isn't being used on playbook
        if(behavior==NULL)
            std::cout << "[WARNING] " << name().toStdString() << ", player #" << (int)id << " has no Behavior assigned!\n";
    }
}

void Playbook::setCurrBehaviorsToOld() {
    while(_behaviorsList.empty()==false)
        _oldBehaviors.push_back(_behaviorsList.takeFirst());
}

void Playbook::clearOldBehaviors() {
    while(_oldBehaviors.empty()==false)
        delete _oldBehaviors.takeFirst();
}

void Playbook::usesBehavior(Behavior *behavior) {
    // Check if call is inside configure()
    if(_configureEnabled==false) {
        std::cout << "[WARNING] Blocked playbook '" << name().toStdString() << "' setting Behavior to use outside configure().\n";
        return;
    }

    // Check null pointer
    if(behavior==NULL) {
        std::cout << "[ERROR] " << name().toStdString() << "::usesBehavior(NULL), &behavior==NULL.\n";
        return;
    }

    // Add behavior
    if(_behaviorsList.contains(behavior)==false)
        _behaviorsList.push_back(behavior);
}

void Playbook::setPlayerBehavior(quint8 id, Behavior *behavior) {
    if(id==DIST_INVALID_ID)
        return;

    // Check if player is on this playbook
    if(_players.contains(id)==false) {
        std::cout << "[ERROR] " << name().toStdString() << "::setPlayerBehavior(" << (int)id << ", " << behavior->name().toStdString() << "), player #" << (int)id << " wasn't added to this playbook!\n";
        return;
    }

    // Check pointer
    if(behavior==NULL) {
        std::cout << "[ERROR] " << name().toStdString() << "::setPlayerBehavior(" << (int)id << ", NULL), &behavior==NULL!\n";
        return;
    }

    // Check if behavior is on this playbook
    if(_behaviorsList.contains(behavior)==false) {
        std::cout << "[ERROR] " << name().toStdString() << "::setPlayerBehavior(" << (int)id << ", " << behavior->name().toStdString() << "), behavior " << behavior->name().toStdString() << " wasn't added to this playbook!\n";
        return;
    }

    // Check if old player still has the behavior
    if(_assignmentTable.values().contains(behavior)) {
        quint8 oldID = _assignmentTable.key(behavior);
        if(_players.contains(oldID))
            _assignmentTable.insert(oldID, NULL);
    }

    // Assign behavior
    _assignmentTable.insert(id, behavior);
}

bool Playbook::hasPlayer(quint8 id) {
    // Check if player is added to playbook
    return _players.contains(id);
}

void Playbook::addPlayer(quint8 id) {
    if(id==DIST_INVALID_ID)
        return;

    // Check if player id is on available players hash
    QHash<quint8,Player*> avPlayers = _ourTeam->avPlayers();
    if(avPlayers.contains(id)==false) {
		std::cout << "[ERROR] " << name().toStdString() << "::addPlayer(" << (int)id << "), player #" << (int)id << " doesn't exist!\n";
        return;
    }

    // Add player id to this playbook and to distribution
    if(_players.contains(id)==false) {
        _players.push_back(id);
        _dist->insert(id);
    }

}

void Playbook::addPlayers(const QList<quint8> &ids) {
    if(ids.empty())
        return;

    // Add list of players
    QList<quint8>::const_iterator it;
    for(it=ids.constBegin(); it!=ids.constEnd(); it++) {
        const quint8 id = *it;
        addPlayer(id);
    }
}

void Playbook::clearPlayers() {
    _players.clear();
    _dist->clear();
}

int Playbook::numPlayers() {
    return _players.size();
}

bool Playbook::canKickBall() const {
    return _ref->getGameInfo(_ourTeam->teamColor())->canKickBall();
}
