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

#include "wrteam.hh"
#include <WRCoach/entity/player/player.hh>
#include <WRCoach/entity/locations.hh>

WRTeam::WRTeam(Competitions::Competition competition, quint8 teamId, WorldMap *wm) : _competition(competition) {
    _teamId = teamId;
    _wm = wm;

    _loc = new Locations(this);
}

WRTeam::~WRTeam() {
    delete _loc;
}

Colors::Color WRTeam::teamColor() const {
    QMutexLocker locker(&_mutexTeamColor);
    return _teamColor;
}

FieldSide WRTeam::fieldSide() const {
    QMutexLocker locker(&_mutexFieldSide);
    return _fieldSide;
}

void WRTeam::setTeamColor(Colors::Color color) {
    QMutexLocker locker(&_mutexTeamColor);
    _teamId = _teamColor = color;
}

void WRTeam::setFieldSide(FieldSide fieldSide) {
    QMutexLocker locker(&_mutexFieldSide);
    _fieldSide = fieldSide;
}

bool WRTeam::hasBallPossession() {
    // Iterate players
    const QList<Player*> players = _players.values();
    QList<Player*>::const_iterator it;
    for(it=players.constBegin(); it!=players.end(); it++) {
        const Player* player = *it;

        // Check ball possession
        if(player->hasBallPossession())
            return true;
    }

    return false;
}

quint8 WRTeam::ballPossession() {
    // Iterate players
    const QList<Player*> players = _players.values();
    QList<Player*>::const_iterator it;
    for(it=players.constBegin(); it!=players.end(); it++) {
        const Player* player = *it;

        // Check ball possession
        if(player->hasBallPossession())
            return player->playerId();
    }

    return BALLPOSS_NONE;
}

void WRTeam::addPlayer(Player *player) {
    _players.insert(player->playerId(), player);
}

int WRTeam::avPlayersSize() {
    _muxPlayers.lock();
    int result = _avPlayers.size();
    _muxPlayers.unlock();
    return result;
}

QHash<quint8,Player*> WRTeam::avPlayers() {
    _muxPlayers.lock();
    QHash<quint8,Player*>result(_avPlayers);
    _muxPlayers.unlock();
    return result;
}

void WRTeam::updateAvailablePlayers() {
    _muxPlayers.lock();

    // Clear hash
    _avPlayers.clear();

    // Update players
    QHash<quint8,Player*>::iterator it;
    for(it=_players.begin(); it!=_players.end(); it++) {
        quint8 id = it.key();
        Player *player = it.value();

        // If position is known, player is available
        if(player->position().isUnknown()==false)
            _avPlayers.insert(id, player);
    }

    _muxPlayers.unlock();
}
