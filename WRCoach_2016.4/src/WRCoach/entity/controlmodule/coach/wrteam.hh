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

#ifndef WRTEAM_HH
#define WRTEAM_HH

#include <QReadWriteLock>
#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/utils/fieldside/fieldside.hh>
#include <WRCoach/utils/basics/color.hh>
#include <WRCoach/utils/basics/competition.hh>
#include <WRCoach/utils/fields/fields.hh>

#define BALLPOSS_NONE 200

class WRTeam {
public:
    WRTeam(Competitions::Competition competition, quint8 teamId, WorldMap *wm);
    virtual ~WRTeam();

    // Constant infos access
    quint8 teamId() const { return _teamId; }
    Competitions::Competition competition() const { return _competition; }

    // Infos getters
    Colors::Color teamColor() const;
    FieldSide fieldSide() const;

    // Infos setters
    void setTeamColor(Colors::Color color);
    void setFieldSide(FieldSide fieldSide);

    // Opponent team info
    void setOpponentTeam(WRTeam *team) { _opTeam = team; }
    WRTeam* opTeam() const { return _opTeam; }

    // Ball possession
    bool hasBallPossession();
    quint8 ballPossession();

    // Add players
    void addPlayer(Player *player);

    // Available players access
    int avPlayersSize();
    QHash<quint8,Player*> avPlayers();
    void updateAvailablePlayers();

    // Loc and WorldMap access
    Locations* loc() { return _loc; }
    WorldMap* wm() { return _wm; }
private:
    // Competition info
    const Competitions::Competition _competition;

    // Team id
    quint8 _teamId;

    // Team color
    Colors::Color _teamColor;
    mutable QMutex _mutexTeamColor;

    // Team field side
    FieldSide _fieldSide;
    mutable QMutex _mutexFieldSide;

    // Opp. team access
    WRTeam *_opTeam;

    // Players
    QMutex _muxPlayers;
    QHash<quint8,Player*> _players;
    QHash<quint8,Player*> _avPlayers;

    // Loc and WorldMap
    Locations *_loc;
    WorldMap *_wm;
};

#endif // WRTEAM_HH
