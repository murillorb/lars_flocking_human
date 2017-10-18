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

#ifndef PLAYBOOK_HH
#define PLAYBOOK_HH

#include <climits>
#include <WRCoach/entity/controlmodule/coach/basecoach.hh>
#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/entity/referee/sslreferee/sslgameinfo.hh>
#include <WRCoach/entity/player/behavior/wrbehaviorsbase.hh>

class Playbook : public QObject {
    Q_OBJECT
public:
    Playbook();
    virtual ~Playbook();

    // Called one time before run is first called
    bool isInitialized() { return _initialized; }
    void initialize(WRTeam *ourTeam, WRTeam *opTeam, CoachUtils *utils, qint8 *kickerId, SSLGameInfo::RefProcessedState *lastState, SSLReferee *ref);

    // Called in StrategyState loop
    void runPlaybook(QString strategyState);
    void clearOldBehaviors();

    // Players manipulation
    void addPlayer(quint8 id);
    void addPlayers(const QList<quint8> &ids);
    void clearPlayers();
    int numPlayers();

    virtual QString name() = 0;
protected:

    // Players on this playbook access
    bool hasPlayer(quint8 id);
    void setPlayerBehavior(quint8 id, Behavior *behavior);

    // Behavior functions
    void usesBehavior(Behavior *behavior);

    // canKickBall access (normal start)
    bool canKickBall() const;

    // Utils, loc and dist access
    CoachUtils* utils() { return _utils; }
    Locations* loc() const;
    PlayersDistribution* dist() { return _dist; }

private:
    // Implemented by children
    virtual void run(int numPlayers) = 0;
    virtual void configure(int numPlayers) = 0;
    virtual int maxNumPlayer() = 0;

    // Configure info
    bool _configureEnabled;
    int _lastNumPlayers;
    void updatePlayersBehaviors();

    // Behaviors garbage collector
    QList<Behavior*> _oldBehaviors;
    void setCurrBehaviorsToOld();

    // Game info
    WRTeam *_ourTeam;
    WRTeam *_opTeam;
    CoachUtils *_utils;
    SSLReferee *_ref;

    // Players in this playbook list
    PlayersDistribution *_dist;
    QList<quint8> _players;
    QHash<quint8,Behavior*> _assignmentTable;
    QList<Behavior*> _behaviorsList;

    // Playbook initialized
    bool _initialized;
};

#endif // PLAYBOOK_HH
