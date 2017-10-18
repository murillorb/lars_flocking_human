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

#ifndef STRATEGY_HH
#define STRATEGY_HH

#include <WRCoach/entity/controlmodule/coach/basecoach.hh>
#include <WRCoach/entity/referee/sslreferee/sslgameinfo.hh>

class Strategy {
public:
    Strategy();
    virtual ~Strategy();

    // Called one time before run is first called
    bool isInitialized() { return _initialized; }
    void initialize(SSLReferee *ref, WRTeam *ourTeam, WRTeam *opTeam, CoachUtils *utils);

    // Called in Coach loop
    virtual void runStrategy() = 0;

    virtual QString name() = 0;
protected:
    SSLGameInfo::RefProcessedState getGameState();
    virtual void setStrategyState(int gameState, StrategyState *strategyState);
    void runStrategy(int gameState, SSLGameInfo::RefProcessedState refState);
private:
    virtual QString state2str(int gameState) = 0;
    StrategyState* getStrategyState(int gameState);

    // Implemented by children
    virtual void configure() = 0;

    // Referee
    SSLReferee *_ref;

    // Game info
    CoachUtils *_utils;

    // Teams
    WRTeam *_ourTeam;
    WRTeam *_opTeam;

    // Players distribution
    qint8 _kickerId;
    SSLGameInfo::RefProcessedState _lastState;
    PlayersDistribution *_dist;

    // Game states table
    QHash<int,StrategyState*> _strategyStatesTable;

    // Strategy initialized
    bool _initialized;
};

#endif // STRATEGY_HH
