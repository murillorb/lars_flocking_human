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

#ifndef SSLSTRATEGY_HH
#define SSLSTRATEGY_HH

#include <WRCoach/entity/controlmodule/coach/strategy/strategy.hh>

class SSLStrategy : public Strategy {
public:
    SSLStrategy();
    virtual ~SSLStrategy();

    virtual QString name() = 0;
protected:
    typedef enum {
        HALT,
        GAMEON,
        GAMEOFF,
        OURINDIRECTKICK,
        OURDIRECTKICK,
        OURKICKOFF,
        OURPENALTY,
        THEIRKICKOFF,
        THEIRPENALTY,
        THEIRDIRECTKICK,
        THEIRINDIRECTKICK,
        UNDEFINED
    } SSLGameState;
    void setStrategyState(SSLGameState gameState, StrategyState *strategyState);
private:
    virtual void configure() = 0;
    void runStrategy();

    QString state2str(int gameState);
    SSLGameState refState2SSLGameState(SSLGameInfo::RefProcessedState refState) const;
};

#endif // SSLSTRATEGY_HH
