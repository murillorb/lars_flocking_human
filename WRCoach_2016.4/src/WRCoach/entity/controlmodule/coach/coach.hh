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

#ifndef COACH_HH
#define COACH_HH

#include <WRCoach/entity/controlmodule/coach/basecoach.hh>
#include <WRCoach/entity/controlmodule/controlmodule.hh>

class Coach : public ControlModule {
public:
    Coach(SSLReferee *ref, WRTeam *ourTeam, WRTeam *opTeam);
    virtual ~Coach();
    QString name();

    void setStrategy(Strategy *strat);
private:
    // ControlModule inherit virtual methods
    void run();

    // Game info, used to initialize strategy
    SSLReferee *_ref;
    CoachUtils *_utils;

    // Teams
    WRTeam *_ourTeam;
    WRTeam *_opTeam;

    // Strategy
    QMutex _mutexStrategy;
    Strategy *_strat;
    Strategy* strategy();
};

#endif // COACH_HH
