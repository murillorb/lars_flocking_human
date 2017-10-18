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

#ifndef WRCOACH_HH
#define WRCOACH_HH

#include <QHostAddress>
#include <GEARSystem/gearsystem.hh>
#include <WRCoach/utils/basics/color.hh>
#include <WRCoach/utils/basics/competition.hh>
#include <WRCoach/utils/fieldside/fieldside.hh>
#include <WRCoach/utils/fields/fields.hh>

#include <WRCoach/entity/baseentity.hh>
#include <WRCoach/entity/controlmodule/coach/basecoach.hh>

#include <WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wrsslstrategy.hh>
#include <WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wrvssstrategy.hh>

namespace WR {

class WRCoach {
public:
    WRCoach(quint8 teamId, Colors::Color teamColor, FieldSide fieldSide, Competitions::Competition competition, bool enableGUI);
    ~WRCoach();

    // Start
    bool start();
    void stop();

    // Configuration
    void setServerAddress(QString ipAddress, int port);

private:
    // Internal methods
    void initializeConstants();
    void initializeUtils();
    bool connectToServer();
    void setupTeams(quint8 opTeamId, Colors::Color opTeamColor, FieldSide opFieldSide);
    void setupOurPlayers();
    void setupOppPlayers(quint8 opTeamId);

    // Parameters
    const quint8 _teamId;
    const Colors::Color _teamColor;
    const FieldSide _fieldSide;
    const Competitions::Competition _competition;
    const bool _enableGUI;

    // Server connection
    QString _serverAddress;
    int _serverPort;

    // Internal
    Controller *_ctr;
    Fields::Field* _defaultField;

    // Modules
    World *_world;
    SSLReferee *_ref;
    Sensor_BallMovimentDetect *_ballMov;
    Coach *_coach;
    CoachView *_coachView;

    // Teams
    WRTeam *_ourTeam;
    WRTeam *_opTeam;
};

}

#endif // WRCOACH_HH
