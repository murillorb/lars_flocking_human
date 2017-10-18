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

#include "wrcoach.hh"

#include <WRCoach/const/globalconstants.hh>
#include <WRCoach/const/pfconstants.hh>
#include <WRCoach/const/opfconstants.hh>
#include <WRCoach/entity/world/world.hh>
#include <WRCoach/entity/controlmodule/coach/coach.hh>
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>
#include <WRCoach/entity/referee/sslreferee/sslreferee.hh>
#include <WRCoach/entity/player/sslplayer/sslplayer.hh>
#include <WRCoach/entity/player/vssplayer/vssplayer.hh>
#include <WRCoach/entity/player/navigation/navigation.hh>
#include <WRCoach/entity/sensor/wrsensors.hh>
#include <WRCoach/entity/player/navigation/wrnavigation.hh>
#include <WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wrsslstrategy.hh>
#include <WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wrvssstrategy.hh>
#include <WRCoach/entity/player/behavior/wrbehaviors.hh>
#include <WRCoach/utils/fields/wrfields.hh>
#include <WRCoach/entity/coachview/coachview.hh>

#define MAX_ROBOT_ID 12

WR::WRCoach::WRCoach(quint8 teamId, Colors::Color teamColor, FieldSide fieldSide, Competitions::Competition competition, bool enableGUI)
    : _teamId(teamId), _teamColor(teamColor), _fieldSide(fieldSide), _competition(competition), _enableGUI(enableGUI) {

    // Create controller
    _ctr = new Controller();

    // Default field setup
    if(_competition==Competitions::SSL)
        _defaultField = new Fields::SSL2015();
    else
        _defaultField = new Fields::VSS2008();

    // Initialize default values
    this->setServerAddress("localhost", 0);
}

WR::WRCoach::~WRCoach() {
    // Delete controller
    delete _ctr;

    // Delete default field
    delete _defaultField;
}

bool WR::WRCoach::start() {
    // Op team infos parsing
    quint8 opTeamId = (_teamId==0? 1:0);
    Colors::Color opTeamColor = (_teamColor==Colors::YELLOW? Colors::BLUE : Colors::YELLOW);
    FieldSide opFieldSide = (_fieldSide.isRight()? Sides::LEFT : Sides::RIGHT);

    // Initialize constants and utils
    initializeConstants();
    initializeUtils();

    // Server connection
    if(connectToServer()==false)
        return false;

    // Create World
    _world = new World(_ctr, _competition, _defaultField);

    // Create SSLReferee
    _ref = new SSLReferee();
    _world->addEntity(_ref, 0);
    _ref->addGameInfo(Colors::YELLOW);
    _ref->addGameInfo(Colors::BLUE);

    // Ball moviment detect sensor
    _ballMov = new Sensor_BallMovimentDetect(_ref, _world->getWorldMap());
    _world->addEntity(_ballMov, 0);

    // Setup teams
    setupTeams(opTeamId, opTeamColor, opFieldSide);
    _world->setTeams(_ourTeam, _opTeam);

    // Setup team players
    setupOurPlayers();
    setupOppPlayers(opTeamId);

    // Create Coach
    _coach = new Coach(_ref, _ourTeam, _opTeam);
    _world->setControlModule(_coach);

    // Setup strategy for Coach
    Strategy *strategy = NULL;
    switch(_competition) {
        default:
        case Competitions::SSL:
//            strategy = new SSLStrategy_Template();
//            strategy = new SSLStrategy_Test();
//            strategy = new SSLStrategy_WR2015();
            strategy = new SSLStrategy_Flock();

        break;
        case Competitions::VSS:
//            strategy = new SSLStrategy_Test();
            strategy = new VSSStrategy_WR2016();
        break;
    }
    _coach->setStrategy(strategy);

    // Coach view
    if(_enableGUI) {
        _coachView = new CoachView(_competition, _ourTeam, _opTeam);
        _world->addEntity(_coachView, 2);
    }

    // Initialize Utils
    initializeUtils();

    // Start world
    _world->start();

    return true;
}

void WR::WRCoach::stop() {
    // Stop world and wait for it
    _world->stopEntity();
    _world->wait();

    // Delete teams
    delete _ourTeam;
    delete _opTeam;

    // Delete modules
    delete _world;
    delete _ref;
    delete _ballMov;
    delete _coach;
    delete _coachView;

    // Disconnect controller
    _ctr->disconnect();
}

void WR::WRCoach::setServerAddress(QString ipAddress, int port) {
    _serverAddress = ipAddress;
    _serverPort = port;
}

void WR::WRCoach::initializeConstants() {
    GlobalConstants::setCompetition(_competition);
    PFConstants::setCompetition(_competition);
    OPFConstants::setCompetition(_competition);
    SensorConstants::setCompetition(_competition);
}

void WR::WRCoach::initializeUtils() {
    WR::Utils::initialize(_ourTeam, _opTeam);
}

bool WR::WRCoach::connectToServer() {
    // Server connection
    if(_ctr->connect(_serverAddress, _serverPort) == false) {
        std::cout << "[ERROR] failed to connect to GEARSystem's server!";
        return false;
    }

    // Check if GEARSystem's Sensor is connected
    if(_ctr->teams().size() < 2) {
        std::cout << "[ERROR] no teams on controller, GEARSystem's sensor is not connected!";
        return false;
    }

    return true;
}

void WR::WRCoach::setupTeams(quint8 opTeamId, Colors::Color opTeamColor, FieldSide opFieldSide) {
    // Create teams
    _ourTeam = new WRTeam(_competition, _teamId, _world->getWorldMap());
    _opTeam = new WRTeam(_competition, opTeamId, _world->getWorldMap());

    // Set opponent teams
    _ourTeam->setOpponentTeam(_opTeam);
    _opTeam->setOpponentTeam(_ourTeam);

    // Set our team info
    _ourTeam->setTeamColor(_teamColor);
    _ourTeam->setFieldSide(_fieldSide);

    // Set op team info
    _opTeam->setTeamColor(opTeamColor);
    _opTeam->setFieldSide(opFieldSide);
}

void WR::WRCoach::setupOurPlayers() {
    // Create OUR PLAYERS
    QList<quint8> playerList = _world->getWorldMap()->players(_teamId);
    for(quint8 i=0; i<playerList.size() && i<=MAX_ROBOT_ID; i++) {
        // Create navigation algorithm for player
        NavigationAlgorithm *navAlg = new PF();
//        NavigationAlgorithm *navAlg = new RRT();
        ControlAlgorithm *linCtrAlg = new PID();
        ControlAlgorithm *angCtrAlg = new PID();

        // Create Player
        Player *player;
        switch(_competition) {
            default:
            case Competitions::SSL:
                player = new SSLPlayer(_world, _ourTeam, playerList.at(i), _ctr, navAlg, linCtrAlg, angCtrAlg, new Behavior_DoNothing(), _ref);
            break;
            case Competitions::VSS:
                player = new VSSPlayer(_world, _ourTeam, playerList.at(i), _ctr, navAlg, linCtrAlg, angCtrAlg, new Behavior_DoNothing(), _ref);
            break;
        }

        // Enable
        player->enable(true);

        // Add to team
        _ourTeam->addPlayer(player);

        // Add to world
        _world->addEntity(player, 2);
    }
}

void WR::WRCoach::setupOppPlayers(quint8 opTeamId) {
    // Create opp. players
    const QList<quint8> opPlayerList = _world->getWorldMap()->players(opTeamId);
    for(quint8 i=0; i<opPlayerList.size() && i<=MAX_ROBOT_ID; i++) {
        // Create Player
        Player *opPlayer=NULL;
        switch(_competition) {
            default:
            case Competitions::SSL:
                opPlayer = new SSLPlayer(NULL, _opTeam, opPlayerList.at(i), _ctr, NULL, NULL, NULL, NULL, _ref);
            break;
            case Competitions::VSS:
                opPlayer = new VSSPlayer(NULL, _opTeam, opPlayerList.at(i), _ctr, NULL, NULL, NULL, NULL, _ref);
            break;
        }

        // Disable (op team doesnt run)
        opPlayer->enable(false);

        // Add to team
        _opTeam->addPlayer(opPlayer);

        // Add to world
        _world->addEntity(opPlayer, 2);
    }
}
