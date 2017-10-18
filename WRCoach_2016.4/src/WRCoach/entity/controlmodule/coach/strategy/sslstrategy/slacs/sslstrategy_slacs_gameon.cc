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

    #include "sslstrategy_slacs_gameon.hh"
    #include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

    #pragma GCC diagnostic ignored "-Wunused-parameter"

    QString SSLStrategy_Slacs_GameOn::name() {
        return "SSLStrategy_Slacs_GameOn";
    }

    SSLStrategy_Slacs_GameOn::SSLStrategy_Slacs_GameOn() {
        _pb_slacs = NULL;
    }

    void SSLStrategy_Slacs_GameOn::configure(int numOurPlayers) {
        std::cout << "SSLStrategy_Slacs_GameOn::configurate(" << numOurPlayers << ")\n";

        usesPlaybook(_pb_slacs = new Playbook_Slacs());
    }

    void SSLStrategy_Slacs_GameOn::run(int numOurPlayers) {
        std::cout << "SSLStrategy_Slacs_GameOn::run()\n";

        _pb_slacs->addPlayers(dist()->getAllPlayers());
        _pb_slacs->setEntityDistance(0.8);
        _pb_slacs->setMaxBeam(20);
        _pb_slacs->setSamplesDistance(0.1);
        _pb_slacs->setLaserRange(4.0);
        _pb_slacs->setWGNoiseGain(0.0001);
        _pb_slacs->setEntityId(3);
        _pb_slacs->setForceWeight(0.5);
    }
