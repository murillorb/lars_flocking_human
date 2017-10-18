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

    #include "sslstrategy_flock_gameon.hh"
    #include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

    #pragma GCC diagnostic ignored "-Wunused-parameter"

    QString SSLStrategy_Flock_GameOn::name() {
        return "SSLStrategy_Flock_GameOn";
    }

    SSLStrategy_Flock_GameOn::SSLStrategy_Flock_GameOn() {
        _pb_flock = NULL;
    }

    void SSLStrategy_Flock_GameOn::configure(int numOurPlayers) {
        std::cout << "SSLStrategy_Flock_GameOn::configurate(" << numOurPlayers << ")\n";

        usesPlaybook(_pb_flock = new Playbook_Flock());
    }

    void SSLStrategy_Flock_GameOn::run(int numOurPlayers) {
        std::cout << "SSLStrategy_Flock_GameOn::run()\n";

        _pb_flock->addPlayers(dist()->getAllPlayers());
        _pb_flock->setEntityDistance(0.995);
        _pb_flock->setMaxBeam(10);
        _pb_flock->setSamplesDistance(0.1);
        _pb_flock->setLaserRange(2.0);
        _pb_flock->setWGNoiseGain(0.0001);
        _pb_flock->setEntityId(3);
        _pb_flock->setForceWeight(0.5);



        _pb_flock->setCohesionForce(0);
        _pb_flock->setAvoidForce(0.25);//0.2 vs 0.5
        _pb_flock->setAvoidHumanForce(0);
        _pb_flock->setGaussForce(5);//5 vs 0.5
       // _pb_flock->setAlignForce(0);
        _pb_flock->setGaussContour(0);
        _pb_flock->setNeighborReach(99999);
        for (int i = 0; i < 6; i++)
            _pb_flock->setStopState(i,0);
        _pb_flock->setSqObstacleSize(0.5);
        _pb_flock->setSideMode(1);// { _sideMode = mode; }
        _pb_flock->setEnableSlacs(0);//(int mode) { _enableSlacs = mode; }
    }
