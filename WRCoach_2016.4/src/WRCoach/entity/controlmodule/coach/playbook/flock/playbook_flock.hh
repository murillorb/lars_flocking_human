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

#ifndef PLAYBOOK_FLOCK_HH
#define PLAYBOOK_FLOCK_HH

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>
#include <WRCoach/entity/controlmodule/coach/playbook/flock/assymetricgaussfunction.h>

class Playbook_Flock : public Playbook {
private:
    // Parameters
    float _distEntity;
    int _maxBeam;
    float _samplesDistance;
    float _laserRange;
    float _noiseGain;
    int _entityId;
    float _forceWeight;


    //Flocking parameters; remember to clear SLACS-related properties that are left unused

    float _cohesionForce;
    float _avoidForce;
    float _avoidHumanForce;
    float _gaussForce;
    float _alignForce;
    float _gaussContour;
    float _neighborReach;
    float _sqObstacleSize;
    int _stopState[6];
    int _sideMode;
    int _enableSlacs;

    //Gaussian control class

    AssymetricGaussFunction AGS;

    // Behaviors
    QList<Behavior_FollowEntity*> _bh_followentity;
    QList<Behavior_DoNothing*> _bh_donothing;
    QList<Behavior_GoTo*> _bh_goto;

    QList<int> _samples;
    bool sampleIsValid(int id, const Position &pos, int numPlayers);
    float whiteGaussianNoise(float noiseGain);
    float getGaussianPower(int id, const Position &hpos);
    int findNearest(int id, int numPlayers);
    int findNearestOpponent(int id, int numPlayers);

    //Miscellaneous functions

    double getDx(Position robot, Position target, double targAng);
    double getDy(Position robot, Position target, double targAng);
    int getStopState(int index) {return _stopState[index];}

    void normalizeForces();
    void squareObstacle(int id, int nid, Position *avoidPos);
    void configure(int numPlayers);
    void run(int numPlayers);
public:
    Playbook_Flock();
    QString name();
    int maxNumPlayer();

    void setEntityDistance(float distEntity) { _distEntity = distEntity; }
    void setMaxBeam(float maxBeam) { _maxBeam = maxBeam; }
    void setSamplesDistance(float samplesDistance) { _samplesDistance = samplesDistance; }
    void setLaserRange(float laserRange) { _laserRange = laserRange; }
    void setWGNoiseGain(float noiseGain) { _noiseGain = noiseGain; }
    void setEntityId(int entityId) { _entityId = entityId; }
    void setForceWeight(float forceWeight) { _forceWeight = forceWeight; }
    void setCohesionForce(float cohesionForce) { _cohesionForce = cohesionForce; }
    void setAvoidForce(float avoidForce) { _avoidForce = avoidForce; }
    void setAlignForce(float alignForce) { _alignForce = alignForce; }
    void setSideMode(int mode) { _sideMode = mode; }
    void setEnableSlacs(int mode) { _enableSlacs = mode; }


    void setAvoidHumanForce(float avoidHumanForce) { _avoidHumanForce = avoidHumanForce; }
    void setGaussForce(float gaussForce) { _gaussForce = gaussForce; }
    void setGaussContour(float gaussContour) { _gaussContour = gaussContour; }
    void setNeighborReach(float neighborReach) { _neighborReach = neighborReach; }
    void setStopState(int index, int value) {_stopState[index] = value;}
    void setSqObstacleSize(double value){_sqObstacleSize = value;}
    float getNeighborReach() {return _neighborReach; }



};

#endif // PLAYBOOK_FLOCK_HH
