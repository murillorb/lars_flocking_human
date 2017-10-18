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

#include "playbook_slacs.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#include <math.h>

QString Playbook_Slacs::name() {
    return "Playbook_Slacs";
}

Playbook_Slacs::Playbook_Slacs() {
    setEntityDistance(0.6);
    setMaxBeam(20);
    setSamplesDistance(0.1);
    setWGNoiseGain(0.0);
    setEntityId(0);
}

int Playbook_Slacs::maxNumPlayer() {
    return INT_MAX;
}

void Playbook_Slacs::configure(int numPlayers) {
    _bh_followentity.clear();
    _bh_donothing.clear();
    for(quint8 i=0; i<numPlayers; i++) {
        Behavior_FollowEntity *bh_followentity = new Behavior_FollowEntity();
        usesBehavior(bh_followentity);
        _bh_followentity.push_back(bh_followentity);
    }
    _bh_goto.clear();
    for(quint8 i=0; i<numPlayers; i++) {
        Behavior_GoTo *bh_goto = new Behavior_GoTo();
        usesBehavior(bh_goto);
        _bh_goto.push_back(bh_goto);
    }
}

void Playbook_Slacs::run(int numPlayers) {
    // Parameters
    float beamStep = 2*M_PI/_maxBeam;

    // Generate samples with white gaussian noise
    QList<Position> samples[numPlayers][_maxBeam];
    for(int i=0; i<numPlayers; i++) {
        for(int j=0; j<_maxBeam; j++) {
            for(int k=0; true; k++) {
                Position samplePos(true,
                                   PlayerBus::ourPlayer(i)->position().x() + k*_samplesDistance*cos(j*beamStep + PlayerBus::ourPlayer(i)->orientation().value()) + whiteGaussianNoise(_noiseGain),
                                   PlayerBus::ourPlayer(i)->position().y() + k*_samplesDistance*sin(j*beamStep + PlayerBus::ourPlayer(i)->orientation().value()) + whiteGaussianNoise(_noiseGain),
                                   0.0);

                // If there's an obstacle, break!
                if(sampleIsValid(i, samplePos, numPlayers)==false)
                    break;

                samples[i][j].push_back(samplePos);
            }
        }
    }

    // Allocate the samples to the correct player
    QList<Position> allocatedSamples[numPlayers];
    for(int i=0; i<numPlayers; i++) {
        for(int j=0; j<_maxBeam; j++) {
            for(int k=0; k<samples[i][j].size(); k++) {
                float minDistance = WR::Utils::distance(samples[i][j].at(k), PlayerBus::ourPlayer(0)->position());
                int minDistanceIndex = 0;
                for(int l=1; l<numPlayers; l++) {
                    float distance = WR::Utils::distance(samples[i][j].at(k), PlayerBus::ourPlayer(l)->position());
                    if(distance<minDistance) {
                        minDistance = distance;
                        minDistanceIndex = l;
                    }
                }
                allocatedSamples[minDistanceIndex].push_back(samples[i][j].at(k));
            }
        }
    }

    // Calculate sample's average and set behaviors
    for(int i=0; i<numPlayers; i++) {
        float xSum = 0.0;
        float ySum = 0.0;
        for(int j=0; j<allocatedSamples[i].size(); j++) {
            xSum += allocatedSamples[i].at(j).x();
            ySum += allocatedSamples[i].at(j).y();
        }

        Position posToGo(true, xSum/(float)allocatedSamples[i].size(), ySum/(float)allocatedSamples[i].size(), 0.0);

        float distance = WR::Utils::distance(PlayerBus::ourPlayer(i)->position(), PlayerBus::theirPlayer(_entityId)->position());

        float lineAtan = atan2((PlayerBus::ourPlayer(i)->position().y()) - PlayerBus::theirPlayer(_entityId)->position().y(),((PlayerBus::ourPlayer(i)->position().x()) - PlayerBus::theirPlayer(_entityId)->position().x()));

        float referencePosition[2], distToDest[2];
        referencePosition[0] = PlayerBus::theirPlayer(_entityId)->position().x() + _distEntity*cos(lineAtan);
        referencePosition[1] = PlayerBus::theirPlayer(_entityId)->position().y() + _distEntity*sin(lineAtan);



      //  if(distance > _distEntity) {
            posToGo = Position(true,
                               ((1-_forceWeight)*posToGo.x() + _forceWeight*referencePosition[0]),//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().x()),
                               ((1-_forceWeight)*posToGo.y() + _forceWeight*referencePosition[1]),//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().y()),
                               0.0);

        distToDest[0] = ((1-_forceWeight)*posToGo.x() + _forceWeight*referencePosition[0]) - PlayerBus::ourPlayer(i)->position().x();
        distToDest[1] = ((1-_forceWeight)*posToGo.y() + _forceWeight*referencePosition[1]) - PlayerBus::ourPlayer(i)->position().y();

        if (pow((distToDest[0]*distToDest[0])+(distToDest[1]*distToDest[1]),0.5) < 0.2)
            posToGo = Position(true,
                               PlayerBus::ourPlayer(i)->position().x(),//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().x()),
                               PlayerBus::ourPlayer(i)->position().y(),//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().y()),
                               0.0);

        Behavior_GoTo *bh_goto = _bh_goto.at(i);
        setPlayerBehavior(i, bh_goto);
        bh_goto->setDestination(posToGo);
        bh_goto->setPositionToLook(PlayerBus::theirPlayer(_entityId)->position());
    }
}

bool Playbook_Slacs::sampleIsValid(int id, const Position &pos, int numPlayers) {
    if((pos.isValid()==true) &&
       (WR::Utils::distance(PlayerBus::ourPlayer(id)->position(), pos) <= _laserRange) &&
       (WR::Utils::distance(PlayerBus::theirPlayer(_entityId)->position(),pos) > 0.3)) {
        for(int i=0; i<numPlayers; i++) {
            if(WR::Utils::distance(pos, PlayerBus::ourPlayer(i)->position()) < 0.01)
                return false;
            if(PlayerBus::theirPlayerAvailable(i)) {
                if(WR::Utils::distance(pos, PlayerBus::theirPlayer(i)->position()) < 0.005)
                    return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

float Playbook_Slacs::whiteGaussianNoise(float noiseGain) {
    float r1 = (rand()%10000)/10000.0;
    float r2 = (rand()%10000)/10000.0;
    return (pow(-2*noiseGain*log(r1),0.5)*cos(2*M_PI*r2));
}
