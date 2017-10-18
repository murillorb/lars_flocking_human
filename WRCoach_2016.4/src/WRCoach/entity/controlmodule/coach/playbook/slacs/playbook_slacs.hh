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

#ifndef PLAYBOOK_SLACS_HH
#define PLAYBOOK_SLACS_HH

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>

class Playbook_Slacs : public Playbook {
private:
    // Parameters
    float _distEntity;
    int _maxBeam;
    float _samplesDistance;
    float _laserRange;
    float _noiseGain;
    int _entityId;
    float _forceWeight;

    // Behaviors
    QList<Behavior_FollowEntity*> _bh_followentity;
    QList<Behavior_DoNothing*> _bh_donothing;
    QList<Behavior_GoTo*> _bh_goto;

    QList<int> _samples;
    bool sampleIsValid(int id, const Position &pos, int numPlayers);
    float whiteGaussianNoise(float noiseGain);

    void configure(int numPlayers);
    void run(int numPlayers);
public:
    Playbook_Slacs();
    QString name();
    int maxNumPlayer();

    void setEntityDistance(float distEntity) { _distEntity = distEntity; }
    void setMaxBeam(float maxBeam) { _maxBeam = maxBeam; }
    void setSamplesDistance(float samplesDistance) { _samplesDistance = samplesDistance; }
    void setLaserRange(float laserRange) { _laserRange = laserRange; }
    void setWGNoiseGain(float noiseGain) { _noiseGain = noiseGain; }
    void setEntityId(int entityId) { _entityId = entityId; }
    void setForceWeight(float forceWeight) { _forceWeight = forceWeight; }
};

#endif // PLAYBOOK_SLACS_HH
