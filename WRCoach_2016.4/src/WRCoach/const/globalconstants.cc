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

#include "globalconstants.hh"

bool GlobalConstants::_competitionSet = false;
float GlobalConstants::_minBallDistToKick = 0.0f;
float GlobalConstants::_gameOffMaxLSpeed = 0.0f;

// Competition independent default values
Competitions::Competition GlobalConstants::_competition = Competitions::SSL;
int GlobalConstants::_threadLoopFrequency = 60; // Hz
float GlobalConstants::_maxKickPower = 8.0f;
float GlobalConstants::_ballRadius = 0.03f;
float GlobalConstants::_maxPushDistance = 0.9f;
float GlobalConstants::_highDistance = 1000.0f;

void GlobalConstants::setCompetition(Competitions::Competition competition) {
    using namespace Competitions;

    // Avoid setting competition two times
    if(GlobalConstants::_competitionSet==true)
        return;

    // Set constants based on competition
    switch(competition) {
        default:
        case SSL:
            GlobalConstants::_minBallDistToKick = 0.25f;
            GlobalConstants::_gameOffMaxLSpeed = 1.5f;
        break;
        case VSS:
            GlobalConstants::_minBallDistToKick = 0.0f;
            GlobalConstants::_gameOffMaxLSpeed = 0.5f;
        break;
    }

    GlobalConstants::_competition = competition;
    GlobalConstants::_competitionSet = true;
}
