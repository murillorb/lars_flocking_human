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

#include "vssskill_takeout.hh"
#include <WRCoach/entity/player/skill/wrskill.hh>

QString VSSSkill_Takeout::name() {
	return "VSSSkill_Takeout";
}

VSSSkill_Takeout::VSSSkill_Takeout() {
    _canRun = false;
}

void VSSSkill_Takeout::run() {

    // Reset PID calibration
    player()->setLinearPIDParameters(3.5, 0.0, 0.2, 0.0);
    player()->setMaxLSpeed(0.9);
    player()->setMaxLAcceleration(0.75);

    // Check ball inline
    Position pos1(true, player()->position().x(), loc()->ourGoalRightPost().y(), 0.0);
    Position pos2(true, player()->position().x(), loc()->ourGoalLeftPost().y(), 0.0);

    float distGoalLine = WR::Utils::distanceToLine(pos1, pos2, loc()->ball());
    if(distGoalLine < 1.10*player()->robotLength()/2) { // ball inline

        // Calc position
        float signal = (loc()->ball().y()<player()->position().y()? -1 : 1);
        Position desiredPos(true, player()->position().x(), signal*loc()->fieldMaxY(), 0.0);

        // Run fast to ball
        player()->goTo(desiredPos, false, false, false);

        _canRun = false;

    } else { // ball not inline
        /// [TODO] refs #422
        /// Descrição: Não há nenhum tratamento para caso o player esteja mais longe do próprio gol que a bola, o que pode fazer ele cometer um gol contra.

        // Specific PID calibration
        player()->setLinearPIDParameters(3.0, 0.0, 0.0, 0.0);
        player()->setMaxLSpeed(0.6);
        player()->setMaxLAcceleration(0.6);

        Position desiredPos = WR::Utils::threePoints(loc()->ball(), player()->position(), 0.20, GEARSystem::Angle::pi);
        player()->goTo(desiredPos, false, false, false);
    }

}
