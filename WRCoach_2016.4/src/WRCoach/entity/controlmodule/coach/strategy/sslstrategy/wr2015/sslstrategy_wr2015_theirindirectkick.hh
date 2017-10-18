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

#ifndef SSLSTRATEGY_WR2015_THEIRINDIRECTKICK_HH
#define SSLSTRATEGY_WR2015_THEIRINDIRECTKICK_HH

#include <WRCoach/entity/controlmodule/coach/strategy/strategystate.hh>

class SSLStrategy_WR2015_TheirIndirectKick : public StrategyState {
private:
    // Playbooks
	Playbook_Goal *_pb_goal;
    Playbook_Defense *_pb_defense;

    // Internal
    bool isMarkable(quint8 toMarkID);

	void configure(int numOurPlayers);
	void run(int numOurPlayers);
public:
	SSLStrategy_WR2015_TheirIndirectKick();
	QString name();
};
#endif // SSLSTRATEGY_WR2015_THEIRINDIRECTKICK_HH
