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

#ifndef PLAYBOOK_OURKICKOFF_HH
#define PLAYBOOK_OURKICKOFF_HH

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>

class Playbook_OurKickOff : public Playbook {
private:
    // Parameters
    qint8 _kicker;

    // Behaviors
    Behavior_Pos *_bh_kickerPos;
    Behavior_Attack *_bh_kickerAtk;
    QList<Behavior_Pos*> _bh_assistsPos;

    // Internal
    Position _assistantPos;

	void configure(int numPlayers);
	void run(int numPlayers);
public:
	Playbook_OurKickOff();
	QString name();

    int maxNumPlayer();

    void setKicker(quint8 kicker) { _kicker = kicker; }
};

#endif // PLAYBOOK_OURKICKOFF_HH
