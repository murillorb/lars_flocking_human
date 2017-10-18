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

#ifndef SKILL_HH
#define SKILL_HH

#include <WRCoach/entity/player/baseplayer.hh>

class Skill {
public:
	Skill();
	virtual ~Skill();

	// Called one time before run is first called
	bool isInitialized() const { return _initialized; }
    void initialize(Locations *loc);
	void setPlayer(Player *player, PlayerUtils *utils);

	// Called every time before run is called; update general variables
	void runSkill();

    virtual QString name() = 0;
protected:
	Player *player() { return _player; }
	Locations *loc() { return _loc; }
	PlayerUtils *utils() { return _utils; }
private:
	virtual void run() = 0;

    // Player access
    Player *_player;
    PlayerUtils *_utils;

    // Game info
	Locations *_loc;

    // Skill initialized
	bool _initialized;
};

#endif // SKILL_HH
