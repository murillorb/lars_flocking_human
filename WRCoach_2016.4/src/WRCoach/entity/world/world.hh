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

#ifndef WORLD_HH
#define WORLD_HH

#include <QReadWriteLock>
#include <WRCoach/entity/baseentity.hh>
#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/entity/entity.hh>
#include <WRCoach/utils/basics/competition.hh>
#include <WRCoach/utils/fields/fields.hh>

class World : public Entity {
public:
    World(Controller *ctr, Competitions::Competition competition, Fields::Field *defaultField);
    ~World();
    QString name();

    // Management
    void setTeams(WRTeam *ourTeam, WRTeam *opTeam);
    void setControlModule(ControlModule *ctrModule);

    // Entities management
    void addEntity(Entity *e, int priority);
    void removeEntity(int id);

    // WorldMap read-write synchronization
    void wmLockRead();
    void wmUnlock();

    // Controller/WorldMap access
    Controller* getController() const { return _ctr; }
    WorldMap* getWorldMap() const { return _wm; }
private:
    // Entity implementation
    void initialization();
    void loop();
    void finalization();

    // Controller access
    Controller *_ctr;

    // Teams access
    WRTeam *_ourTeam;
    WRTeam *_opTeam;

    // Control module
    ControlModule *_ctrModule;
    QMutex _ctrModuleMutex;

    // WorldMap
    WorldMap *_wm;
    WorldMapUpdater *_wmUpdater;

    // WorldMap read-write synchronization
    QReadWriteLock _wmLock;
    void wmLockWrite();

    // Entities by priority
    QMap<int,QHash<int,Entity*>*> _priorityLevels; // Map<priority, Hash<id,Entity> >

    // Internal
    void setupWorldMap();
    void stopAndDeleteEntities();
};

#endif // WORLD_HH
