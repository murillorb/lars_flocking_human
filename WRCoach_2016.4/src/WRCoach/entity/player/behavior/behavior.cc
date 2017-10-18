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

#include "behavior.hh"
#include <WRCoach/entity/player/skill/skill.hh>

/// REMOVE
#include <WRCoach/entity/player/player.hh>

Behavior::Behavior() {
    _skill = NULL;
    _configureEnabled = false;

    _player = NULL;
    _utils = NULL;
    _playerAccess = NULL;
    _loc = NULL;

    _initialized = false;
}

Behavior::~Behavior() {
    // Delete skills transitions
    QList<SkillTransition*> transitions = _transitionTable.values();
    QList<SkillTransition*>::iterator it;
    for(it=transitions.begin(); it!=transitions.end(); it++)
        delete *it;
    _transitionTable.clear();

    // Delete skills
    QList<Skill*>::iterator it2;
    for(it2=_skillList.begin(); it2!=_skillList.end(); it2++)
        delete *it2;
    _skillList.clear();
}

void Behavior::initialize(Locations *loc) {
    _loc = loc;

    // Configurate skills to be used
    _configureEnabled = true;
    configure();
    _configureEnabled = false;

    _initialized = true;
}

void Behavior::setPlayer(Player *player, PlayerUtils *utils, PlayerAccess *playerAccess) {
    _player = player;
    _utils = utils;
    _playerAccess = playerAccess;
}

void Behavior::runBehavior() {
    // Check if behavior has at least one skill set
    if(_skillList.size()==0) {
        std::cout << "[ERROR] " << name().toStdString() << " has no skills set!\n";
        return;
    }

    // Run behavior (implemented by child)
    run();

    // Run skill (implemented by skill)
    if(_skill->isInitialized()==false)
        _skill->initialize(_loc);
    _skill->setPlayer(_player, _utils);
    _skill->runSkill();
}

void Behavior::usesSkill(Skill *skill) {
    if(_configureEnabled==false) {
        std::cout << "[WARNING] Blocked '" << name().toStdString() << "' setting Skill to use outside configure().\n";
        return;
    }

    if(_skillList.contains(skill)==false) {
        // Add to list
        _skillList.push_back(skill);

        // If setting first skill, use as initial state (default)
        if(_skill==NULL)
            _skill = skill;
    }
}

void Behavior::setInitialSkill(Skill *skill) {
    // Set initial skill
    if(_skillList.contains(skill))
        _skill = skill;
    else
        std::cout << "[ERROR] " << name().toStdString() << " setting initial skill '" << skill->name().toStdString() << "' that isn't at skill list!\n";
}

void Behavior::addTransition(int id, Skill *source, Skill *target) {
    // Check if isn't a invalid transition (same source for same id)
    if(_transitionTable.contains(id)) {
        QList<SkillTransition*> transitions = _transitionTable.values(id);
        QList<SkillTransition*>::const_iterator it;
        for(it=transitions.constBegin(); it!=transitions.constEnd(); it++) {
            const SkillTransition *transition = *it;
            if(transition->source()==source) {
                std::cout << "[ERROR] " << name().toStdString() << " adding invalid transition from " << source->name().toStdString() << " to " << target->name().toStdString() << "; source already has a transition with id " << id << "!\n";
                return;
            }
        }
    }

    // Add transition to transitionTable
    SkillTransition *tmpTransition = new SkillTransition(source, target);
    _transitionTable.insertMulti(id, tmpTransition);
}

void Behavior::enableTransition(int id) {
    // Check if transition exists
    if(_transitionTable.contains(id)==false) {
        std::cout << "[ERROR] " << name().toStdString() << ", enabling transition id '" << id << "' not found!\n";
        return;
    }

    // Get transitions in transition table
    QList<SkillTransition*> transitions = _transitionTable.values(id);

    // For each transition, add transition and emit signal
    QList<SkillTransition*>::const_iterator it;
    for(it=transitions.constBegin(); it!=transitions.constEnd(); it++) {
        const SkillTransition *transition = *it;

        // Check if curr state is source state
        if(_skill!=transition->source())
            continue;

        // Do transition
        _skill = transition->target();

        break;
    }
}

PlayerAccess* Behavior::player() {
    if(_playerAccess==NULL)
        std::cout << "[ERROR] " << name().toStdString() << ", requesting player(), playerAccess not set!\n";
    return _playerAccess;
}

Locations* Behavior::loc() {
    if(_loc==NULL)
        std::cout << "[ERROR] " << name().toStdString() << ", requesting loc(), loc not initialized!\n";
    return _loc;
}

PlayerUtils* Behavior::utils() {
    if(_utils==NULL)
        std::cout << "[ERROR] " << name().toStdString() << ", requesting utils(), utils not set!\n";
    return _utils;
}
