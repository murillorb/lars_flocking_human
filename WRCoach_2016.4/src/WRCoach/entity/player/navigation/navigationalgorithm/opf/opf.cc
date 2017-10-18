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

#include "opf.hh"
#include <WRCoach/const/opfconstants.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

int OPF::_count = 0;

OPF::OPF() {
    const float width = OPFConstants::gridWidth();
    const float height = OPFConstants::gridHeight();
    const float widthRes = OPFConstants::gridWidthRes();
    const float heightRes = OPFConstants::gridHeightRes();

    _opfe = OPFE_new(200, 200, 200, 200, width/widthRes+0.5, height/heightRes+0.5, 0, 0);
    this->_height = height;
    this->_width = width;
    this->_widthRes = widthRes;
    this->_heightRes = heightRes;
    this->_maxError = 1e-5;
    this->_maxIterations = 15e3;
    this->_cpoPathNumber = 0;
    _count++;
}

OPF::~OPF() {
    OPFE_delete(_opfe);
}

void OPF::reset() {
    _mOpfe.lock();
    OPFE_clearListOfGoal(_opfe);
    OPFE_clearListOfObstacle(_opfe);
    OPFE_clearListOfEnemyRobot(_opfe);
    OPFE_clearListOfOwnRobot(_opfe);
    //OPFE_clearGridWithStartPotential_UNSAFE(_opfe);
    _mOpfe.unlock();
}

void OPF::setOrigin(const Position &pos, const Angle &ori, const Velocity &vel) {
    _centerPos = position2pos(pos);
}

void OPF::setGoal(const Position &pos, const Angle &angle) {
    Pos aux = position2pos(pos);
    _mOpfe.lock();
    OPFE_addGoal(_opfe, &aux);
    OPFE_setVector(_opfe, angle.value());
    _mOpfe.unlock();
}

void OPF::addOwnRobot(const Position &pos, const Velocity &vel) {
    Pos aux = position2pos(pos);
    _mOpfe.lock();
    OPFE_addOwnRobot(_opfe, &aux);
    _mOpfe.unlock();
}

void OPF::addEnemyRobot(const Position &pos, const Velocity &vel) {
    Pos aux = position2pos(pos);
    _mOpfe.lock();
    OPFE_addEnemyRobot(_opfe, &aux);
    _mOpfe.unlock();
}

void OPF::addBall(const Position &pos, const Velocity &vel) {
    Pos aux = position2pos(pos);
    _mOpfe.lock();
    //OPFE_addEnemyRobot(_opfe, &aux);
    OPFE_addObstacle(_opfe, &aux);
    _mOpfe.unlock();
}

Angle OPF::getDirection() {
    _mOpfe.lock();

    // Process
    //OPFE_solveIE(_opfe, OPFE_gsc, _maxInteracoes, _maxErro);
    OPFE_solveIE(_opfe, OPFE_gsc, _maxIterations, _maxError, OPFE_printTimeAtEnd);


    // Result
    double result =
            atan2( -OPFE_getPotential_UNSAFE(_opfe, _centerPos.x__, _centerPos.y__+2)+OPFE_getPotential_UNSAFE(_opfe, _centerPos.x__, _centerPos.y__-2),
                    OPFE_getPotential_UNSAFE(_opfe, _centerPos.x__-2, _centerPos.y__)-OPFE_getPotential_UNSAFE(_opfe, _centerPos.x__+2, _centerPos.y__));


    //	Pos pres2;
    //	OPFE_getGradientDescent(_opfe,&_centerPos,&pres2);
    //	double res2 = atan2(pres2.y__,pres2.x__);
    //
    //	cout << "result: " << result << "   pres2: " << res2 << endl;

    mExtern.lock();
    _cpoPathNumber = _getPath(_cpoPath, 1000);
    mExtern.unlock();

    _mOpfe.unlock();

    return Angle(true, result);
}

int OPF::_getPath(Pos* path, int max) {
    int nsteps = OPFE_getPathFrom(_opfe, max, &_centerPos, path);
    return nsteps;
}

Pos OPF::position2pos(const Position &pos) {
    Pos result;
    result.x__ = (int)((pos.x()+_width/2)/(_widthRes));
    result.y__ = (int)((pos.y()+_height/2)/(_heightRes));
    return result;
}

QLinkedList<Position> OPF::getPath() {
    QLinkedList<Position> retn;
    mExtern.lock();
    for(int c = 0; c < _cpoPathNumber; c++)
        retn.append(Position(true, _cpoPath[c].x__, _cpoPath[c].y__, 0));
    mExtern.unlock();

    return retn;
}

float OPF::getDistance() {
    double dist = 0;
    mExtern.lock();
    for(int c=1;  c<_cpoPathNumber; c++) {
        if(_cpoPath[c].x__ == _cpoPath[c-1].x__)
            dist+=_widthRes;
        else if(_cpoPath[c].y__ == _cpoPath[c-1].y__)
            dist+=_heightRes;
        else
            //dist+=WR::Utils::fastSQRT(widthRes*widthRes+heightRes*heightRes);
            dist+=sqrt(_widthRes*_widthRes+_heightRes*_heightRes);
    }
    mExtern.unlock();
    return dist;
}
