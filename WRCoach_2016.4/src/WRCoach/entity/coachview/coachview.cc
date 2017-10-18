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

#include "coachview.hh"
#include <WRCoach/gui/soccerview/soccerview.hh>

#define COACHVIEW_LOOPFREQUENCY 30 // Hz

QString CoachView::name() {
    return "CoachView";
}

CoachView::CoachView(const Competitions::Competition &compType, WRTeam *ourTeam, WRTeam *opTeam) : Entity(ENT_GUI) {
    _ourTeam = ourTeam;
	_opTeam = opTeam;

    _view = new GLSoccerView(compType);
	_view->show();
}

CoachView::~CoachView() {
	_view->close();
	delete _view;
}

void CoachView::initialization() {
    this->setLoopFrequency(COACHVIEW_LOOPFREQUENCY);
}

void CoachView::loop() {
	_view->updateDetection(_ourTeam, _opTeam);
    _view->updateFieldGeometry(_ourTeam->loc(), _compType);
}

void CoachView::finalization() {

}
