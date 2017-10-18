#
# Warthog Robotics
# University of Sao Paulo (USP) at Sao Carlos
# http://www.warthog.sc.usp.br/
#
# This file is part of WRCoach project.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# Application info
TEMPLATE = app
DESTDIR = bin
TARGET = WRCoach
VERSION = 3.4.0
CONFIG -= c++11

# Qt config
CONFIG += console
CONFIG -= app_bundle
QT += core \
    gui \
    widgets \
    network \
    opengl

# Optimization flags
QMAKE_CXXFLAGS  -= -O
QMAKE_CXXFLAGS  -= -O1
QMAKE_CXXFLAGS  += -O2 -ansi

# GEARSystem info
LIBS += -lomniORB4 \
    -lomnithread \
    -lGEARSystem \
    -lprotobuf \
    -lGLU

# Temporary dirs
OBJECTS_DIR = tmp/obj
MOC_DIR = tmp/moc
UI_DIR = tmp/moc
RCC_DIR = tmp/rc

# Source files
INCLUDEPATH *= . src

SOURCES += \
    src/WRCoach/entity/player/vssplayer/vssplayer.cc \
    src/WRCoach/const/globalconstants.cc \
    src/WRCoach/utils/utils.cc \
    src/WRCoach/entity/entity.cc \
    src/WRCoach/entity/player/player.cc \
    src/WRCoach/entity/player/navigation/navigation.cc \
    src/WRCoach/entity/player/behavior/behavior.cc \
    src/WRCoach/entity/player/behavior/basics/behavior_donothing.cc \
    src/WRCoach/entity/locations.cc \
    src/WRCoach/entity/player/playerutils.cc \
    src/WRCoach/entity/referee/referee.cc \
    src/WRCoach/exithandler.cc \
    src/WRCoach/entity/player/skill/skill.cc \
    src/WRCoach/entity/player/skill/basics/skill_donothing.cc \
    src/WRCoach/entity/player/skill/basics/skill_goto.cc \
    src/WRCoach/const/pfconstants.cc \
    src/WRCoach/const/opfconstants.cc \
    src/WRCoach/const/sensorconstants.cc \
    src/WRCoach/entity/player/behavior/basics/behavior_followball.cc \
    src/WRCoach/entity/player/behavior/basics/behavior_pos.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_be.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_hpf.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_opf.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/opf.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/pf/pf.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/navigationalgorithm.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/fana/fana.cc \
    src/WRCoach/utils/knn/knn.cc \
    src/WRCoach/utils/freeangles/freeangles.cc \
    src/WRCoach/utils/freeangles/obstacle.cc \
    src/WRCoach/utils/line/line.cc \
    src/WRCoach/entity/player/playerbus.cc \
    src/WRCoach/entity/player/playeraccess.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf-threaded/lib/libopf-threaded.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf-threaded/opf-threaded.cc \
    src/WRCoach/entity/player/sslplayer/sslplayer.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_gk.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_attack.cc \
    src/WRCoach/entity/player/skill/vss/vssskill_takeout.cc \
    src/WRCoach/entity/player/skill/vss/vssskill_pushball.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_defense.cc \
    src/WRCoach/entity/player/behavior/test/behavior_followplayer.cc \
    src/WRCoach/entity/player/skill/ssl/skill_interceptball.cc \
    src/WRCoach/entity/player/skill/ssl/skill_pushball.cc \
    src/WRCoach/entity/player/skill/ssl/skill_takeout.cc \
    src/WRCoach/entity/player/skill/ssl/skill_kick.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_attack.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_gk.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_gka.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_markball.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_markballa.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_markplayer.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_penaltyassistant.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_penaltygk.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_penaltykick.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_mark.cc \
    src/WRCoach/entity/player/behavior/ssl/behavior_recv.cc \
    src/WRCoach/instancechecker.cc \
    src/WRCoach/utils/basics/color.cc \
    src/WRCoach/utils/basics/competition.cc \
    src/WRCoach/utils/basics/quadrant.cc \
    src/WRCoach/utils/basics/side.cc \
    src/WRCoach/utils/basics/wall.cc \
    src/WRCoach/utils/fieldside/fieldside.cc \
    src/WRCoach/utils/wrtimer/wrtimer.cc \
    src/main.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/uvf/uvf.cc \
    src/WRCoach/entity/controlmodule/controlmodule.cc \
    src/WRCoach/entity/world/world.cc \
    src/WRCoach/entity/world/worldmapupdater.cc \
    src/WRCoach/utils/fields/field_ssl2014.cc \
    src/WRCoach/utils/fields/field_ssl2015.cc \
    src/WRCoach/utils/fields/field_vss2008.cc \
    src/WRCoach/utils/fields/fields.cc \
    src/WRCoach/entity/player/navigation/controlalgorithm/controlalgorithm.cc \
    src/WRCoach/entity/player/navigation/controlalgorithm/pid/pid.cc \
    src/WRCoach/entity/player/navigation/controlalgorithm/controlalgorithmparameters.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_attacka.cc \
    src/WRCoach/entity/player/navigation/controlalgorithm/cascadepid/cascadepid.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_gka.cc \
    src/WRCoach/entity/player/skill/basics/skill_spin.cc \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_freeball.cc \
    src/WRCoach/const/uvfconstants.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrt.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtpoint.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtobstacle.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/uvf/uvfobstacle.cc \
    src/3rdparty/referee.pb.cc \
    src/WRCoach/entity/referee/sslreferee/sslgameinfo.cc \
    src/WRCoach/entity/referee/sslreferee/sslreferee.cc \
    src/WRCoach/entity/sensor/ballmovimentdetect/sensor_ballmovimentdetect.cc \
    src/WRCoach/entity/sensor/sensor.cc \
    src/WRCoach/gui/soccerview/util/field.cc \
    src/WRCoach/gui/soccerview/gltext.cc \
    src/WRCoach/gui/soccerview/soccerview.cc \
    src/WRCoach/entity/coachview/coachview.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/basics/playbook_donothing.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/basics/playbook_followball.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/basics/playbook_pos.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_attack.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_defense.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_goal.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_ourkickoff.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_ourpenalty.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_theirpenalty.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_attack.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_attacka.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_defense.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_freeball.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_goal.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/playbook.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_gameoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_gameon.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_halt.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourdirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourindirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourkickoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourpenalty.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirdirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirindirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirkickoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirpenalty.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_gameoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_gameon.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_halt.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_gameoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_gameon.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_halt.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourdirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourindirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourkickoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourpenalty.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirdirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirindirectkick.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirkickoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirpenalty.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/sslstrategy.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_gameoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_gameon.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_halt.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_freeball.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_gameoff.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_gameon.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_gameona.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_halt.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/vssstrategy.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/strategy.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/strategystate.cc \
    src/WRCoach/entity/controlmodule/coach/coach.cc \
    src/WRCoach/entity/controlmodule/coach/coachutils.cc \
    src/WRCoach/entity/controlmodule/coach/playersdistribution.cc \
    src/WRCoach/entity/controlmodule/coach/wrteam.cc \
    src/WRCoach/wrcoach.cc \
    src/WRCoach/utils/graph/graph.cc \
    src/WRCoach/utils/graph/vertex.cc \
    src/WRCoach/utils/graph/edge.cc \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtvertex.cc \
    src/WRCoach/entity/player/navigation/controlalgorithm/discretepid/discretepid.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/slacs/playbook_slacs.cc \
    src/WRCoach/entity/player/behavior/slacs/behavior_followentity.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/slacs/sslstrategy_slacs.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/slacs/sslstrategy_slacs_gameon.cc \
    src/WRCoach/entity/player/behavior/slacs/behavior_goto.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/flock/playbook_flock.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/flock/sslstrategy_flock.cc \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/flock/sslstrategy_flock_gameon.cc \
    src/WRCoach/entity/controlmodule/coach/playbook/flock/assymetricgaussfunction.cpp

# Header files
HEADERS += \
    src/WRCoach/entity/player/vssplayer/vssplayer.hh \
    src/WRCoach/const/globalconstants.hh \
    src/WRCoach/utils/utils.hh \
    src/WRCoach/entity/player/sslplayer/sslplayer.hh \
    src/WRCoach/entity/entity.hh \
    src/WRCoach/entity/player/player.hh \
    src/WRCoach/entity/player/navigation/navigation.hh \
    src/WRCoach/entity/baseentity.hh \
    src/WRCoach/entity/player/baseplayer.hh \
    src/WRCoach/entity/player/behavior/behavior.hh \
    src/WRCoach/entity/player/behavior/basics/behavior_donothing.hh \
    src/WRCoach/entity/player/behavior/wrbehaviors.hh \
    src/WRCoach/entity/player/navigation/wrnavigation.hh \
    src/WRCoach/entity/locations.hh \\
    src/WRCoach/entity/player/playerutils.hh \
    src/WRCoach/entity/referee/referee.hh \
    src/WRCoach/exithandler.hh \
    src/WRCoach/entity/player/skill/skill.hh \
    src/WRCoach/entity/player/skill/basics/skill_donothing.hh \
    src/WRCoach/entity/player/skill/wrskills.hh \
    src/WRCoach/entity/player/skill/basics/skill_goto.hh \
    src/WRCoach/const/pfconstants.hh \
    src/WRCoach/const/opfconstants.hh \
    src/WRCoach/const/sensorconstants.hh \
    src/WRCoach/entity/player/behavior/basics/behavior_followball.hh \
    src/WRCoach/entity/player/behavior/basics/behavior_pos.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_be.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_hpf.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/lib_opf.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/queue.h \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/lib/utils.h \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf/opf.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/pf/pf.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/pf/vector.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/navigationalgorithm.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/fana/fana.hh \
    src/WRCoach/utils/knn/knn.hh \
    src/WRCoach/utils/freeangles/freeangles.hh \
    src/WRCoach/utils/freeangles/obstacle.hh \
    src/WRCoach/entity/player/skill/wrskillsbase.hh \
    src/WRCoach/entity/player/behavior/wrbehaviorsbase.hh \
    src/WRCoach/utils/line/line.hh \
    src/WRCoach/entity/player/playerbus.hh \
    src/WRCoach/entity/player/playeraccess.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/opf-threaded/opf-threaded.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_gk.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_attack.hh \
    src/WRCoach/entity/player/skill/vss/vssskill_takeout.hh \
    src/WRCoach/entity/player/skill/vss/vssskill_pushball.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_defense.hh \
    src/WRCoach/entity/player/behavior/test/behavior_followplayer.hh \
    src/WRCoach/entity/player/skill/ssl/skill_interceptball.hh \
    src/WRCoach/entity/player/skill/ssl/skill_pushball.hh \
    src/WRCoach/entity/player/skill/ssl/skill_takeout.hh \
    src/WRCoach/entity/player/skill/ssl/skill_kick.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_attack.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_gk.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_gka.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_markball.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_markballa.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_markplayer.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_penaltyassistant.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_penaltygk.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_penaltykick.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_mark.hh \
    src/WRCoach/entity/player/skill/wrskill.hh \
    src/WRCoach/entity/player/behavior/wrbehavior.hh \
    src/WRCoach/entity/player/behavior/ssl/behavior_recv.hh \
    src/WRCoach/instancechecker.hh \
    src/WRCoach/utils/basics/color.hh \
    src/WRCoach/utils/basics/competition.hh \
    src/WRCoach/utils/basics/quadrant.hh \
    src/WRCoach/utils/basics/side.hh \
    src/WRCoach/utils/basics/wall.hh \
    src/WRCoach/utils/fieldside/fieldside.hh \
    src/WRCoach/utils/wrtimer/wrtimer.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/uvf/uvf.hh \
    src/WRCoach/entity/controlmodule/controlmodule.hh \
    src/WRCoach/entity/world/world.hh \
    src/WRCoach/entity/world/worldmapupdater.hh \
    src/WRCoach/utils/fields/field_ssl2014.hh \
    src/WRCoach/utils/fields/field_ssl2015.hh \
    src/WRCoach/utils/fields/field_vss2008.hh \
    src/WRCoach/utils/fields/fields.hh \
    src/WRCoach/utils/fields/wrfields.hh \
    src/WRCoach/entity/player/navigation/controlalgorithm/controlalgorithm.hh \
    src/WRCoach/entity/player/navigation/controlalgorithm/pid/pid.hh \
    src/WRCoach/entity/player/navigation/controlalgorithm/controlalgorithmparameters.hh \
    src/WRCoach/entity/player/navigation/controlalgorithm/wrcontrolalgorithmbase.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_attacka.hh \
    src/WRCoach/entity/player/navigation/controlalgorithm/cascadepid/cascadepid.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_gka.hh \
    src/WRCoach/entity/player/skill/basics/skill_spin.hh \
    src/WRCoach/entity/player/behavior/vss/vssbehavior_freeball.hh \
    src/WRCoach/const/uvfconstants.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrt.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtpoint.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtobstacle.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/uvf/uvfobstacle.hh \
    src/3rdparty/referee.pb.h \
    src/WRCoach/entity/referee/sslreferee/sslgameinfo.hh \
    src/WRCoach/entity/referee/sslreferee/sslreferee.hh \
    src/WRCoach/entity/sensor/ballmovimentdetect/sensor_ballmovimentdetect.hh \
    src/WRCoach/entity/sensor/sensor.hh \
    src/WRCoach/entity/sensor/wrsensors.hh \
    src/WRCoach/gui/soccerview/util/field.hh \
    src/WRCoach/gui/soccerview/util/field_default_constants.hh \
    src/WRCoach/gui/soccerview/util/geometry.hh \
    src/WRCoach/gui/soccerview/util/gvector.hh \
    src/WRCoach/gui/soccerview/util/timer.hh \
    src/WRCoach/gui/soccerview/gltext.hh \
    src/WRCoach/gui/soccerview/soccerview.hh \
    src/WRCoach/entity/coachview/coachview.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/basics/playbook_donothing.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/basics/playbook_followball.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/basics/playbook_pos.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_attack.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_defense.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_goal.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_ourkickoff.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_ourpenalty.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_theirpenalty.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_attack.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_attacka.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_defense.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_freeball.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/vss/vssplaybook_goal.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/playbook.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/wrplaybookbase.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/wrplaybooks.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_gameoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_halt.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourdirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourindirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourkickoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_ourpenalty.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_states.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirdirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirindirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirkickoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/template/sslstrategy_template_theirpenalty.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_gameoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_halt.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/test/sslstrategy_test_states.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_gameoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_halt.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourdirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourindirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourkickoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_ourpenalty.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_states.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirdirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirindirectkick.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirkickoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wr2015/sslstrategy_wr2015_theirpenalty.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/sslstrategy.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/wrsslstrategy.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_gameoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_halt.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2015/vssstrategy_wr2015_states.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_freeball.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_gameoff.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_gameona.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_halt.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wr2016/vssstrategy_wr2016_states.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/vssstrategy.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/vssstrategy/wrvssstrategy.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/strategy.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/strategystate.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh \
    src/WRCoach/entity/controlmodule/coach/basecoach.hh \
    src/WRCoach/entity/controlmodule/coach/coach.hh \
    src/WRCoach/entity/controlmodule/coach/coachutils.hh \
    src/WRCoach/entity/controlmodule/coach/playersdistribution.hh \
    src/WRCoach/entity/controlmodule/coach/wrteam.hh \
    src/WRCoach/gui/soccerview/util/util.h \
    src/WRCoach/wrcoach.hh \
    src/WRCoach/utils/graph/graph.hh \
    src/WRCoach/utils/graph/vertex.hh \
    src/WRCoach/utils/graph/edge.hh \
    src/WRCoach/utils/graph/basegraph.hh \
    src/WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtvertex.hh \
    src/WRCoach/entity/player/navigation/controlalgorithm/discretepid/discretepid.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/slacs/playbook_slacs.hh \
    src/WRCoach/entity/player/behavior/slacs/behavior_followentity.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/slacs/sslstrategy_slacs.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/slacs/sslstrategy_slacs_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/slacs/sslstrategy_slacs_states.hh \
    src/WRCoach/entity/player/behavior/slacs/behavior_goto.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/flock/playbook_flock.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/flock/sslstrategy_flock.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/flock/sslstrategy_flock_gameon.hh \
    src/WRCoach/entity/controlmodule/coach/strategy/sslstrategy/flock/sslstrategy_flock_states.hh \
    src/WRCoach/entity/controlmodule/coach/playbook/flock/assymetricgaussfunction.h

# Other files
DISTFILES += \
    TODO.txt \
    README.txt \
    FAQ.txt \
    pre-build.sh \
    protobuf.sh \
    template.txt \
    LICENSE \
    src/3rdparty/referee.proto \
    src/3rdparty/messages_robocup_ssl_wrapper.proto \
    src/3rdparty/messages_robocup_ssl_refbox_log.proto \
    src/3rdparty/messages_robocup_ssl_geometry.proto \
    src/3rdparty/messages_robocup_ssl_detection.proto \
    src/3rdparty/protoc.sh
