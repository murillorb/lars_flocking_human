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

#include <iostream>
#include <QApplication>

#include <WRCoach/wrcoach.hh>
#include <WRCoach/exithandler.hh>
#include <WRCoach/instancechecker.hh>

int adjTeam = 0;

Competitions::Competition validateCategory(const QString &input, bool *valid) {
    *valid = true;

    if(input.toLower()=="ssl")
        return Competitions::SSL;
    else if(input.toLower()=="vss")
        return Competitions::VSS;
    else {
        *valid = false;
        return Competitions::SSL; // return default
    }
}

Colors::Color validateTeamColor(const QString &input, bool *valid) {
    *valid = true;

    if(input.toLower()=="yellow")
        return Colors::YELLOW;
    else if(input.toLower()=="blue")
        return Colors::BLUE;
    else {
        *valid = false;
        if (adjTeam)
            return Colors::BLUE;
        else
            return Colors::YELLOW; // return default
    }
}

FieldSide validateFieldSide(const QString &input, bool *valid) {
    *valid = true;

    if(input.toLower()=="right")
        return Sides::RIGHT;
    else if(input.toLower()=="left")
        return Sides::LEFT;
    else {
        *valid = false;
        if (adjTeam)
            return Sides::LEFT;
        else
            return Sides::RIGHT; // return default
    }
}

bool validateEnableGUI(const QString &input, bool *valid) {
    *valid = true;

    if(input.toLower()=="true")
        return true;
    else if(input.toLower()=="false")
        return false;
    else {
        *valid = false;
        return true; // return default
    }
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("WRCoach");
    app.setApplicationVersion("3.4.0");

    // Duplicated instance checking
    //InstanceChecker::waitIfDuplicated(app.applicationName());

    // Command line parser, get arguments
    QCommandLineParser parser;
    parser.setApplicationDescription("WRCoach application help.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("category", "Sets the category ('ssl' or 'vss', default='ssl').");
    parser.addPositionalArgument("teamColor", "Sets the team color ('yellow' or 'blue', default='yellow').");
    parser.addPositionalArgument("fieldSide", "Sets the field side ('right' or 'left', default='right').");
    parser.addPositionalArgument("enableGUI", "Enable or disable the GUI ('true' or 'false', default='true').");
    parser.process(app);
    QStringList args = parser.positionalArguments();

    // WRCoach parameters (with default values)
    Competitions::Competition competition = Competitions::SSL;
    quint8 ourTeamId = 0;
    Colors::Color ourTeamColor = Colors::YELLOW;
    FieldSide ourFieldSide = Sides::RIGHT;

    bool enableGUI = true;

    // Check arguments
    // Category
    if(args.size() >= 1) {
        bool valid;
        competition = validateCategory(args.at(0), &valid);
        if(valid==false) {
            std::cout << ">> WRCoach: Invalid category argument '" << args.at(0).toStdString() << "'.\n>> Please check help below.\n\n";
            parser.showHelp();
            return EXIT_FAILURE;
        }
    }

    // Team color
    if(args.size() >= 2) {
        bool valid;
        ourTeamColor = validateTeamColor(args.at(1), &valid);
        if(valid==false) {
            std::cout << ">> WRCoach: Invalid team color argument '" << args.at(1).toStdString() << "'.\n>> Please check help below.\n\n";
            parser.showHelp();
            return EXIT_FAILURE;
        }
    }

    // Field side
    if(args.size() >= 3) {
        bool valid;
        ourFieldSide = validateFieldSide(args.at(2), &valid);
        if(valid==false) {
            std::cout << ">> WRCoach: Invalid field side argument '" << args.at(2).toStdString() << "'.\n>> Please check help below.\n\n";
            parser.showHelp();
            return EXIT_FAILURE;
        }
    }

    // Enable GUI
    if(args.size() >= 4) {
        bool valid;
        enableGUI = validateEnableGUI(args.at(3), &valid);
        if(valid==false) {
            std::cout << ">> WRCoach: Invalid enable GUI argument '" << args.at(2).toStdString() << "'.\n>> Please check help below.\n\n";
            parser.showHelp();
            return EXIT_FAILURE;
        }
    }

    // Setup ExitHandler
    ExitHandler::setup(&app);

    // Create and start WRCoach
    WR::WRCoach wrcoach(ourTeamId, ourTeamColor, ourFieldSide, competition, enableGUI);
    wrcoach.start();

    // Block main thread
    int retn = app.exec();

    // Stop WRCoach
    wrcoach.stop();

    return retn;
}
