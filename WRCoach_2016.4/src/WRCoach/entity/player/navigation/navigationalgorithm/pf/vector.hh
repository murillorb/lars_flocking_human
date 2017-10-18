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

#ifndef VECTOR_H_
#define VECTOR_H_

#include <cmath>
#include <cassert>

class Vector {
private:
    float _module;
    float _x;
    float _y;

    void updateModule() {
        _module = sqrt(pow(_x,2) + pow(_y,2));
    }

public:

    Vector() {
        setVector(0.0, 0.0);
    }

    Vector(float x, float y) {
        setVector(x, y);
    }

    float x() const { return _x; }
    float y() const { return _y; }
    float getModule() const { return _module; }

    void setVector(float x, float y) {
        _x = x;
        _y = y;
        updateModule();
    }

    Vector getUnitary() const {
        return (*this)/_module;
    }

    // Operators Vector-Vector
    Vector operator+(const Vector &v) const {
        return Vector(_x+v.x(), _y+v.y());
    }

    Vector operator-(const Vector &v) const {
        return Vector(_x-v.x(), _y-v.y());
    }

    float operator*(const Vector &v) const {
        return (_x * v.x()) + (_y * v.y());
    }

    // Operators Vector-float
    Vector operator+(const float &k) const {
        return Vector(_x+k, _y+k);
    }

    Vector operator-(const float &k) const {
        return Vector(_x-k, _y-k);
    }

    Vector operator*(const float &k) const {
        return Vector(_x*k, _y*k);
    }

    Vector operator/(const float &k) const {
        if(k==0)
            return Vector(0.0, 0.0);
        return Vector(_x/k, _y/k);
    }

    // Self-operators Vector-Vector
    void operator+=(const Vector &v) {
        _x += v.x();
        _y += v.y();
        updateModule();
    }

    void operator-=(const Vector &v) {
        _x -= v.x();
        _y -= v.y();
        updateModule();
    }

    // Self-operators Vector-float
    void operator*=(const float &k) {
        _x *= k;
        _y *= k;
        updateModule();
    }

    void operator/=(const double &k) {
        assert(k!=0);
        _x /= k;
        _y /= k;
        updateModule();
    }
};

#endif // VECTOR_H_
