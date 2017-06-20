#ifndef ALGVEC_H
#define ALGVEC_H

#include "math.h"

class AlgVec
{
public:
    AlgVec(double x, double y): x(x), y(y) {}

    int operator==(AlgVec rhs) {
        return(x == rhs.x && y == rhs.y);
    }

    AlgVec operator+(AlgVec rhs) {
        return AlgVec( x + rhs.x,
                     y + rhs.y);
    }

    AlgVec operator-(AlgVec rhs) {
        return AlgVec( x - rhs.x,
                     y - rhs.y);
    }

    AlgVec operator*(AlgVec rhs) {
        return AlgVec( x * rhs.x,
                     y * rhs.y);
    }

    AlgVec operator/(AlgVec rhs) {
        return AlgVec( x / rhs.x,
                     y / rhs.y);
    }

    AlgVec operator+(double scalar) {
        return AlgVec(x + scalar,
                      y + scalar);
    }

    AlgVec operator-(double scalar) {
        return AlgVec(x - scalar,
                      y - scalar);
    }

    AlgVec operator*(double scalar) {
        return AlgVec(x * scalar,
                      y * scalar);
    }

    AlgVec operator/(double scalar) {
        return AlgVec(x / scalar,
                      y / scalar);
    }

    float dot(AlgVec rhs) {
        return (x * rhs.x +
                y * rhs.y);
    }

    double length() {
        return double(sqrt( x*x + y*y ));
    }

    AlgVec normalize () {
        double factor = 1.0 / this->length();
        return ((*this) * factor);
    }

    double x;
    double y;
};

#endif // ALGVEC_H
