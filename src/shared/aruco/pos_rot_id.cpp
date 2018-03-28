//
// Created by Wouter Timmermans on 18-09-17.
//

#include "pos_rot_id.h"

PosRotId::PosRotId(int id, double x, double y, double theta) {
    this -> id = id;
    this -> x = x;
    this -> y = y;
    this -> theta = theta;
}

int PosRotId::getID() {
    return id;
}

double PosRotId::getX() {
    return x;
}

double PosRotId::getY() {
    return y;
}

double PosRotId::getTheta() {
    return theta;
}