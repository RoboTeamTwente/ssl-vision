//
// Created by Wouter Timmermans on 18-09-17.
//

#ifndef SURFBOTTRACKING_POSROT_H
#define SURFBOTTRACKING_POSROT_H


class PosRotId {
public:
    PosRotId(int id, double x, double y, double theta);

    int    getID();
    double getX();
    double getY();
    double getTheta();

private:
    int id;
    double x,y,theta;
};


#endif //SURFBOTTRACKING_POSROT_H
