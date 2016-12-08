#ifndef ENVIRONMENTOPTIONS_H
#define ENVIRONMENTOPTIONS_H
#include "gl_const.h"

class EnvironmentOptions
{
public:
    EnvironmentOptions(int MT, bool AS, double LC, double DC, bool AD, bool RP);
    EnvironmentOptions();
    int     metrictype;     //Can be chosen Euclidean, Manhattan and Diagonal distance
    bool    allowsqueeze;   //Option that allows to move throught "bottleneck"
    double  linecost;       //Cost of straight moves
    double  diagonalcost;   //Cost of diagonal moves
    bool    allowdiagonal;  //Option that allows to make diagonal moves
    bool    useresetparent; //Option that allows JPS use resetParent function like in Theta*

};

#endif // ENVIRONMENTOPTIONS_H
