


#ifndef Point_h
#define Point_h

#include "FastPoint.h"

typedef FastPoint<double> Point;


class SemanticPoint{

public:
    double x=0;
    double y=0;
    double z=0;
    int label=0;
    double w=0;
};

#endif

