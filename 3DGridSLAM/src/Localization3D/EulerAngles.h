#ifndef EulerAngles_H
#define EulerAngles_H


#include "Quaternion.h"
#include <sstream>
#include <cmath>
#include <math.h>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

class EulerAngles{

public:
    double roll;
    double pitch;
    double yaw;


    double rad2deg(double rad){

        return ((rad)*180/M_PI);
    }


    double deg2rad(double deg){
        return ((deg)*((M_PI)/(180.0)));
    }


    EulerAngles(){

        roll=0;
        pitch=0;
        yaw=0;

    }

    EulerAngles(double x,double y,double z){
        roll=x;
        pitch=y;
        yaw=z;
    }

    EulerAngles(Quaternion q){
        roll =atan2(2.0*(q.q0*q.q1+q.q2*q.q3),1.0-2.0*(q.q1*q.q1+q.q2*q.q2));
        pitch=asin(2.0*(q.q0*q.q2-q.q3*q.q1));
        yaw=atan2(2.0*(q.q0*q.q3+q.q1*q.q2),1.0-2.0*(q.q2*q.q2+q.q3*q.q3));

    }

    void fromQuaternion(Quaternion q){

        roll =atan2(2.0*(q.q0*q.q1+q.q2*q.q3),1.0-2.0*(q.q1*q.q1+q.q2*q.q2));
        pitch=asin(2.0*(q.q0*q.q2-q.q3*q.q1));
        yaw=atan2(2.0*(q.q0*q.q3+q.q1*q.q2),1.0-2.0*(q.q2*q.q2+q.q3*q.q3));


    }

    void fromQuaternion(double q0,double q1,double q2,double q3){

        //http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        roll =atan2(2.0*(q0*q1+q2*q3),1.0-2.0*(q1*q1+q2*q2));
        pitch=asin(2.0*(q0*q2-q3*q1));
        yaw=atan2(2.0*(q0*q3+q1*q2),1.0-2.0*(q2*q2+q3*q3));
    }

    Quaternion toQuaternion(){

//            double c1 = cos(heading);
//            double s1 = sin(heading);
//            double c2 = cos(attitude);
//            double s2 = sin(attitude);
//            double c3 = cos(bank);
//            double s3 = sin(bank);

//            Quaternion qt;
//            qt.q0 = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
//            double w4 = (4.0 * qt.q0);
//            qt.q1 = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
//            qt.q2 = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
//            qt.q3 = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;



        // Assuming the angles are in radians.
        double c1 = cos(pitch/2.0);
        double s1 = sin(pitch/2.0);
        double c2 = cos(yaw/2.0);
        double s2 = sin(yaw/2.0);
        double c3 = cos(roll/2.0);
        double s3 = sin(roll/2.0);
        double c1c2 = c1*c2;
        double s1s2 = s1*s2;


       Quaternion qt;

        qt.q0 =c1c2*c3 - s1s2*s3;
        qt.q1 =c1c2*s3 + s1s2*c3;
        qt.q2 =s1*c2*c3 + c1*s2*s3;
        qt.q3 =c1*s2*c3 - s1*c2*s3;

        return qt;

    }



    std::string toString(){

        std::stringstream output;

        output<<"roll : "<<roll<<std::endl;
        output<<"pitch : "<<pitch<<std::endl;
        output<<"yaw : "<<yaw<<std::endl;

        return output.str();

    }

    std::string toStringDeg(){

        std::stringstream output;

        output<<"roll : "<<rad2deg(roll)<<std::endl;
        output<<"pitch : "<<rad2deg(pitch)<<std::endl;
        output<<"yaw : "<<rad2deg(yaw)<<std::endl;

        return output.str();
    }




};

#endif // EulerAngles_H
