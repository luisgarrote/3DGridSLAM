#ifndef Euler_H
#define Euler_H

#include "Matrix.h"
#include "Quaternion.h"
class Euler {


public:
    double roll;
    double pitch;
    double yaw;


    Euler(){
        roll=0.0;
        pitch=0.0;
        yaw=0.0;
    }
    Euler(double roll_,double pitch_,double yaw_){
        roll=roll_;
        pitch=pitch_;
        yaw=yaw_;
    }

    ~Euler(){

    }

    Euler operator + (const Euler& q)
    {
        return Euler(roll+q.roll,pitch+q.pitch,yaw+q.yaw);
    }

    Euler operator - (const Euler& q)
    {
        return Euler(roll-q.roll,pitch-q.pitch,yaw-q.yaw);
    }


     Quaternion toQuaternion(){

        geometry_msgs::Quaternion pose;
        Quaternion cm;

        cm.setRPY(roll,pitch,yaw);



        return cm;
    }
    geometry_msgs::Quaternion toQuaternionROS(){

        geometry_msgs::Quaternion pose;
        Quaternion cm;

        cm.setRPY(roll,pitch,yaw);

        pose.x=cm.q1;
        pose.y=cm.q2;
        pose.z=cm.q3;
        pose.w=cm.q0;

        return pose;
    }

/*
@article{slabaugh1999computing,
  title={Computing Euler angles from a rotation matrix},
  author={Slabaugh, Gregory G},
  journal={denoted as TRTA implementation from: http://www. starfireresearch. com/services/java3d/samplecode/FlorinE ulers. html},
  year={1999}
}
*/


//    Euler fromMatrix(Garrote::Math::Matrix::Matrix<double> *matrix){

//        Euler euler;
//        //     phi - x - roll
//        //     theta - y - pitch
//        //     theta - z - yaw

//        if(matrix->get(3,1)!=-1.0 && matrix->get(3,1)!=1.0){



//            euler.pitch=-asin(matrix->get(3,1));
//            euler.roll=atan2(matrix->get(3,2)/cos(euler.pitch),matrix->get(3,3)/cos(euler.pitch));
//            euler.yaw=atan2(matrix->get(2,1)/cos(euler.pitch),matrix->get(1,1)/cos(euler.pitch));


//        }else{


//            euler.yaw=0;

//            if(matrix->get(3,1)==-1.0){

//                euler.pitch=M_PI/2.0;
//                euler.roll=atan2(matrix->get(1,2),matrix->get(1,3));



//            }else{
//                euler.pitch=-M_PI/2.0;
//                euler.roll=atan2(-matrix->get(1,2),-matrix->get(1,3));


//            }

//        }

//        return euler;


//    }



};
#endif // Euler_H
