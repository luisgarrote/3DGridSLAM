/** 
*@file //TODO 
*  @date 9 Oct 2013 
*  @version 1.0 
*  @brief //TODO. 

This file is part of Luis Garrote Phd work. 
Copyright (C) 2012-2014 Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt) 
All rights reserved.

*
*  @author Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt)
*  @bug No know bugs.
*/ 
#ifndef QUATERNION_H
#define QUATERNION_H

#include <iostream>
#include <cmath>
#include "Point.h"
class Quaternion{

    //http://docs.ros.org/indigo/api/tf/html/c++/Quaternion_8h_source.html
public:
    double q0;
    double q1;
    double q2;
    double q3;




    void toStream(){

        std::cout<<" q0 "<<q0<<" q1 "<<q1<<" q2 "<<q2<<" q3 "<<q3<<std::endl;

    }

    Quaternion(){

        q0=1;
        q1=0;
        q2=0;
        q3=0;

    }

    Quaternion(double x_,double y_,double z_,double w_){
        q0=w_;
        q1=x_;
        q2=y_;
        q3=z_;
    }
    Quaternion(double w_,double x_,double y_,double z_,bool reordered){
        if(reordered){
        q0=w_;
        q1=x_;
        q2=y_;
        q3=z_;
        }else{
            q0=z_;
            q1=w_;
            q2=x_;
            q3=y_;

        }
    }

    inline void set(double x_,double y_,double z_,double w_){
        q0=w_;
        q1=x_;
        q2=y_;
        q3=z_;


    }
    void  FromAngleAxis (const double& rfAngle,
                         const Point& rkAxis)
    {
        // assert:  axis[] is unit length
        //https://bitbucket.org/sinbad/ogre/src/9db75e3ba05c/OgreMain/include/OgreVector3.h?fileviewer=file-view-default#cl-651
        // The quaternion representing the rotation is
        //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

        double fHalfAngle ( 0.5*rfAngle );
        double fSin = std::sin(fHalfAngle);
        q0 = std::cos(fHalfAngle);
        q1 = fSin*rkAxis.x;
        q2 = fSin*rkAxis.y;
        q3 = fSin*rkAxis.z;
    }
    inline void normalize()
    {
        double fLength = std::sqrt( q1 * q1 + q2 * q2 + q3 * q3+ q0 * q0 );


        if ( fLength > 0.0001 )
        {
            double fInvLength = 1.0f / fLength;
            q0 *= fInvLength;
            q1 *= fInvLength;
            q2 *= fInvLength;
            q3 *= fInvLength;

        }

    }

    //https://github.com/moble/Quaternions/blob/master/Quaternions.hpp
    //https://github.com/moble/Quaternions/blob/master/Quaternions.cpp




    Quaternion exp() const  {
        Quaternion Result;
        const double b = std::sqrt(q1*q1 + q2*q2 + q3*q3);
        if(std::abs(b)<=1.0e-14*std::abs(q0)) {
            Result.q0 = std::exp(q0);
        } else {
            double e = std::exp(q0);
            double f = std::sin(b)/b; // Note: b is never 0.0 at this point
            Result.q0 = e*std::cos(b);
            Result.q1 = e*f*q1;
            Result.q2 = e*f*q2;
            Result.q3 = e*f*q3;
        }
        return Result;
    }


    static Quaternion exp(const Quaternion&  qr){

        return qr.exp();
    }

    inline Quaternion operator-() const { return Quaternion(-q0, -q1, -q2, -q3,true); }
    inline Quaternion operator+(const double t) const { return Quaternion(q0+t, q1, q2, q3,true); }
    inline Quaternion operator-(const double t) const { return Quaternion(q0-t, q1, q2, q3,true); }
    inline Quaternion operator*(const double t) const { return Quaternion(q0*t, q1*t, q2*t, q3*t,true); }
    inline Quaternion operator/(const double t) const { return Quaternion(q0/t, q1/t, q2/t, q3/t,true); }

    Quaternion operator + (const Quaternion& q)
    {
        return Quaternion(q0+q.q0, q1+q.q1, q2+q.q2, q3+q.q3,true);
    }

    Quaternion operator - (const Quaternion& q)
    {
        return Quaternion(q0-q.q0, q1-q.q1, q2-q.q2, q3-q.q3,true);
    }

    Quaternion operator * (const Quaternion& q)
    {
        return Quaternion(
                    q0*q.q0 - q1*q.q1 - q2*q.q2 - q3*q.q3,
                    q0*q.q1 + q1*q.q0 + q2*q.q3 - q3*q.q2,
                    q0*q.q2 + q2*q.q0 + q3*q.q1 - q1*q.q3,
                    q0*q.q3 + q3*q.q0 + q1*q.q2 - q2*q.q1);
    }

    Quaternion operator / (Quaternion& q)
    {
        return ((*this) * (q.inverse()));
    }
    Quaternion&operator += (const Quaternion& q)
    {
        q0 += q.q0;
        q1 += q.q1;
        q2 += q.q2;
        q3 += q.q3;

        return (*this);
    }
    Quaternion & operator -= (const Quaternion & q)
    {
        q0 -= q.q0;
        q1 -= q.q1;
        q2 -= q.q2;
        q3 -= q.q3;


        return (*this);
    }

//    Quaternion & operator = (const Quaternion & q)
//    {
//        q0 = q.q0;
//        q1 = q.q1;
//        q2 = q.q2;
//        q3 = q.q3;


//        return (*this);
//    }

    Quaternion & operator *= (const Quaternion & q)
    {
        Quaternion data(
                    q0*q.q0 - q1*q.q1 - q2*q.q2 - q3*q.q3,
                    q0*q.q1 + q1*q.q0 + q2*q.q3 - q3*q.q2,
                    q0*q.q2 + q2*q.q0 + q3*q.q1 - q1*q.q3,
                    q0*q.q3 + q3*q.q0 + q1*q.q2 - q2*q.q1);



        this->q0=data.q0;
        this->q1=data.q1;
        this->q2=data.q2;
        this->q3=data.q3;


        return (*this);
    }

    Quaternion&  operator /= (Quaternion& q)
    {
        (*this) = (*this)*q.inverse();
        return (*this);
    }

    bool operator == (const Quaternion& q)
    {
        return (q0==q.q0 && q1==q.q1 && q2==q.q2 && q3==q.q3) ? true : false;
    }

    double norm() const
    {
        return (q0*q0 + q1*q1 + q2*q2 + q3*q3);
    }

    inline double x(){
        return q1;
    }
    inline  double y(){
        return q2;
    }
    inline  double z(){
        return q3;
    }
    inline  double w(){
        return q0;
    }

    double magnitude()
    {
        return sqrt(norm());
    }

    Quaternion inverse() const
    {
        return conjugate().scale(1.0/norm());
    }
    Quaternion  conjugate() const
    {
        return Quaternion(q0, -q1, -q2, -q3,true);
    }

    Quaternion   UnitQuaternion()
    {
        return (*this).scale(1/(*this).magnitude());
    }

    Quaternion scale(double  s)
    {
        return Quaternion(q0*s, q1*s, q2*s, q3*s,true);
    }


    //http://docs.ros.org/api/tf/html/c++/Quaternion_8h_source.html#l00094

    void setRPY(double roll, double pitch, double yaw){


        /* btScalar halfYaw = btScalar(yaw) * btScalar(0.5);
                           btScalar halfPitch = btScalar(pitch) * btScalar(0.5);
                           btScalar halfRoll = btScalar(roll) * btScalar(0.5);
                           btScalar cosYaw = btCos(halfYaw);
                           btScalar sinYaw = btSin(halfYaw);
                            btScalar cosPitch = btCos(halfPitch);
                           btScalar sinPitch = btSin(halfPitch);
                           btScalar cosRoll = btCos(halfRoll);
                           btScalar sinRoll = btSin(halfRoll);
                           setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
                                   cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
                                   sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
                                   cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);*/



        //        double e1=roll;
        //        double e2=pitch;
        //        double e3=yaw;



        double halfYaw = double(yaw) * double(0.5);
        double halfPitch = double(pitch) * double(0.5);
        double halfRoll = double(roll) * double(0.5);
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        double cosPitch = cos(halfPitch);
        double sinPitch = sin(halfPitch);
        double cosRoll = cos(halfRoll);
        double sinRoll = sin(halfRoll);
        set(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
            cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
            cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
            cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx




        //          q0 = sqrt(cos(e2)*cos(e1)+cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)* cos(e3)+1.0)/2.0;
        //          q1 = (cos(e1)*sin(e3)+cos(e2)*sin(e3)+sin(e2)*sin(e1)*cos(e3))/sqrt(cos(e2)* cos(e1)+cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)*cos(e3)+1.0)/2.0;
        //          q2 = (sin(e2)*sin(e3)-cos(e2)*sin(e1)*cos(e3)-sin(e1))/sqrt(cos(e2)*cos(e1)+ cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)*cos(e3)+1.0)/2.0;
        //          q3 = (sin(e2)*cos(e1)+sin(e2)*cos(e3)+cos(e2)*sin(e1)*sin(e3))/sqrt(cos(e2)* cos(e1)+cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)*cos(e3)+1.0)/2.0;



    }


    void setY( double yaw){



        double halfYaw = double(yaw) * double(0.5);

        set(0, //x
            0, //y
            sin(halfYaw), //z
            cos(halfYaw)); //formerly yzx

        //          q0 = sqrt(cos(e2)*cos(e1)+cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)* cos(e3)+1.0)/2.0;
        //          q1 = (cos(e1)*sin(e3)+cos(e2)*sin(e3)+sin(e2)*sin(e1)*cos(e3))/sqrt(cos(e2)* cos(e1)+cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)*cos(e3)+1.0)/2.0;
        //          q2 = (sin(e2)*sin(e3)-cos(e2)*sin(e1)*cos(e3)-sin(e1))/sqrt(cos(e2)*cos(e1)+ cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)*cos(e3)+1.0)/2.0;
        //          q3 = (sin(e2)*cos(e1)+sin(e2)*cos(e3)+cos(e2)*sin(e1)*sin(e3))/sqrt(cos(e2)* cos(e1)+cos(e2)*cos(e3)-sin(e2)*sin(e1)*sin(e3)+cos(e1)*cos(e3)+1.0)/2.0;



    }


};

inline Quaternion operator+(const double a, const Quaternion& Q) { return Q+a; }
inline Quaternion operator-(const double a, const Quaternion& Q) { return (-Q)+a; }
inline Quaternion operator*(const double a, const Quaternion& Q) { return Q*a; }
inline Quaternion operator/(const double a, const Quaternion& Q) { return Q.inverse()*a; }

#endif // QUATERNION_H
