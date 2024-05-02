/*
*
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

#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <cmath>
#include <random>
#include <chrono>
#include <stdlib.h>
#include "Matrix.h"
template <typename T,bool cached>
class normal_distribution{

    double x0;
    double sigx;

    Garrote::Math::Matrix::Matrix<double> cache;

    double compute(double x){
        return std::exp(-(((x-x0)*(x-x0)/(2.0*sigx*sigx))));

    }

public:
    double A;

    normal_distribution(double x0_,double sigx_){
        x0=x0_;
        sigx=sigx_;
        A=1.0/(sigx*sqrt(2.0*M_PI));

        if(cached==true){
            int n=floor(sigx*4.0/0.01);
            cache.reshape(n,1);

            for(int i=0;i<n;i++){
                cache.set(i,0,compute(((double)i)*0.01));
            }

        }
    }
    normal_distribution(double x0_,double sigx_,double A_){
        x0=x0_;
        sigx=sigx_;
        A=A_;

        if(cached==true){
            int n=floor(sigx*4.0/0.01);
            cache.reshape(n,1);

            for(int i=0;i<n;i++){
                cache.set(i,0,compute(((double)i)*0.01));
            }

        }
    }
    normal_distribution(double x0_,double sigx_,bool ignore){
        x0=x0_;
        sigx=sigx_;


        if(ignore){
            A=1;
        }else{
            A=1.0/(sigx*sqrt(2.0*M_PI));
        }
        if(cached==true){
            int n=floor(sigx*4.0/0.01);
            cache.reshape(n,1);

            for(int i=0;i<n;i++){
                cache.set(i,0,compute(((double)i)*0.01));
            }

        }
    }



    double operator ()(double x){

        if(cached){
            int n=round(fabs(x)/0.01);
            if(n>=cache.getRows()){
                return 0;
            }
            return A*cache[n];
        }else{
            return A*std::exp(-(((x-x0)*(x-x0)/(2.0*sigx*sigx))));
        }
    }


    double sample(){
        unsigned seed1 = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        std::default_random_engine generator(seed1);
        std::uniform_real_distribution<double> distribution(-sigx,sigx);
        double samp=0;
        for(int i=1;i<=12;i++){

            samp+=distribution(generator);
        }
        return 0.5*samp;
    }


    static double sample(double sig2){
        //        double sigx_=sqrt(sig2);
        //        unsigned seed1 = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        //        std::default_random_engine generator(seed1);
        //        std::uniform_real_distribution<double> distribution(-sigx_,sigx_);
        //        double samp=0;
        //        for(int i=1;i<=12;i++){
        //            samp+=distribution(generator);
        //        }
        //        return 0.5*samp;


        //Box-Muller transformation

        double sigma=std::sqrt(sig2);

        if(sigma<0.001){
            return 0.0;
        }

        //        std::cout<<sigma<<std::endl;
        double x1, x2, w, r;

        do
        {
            do { r = rand() / (RAND_MAX + 1.0); } while (r==0.0);
            x1 = 2.0 * r - 1.0;
            do { r =rand() / (RAND_MAX + 1.0); } while (r==0.0);
            x2 = 2.0 * r - 1.0;
            w = x1*x1 + x2*x2;
        } while(w > 1.0 || w==0.0);

        return(sigma * x2 * sqrt(-2.0*log(w)/w));
        //http://www.lmpt.univ-tours.fr/~nicolis/Licence_NEW/08-09/boxmuller.pdf
        //https://www.johndcook.com/blog/cpp_tr1_random/#normal
        //return  sigma * sqrt(-2.0*log(1.0-x1))*cos(2.0*M_PI*x2);
    }


};








template <typename Point,int N>
class NdInverseMultiquadratic{
public:

    double A=1.0;

    Garrote::Math::Matrix::Matrix<double> mu;
    Garrote::Math::Matrix::Matrix<double> P;
    Garrote::Math::Matrix::Matrix<double> Pinv;

    int samples=0;


    void setP(const std::vector<double> &diag){

        int f=std::min(std::min(P.getColumns(),P.getRows()),(unsigned int)diag.size());
        for(int i=0;i<f;i++){
            P.set(i,i,diag[i]);
        }

        Pinv=P.inverse();

    }


    void setMu(const std::vector<double> &diag){

        int f=std::min(mu.getRows(),(unsigned int)diag.size());
        for(int i=0;i<f;i++){
            mu.set(i,0,diag[i]);
        }

    }

    NdInverseMultiquadratic(){

        mu.reshape(N,1);
        mu.zero();
        P.reshape(N,N);
        P.zero();
    }


    void fromPoints(const std::vector<Point> &pts){


        samples=pts.size();
        mu.zero();
        P.zero();

        if(N==2){

            for(unsigned int i=0;i<pts.size();i++){
                mu(0,0)+=pts[i].x;
                mu(1,0)+=pts[i].y;
            }
            mu=mu/((double)pts.size());

            double x;
            double y;
            for(unsigned int i=0;i<pts.size();i++){

                x=(pts[i].x)-mu(0,0);
                y=(pts[i].y)-mu(1,0);

                P(0,0)+=x*x;
                P(1,1)+=y*y;
                P(1,0)+=x*y;
                P(0,1)+=y*x;

            }
            P=P/((double)(pts.size()-1));

            //            P.toOutput();
            //            std::cout<<pts.size()<<std::endl;





        }else if(N==3){
            for(unsigned int i=0;i<pts.size();i++){

                mu(0,0)+=pts[i].x;
                mu(1,0)+=pts[i].y;
                mu(2,0)+=pts[i].z;
            }
            mu=mu/((double)pts.size());

            double x;
            double y;
            double z;
            for(unsigned int i=0;i<pts.size();i++){
                x=pts[i].x-mu(0,0);
                y=pts[i].y-mu(1,0);
                z=pts[i].z-mu(2,0);

                P(0,0)+=x*x;
                P(1,0)+=x*y;
                P(2,0)+=x*z;

                P(0,1)+=y*x;
                P(1,1)+=y*y;
                P(2,1)+=y*z;

                P(0,2)+=z*x;
                P(1,2)+=z*y;
                P(2,2)+=z*z;

            }

            P=P/((double)pts.size());
        }

        Pinv=P.inverse();
    }

    double compute(const Point &p){

        Garrote::Math::Matrix::Matrix<double> q(N,1);
        q(0,0)=p.x-mu(0,0);
        q(1,0)=p.y-mu(1,0);
        if(N>2)
            q(2,0)=p.z-mu(2,0);


        //inverse multiquadratic
        return 1.0/(std::sqrt(1.0+std::pow((q.transpose()*Pinv*q)(0,0),2)));
        //Multiquadric:


    }




    double compute(Point &p,double dx,double dy){


        Garrote::Math::Matrix::Matrix<double> q(1,N);
        Garrote::Math::Matrix::Matrix<double> L(N,1);
        L<<dx,dy;
        if(N==2){
            q<<(p.x-mu(0,0))*dx,(p.y-mu(1,0))*dy;
        }else if(N==3){
            q<<(p.x-mu(0,0))*dx,(p.y-mu(1,0))*dy,p.z-mu(2,0);
        }
        return 0;// -((2.0*q*q*q)/(P*P) + (2.0*q*q*q)/(P*P))/(2.0*std::pow((q*q*q*q)/(P*P) + 1,1.5))(0,0);
    }

    double compute(double dx,double dy,double dx2,double dy2){


        Garrote::Math::Matrix::Matrix<double> L1(1,N);
        Garrote::Math::Matrix::Matrix<double> L2(N,1);
        L1<<dx,dy;
        L2<<dx2,dy2;
        return (L1*Pinv*L2)(0,0);
    }

};



template <typename Point,int N>
class NdGaussian{
public:

    double A=1.0;

    Garrote::Math::Matrix::Matrix<double> mu;
    Garrote::Math::Matrix::Matrix<double> P;
    Garrote::Math::Matrix::Matrix<double> Pinv;

    int samples=0;


    void setP(const std::vector<double> &diag){

        int f=std::min(std::min(P.getColumns(),P.getRows()),(unsigned int)diag.size());
        for(int i=0;i<f;i++){
            P.set(i,i,diag[i]);
        }

        Pinv=P.inverse();

    }


    void setMu(const std::vector<double> &diag){

        int f=std::min(mu.getRows(),(unsigned int)diag.size());
        for(int i=0;i<f;i++){
            mu.set(i,0,diag[i]);
        }

    }

    NdGaussian(){

        mu.reshape(N,1);
        mu.zero();
        P.reshape(N,N);
        P.zero();
    }


    void fromPoints(const std::vector<Point> &pts){


        samples=pts.size();
        mu.zero();
        P.zero();

        if(N==2){

            for(unsigned int i=0;i<pts.size();i++){
                mu(0,0)+=pts[i].x;
                mu(1,0)+=pts[i].y;
            }
            mu=mu/((double)pts.size());

            double x;
            double y;
            for(unsigned int i=0;i<pts.size();i++){

                x=(pts[i].x)-mu(0,0);
                y=(pts[i].y)-mu(1,0);

                P(0,0)+=x*x;
                P(1,1)+=y*y;
                P(1,0)+=x*y;
                P(0,1)+=y*x;

            }
            P=P/((double)(pts.size()-1));

            //            P.toOutput();
            //            std::cout<<pts.size()<<std::endl;





        }else if(N==3){
            for(unsigned int i=0;i<pts.size();i++){

                mu(0,0)+=pts[i].x;
                mu(1,0)+=pts[i].y;
                mu(2,0)+=pts[i].z;
            }
            mu=mu/((double)pts.size());

            double x;
            double y;
            double z;
            for(unsigned int i=0;i<pts.size();i++){
                x=pts[i].x-mu(0,0);
                y=pts[i].y-mu(1,0);
                z=pts[i].z-mu(2,0);

                P(0,0)+=x*x;
                P(1,0)+=x*y;
                P(2,0)+=x*z;

                P(0,1)+=y*x;
                P(1,1)+=y*y;
                P(2,1)+=y*z;

                P(0,2)+=z*x;
                P(1,2)+=z*y;
                P(2,2)+=z*z;

            }

            P=P/((double)pts.size());
        }

        Pinv=P.inverse();
    }
    //http://www-biba.inrialpes.fr/Jaynes/cappe1.pdf
    double integral(){

        return  std::pow(2.0*M_PI,N/2.0)/std::sqrt(Pinv.determinant());
    }

    double INTcompute(const Point &p1,const Point &p2){

        Garrote::Math::Matrix::Matrix<double> Lx(N,1);
        Lx<<1,0;
        Garrote::Math::Matrix::Matrix<double> Ly(N,1);
        Ly<<0,1;

        Garrote::Math::Matrix::Matrix<double> q1(N,1);
        q1<<p1.x,p1.y;
        Garrote::Math::Matrix::Matrix<double> q2(N,1);
        q2<<p2.x,p2.y;
        Garrote::Math::Matrix::Matrix<double> q3(N,1);
        q3<<p1.x,p2.y;
        Garrote::Math::Matrix::Matrix<double> q4(N,1);
        q4<<p2.x,p1.y;



        //        std::cout<<(-2.0/((q2.transpose()*Pinv*Ly)(0,0)))<<std::endl;
        //        std::cout<<(-2.0/((q2.transpose()*Pinv*Lx)(0,0)))<<std::endl;
        //        std::cout<<std::exp(-(((q2.transpose()*Pinv*q2)/2.0)(0,0)))<<std::endl;
        //        std::cout<<(2.0/((q3.transpose()*Pinv*Lx)(0,0)))<<std::endl;
        //        std::cout<<std::exp(-(((q3.transpose()*Pinv*q3)/2.0)(0,0)))<<std::endl;
        //        std::cout<<(-2.0/((q1.transpose()*Pinv*Ly)(0,0)))<<std::endl;
        //        std::cout<<(-2.0/((q4.transpose()*Pinv*Lx)(0,0)))<<std::endl;
        //        std::cout<<std::exp(-(((q4.transpose()*Pinv*q4)/2.0)(0,0)))<<std::endl;
        //        std::cout<<(2.0/((q1.transpose()*Pinv*Lx)(0,0)))<<std::endl;
        //        std::cout<<std::exp(-(((q1.transpose()*Pinv*q1)/2.0)(0,0)))<<std::endl;


        return (-2.0/((q2.transpose()*Pinv*Ly)(0,0)))*
                ((-2.0/((q2.transpose()*Pinv*Lx)(0,0)))*
                 std::exp(-(((q2.transpose()*Pinv*q2)/2.0)(0,0)))
                 +(2.0/((q3.transpose()*Pinv*Lx)(0,0)))*std::exp(-(((q3.transpose()*Pinv*q3)/2.0)(0,0))))
                - (-2.0/((q1.transpose()*Pinv*Ly)(0,0)))*
                ((-2.0/((q4.transpose()*Pinv*Lx)(0,0)))*
                 std::exp(-(((q4.transpose()*Pinv*q4)/2.0)(0,0)))
                 +(2.0/((q1.transpose()*Pinv*Lx)(0,0)))*std::exp(-(((q1.transpose()*Pinv*q1)/2.0)(0,0))));


    }



    double compute2(const Point &p){

        Garrote::Math::Matrix::Matrix<double> q(N,1);
        q(0,0)=p.x-mu(0,0);
        q(1,0)=p.y-mu(1,0);
        if(N>2)
            q(2,0)=p.z-mu(2,0);

        //inverse quadratic
        //        return 1.0/(1.0+std::pow((q.transpose()*Pinv*q)(0,0),2));
        //inverse multiquadratic
        return (q*q.transpose())(0,0);
        //Multiquadric:
        //        return std::sqrt(1+std::pow((q.transpose()*Pinv*q)(0,0),2));

    }


    double compute(const Point &p){

        Garrote::Math::Matrix::Matrix<double> q(N,1);
        q(0,0)=p.x-mu(0,0);
        q(1,0)=p.y-mu(1,0);
        if(N>2)
            q(2,0)=p.z-mu(2,0);

        return A*std::exp(-(((q.transpose()*Pinv*q)/2.0)(0,0)));

    }

    double compute(Point &p,double dx,double dy){


        Garrote::Math::Matrix::Matrix<double> q(1,N);
        Garrote::Math::Matrix::Matrix<double> L(N,1);
        L<<dx,dy;
        if(N==2){
            q<<p.x-mu(0,0),p.y-mu(1,0);
        }else if(N==3){
            q<<p.x-mu(0,0),p.y-mu(1,0),p.z-mu(2,0);
        }
        return (q*Pinv*L)(0,0);
    }

    double compute(double dx,double dy,double dx2,double dy2){


        Garrote::Math::Matrix::Matrix<double> L1(1,N);
        Garrote::Math::Matrix::Matrix<double> L2(N,1);
        L1<<dx,dy;
        L2<<dx2,dy2;
        return (L1*Pinv*L2)(0,0);
    }

};

class Gaussian{



public:

    double A;
    double x0;
    double y0;
    double sigx;
    double sigy;

    Gaussian(){
        A=0;
        x0=0;
        y0=0;
        sigx=0;
        sigy=0;
    }
    Gaussian(double A_,double x0_,double y0_,double sigx_,double sigy_){
        A=A_;
        x0=x0_;
        y0=y0_;
        sigx=sigx_;
        sigy=sigy_;
    }


    void setGain(double A_){
        A=A_;
    }

    double compute(double x,double y){

        return A*std::exp(-(((x-x0)*(x-x0)/(2.0*sigx*sigx))+((y-y0)*(y-y0)/(2.0*sigy*sigy))));

    }

};

#endif // GAUSSIAN_H
