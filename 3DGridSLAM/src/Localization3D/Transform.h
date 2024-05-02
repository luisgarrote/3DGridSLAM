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
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "Matrix.h"
#include "Point.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include <geometry_msgs/Pose.h>
#include "stringformat.h"




//matrix!!
//0 4 8 12
//1 5 9 13
//2 6 10 14
//3 7 11 15

using namespace Garrote::Math::Matrix;
class Transform{
    //friend class FastPoint<double>;


public:

    Transform *Parent=0;
    std::string frame;
    Garrote::Math::Matrix::Matrix<double>* matrix=0;
    bool dispose=true;


    void setDisposeable(bool dispose_);



    Transform * getParentTransform();

    void setParentTransform(Transform * Pr);

    std::string getTransformFrame();

    void setTransformFrame(std::string f);

    double get(unsigned int i,unsigned int j);
    void set(unsigned int i,unsigned int j,double d);

    double* getMatrix();

    bool isTranslation();

    bool isIdentity();

    Garrote::Math::Matrix::Matrix<float>* getMatrixFloatingPoint();


    std::vector<double> scale();


    Transform transpose(){

        Transform out;
        auto mat=this->matrix->transpose();

        out.matrix->set(0,mat.get(0));
        out.matrix->set(1,mat.get(1));
        out.matrix->set(2,mat.get(2));
        out.matrix->set(3,mat.get(3));
        out.matrix->set(4,mat.get(4));
        out.matrix->set(5,mat.get(5));
        out.matrix->set(6,mat.get(6));
        out.matrix->set(7,mat.get(7));
        out.matrix->set(8,mat.get(8));
        out.matrix->set(9,mat.get(9));
        out.matrix->set(10,mat.get(10));
        out.matrix->set(11,mat.get(11));
        out.matrix->set(12,mat.get(12));
        out.matrix->set(13,mat.get(13));
        out.matrix->set(14,mat.get(14));
        out.matrix->set(15,mat.get(15));


        return out;
    }
    Transform();

    //    Transform(bool eye,bool temp){
    //        matrix=new Garrote::Math::Matrix::Matrix<double>(4,4);

    //        if(eye){
    //            matrix->matrix[0]=1;
    //            matrix->matrix[1]=0;
    //            matrix->matrix[2]=0;
    //            matrix->matrix[3]=0;

    //            matrix->matrix[4]=0;
    //            matrix->matrix[5]=1;
    //            matrix->matrix[6]=0;
    //            matrix->matrix[7]=0;

    //            matrix->matrix[8]=0;
    //            matrix->matrix[9]=0;
    //            matrix->matrix[10]=1;
    //            matrix->matrix[11]=0;

    //            matrix->matrix[12]=0;
    //            matrix->matrix[13]=0;
    //            matrix->matrix[14]=0;
    //            matrix->matrix[15]=1;
    //        }else{
    //            // do nothing?
    //        }

    //        Parent=NULL;
    //        frame="";
    //    }



    Transform(Garrote::Math::Matrix::Matrix<float>* matrix_);

    Transform(Garrote::Math::Matrix::Matrix<double>* matrix_);

    Transform(Garrote::Math::Matrix::Matrix<double>* matrix_,bool reuse);


    Transform(const Transform& other );

    Transform(std::string mat);

    Transform(Quaternion q);

    void  setRotation(double w,double x,double y,double z);

    //rever Transforms
    struct Euler
    {
        double yaw;
        double pitch;
        double roll;
    };


    void getEulerYPR(double &yaw,double &pitch,double &roll,int solution_number=1);




    double getYaw();

    double getYawFast();



    void getRPY(double &roll,double &pitch,double &yaw,int solution_number=1);


    static Transform  byPose(geometry_msgs::Pose pose);
    static Transform *createTransformFromPose(geometry_msgs::Pose pose);


    static Transform *fromPose(geometry_msgs::Pose pose);


    void  updatefromPose(geometry_msgs::Pose pose);

    //inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
    //inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}
    static geometry_msgs::Pose toPose( Transform *r);

    geometry_msgs::Pose toPose();

    geometry_msgs::Quaternion toQuaternion();



    void copy(Transform *tf);


    void fromString(std::string mat);


    void cloneRotation(Transform & tf);

    Matrix<double> getRotation();

    void cloneRotation3x3(Matrix<double> * tf);

    void cloneRotation(Matrix<double> * tf);
    void cloneMatrix(Garrote::Math::Matrix::Matrix<float> & tfmatrix);

    void cloneMatrix(Garrote::Math::Matrix::Matrix<double> & tfmatrix);




    static Transform identity();


    void setIdentity();


    Point point();

    double x();

    double y();

    double z();

    Point point( double x,double y,double z);
    Point point( double x,double y);

    void removeTransform();
    void removeRotation();


    template <typename T>
    T operator*(const T &value){


        //                Garrote::Math::Matrix:: Matrix <double> * point=new Garrote::Math::Matrix::Matrix<double> (4,1);
        //                (*point)(0,0)=value.getX();
        //                (*point)(1,0)=value.getY();
        //                (*point)(2,0)=value.getZ();
        //                (*point)(3,0)=1.0;

        //                //Transform result;


        //                Garrote::Math::Matrix::Matrix<double> result;
        //                result=(*matrix)*(*point);

        //                Point output( result.get(0,0), result.get(1,0), result.get(2,0));

        //                delete point;

        //                return output;


        T output;


        //        output.X()=matrix->get(0,0)*value.getX() + matrix->get(0,1)*value.getY()  + matrix->get(0,2)*value.getZ() + matrix->get(0,3);
        //        output.Y()=matrix->get(1,0)*value.getX()  + matrix->get(1,1)*value.getY() + matrix->get(1,2)*value.getZ() + matrix->get(1,3);
        //        output.Z()=matrix->get(2,0)*value.getX()  + matrix->get(2,1)*value.getY() + matrix->get(2,2)*value.getZ() + matrix->get(2,3);
        output.x=matrix->matrix[0]*value.x + matrix->matrix[4]*value.y + matrix->matrix[8]*value.z + matrix->matrix[12];
        output.y=matrix->matrix[1]*value.x + matrix->matrix[5]*value.y + matrix->matrix[9]*value.z + matrix->matrix[13];
        output.z=matrix->matrix[2]*value.x + matrix->matrix[6]*value.y + matrix->matrix[10]*value.z + matrix->matrix[14];
        return output;
    }

    geometry_msgs::Pose operator*(const geometry_msgs::Pose &value){



        Transform p;
        p.updatefromPose(value);
        //                Garrote::Math::Matrix:: Matrix <double> * point=new Garrote::Math::Matrix::Matrix<double> (4,1);
        //                (*point)(0,0)=value.getX();
        //                (*point)(1,0)=value.getY();
        //                (*point)(2,0)=value.getZ();
        //                (*point)(3,0)=1.0;

        //                //Transform result;


        //                Garrote::Math::Matrix::Matrix<double> result;
        //                result=(*matrix)*(*point);

        //                Point output( result.get(0,0), result.get(1,0), result.get(2,0));

        //                delete point;

        //                return output;


        geometry_msgs::Pose output;

        auto f=this->operator *(p);
        output=f.toPose();





//        //        output.X()=matrix->get(0,0)*value.getX() + matrix->get(0,1)*value.getY()  + matrix->get(0,2)*value.getZ() + matrix->get(0,3);
//        //        output.Y()=matrix->get(1,0)*value.getX()  + matrix->get(1,1)*value.getY() + matrix->get(1,2)*value.getZ() + matrix->get(1,3);
//        //        output.Z()=matrix->get(2,0)*value.getX()  + matrix->get(2,1)*value.getY() + matrix->get(2,2)*value.getZ() + matrix->get(2,3);
//        output.position.x=matrix->matrix[0]*value.position.x + matrix->matrix[4]*value.position.y + matrix->matrix[8]*value.position.z + matrix->matrix[12];
//        output.position.y=matrix->matrix[1]*value.position.x + matrix->matrix[5]*value.position.y + matrix->matrix[9]*value.position.z + matrix->matrix[13];
//        output.position.z=matrix->matrix[2]*value.position.x + matrix->matrix[6]*value.position.y + matrix->matrix[10]*value.position.z + matrix->matrix[14];
//



        return output;
    }
    template <typename T> std::vector<T> operator*(const std::vector<T> &value){



        std::vector<T> out;
        T output;

        out.reserve(value.size());
        for(unsigned int i=0;i<value.size();i++){

            output=value[i];
            output.x=matrix->matrix[0]*value[i].x + matrix->matrix[4]*value[i].y + matrix->matrix[8]*value[i].z + matrix->matrix[12];
            output.y=matrix->matrix[1]*value[i].x + matrix->matrix[5]*value[i].y + matrix->matrix[9]*value[i].z + matrix->matrix[13];
            output.z=matrix->matrix[2]*value[i].x + matrix->matrix[6]*value[i].y + matrix->matrix[10]*value[i].z + matrix->matrix[14];
            out.push_back(output);
        }


        return out;
    }



    std::vector<geometry_msgs::Pose> operator*(const std::vector<geometry_msgs::Pose> &value){



        std::vector<geometry_msgs::Pose> out;
        geometry_msgs::Pose output;

        Transform p;

        for(unsigned int i=0;i<value.size();i++){


            p.updatefromPose(value[i]);

//            output=value[i];
//            output.x=matrix->matrix[0]*value[i].x + matrix->matrix[4]*value[i].y + matrix->matrix[8]*value[i].z + matrix->matrix[12];
//            output.y=matrix->matrix[1]*value[i].x + matrix->matrix[5]*value[i].y + matrix->matrix[9]*value[i].z + matrix->matrix[13];
//            output.z=matrix->matrix[2]*value[i].x + matrix->matrix[6]*value[i].y + matrix->matrix[10]*value[i].z + matrix->matrix[14];
            auto f=this->operator *(p);
            output=f.toPose();
            out.push_back(output);
        }


        return out;
    }



    std::vector<geometry_msgs::Pose> operator^(const std::vector<geometry_msgs::Pose> &value){



        std::vector<geometry_msgs::Pose> out;
        geometry_msgs::Pose output;

        out.reserve(value.size());
        for(unsigned int i=0;i<value.size();i++){

            output=value[i];
            output.position.x= value[i].position.x + matrix->matrix[12];
            output.position.y= value[i].position.y + matrix->matrix[13];
            output.position.z= value[i].position.z + matrix->matrix[14];
            out.push_back(output);
        }


        return out;
    }


    std::vector<Point> operator*(const std::vector<Point> &values);






    //    friend Point operator*(Point value, const Transform &p) {


    //        Garrote::Math::Matrix:: Matrix <double> * point=new Garrote::Math::Matrix::Matrix<double> (4,1);
    //        (*point)(0,0)=value.X();
    //        (*point)(1,0)=value.Y();
    //        (*point)(2,0)=value.Z();
    //        (*point)(3,0)=1.0;

    //        Transform result;

    //        (*result.matrix)=(*point)*(*p.matrix);

    //        Point output( (*result.matrix)(0,0), (*result.matrix)(1,0), (*result.matrix)(2,0));


    //        delete point;

    //        return output;



    //    }


#define SWAP_ROWS(a, b) { double *_tmp = a; (a)=(b); (b)=_tmp; }
#define MAT(m,r,c) (m)[(c)*4+(r)]

    //This code comes directly from GLU except that it is for float
    int inverter(double *m, double *out);




    //matrix!!
    //0 4 8 12
    //1 5 9 13
    //2 6 10 14
    //3 7 11 15


    Transform inverse();


    void onlyRotation();

    Transform inverse4x4();

    Transform operator*(const double &value);

    Transform operator*(const Transform &value);


    //    void transcribeOperationX(const Transform &value){



    //        std::cout<<"START"<<std::endl;
    //        for(unsigned int i=0;i<4;i++){
    //            for(unsigned int f=0;f<4;f++){

    //                std::cout<<"val=0.0;"<<std::endl;
    //                for( unsigned int k=0;k<4;k++){

    //                    std::cout<<"val+= A["<<matrix->ik(i,k)<<"]*B["<<value.matrix->ik(k,f)<<"];"<<std::endl;

    //                    //                        val=val+matrix->ik(i,k)*value.matrix->ik(k,f);
    //                }
    //                std::cout<<"out["<<matrix->ik(i,f)<<"]=val;"<<std::endl;
    //                // result.set(i,f,val);
    //            }
    //        }
    //        std::cout<<"END"<<std::endl;
    //    }

    //    Transform operator-(const Transform &value){


    ////        Transform result;

    ////        //result.matrix=matrix*value.matrix;
    ////        (result.matrix)=&matrix->operator -(*value.matrix);

    ////        return result;


    //    }
    //    Transform operator+(const Transform &value){


    //        Transform result;

    //        //result.matrix=matrix*value.matrix;
    //        //(result.matrix)=(*matrix) +(*value.matrix);

    //        (result.matrix)=&matrix->operator -(*value.matrix);

    //        return result;



    //    }




    void  applyRotationXinOrigin(double xs);

    void  applyRotationYinOrigin(double xs);

    void  applyRotationZinOrigin(double xs);

    void applyScale(double sx,double sy,double sz);

    void take(  Transform &value);
    void add(  Transform &value);

    Transform & operator=(const Transform &value);

    bool operator==(const Transform &other);

    bool operator!=(const Transform &other);

    Transform & operator*=(const Transform &Tf);


    void applyRotationX(double a);
    void applyRotationY(double a);
    void applyRotationZ(double a);


    static void unit();



    void applyRotationXr(double a);
    void applyRotationYr(double a);
    void applyRotationZr(double a);

    void applyTranslation(Point data);

    void applyTranslationr(Point data);

    void applyTranslation(double x,double y,double z);

    void applyTranslationr(double x,double y,double z);

    Transform rotateX(double a);

    Transform rotateY(double a);
    Transform rotateZ(double a);

    Transform translate(Point data);

    Transform translate(double x,double y,double z);
    void setTranslation(double x,double y,double z);

    void expand(double val);

    std::string toString();
    std::string toStringTranslation();


    std::string toStringQuaternion();


    Transform transformationFromBase();


    ~Transform();

};



#endif // TRANSFORM_H
