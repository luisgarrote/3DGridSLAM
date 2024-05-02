
#ifndef POSETOOLSq_H
#define POSETOOLSq_H
#pragma once


#include <iostream>
#include <cmath>
#include <vector>

#include "stringformat.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "Point.h"
#include "Quaternion.h"

#include <algorithm>
class Euler;
namespace TwistTools{

geometry_msgs::Twist fromString(std::string v);


inline std::string  toStringCompressed(std::vector<geometry_msgs::Twist> &pose1){

    std::stringstream str;

    for(unsigned int i=0;i<pose1.size();i++){
        str<< "["+$("{}").format(pose1[i].linear.x)+
              ","+$("{}").format(pose1[i].linear.y)+
              ","+$("{}").format(pose1[i].linear.z)+
              ","+$("{}").format(pose1[i].angular.x)+
              ","+$("{}").format(pose1[i].angular.y)+
              ","+$("{}").format(pose1[i].angular.z)+"];";
    }
    return str.str();


}

std::string toString(geometry_msgs::Twist  pos);

inline double  norm2D(geometry_msgs::Twist pos1){
    return sqrt((pos1.linear.x)*(pos1.linear.x)+(pos1.angular.z)*(pos1.angular.z));
}


inline double  norm(geometry_msgs::Twist pos1){
    return sqrt((pos1.linear.x)*(pos1.linear.x)+(pos1.linear.y)*(pos1.linear.y)+(pos1.linear.z)*(pos1.linear.z)+(pos1.angular.x)*(pos1.angular.x)+(pos1.angular.y)*(pos1.angular.y)+(pos1.angular.z)*(pos1.angular.z));
}
inline double   distance2D(geometry_msgs::Twist pos1, geometry_msgs::Twist pos2){
    return sqrt((pos1.linear.x-pos2.linear.x)*(pos1.linear.x-pos2.linear.x)+(pos1.angular.z-pos2.angular.z)*(pos1.angular.z-pos2.angular.z));
}


//inline double distance2D(geometry_msgs::Twist  pos1,geometry_msgs::Twist  pos2);

//inline double norm2D(geometry_msgs::Twist pos1);

}

template < class T >
std::vector<T> &operator += ( std::vector<T>& v,T p){

    for(unsigned int i=0;i<v.size();i++){
        v[i]+=p;
    }
    return v;
}


namespace VectorTools {


template <typename T>
inline unsigned int replace(std::vector<T> &vec,T val,T th){

    unsigned int out=0;
    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]<th){
            vec[i]=val;
            out++;
        }
    }
    return out;
}




template <typename T>
inline std::vector<T> fromString(std::string &vec){

    return $(vec).numbers<T>();

}





template <typename T>
inline std::string toString(std::vector<T> &vec){

    std::stringstream str;
    str<< "[";
    if(vec.size()>0){
        str<<vec[0];
        for(unsigned int i=1;i<vec.size();i++){
            str<<" , "<<vec[i];
        }
    }
    str<<"];";
    return str.str();

}
template <typename T,int N>
inline std::string toString(std::array<T,N> &vec){

    std::stringstream str;
    str<< "[";
         str<<vec[0];
        for(unsigned int i=1;i<N;i++){
            str<<" , "<<vec[i];
        }
     str<<"];";
    return str.str();

}
template <typename T>
inline std::string toStringMatlab(std::vector<T> &vec){

    std::stringstream str;
    str<< "[";
         str<<vec[0];
        for(unsigned int i=1;i<vec.size();i++){
            str<<" , "<<vec[i];
        }
     str<<"]";
    return str.str();

}
template <typename T>
inline void divide(std::vector<T> &vec,T val){

    for(unsigned int i=0;i<vec.size();i++){
        vec[i]=vec[i]/val;
    }
}

template <typename T>
inline int count(std::vector<T> &vec,T val){
    int ct=0;
    for(unsigned int i=0;i<vec.size();i++){
        if(vec[i]==val){
            ct++;
        }
    }
    return ct;
}


template <typename T>
double sum(std::vector<T> &vec){
    double ct=0.0;
    for(unsigned int i=0;i<vec.size();i++){
        ct=ct+vec[i];
    }
    return ct;
}

template <typename T>
inline int argmin(std::vector<T> &vec){

    T min=std::numeric_limits<T>::max();
    int index =-1;
    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]<min){

            min=vec[i];
            index=i;
        }


    }

    return index;
}


template <typename T>
inline int argmax(std::vector<T> &vec){

    T min=std::numeric_limits<T>::lowest();
    int index =-1;
    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]>min){

            min=vec[i];
            index=i;
        }


    }

    return index;
}


template <typename T,int N>
inline int argmax(std::array<T,N> &vec){

    T min=std::numeric_limits<T>::lowest();
    int index =-1;
    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]>min){

            min=vec[i];
            index=i;
        }


    }

    return index;
}
template <typename T>
inline bool contains(std::vector<T> &vec,T da){

    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]==da){
            return true;
        }
    }
    return false;
}



template <class T> inline double var_unbias(const std::vector<T> &data )
{
    double mean=0;
    for(int i=0;i<data.size();i++)
    {
        mean=mean+data[i];
    }

    mean=mean/(double)((data.size()));

    double vr=0;
    for(int i=0;i<data.size();i++)
    {
        vr=vr+(data[i]-mean)*(data[i]-mean);
    }

    return vr/(double)((data.size())-1.0);
}



template <class T> inline double var(const std::vector<T> &data )
{
    double mean=0;
    for(int i=0;i<data.size();i++)
    {
        mean=mean+data[i];
    }

    mean=mean/(double)((data.size()));

    double vr=0;
    for(int i=0;i<data.size();i++)
    {
        vr=vr+(data[i]-mean)*(data[i]-mean);
    }

    return vr/(double)((data.size()));
}


template <class T> inline double std(const std::vector<T> &data)
{
    return sqrt(var(data));
}
template <class T> inline double std_unbias(const std::vector<T> &data)
{
    return sqrt(var_unbias(data));
}


template <typename T>
inline T mean(const std::vector<T> &vec){

    T min=0;
    for(unsigned int i=0;i<vec.size();i++){


        min+=vec[i];
    }



    return min/(T)(vec.size());
}




template <typename T>
inline T rmse(const std::vector<T> &vec){

    T min=0;
    for(unsigned int i=0;i<vec.size();i++){


        min+=(vec[i]*vec[i]);
    }



    return std::sqrt(min/(T)(vec.size()));
}




template <typename T>
inline T L1Norm(const std::vector<T> &vec){

    T min=0;
    for(unsigned int i=0;i<vec.size();i++){


        min+=fabs(vec[i]);
    }



    return min;
}


template <typename T>
inline T L2Norm(const std::vector<T> &vec){

    T min=0;
    for(unsigned int i=0;i<vec.size();i++){
        min+=vec[i]*vec[i];
    }



    return sqrt(min);
}
template <typename T>
inline T LInfinityNorm(const std::vector<T> &vec){

    T min=-9999999999;
    for(unsigned int i=0;i<vec.size();i++){
        min=std::max(vec[i],min);
    }



    return (min);
}


template <typename T>
inline std::vector<T> abs(std::vector<T> &vec){
    std::vector<T>  vecout;
    vecout.reserve(vec.size());
    for(unsigned int i=0;i<vec.size();i++){

        vecout.push_back(std::fabs(vec[i]));
    }



    return vecout;
}


template <typename T>
inline std::vector<T> unique(  std::vector<T> nodes){
    std::sort(nodes.begin(), nodes.end());
    auto last = std::unique(nodes.begin(), nodes.end());
    nodes.erase(last, nodes.end());

    return nodes;
 }

template <typename T>
inline T median(const std::vector<T> &vec){



    if(vec.size()%2==0){
        return vec[(vec.size()-1)/2.0]+vec[(vec.size()+1)/2.0];
    }else{

        return vec[vec.size()/2.0];
    }

}

template <typename T>
inline T max(const  std::vector<T> &vec){

    T max=std::numeric_limits<T>::lowest();
    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]>max){

            max=vec[i];
        }


    }

    return max;
}


template <typename T>
inline T min(const std::vector<T> &vec){

    T min=std::numeric_limits<T>::max();
    for(unsigned int i=0;i<vec.size();i++){

        if(vec[i]<min){

            min=vec[i];
        }


    }

    return min;
}



}


namespace PointTools {



template <typename T>

inline double angleBetween(const T &a,const T &b,const  T &c){

    T   ab;
    ab.x=( b.x - a.x);
    ab.y=( b.y - a.y);
    T   cb;
    cb.x=( b.x - c.x);
    cb.y=( b.y - c.y);

    double dot = ((ab.x * cb.x) + (ab.y * cb.y)); // dot product
    double cross = ((ab.x * cb.y) - (ab.y * cb.x)); // cross product

    return  atan2(cross, dot);

}


template <typename T>
inline std::vector<T> lineCircumferenceIntersection(T p1,T p2,T c,double r){

    T p3;
    p3.x= p1.x - c.x;
    p3.y=  p1.y - c.y;
    T p4;
    p4.x= p2.x - c.x;
    p4.y=p2.y - c.y;

    double m = (p4.y - p3.y) / (p4.x - p3.x); //%slope of the line
    double b = p3.y - m * p3.x; //%y-intercept of line

    double  underRadical = (r*r)*m*m + r*r - b*b; //%the value under the square root sign



    if (underRadical < 0){
        std::cout<<"line completely missed"<<std::endl;
        return std::vector<T>();
    }else{
        std::vector<T> out;
        double t1 = (-m*b + sqrt(underRadical))/(m*m + 1.0);// %one of the intercept x's
        double t2 = (-m*b - sqrt(underRadical))/(m*m + 1.0);// %other intercept's x
        T i1,i2;
        i1.x=t1+c.x;
        i1.y=m*t1+b+c.y;// %intercept point 1
        i2.x = t2+c.x;
        i2.y=m*t2+b+c.y;// %intercept point 2

        out.push_back(i1);
        out.push_back(i2);

        return out;
    }

}



template <typename T>
double distance2D(const T &pose1,const T &pose2){

    double a=(pose1.x-pose2.x);
    double b=(pose1.y-pose2.y);
    return std::sqrt(a*a+b*b);
}

template <typename T>
double normSquared(const T &pose1){

    double a=(pose1.x*pose1.x);
    double b=(pose1.y*pose1.y);
    return a+b;
}
template <typename T>
double squaredistance2D(const T &pose1,const T &pose2){

    double a=(pose1.x-pose2.x);
    double b=(pose1.y-pose2.y);
    return  (a*a+b*b);
}
template <typename T,typename F>
double distance2D(const T &pose1,const F &pose2){

    double a=(pose1.x-pose2.x);
    double b=(pose1.y-pose2.y);
    return std::sqrt(a*a+b*b);
}


template <typename T>
double distance(const T &pose1,const T &pose2){
    double a=(pose1.x-pose2.x);
    double b=(pose1.y-pose2.y);
    double c=(pose1.z-pose2.z);
    return std::sqrt(a*a+b*b+c*c);
}

template <typename T,typename F>
double distance(T &pose1,F &pose2){
    double a=(pose1.x-pose2.x);
    double b=(pose1.y-pose2.y);
    double c=(pose1.z-pose2.z);
    return std::sqrt(a*a+b*b+c*c);
}

template <typename T>
double maxDistance2D(std::vector<T> &pose1){
    T base;
    base.x=0;
    base.y=0;
    double dist=0;
    for(unsigned int i=0;i<pose1.size();i++){

        double di= distance2D(pose1[i],base);

        if(di>dist){
            dist=di;
        }

    }


    return dist;
}
template <typename T>
T nearest(std::vector<T> &pose1,T &pose2,std::vector<int> &d){
    T out;
    double dist=99999;
    int idx=-1;
    for(unsigned int i=0;i<pose1.size();i++){

        if(d[i]!=0){

            double di= distance2D(pose1[i],pose2);

            if(di<dist){
                dist=di;
                idx=i;
            }

        }

    }

    if(idx>-1){
        out=pose1[idx];
        d[idx]=0;
    }
    return out;
}



template <typename T>
double distanceToSegment(T Point, T LineStart,T LineEnd){

    float LineMag;
    float U;
    T Intersection;

    LineMag = distance2D( LineEnd, LineStart );

    U = ( ( ( Point.x - LineStart.x ) * ( LineEnd.x - LineStart.x ) ) +
          ( ( Point.y  - LineStart.y  ) * ( LineEnd.y  - LineStart.y ) ) +
          ( ( Point.z - LineStart.z ) * ( LineEnd.z - LineStart.z ) ) ) /
            ( LineMag * LineMag );

    if( U < 0.0f || U > 1.0f )
        return std::max(distance2D(LineStart,Point),distance2D(Point,LineEnd));   // closest point does not fall within the line segment

    Intersection.x  = LineStart.x  + U * ( LineEnd.x  - LineStart.x  );
    Intersection.y  = LineStart.y  + U * ( LineEnd.y - LineStart.y );
    Intersection.z  = LineStart.z  + U * ( LineEnd.z - LineStart.z );

    // *Distance = Magnitude( Point, &Intersection );

    return distance2D(Point, Intersection);
}



template <typename T>
T mean(std::vector<T> &pose1){
    T out;
    out.x=0;
    out.y=0;
    out.z=0;

    for(unsigned int i=0;i<pose1.size();i++){

        out.x+=pose1[i].x;
        out.y+=pose1[i].y;
        out.z+=pose1[i].z;

    }


    out.x=out.x/((double)pose1.size());
    out.y=out.y/((double)pose1.size());
    out.z=out.z/((double)pose1.size());
    return out;
}



template <typename T>
std::vector<double> distance2D(T d,std::vector<T> &pose1){
    std::vector<double> out;
    out.resize(pose1.size());

    for(unsigned int i=0;i<pose1.size();i++){

        out[i]=PointTools::distance2D(pose1[i],d);

    }

    return out;
}


template <typename T>
std::vector<double> squaredistance2D(T d,std::vector<T> &pose1){
    std::vector<double> out;
    out.resize(pose1.size());

    for(unsigned int i=0;i<pose1.size();i++){

        out[i]=PointTools::squaredistance2D(pose1[i],d);

    }


    return out;
}

template <typename T>
std::vector<double> distance(T d,std::vector<T> &pose1){
    std::vector<double> out;
    out.resize(pose1.size());

    for(unsigned int i=0;i<pose1.size();i++){

        out[i]=PointTools::distance(pose1[i],d);

    }


    return out;
}
template <typename T>
T mean2D(std::vector<T> &pose1){
    T out;
    out.x=0;
    out.y=0;

    for(unsigned int i=0;i<pose1.size();i++){

        out.x+=pose1[i].x;
        out.y+=pose1[i].y;

    }


    out.x=out.x/((double)pose1.size());
    out.y=out.y/((double)pose1.size());
    return out;
}

template <typename T>
T std2D(std::vector<T> &pose1){
    T mean=mean2D(pose1);
    double sqr=0;

    for(unsigned int i=0;i<pose1.size();i++){

        sqr+=(distance2D(pose1[i],mean)*distance2D(pose1[i],mean));

    }


    return sqr/((double)pose1.size()-1.0);
}
template <typename T>
double angleBetween(T pose1, T pose2){
    return atan2(pose2.y-pose1.y,pose2.x-pose1.x);
}



template <typename T>
std::vector<T> segment(T pose1,T pose2,int delta){
    std::vector<T> out;


    double angle= angleBetween(pose1,pose2);
    double d=  distance2D(pose1,pose2);


    double sigma=d/(delta+1.0);
    T p=pose1;

    out.push_back(p);
    for(unsigned int i=0;i<delta;i++){
        p.x+=sigma*cos(angle);
        p.y+=sigma*sin(angle);
        out.push_back(p);
    }

    out.push_back(pose2);



    return out;
}



template <typename T>
std::string toString(T pose1){
    return "["+$("{}").format(pose1.x)+
            ","+$("{}").format(pose1.y)+
            ","+$("{}").format(pose1.z)+"]";
}
inline std::string toStringMatlab(geometry_msgs::Pose pose1);
inline std::string toStringMatlab(std::vector<geometry_msgs::Pose> &poses);

template <typename T>
std::string toStringMatlab(T pose1){
    return ""+$("{}").format(pose1.x)+
            ","+$("{}").format(pose1.y)+
            ","+$("{}").format(pose1.z);
}
template <typename T>
std::string toString(std::vector<T> &poses){

    std::stringstream str;
    for(unsigned int i=0;i<poses.size();i++){
        str<<toString(poses[i])<<";";
    }

    return str.str();
}



template <typename T>
std::string toStringMatlab(std::vector<T> &poses){

    std::stringstream str;
    for(unsigned int i=0;i<poses.size();i++){
        str<<toStringMatlab(poses[i])<<";";
    }

    return str.str();
}






template <typename T>
double distance2D(std::vector<T> &poses){

    double out=0;
    for(unsigned int i=1;i<poses.size();i++){
        out=out+poses[i-1].distance2D(poses[i]);
    }

    return out;
}





template <typename T>
T fromString(std::string v){
    T pos;
    std::vector<double> vec=$(v).numbers<double>();

    if(vec.size()==3){
        pos.x= (vec[0]);
        pos.y= (vec[1]);
        pos.z= (vec[2]);
    }
    return pos;
}

template <typename T>
std::vector<T>  fromStringToVector(std::string v){

    std::vector<T> out;
    auto lines= $(v).split(";");
    for(unsigned int i=0;i<lines.size();i++){
        out.push_back(fromString<T>(lines[i]));
    }
    return out;
}



template <typename T>
inline T diff(T p1,T p2){
    T out;
    out.x=p1.x-p2.x;
    out.y=p1.y-p2.y;
    out.z=p1.z-p2.z;
    return out;
}
template <typename T>
void minmax(const std::vector<T> &points,T &min_,T &max_){


    if(points.size()==0){
        min_.x=0;
        min_.y=0;
        max_.x=0;
        max_.y=0;
    }

    min_.x=999999;
    min_.y=999999;
    max_.x=-999999;
    max_.y=-999999;

    for(unsigned int i=0;i< points.size();i++){
        min_.x=std::min(points[i].x,min_.x);
        min_.y=std::min(points[i].y,min_.y);
        max_.x=std::max(points[i].x,max_.x);
        max_.y=std::max(points[i].y,max_.y);
    }


}

}
namespace PoseTools{


geometry_msgs::Pose  PointInLine ( geometry_msgs::Pose base, geometry_msgs::Pose v, geometry_msgs::Pose w);


std::string toString(geometry_msgs::Pose pose1);
std::string toString2(geometry_msgs::Pose pose1);
std::vector<geometry_msgs::Pose >  fromStringToVector(std::string v);
std::string  toStringCompressed(std::vector<geometry_msgs::Pose> pose1);
std::string  toStringPointCompressedMatlab(std::vector<geometry_msgs::Pose> &pose1);
geometry_msgs::Pose fromString(std::string v);

double getYaw(geometry_msgs::Pose pose);
double getYaw_old(geometry_msgs::Pose pose1);
Euler getEuler(geometry_msgs::Pose pose1);

geometry_msgs::Pose move2D(geometry_msgs::Pose lp,double La);

void updateYaw(geometry_msgs::Pose &pose,double yaw);

void  updateRPY(geometry_msgs::Pose &pose, double roll, double pitch, double yaw);

void  setRPY(geometry_msgs::Pose &pose, double roll, double pitch, double yaw);

bool similarYaw(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2,double v);

double diffYaw(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);

double distance(geometry_msgs::Pose &pose1,geometry_msgs::Pose &pose2);
double distance2D(geometry_msgs::Pose &pose1,geometry_msgs::Pose &pose2);
double squareDistance2D(geometry_msgs::Pose &pose1,geometry_msgs::Pose &pose2);
double distance2D(double x0,double y0,double x1,double y1);
double distance(double x0,double y0,double z0,double x1,double y1,double z1);

double distance(geometry_msgs::Pose &pose1,geometry_msgs::Point &pose2);

std::vector<geometry_msgs::Pose> segment(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2,int delta);


template <typename T>
inline  double distance(T &pose1,T &pose2){
    double dx=pose1.x-pose2.x;
    double dy=pose1.y-pose2.y;
    double dz=pose1.z-pose2.z;
    return sqrt(dx*dx+dy*dy+dz*dz);
}

double distance2D(std::vector<geometry_msgs::Pose> &poses);
double distance(std::vector<geometry_msgs::Pose> &poses);
geometry_msgs::Pose cross(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);

geometry_msgs::Pose operator+( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2);

bool operator==( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2);

bool operator!=( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2);

geometry_msgs::Pose operator*( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2);

geometry_msgs::Pose operator*( const geometry_msgs::Pose value1 ,double value2);

geometry_msgs::Pose operator*( double value2,const geometry_msgs::Pose value1);

geometry_msgs::Pose operator-( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2);

extern  geometry_msgs::Pose operator/( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2);

double distanceSquared(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);

double dot(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);

double angleBetween(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2);

double angleBetween(geometry_msgs::Pose pose1,Point pose2);
double distanceToLine ( geometry_msgs::Pose base, geometry_msgs::Pose v, geometry_msgs::Pose w);

geometry_msgs::Pose projectionInLine(geometry_msgs::Pose Point, geometry_msgs::Pose LineStart,geometry_msgs::Pose LineEnd);

geometry_msgs::Pose mean(geometry_msgs::Pose base,geometry_msgs::Pose h1);

double mean(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &h1);



double min(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &h1);

double min2D(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &h1);


double mean3p(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &h1);


double min3p(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &h1);
double min3p2D(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &h1);
geometry_msgs::Pose nearPose(geometry_msgs::Pose base,geometry_msgs::Pose h1,geometry_msgs::Pose h2);

double distanceToSegment(geometry_msgs::Pose Point, geometry_msgs::Pose LineStart,geometry_msgs::Pose LineEnd);

double angleBetween(double x0,double y0, double x1,double y1);


double angleBetween(geometry_msgs::Pose a,geometry_msgs::Pose b, geometry_msgs::Pose c);


template <typename PT>
double angleBetween(PT a,PT b, PT c){

    PT ab;
    ab.x=( b.x - a.x);
    ab.y=( b.y - a.y);
    PT cb;

    cb.x=b.x - c.x;
    cb.y=b.y - c.y;


    double dot = ((ab.x * cb.x) + (ab.y * cb.y)); // dot product
    double cross = ((ab.x * cb.y) - (ab.y * cb.x)); // cross product

    return  atan2(cross, dot);

}



//http://geomalgorithms.com/a05-_intersect-1.html

//#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
double perpendicular(geometry_msgs::Pose u,geometry_msgs::Pose v);// perp product  (2D)




int insideSegment( geometry_msgs::Pose P, geometry_msgs::Pose SP0,geometry_msgs::Pose SP1);

// intersect2D_2Segments(): find the 2D intersection of 2 finite segments
//    Input:  two finite segments S1 and S2
//    Output: *I0 = intersect point (when it exists)
//            *I1 =  endpoint of intersect segment [I0,I1] (when it exists)
//    Return: 0=disjoint (no intersect)
//            1=intersect  in unique point I0
//            2=overlap  in segment from I0 to I1
int  intersect2DLines( geometry_msgs::Pose S1P0,geometry_msgs::Pose S1P1,geometry_msgs::Pose S2P0,geometry_msgs::Pose &S2P1, geometry_msgs::Pose* I0, geometry_msgs::Pose* I1 );


// inSegment(): determine if a point is inside a segment
//    Input:  a point P, and a collinear segment S
//    Return: 1 = P is inside S
//            0 = P is  not inside S



std::string toStringCompressed(geometry_msgs::Pose pose1);
std::string toStringCompressed(geometry_msgs::Twist pose1);


geometry_msgs::Pose add(geometry_msgs::Pose p1,geometry_msgs::Pose p2);

geometry_msgs::Pose diff(geometry_msgs::Pose p1,geometry_msgs::Pose p2);
geometry_msgs::Pose fulladd(geometry_msgs::Pose p1,geometry_msgs::Pose p2);

geometry_msgs::Pose fulladd(geometry_msgs::Pose p1,double x,double y,double theta);
//inline void yawUpdate(geometry_msgs::Pose &p1, double theta){


//    Euler p1_=PoseTools::getEuler(p1);
//    p1_.yaw+=theta;

//    p1.orientation=p1_.toQuaternionROS();

//    //    return out;
//}

void fullUpdate(geometry_msgs::Pose &p1,double x,double y, double theta);
geometry_msgs::Pose fulldiff(geometry_msgs::Pose p1,geometry_msgs::Pose p2);
}


#endif // POSETOOLS_H
