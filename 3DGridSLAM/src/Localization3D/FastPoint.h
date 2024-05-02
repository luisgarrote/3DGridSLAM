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
#ifndef FASTPOINT_H
#define FASTPOINT_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include "StringTools.h"


template <typename T>
class FastPoint{

    bool valid;

public:
    T x;
    T y;
    T z;
public:
    FastPoint(){
        //variable[3];
        //variable=new T[3];
        clear();
        valid=true;
    }

    //    FastPoint(bool state){
    //        //variable[3];
    //        //variable=new T[3];
    //        clear();
    //        valid=state;
    //    }

    FastPoint(T var){  //to generate an invalid point
        (void)(var);
        // variable=NULL;
        //var=0;//variable=new T[3];
        //clear();
        x=0;
        y=0;
        z=0;
        valid=false;
    }


    bool hasNaN(){
        return std::isnan(x)||std::isnan(y)||std::isnan(z);
    }


    void setValid(bool r){
        valid=r;
    }

    template<typename Fx>
    static FastPoint convert(Fx &var){
        FastPoint<T> out;
        out.x=var.x;
        out.y=var.y;
        out.z=var.z;
        out.valid=true;
        return out;
    }


    FastPoint(const FastPoint<T> &var){

        //        if(var.isValid()){
        //variable=new T[3];
        x=var.x;
        y=var.y;
        z=var.z;
        valid=var.valid;
        //        }else{
        //            x=0;
        //            y=0;
        //            z=0;
        //            //  variable=NULL;
        //            valid=var.valid;

        //        }
    }

    bool isValid()const{
        return valid;
    }
    void setInvalid(){
        valid=false;}

    FastPoint(T a,T b,T c){
        //variable=new T[3];
        x=a;
        y=b;
        z=c;
        valid=true;

    }
    FastPoint(T a,T b,T c,bool stat){
        //variable=new T[3];
        x=a;
        y=b;
        z=c;
        valid=stat;

    }


    FastPoint moveTo(FastPoint B,double dist){

        double ang=angleBetween(B);

        FastPoint p;

        p.x=x+dist*cos(ang);
        p.y=y+dist*sin(ang);


        return p;

    }

    bool  equal(FastPoint<T> pt){

        if(fabs(pt.x-x)<0.001 && fabs(pt.y-y)<0.001 && fabs(pt.z-z)<0.001){
            return true;
        }

        return false;

    }

    inline T &X(){
        return (x);
    }

    inline T &Y(){
        return (y);
    }

    inline T &Z(){
        return (z);
    }



    inline T getX()const{
        return x;
    }
    inline T getY()const{
        return y;
    }
    inline T getZ()const{
        return z;
    }

    inline T getX2()const{
        return x*x;
    }
    inline T getY2()const{
        return y*y;
    }
    inline T getZ2()const{
        return z*z;
    }

    static FastPoint<T> fromString(std::string data){


        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,']');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,'[');
        Garrote::String::StringTools::removeFromString(data,'}');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,'{');
        Garrote::String::StringTools::removeFromString(data,'(');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,')');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,' ');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,';');
        //std::cout<<data<<std::endl;




        std::vector<std::string> Lines= Garrote::String::StringTools::breakIntoLines(data,',');



        if(Lines.size()>=3){ //for poses....

            return FastPoint<T>(atof(Lines[0].c_str()),atof(Lines[1].c_str()),atof(Lines[2].c_str()));

        }else{
            std::cout<<"ERROR :"<<data<<std::endl;
            return FastPoint<T>(0,0,0);

        }
    }

    static FastPoint<T> *fromStringToPointer(std::string data){


        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,']');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,'[');
        Garrote::String::StringTools::removeFromString(data,'}');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,'{');
        Garrote::String::StringTools::removeFromString(data,'(');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,')');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,' ');
        //std::cout<<data<<std::endl;
        Garrote::String::StringTools::removeFromString(data,';');
        //std::cout<<data<<std::endl;



        std::vector<std::string> Lines= Garrote::String::StringTools::breakIntoLines(data,',');


        //        for(unsigned int i=0;i<Lines.size();i++){

        //            std::cout<<Lines[i]<<std::endl;

        //        }
        if(Lines.size()>=3){

            return new FastPoint<T>(atof(Lines[0].c_str()),atof(Lines[1].c_str()),atof(Lines[2].c_str()));

        }else{
            std::cout<<"ERROR :"<<data<<std::endl;
            return new FastPoint<T>(0,0,0);

        }
    }


    std::string toString(bool show=true){

        std::stringstream str;
        if(isValid()){
            str<<"[ "<<X()<<" , "<<Y()<<" , "<<Z()<<" ]";

        }else{

            str<<"[  Invalid Point  ]";

        }

        if(show){
            std::cout<<str.str()<<std::endl;
        }

        return str.str();
    }

    operator std::string(){
        std::stringstream str;
        str<<toString(false);
        return str.str();
    }


    T *toGL(){
        T* val= new T[3];
        val[0]=x;
        val[1]=y;
        val[2]=z;
        return val;
    }

    inline void clear(){
        x=0;
        y=0;
        z=0;
    }

    inline void setX(T X_){
        x=X_;
    }

    inline void setY(T Y_){
        y=Y_;
    }
    inline void setZ(T Z_){
        z=Z_;
    }

    void set(T a,T b,T c){
        x=a;
        y=b;
        z=c;
    }

    void doRotateX(T ang){

        T aY=cos(ang)*Y() - sin(ang)*Z();
        T aZ=sin(ang)*Y() + cos(ang)*Z();
        setY(aY);
        setZ(aZ);
    }

    void doRotateY(T ang){
        T aX=cos(ang)*X() + sin(ang)*Z();
        T aZ=-sin(ang)*X() + cos(ang)*Z();
        //Y=Y;
        setX(aX);
        setZ(aZ);

    }

    void doRotateZ(T ang){
        T aX=(T)(cos(ang)*X() - sin(ang)*Y());
        T aY=(T)(sin(ang)*X() + cos(ang)*Y());
        setX(aX);
        setY(aY);
    }

    void doTranslate(T dx, T dy, T dz){
        setX(getX()+dx);
        setY(getY()+dy);
        setZ(getZ()+dz);
    }

    void setScale(T sx, T sy, T sz){

        setX(getX()*sx);
        setY(getY()*sy);
        setZ(getZ()*sz);
    }

    void zero(){
        X()=0.0;
        Y()=0.0;
        Z()=0.0;

    }
    T modulo(){

        return sqrt(X()*X()+Y()*Y()+Z()*Z());
    }

    FastPoint<T> & normalize(){


        T d=modulo();

        if(d==(T)(0)){
            d=(T)(1);
        }
        T norms=(T)(1)/d;



        multiply(norms);


        return *this;

    }


    void minimize(const FastPoint<T> &a){

        x=x>a.getX()?a.getX():x;
        y=y>a.getY()?a.getY():y;
        z=z>a.getZ()?a.getZ():z;
    }

    void maximize(const FastPoint<T> &a){
        x=x<a.getX()?a.getX():x;
        y=y<a.getY()?a.getY():y;
        z=z<a.getZ()?a.getZ():z;
    }

    void multiply(T val){

        x=x*val;
        y=y*val;
        z=z*val;

    }

    void add(T valx,T valy,T valz){

        x+=valx;
        y+=valy;
        z+=valz;

    }

    T dot( const FastPoint<T> &value){
        return (x*value.x+y*value.y+z*value.z);
    }
    FastPoint<T> cross( FastPoint<T> &value){
        FastPoint<T> newPoint;

        newPoint.x=(y*value.z - z*value.y);
        newPoint.y=(z*value.x - x*value.z);
        newPoint.z=(x*value.y - y*value.x);
        return newPoint;
    }
    T angleBetween(FastPoint<T> &value){
        return atan2(value.getY()-getY(),value.getX()-getX());
    }

    T angleBetween(T valuegetX,T valuegetY){
        return atan2(valuegetY-getY(),valuegetX-getX());
    }

    FastPoint<T> operator+( const FastPoint<T> &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result += value;            // Use += to add other to the copy.
        return result;
    }


    std::vector<FastPoint<T>> operator+( const std::vector<FastPoint<T>> &value){
        std::vector<FastPoint<T>> out;

        out.reserve(value.size());
        for(unsigned int i=0;i<value.size();i++){
            out.emplace_back(value[i].x+x,value[i].y+y,value[i].z+z);
        }
        return out;
    }

    std::vector<FastPoint<T>> operator-( const std::vector<FastPoint<T>> &value){
        std::vector<FastPoint<T>> out;

        out.reserve(value.size());
        for(unsigned int i=0;i<value.size();i++){
            out.emplace_back(value[i].x-x,value[i].y-y,value[i].z-z);
        }
        return out;
    }

    std::vector<FastPoint<T>> operator*( const std::vector<FastPoint<T>> &value){
        std::vector<FastPoint<T>> out;

        out.reserve(value.size());
        for(unsigned int i=0;i<value.size();i++){
            out.emplace_back(value[i].x*x,value[i].y*y,value[i].z*z);
        }
        return out;
    }

    std::vector<FastPoint<T>> operator/( const std::vector<FastPoint<T>> &value){
        std::vector<FastPoint<T>> out;

        out.reserve(value.size());
        for(unsigned int i=0;i<value.size();i++){
            out.emplace_back(value[i].x/x,value[i].y/y,value[i].z/z);
        }
        return out;
    }
    FastPoint<T> operator+(const  T &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result += value;            // Use += to add other to the copy.
        return result;
    }

    FastPoint<T> operator-(const  FastPoint<T> &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result -= value;            // Use += to add other to the copy.
        return result;
    }
    FastPoint<T> operator-(const T &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result -= value;            // Use += to add other to the copy.
        return result;
    }

    FastPoint<T> operator/(const  FastPoint<T> &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(result.getX()/value.getX());            // Use += to add other to the copy.
        result.setY(result.getY()/value.getY());
        result.setZ(result.getZ()/value.getZ());
        return result;
    }
    FastPoint<T> operator/(const T &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(result.getX()/value);            // Use += to add other to the copy.
        result.setY(result.getY()/value);
        result.setZ(result.getZ()/value);
        return result;
    }

    FastPoint<T> operator*(const FastPoint<T> &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(result.getX()*value.getX());            // Use += to add other to the copy.
        result.setY(result.getY()*value.getY());
        result.setZ(result.getZ()*value.getZ());
        return result;
    }
    FastPoint<T> operator*( T value){

        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(result.getX()*value);            // Use += to add other to the copy.
        result.setY(result.getY()*value);
        result.setZ(result.getZ()*value);
        return result;
    }
    FastPoint<T> operator^(const T &value){
        FastPoint<T> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(pow(result.getX(),value));            // Use += to add other to the copy.
        result.setY(pow(result.getY(),value));
        result.setZ(pow(result.getZ(),value));
        return result;

    }

    T& operator [](int idx) {
        if(idx==0){
            return x;
        }else if(idx==1){
            return y;
        }else if(idx==2){
            return z;
        }
        return x;
    }

    FastPoint<T> & operator=(const FastPoint<T> &value){

        //        if(value.isValid()){
        x=value.getX();
        y=value.getY();
        z=value.getZ();
        valid=value.isValid();
        //        }else{

        //            valid=false;
        //        }
        return *this;

    }
    bool operator==( const FastPoint<T> &other){
        return ((getX()==other.getX())&&(getY()==other.getY())&&(getZ()==other.getZ()));

    }
    bool operator!=(const  FastPoint<T> &other){
        return !(*this==other);
    }
    bool operator>(const  FastPoint<T> &other){
        return ((getX()>other.getX())&&(getY()>other.getY())&&(getZ()>other.getZ()));

    }
    bool operator<( const FastPoint<T> &other){
        return ((getX()<other.getX())&&(getY()<other.getY())&&(getZ()<other.getZ()));

    }
    bool operator<=( const FastPoint<T> &other){
        return ((getX()<=other.getX())&&(getY()<=other.getY())&&(getZ()<=other.getZ()));

    }
    bool operator>=(const  FastPoint<T> &other){
        return ((getX()>=other.getX())&&(getY()>=other.getY())&&(getZ()>=other.getZ()));

    }


    FastPoint<T> & operator+=(const  FastPoint<T> &value){

        setX(  getX()+value.getX());
        setY(  getY()+value.getY());
        setZ(  getZ()+value.getZ());

        return *this;

    }
    FastPoint<T> & operator+=( const T &value){
        setX(getX()+value);
        setY(getY()+value);
        setZ(getZ()+value);

        return *this;

    }

    FastPoint<T> & operator-=(const FastPoint<T> &value){

        setX( getX()-value.getX());
        setY( getY()-value.getY());
        setZ( getZ()-value.getZ());

        return *this;


    }
    FastPoint<T> & operator-=( const T &value){
        setX(  getX()-value);
        setY(  getY()-value);
        setZ(getZ()-value);

        return *this;
    }

//    T squaredDistance2D(FastPoint<T> m){
//        return (getX()-m.getX())*(getX()-m.getX())+(getY()-m.getY())*(getY()-m.getY());
//    }
    T squaredDistance2D(FastPoint<T> &m){
        double xx=(x-m.x);
        double yy=(y-m.y);
        double zz=(z-m.z);

        return  xx*xx+yy*yy+zz*zz;
    }
    T distance(FastPoint<T> m){
        return sqrt((x-m.x)*(x-m.x)+(y-m.y)*(y-m.y)+(z-m.z)*(z-m.z));
    }
    T distance(FastPoint<T> *m){
        return sqrt((getX()-m->getX())*(getX()-m->getX())+(getY()-m->getY())*(getY()-m->getY())+(getZ()-m->getZ())*(getZ()-m->getZ()));
    }
    inline double distance(T X_,T Y_,T Z_){
        return sqrt((getX()-X_)*(getX()-X_)+(getY()-Y_)*(getY()-Y_)+(getZ()-Z_)*(getZ()-Z_));
    }
    double distance2D(FastPoint<T> m){
        return sqrt((getX()-m.getX())*(getX()-m.getX())+(getY()-m.getY())*(getY()-m.getY()));
    }
    double distance2D(FastPoint<T> *m){
        return sqrt((getX()-m->getX())*(getX()-m->getX())+(getY()-m->getY())*(getY()-m->getY()));
    }
    inline double distance2D(T X_,T Y_){
        return sqrt((getX()-X_)*(getX()-X_)+(getY()-Y_)*(getY()-Y_));
    }
    T distanceToLine (  FastPoint<T>& v, FastPoint<T>& w)
    {
        // Return minimum distance between line segment vw and point p
        T l2 = v.squaredDistance2D(w);  // i.e. |w-v|^2 -  avoid a sqrt
        if (l2 == 0.0) return  distance(v);   // v == w case
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        T t = (*this - v).dot( w - v) / l2;
        if (t < 0.0) return 0.0+( distance(v)+ distance(w))/2.0;       // Beyond the 'v' end of the segment
        else if (t > 1.0) return  0.0+( distance(v)+ distance(w))/2.0;  // Beyond the 'w' end of the segment

        // if (t < 0.0) return  distance(v)*99;       // Beyond the 'v' end of the segment
        // else if (t > 1.0) return  distance(w)*99;  // Beyond the 'w' end of the segment
        FastPoint<T> projection = v +  (w - v)*t;  // Projection falls on the segment
        return  distance(projection);
    }

    FastPoint<T> projectionInLine( FastPoint<T>& p0,FastPoint<T>& p1){

        T X_;
        T Y_;
        X_=(getX()*p0.getX()*p0.getX() - 2.0*X()*p0.X()*p1.X() - p0.X()*p0.Y()*p1.Y() + Y()*p0.X()*p0.Y() + p0.X()*p1.Y()*p1.Y() - Y()*p0.X()*p1.Y() + X()*p1.X()*p1.X() + p1.X()*p0.Y()*p0.Y() - p1.X()*p0.Y()*p1.Y() - Y()*p1.X()*p0.Y() + Y()*p1.X()*p1.Y())/(p0.X()*p0.X() - 2.0*p0.X()*p1.X() + p1.X()*p1.X() + p0.Y()*p0.Y() - 2.0*p0.Y()*p1.Y() + p1.Y()*p1.Y());
        Y_=(p0.getX()*p0.getX()*p1.getY() - p0.X()*p1.X()*p0.Y() - p0.X()*p1.X()*p1.Y() + X()*p0.X()*p0.Y() - X()*p0.X()*p1.Y() + p1.X()*p1.X()*p0.Y() - X()*p1.X()*p0.Y() + X()*p1.X()*p1.Y() + Y()*p0.Y()*p0.Y() - 2.0*Y()*p0.Y()*p1.Y() + Y()*p1.Y()*p1.Y())/(p0.X()*p0.X() - 2.0*p0.X()*p1.X() + p1.X()*p1.X() + p0.Y()*p0.Y() - 2.0*p0.Y()*p1.Y() + p1.Y()*p1.Y());


        bool Xl=(X_< fmin(p1.getX() ,p0.getX()));
        bool Xh=(X_> fmax(p1.getX() ,p0.getX()));

        bool Yl=(Y_< fmin(p1.getY() ,p0.getY()));
        bool Yh=(Y_> fmax(p1.getY() ,p0.getY()));

        if(Xl==true||Xh==true||Yl==true||Yh==true){

            return p0.distance(*this)>p1.distance(*this)?p1:p0;
        }

        return FastPoint<T>(X_,Y_,0);
    }

    T norm(){
        return sqrt(getX()*getX()+getY()*getY()+getZ()*getZ());
    }

    friend FastPoint<T> operator*(double value,const FastPoint<T> &p)
    {
        FastPoint<T> result = p;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(p.getX()*value);            // Use += to add other to the copy.
        result.setY(p.getY()*value);
        result.setZ(p.getZ()*value);
        return result;
    }
    friend FastPoint<T> operator*(int value,const  FastPoint<T> &p)
    {
        FastPoint<T> result = p;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(p.getX()*value);            // Use += to add other to the copy.
        result.setY(p.getY()*value);
        result.setZ(p.getZ()*value);
        return result;
    }
    friend FastPoint<T> operator+(double value,const  FastPoint<T> &p)
    {
        FastPoint<T> result = p;     // Make a copy of myself.  Same as MyClass result(*this);
        result.getX(p.getX()+value);            // Use += to add other to the copy.
        result.getY(p.getY()+value);
        result.getZ(p.getZ()+value);
        return result;
    }
    friend FastPoint<T> operator-(double value,const  FastPoint<T> &p)
    {
        FastPoint<T> result = p;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(-p.getX()+value);            // Use += to add other to the copy.
        result.setY(-p.getY()+value);
        result.setZ(-p.getZ()+value);
        return result;
    }
    friend FastPoint<T> operator/(double value,const  FastPoint<T> &p)
    {
        FastPoint<T> result = p;     // Make a copy of myself.  Same as MyClass result(*this);
        result.setX(value/p.getX());            // Use += to add other to the copy.
        result.setY(value/p.getY());
        result.setZ(value/p.getZ());
        return result;
    }

    ~FastPoint(){
        //delete [] variable;
    }


    //    // dist_Point_to_Segment(): get the distance of a point to a segment
    //    //     Input:  a Point P and a Segment S (in any dimension)
    //    //     Return: the shortest distance from P to S

    T  distanceToSegment( FastPoint<T> &P0,FastPoint<T> &P1)
    {
        FastPoint<T> v = P1 - P0;
        FastPoint<T> w =(*this) - P0;

        T c1 = w.dot(v);
        if ( c1 <= 0 )
            return (c1/fabs(c1))* distance(P0);

        T c2 = v.dot(v);
        if ( c2 <= c1 )
            return (c2/fabs(c2))* distance(P1);

        T b = c1 / c2;
        FastPoint<T> Pb = P0 + b * v;
        return (b/fabs(b))* distance( Pb);
    }


    static T angleBetween(FastPoint<T> &a,FastPoint<T> &b, FastPoint<T> &c){

        FastPoint<T>   ab( b.getX() - a.getX(), b.getY() - a.getY(),0);
        FastPoint<T>   cb( b.getX() - c.getX(), b.getY() - c.getY(),0);

        double dot = ((ab.getX() * cb.getX()) + (ab.getY() * cb.getY())); // dot product
        double cross = ((ab.getX() * cb.getY()) - (ab.getY() * cb.getX())); // cross product

        return  atan2(cross, dot);

    }



    //http://geomalgorithms.com/a05-_intersect-1.html

    //#define SMALL_NUM   0.00000001 // anything that avoids division overflow
    // dot product (3D) which allows vector operations in arguments
    static T perpendicular(FastPoint<T> u,FastPoint<T> v){

        return ((u).X() * (v).Y() - (u).Y() * (v).X()) ;

    }// perp product  (2D)

    // Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
    // intersect the intersection point may be stored in the floats i_x and i_y.
    char get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
                               float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
    {
        float s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

        float s, t;
        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Collision detected
            if (i_x != NULL)
                *i_x = p0_x + (t * s1_x);
            if (i_y != NULL)
                *i_y = p0_y + (t * s1_y);
            return 1;
        }

        return 0; // No collision
    }

    // intersect2D_2Segments(): find the 2D intersection of 2 finite segments
    //    Input:  two finite segments S1 and S2
    //    Output: *I0 = intersect point (when it exists)
    //            *I1 =  endpoint of intersect segment [I0,I1] (when it exists)
    //    Return: 0=disjoint (no intersect)
    //            1=intersect  in unique point I0
    //            2=overlap  in segment from I0 to I1
    static int  intersect2DLines( FastPoint<T> &S1P0,FastPoint<T> &S1P1,FastPoint<T> &S2P0,FastPoint<T> &S2P1, FastPoint<T>* I0, FastPoint<T>* I1 )
    {
        FastPoint<T>    u = S1P1 - S1P0;
        FastPoint<T>    v = S2P1 - S2P0;
        FastPoint<T>    w = S1P0 - S2P0;
        float     D = perpendicular(u,v);

        // test if  they are parallel (includes either being a point)
        if (fabs(D) <  0.00000001) {           // S1 and S2 are parallel
            if (perpendicular(u,w) != 0 || perpendicular(v,w) != 0)  {
                return 0;                    // they are NOT collinear
            }
            // they are collinear or degenerate
            // check if they are degenerate  points
            float du = u.dot(u);
            float dv = v.dot(v);
            if (du==0 && dv==0) {            // both segments are points
                if (S1P0 !=  S2P0)         // they are distinct  points
                    return 0;
                *I0 = S1P0;                 // they are the same point
                return 1;
            }
            if (du==0) {                     // S1 is a single point
                if  (insideSegment(S1P0, S2P0,S2P1) == 0)  // but is not in S2
                    return 0;
                *I0 = S1P0;
                return 1;
            }
            if (dv==0) {                     // S2 a single point
                if  (insideSegment(S2P0, S1P0,S1P1) == 0)  // but is not in S1
                    return 0;
                *I0 = S2P0;
                return 1;
            }
            // they are collinear segments - get  overlap (or not)
            float t0, t1;                    // endpoints of S1 in eqn for S2
            FastPoint<T> w2 = S1P1 - S2P0;
            if (v.X() != 0) {
                t0 = w.X() / v.X();
                t1 = w2.X() / v.X();
            }
            else {
                t0 = w.Y() / v.Y();
                t1 = w2.Y() / v.Y();
            }
            if (t0 > t1) {                   // must have t0 smaller than t1
                float t=t0; t0=t1; t1=t;    // swap if not
            }
            if (t0 > 1 || t1 < 0) {
                return 0;      // NO overlap
            }
            t0 = t0<0? 0 : t0;               // clip to min 0
            t1 = t1>1? 1 : t1;               // clip to max 1
            if (t0 == t1) {                  // intersect is a point
                *I0 = S2P0 +  t0 * v;
                return 1;
            }

            // they overlap in a valid subsegment
            *I0 = S2P0 + t0 * v;
            *I1 = S2P0 + t1 * v;
            return 2;
        }

        // the segments are skew and may intersect in a point
        // get the intersect parameter for S1
        float     sI = perpendicular(v,w) / D;
        if (sI < 0 || sI > 1)                // no intersect with S1
            return 0;

        // get the intersect parameter for S2
        float     tI = perpendicular(u,w) / D;
        if (tI < 0 || tI > 1)                // no intersect with S2
            return 0;

        *I0 = S1P0 + sI * u;                // compute S1 intersect point
        return 1;
    }


    // inSegment(): determine if a point is inside a segment
    //    Input:  a point P, and a collinear segment S
    //    Return: 1 = P is inside S
    //            0 = P is  not inside S


    static int insideSegment( FastPoint<T> P, FastPoint<T> SP0,FastPoint<T> SP1)
    {
        if (SP0.X() != SP1.X()) {    // S is not  vertical
            if (SP0.X() <= P.X() && P.X() <= SP1.X())
                return 1;
            if (SP0.X() >= P.X() && P.X() >= SP1.X())
                return 1;
        }
        else {    // S is vertical, so test y  coordinate
            if (SP0.Y() <= P.Y() && P.Y() <= SP1.Y())
                return 1;
            if (SP0.Y() >= P.Y() && P.Y() >= SP1.Y())
                return 1;
        }
        return 0;
    }



    FastPoint<T> toPlanXY(){
        return FastPoint<T>(X(),Y(),0);

    }

    FastPoint<T> toPlanXZ(){
        return FastPoint<T>(X(),0,Z());

    }

    FastPoint<T> toPlanYZ(){

        return FastPoint<T>(0,Y(),Z());
    }


};
template<typename T>
std::ostream& operator<<(std::ostream& s,  FastPoint<T> & v)
{
    s<<(std::string)v;
    return s;
}

template<typename T>
std::vector<FastPoint<T>>operator+(std::vector<FastPoint<T>> &s,  FastPoint<T>   v)
{
std::vector<FastPoint<T>> out=s;

for(unsigned int i=0;i<s.size();i++){
out[i]+=v;
}
    return out;
}

template<typename T>
std::vector<FastPoint<T>> operator-(std::vector<FastPoint<T>> &s,  FastPoint<T>   v)
{
std::vector<FastPoint<T>> out=s;

for(unsigned int i=0;i<s.size();i++){
out[i]-=v;
}
    return out;
}

#endif // FASTPOINT_H
