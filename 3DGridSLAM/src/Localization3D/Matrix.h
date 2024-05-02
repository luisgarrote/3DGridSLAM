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
#ifndef MATRIX_H
#define MATRIX_H
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cmath>
#include <exception>
#include <iomanip>
#include <cstdio>
#include <cstring>

#include "StringTools.h"

//#include "System/Time/Time.h"
//#include "Logging/FakeLog.h";
//namespace _MATRIX_H {
//static bool state=Garrote::FakeLog::instance()->registerLine(std::string(__FILE__)+" file was compiled on "+std::string(__TIME__));
//}

#include <initializer_list>
// http://www.sanfoundry.com/cpp-program-find-inverse-matrix/


namespace Garrote {
namespace Math {
namespace Matrix {
template<class T> //or typename??
class Matrix
{
    unsigned int index;
    unsigned int indexr;
public:


    unsigned int rows;
    unsigned int columns;
    unsigned int count;
public:
    T *matrix;
    T **indexer;

    class MatrixException: public std::exception
    {
    public:
        std::string Message;
        MatrixException(std::string text){
            Message=text;
        }
        virtual const char* what() const noexcept
        {
            return Message.c_str();
        }
        virtual ~MatrixException () noexcept{}
    };

    void dispose(){

        if(matrix!=NULL){
            delete [] matrix;
            delete [] indexer;
        }

        rows=0;
        columns=0;
        count=0;
    }

public:

    Matrix(){
        matrix=NULL;
        indexer=NULL;
        rows=0;
        columns=0;
        count=0;
    }


    Matrix(double val){
        rows=1;
        columns=1;
        matrix =new T [1];
        indexer=new T*[1];
        indexer[0]=&matrix[0];
        matrix[0]=val;
        count=1;
    }



    void addNoise(  double variance,   double mean)
    {
        for(unsigned int r=0; r<rows; r++){
            for(unsigned int c=0; c<columns; c++){
                unprotected_set(r,c, unprotected_get(r,c)+((((double)rand()) / ((double)RAND_MAX)) * variance) -
                                (variance/2.0) + mean);
            }
        }
    }


    double getDeterminant(){

        if(rows != columns){
            std::cerr<<"Can't get determinant from a "<<rows<<"x"<<columns<<" (non-square) matrix!\n"<<std::endl;
            return 0;
        }

        double sign;
        double val;
        unsigned int i;

        switch(rows)
        {
        case 1:
            return  get(0,0);
        case 2:
            return  get(0,0)* get(1,1)- get(0,1)* get(1,0);
        case 3:
            // a11*(a22*a33-a23*a32) - a21*(a32*a13-a12*a33) + a31*(a12*a23-a22*a13)
            return (   get(0,0)*( get(1,1)* get(2,2) -  get(1,2)* get(2,1))
                       -  get(1,0)*( get(2,1)* get(0,2) -  get(0,1)* get(2,2))
                       +  get(2,0)*( get(0,1)* get(1,2) -  get(1,1)* get(0,2)));

        default:
            sign = 1; val = 0;
            for(i=0; i<columns; i++)
            {
                val += sign *  get(0,i) * getMinorMatrix(i).getDeterminant();
                sign *= -1;
            }
            return val;
        }

    }




    Matrix<double> getMinorMatrix(const unsigned int i) const
    {
        Matrix<double> retval(rows-1, columns-1);
        unsigned int r, c, r0, c0;
        r=0;
        for(r0=1; r0<rows; r0++)
        {
            c=0;
            for(c0=0; c0<columns; c0++)
            {
                if(c0 == i) continue;
                set(r,c++, get(r0,c0));
            }
            r++;
        }
        return retval;
    }


    Matrix(const Matrix<T> &value){

        reshapeF(value.getRows(), value.getColumns());

        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                //esta a usar o construtor
                set(i,f,value.get(i,f));
            }
        }

    }


    Matrix(unsigned int rows_, unsigned int columns_){
        rows=rows_;
        columns=columns_;
        count=rows_*columns_;
        matrix =new T [count];
        //        Garrote::Time::start("LoadTime");
        indexer=new T*[columns_];

        for(unsigned int i=0;i<columns_;i++){
            indexer[i]=&matrix[i*rows_];
        }
        //        Garrote::Time::show("LoadTime");

        //        indexer[0]=&matrix[0];
        //        zero();
    }

    void reshapeF(unsigned int rows_, unsigned int columns_){
        rows=rows_;
        columns=columns_;
        count=rows_*columns_;
        matrix =new T [count];
        //        indexer=new T*[rows_];

        //        for(unsigned int i=0;i<columns_;i++){
        //            indexer[i]=&matrix[i*columns_];
        //        }

        indexer=new T*[columns_];

        for(unsigned int i=0;i<columns_;i++){
            indexer[i]=&matrix[i*rows_];
        }
    }
    void reshape(unsigned int rows_, unsigned int columns_){
        dispose();
        rows=rows_;
        columns=columns_;
        count=rows_*columns_;
        matrix =new T [count];
        //        indexer=new T*[rows_];

        //        for(unsigned int i=0;i<columns_;i++){
        //            indexer[i]=&matrix[i*columns_];
        //        }

        indexer=new T*[columns_];

        for(unsigned int i=0;i<columns_;i++){
            indexer[i]=&matrix[i*rows_];
        }
    }




    std::pair<T,int> max(std::vector<int> nrows,int column){
        std::pair<T,int> out;
        out.first=std::numeric_limits<T>::lowest();
        out.second=-1;

        for(unsigned int i=0;i<nrows.size();i++){
            if(get(nrows[i],column)>out.first){
                out.first=get(nrows[i],column);
                out.second=i;
            }
        }
        return out;

    }


    std::pair<T,int> max_debug(std::vector<int> nrows,int column){
        std::pair<T,int> out;
        out.first=std::numeric_limits<T>::lowest();
        out.second=-1;
        std::cout<<"On max_debug"<<std::endl;

        for(unsigned int i=0;i<nrows.size();i++){
            std::cout<<i<<" "<<get(nrows[i],column)<<" "<<nrows[i]<<" "<<column<<" "<<out.first<<" "<<out.second<<" "<<std::endl;
            if(get(nrows[i],column)>out.first){
                out.first=get(nrows[i],column);
                out.second=i;
            }
        }
        return out;

    }


    std::pair<T,int> min(std::vector<int> nrows,int column){
        std::pair<T,int> out;
        out.first=std::numeric_limits<T>::max();
        out.second=-1;

        for(unsigned int i=0;i<nrows.size();i++){
            if(get(i,column)<out.first){
                out.first=get(i,column);
                out.second=i;
            }
        }
        return out;

    }



    T max(int row,int column){


        if(row<0&&column<0){
            return std::numeric_limits<T>::max();
        }

        if(row<0){
            T value=std::numeric_limits<T>::lowest();

            for(unsigned int i=0;i<rows;i++){
                value=std::max(value,get(i,column));
            }
            return value;

        }else if(column<0){

            T value=std::numeric_limits<T>::lowest();

            for(unsigned int i=0;i<columns;i++){
                value=std::max(value,get(row,i));
            }
            return value;
        }


        return max();
    }


    T min(int row,int column){


        if(row<0&&column<0){
            return std::numeric_limits<T>::lowest();
        }

        if(row<0){
            T value=std::numeric_limits<T>::max();

            for(unsigned int i=0;i<rows;i++){
                value=std::min(value,get(i,column));
            }
            return value;

        }else if(column<0){

            T value=std::numeric_limits<T>::max();
            for(unsigned int i=0;i<columns;i++){
                value=std::min(value,get(row,i));
            }
            return value;
        }
        return min();
    }






    std::pair<int,int> maxIndex(){

        std::pair<int,int> out;
        T value=matrix[0]-1;

        for(unsigned int i=0;i<getRows();i++){
            for(unsigned int g=0;g<getColumns();g++){

                if(get(i,g)>value){
                    value=get(i,g);
                    out.first=i;
                    out.second=g;
                }

            }
        }

        return out;
    }

    void leftclamp(T value){

        for(unsigned int i=0;i<count;i++){

            if(matrix[i]<value ){
                matrix[i]=value;
            }

        }
    }

    void rightclamp(T value){
        for(unsigned int i=0;i<count;i++){

            if(matrix[i]>value ){
                matrix[i]=value;
            }

        }
    }





    T max(){

        T value=matrix[0];

        for(unsigned int i=1;i<count;i++){

            if(matrix[i]>value ){
                value=matrix[i];
            }

        }

        return value;
    }


    T min(){

        T value=matrix[0];

        for(unsigned int i=1;i<count;i++){

            if(matrix[i]<value ){
                value=matrix[i];
            }
        }

        return value;

    }

    void zero(){
        memset(matrix,0,count*sizeof(T));
    }
    void zero_deprecated(){
        for(unsigned int i=0;i<count;i++){
            matrix[i]=(T)0;
        }
    }
    void zero_fill(){
        std::fill(matrix, matrix + count, 0);
    }
    void zero_memset(){
        memset(matrix,0,count*sizeof(T));
    }
    void zero_all(){
        all(0);
    }






    void all(T val){
        std::fill(matrix, matrix + count, val);
    }

    void all_deprecated(T val){
        for(unsigned int i=0;i<count;i++){
            matrix[i]=val;
        }
    }

    void all_fill(T val){
        std::fill(matrix, matrix + count, val);
        //        memset(matrix,);
        ////        for(unsigned int i=0;i<count;i++){
        ////            matrix[i]=val;
        ////        }
    }
    void setConstant(T val){
        all(val);
    }

    void setConstant(unsigned int r,unsigned int c,T val){
        set(r,c,val);
    }




    inline T directAccess(unsigned int row,unsigned int column){
        return matrix[column*rows+row];
    }


    unsigned int ik(unsigned int row,unsigned int column) const{
        return column*rows+row;
    }

    T get_old(unsigned int row,unsigned int column)const{
        if((column>=columns)||(row>=rows)){
            return T();
        }else{
            return matrix[column*rows+row];
        }
    }

    inline T get(unsigned int row,unsigned int column)const{
        if((column>=columns)||(row>=rows)){
//            std::cout<<"failed to get"<<row<<"  "<<column<<::std::endl;
            return T();
        }else{
            return indexer[column][row];
        }
    }




    //unit test
    static void unit(){



        /*
unprotected_actual performance:
unprotected_s1 SET : 0.248469
unprotected_s1 GET : 0.193433
1.59429e+07
unprotected_proposed performance:
unprotected_s2 SET : 0.193927
unprotected_s2 GET : 0.192272
1.59429e+07
actual performance:
s1 SET : 0.245071
s1 GET : 0.194278
1.59429e+07
proposed performance:
s2 SET : 0.195557
s2 GET : 0.202587
1.59429e+07
normal all : 0.0895952
changed all : 0.0324751
normal clone : 9e-08
memcpy clone : 1.17e-07
zero base : 0.0192921
zero fill : 0.0227956
zero memset : 0.0205682

*/




        std::cout<<" 5x5 Matrix Zeros "<<std::endl;
        Matrix<double> test(5,5);
        test.zero();
        test.set(0,2,5.0);

        test.set(3,0,5.0);

        test.toOutput();


        std::cout<<" 5x5 Matrix Diagonal "<<std::endl;
        test.identity();
        test.toOutput();

        std::cout<<" 5x5 Matrix with ones + operator"<<std::endl;
        test.zero();
        test=test+1;
        test.toOutput();

        std::cout<<" 5x5 Matrix with 2 operator +"<<std::endl;
        test.zero();
        test=2+test;
        test.toOutput();

        std::cout<<" 5x5 Matrix ones with times 5 operator "<<std::endl;
        test.zero();
        test=(test+1)*5;
        test.toOutput();

        std::cout<<" 3x2 Matrix Input "<<std::endl;
        test.reshape(3,2);
        test<<1,3,2,1,4,1;
        //test=(test+1)*5;
        test.toOutput();

        std::cout<<" 3x2 Matrix Input "<<std::endl;
        test.reshape(3,2);
        test.zero();
        test<<1,2,3,4,5,6;
        test.toOutput();


        std::cout<<" 2x2 Matrix Input "<<std::endl;
        test.reshape(2,2);
        test<<1,2,3,4;
        Matrix<double> test2(2,2);
        test2<<1,2,3,4;
        (test*test2).toOutput();

        test=test*test2;
        test.toOutput();



        std::cout<<" 2x2 Matrix Input "<<std::endl;
        Matrix<double> test3(2,2);
        test3<<1,2,3,4;

        Matrix<double> test4(2,2);
        test4<<4,3,2,1;

        (test3+test4).toOutput();
        (test3-test4).toOutput();
        (test3*test4).toOutput();
        std::cout<<" 2x2 ---------"<<std::endl;

        (4/test4).toOutput();
        std::cout<<" 2x2 +++++++++"<<std::endl;

        (test3/4.0).toOutput();




        std::cout<<" 2x2 Matrix Input "<<std::endl;


        //        Matrix<double> rot(4,4);
        //        rot<<0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15;
        //        //        rot<<{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
        //        //        rot<<{0,1,2,3},{4,5,6,7},{8,9,10,11},{12,13,14,15};
        //        Matrix<double> rotcx(3,3);
        //        rotcx.transcribeOperationX(rotcx);


        //        rot.toOutput();

        //        std::cout<<"New tests"<<std::endl;

        //        Matrix<double> mt(4000,4000);
        //        Matrix<double> mtr(4000,4000);

        //        Matrix<double> mtx(4,4);
        //        int ct=0;
        //        for(unsigned int i=0;i<4;i++){
        //            for(int g=0;g<4;g++){
        //                mtx.set(i,g,ct);
        //                ct++;
        //            }
        //        }
        //        mtx.toOutput();

        //        ct=0;
        //        for(unsigned int i=0;i<4;i++){
        //            for(unsigned int g=0;g<4;g++){
        //                mtx.setu(i,g,ct);
        //                ct++;
        //            }
        //        }

        //        mtx.toOutput();




        //        std::cout<<"unprotected_actual performance: "<<std::endl;

        //        Garrote::Time::start("unprotected_s1 SET");
        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                mtr.unprotected_set(i,g,i*g);
        //            }
        //        }
        //        Garrote::Time::show("unprotected_s1 SET");
        //        double vr=0;

        //        Garrote::Time::start("unprotected_s1 GET");
        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                vr+=(int) (mtr.unprotected_get(i,g)>6000);
        //            }
        //        }
        //        Garrote::Time::show("unprotected_s1 GET");
        //        std::cout<<vr<<std::endl;

        //        std::cout<<"unprotected_proposed performance:"<<std::endl;
        //        Garrote::Time::start("unprotected_s2 SET");
        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                mtr.unprotected_setu(i,g,i*g);
        //            }
        //        }
        //        Garrote::Time::show("unprotected_s2 SET");

        //        double vrr=0;
        //        Garrote::Time::start("unprotected_s2 GET");

        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                vrr+=(int) (mtr.unprotected_getu(i,g)>6000);
        //            }
        //        }

        //        Garrote::Time::show("unprotected_s2 GET");
        //        std::cout<<vrr<<std::endl;

        //        std::cout<<"actual performance: "<<std::endl;

        //        Garrote::Time::start("s1 SET");
        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                mt.set(i,g,i*g);
        //            }
        //        }
        //        Garrote::Time::show("s1 SET");
        //        vr=0;

        //        Garrote::Time::start("s1 GET");
        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                vr+=(int) (mt.get(i,g)>6000);
        //            }
        //        }
        //        Garrote::Time::show("s1 GET");
        //        std::cout<<vr<<std::endl;

        //        std::cout<<"proposed performance:"<<std::endl;
        //        Garrote::Time::start("s2 SET");
        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                mt.setu(i,g,i*g);
        //            }
        //        }
        //        Garrote::Time::show("s2 SET");

        //        vrr=0;
        //        Garrote::Time::start("s2 GET");

        //        for(unsigned int i=0;i<4000;i++){
        //            for(unsigned int g=0;g<4000;g++){
        //                vrr+=(int) (mt.getu(i,g)>6000);
        //            }
        //        }

        //        Garrote::Time::show("s2 GET");
        //        std::cout<<vrr<<std::endl;


        //        Matrix<double> mtrx(4,4);

        //        Garrote::Time::start("normal all");
        //        mtrx.all(22);
        //        Garrote::Time::show("normal all");

        //        Garrote::Time::start("changed all");
        //        mtrx.all_fill(22);
        //        Garrote::Time::show("changed all");




        //        Garrote::Time::start("normal all11");
        //        mtrx.all(2244.35);
        //        Garrote::Time::show("normal all11");

        //        Garrote::Time::start("changed all22");
        //        mtrx.all_fill(2244.35);
        //        Garrote::Time::show("changed all22");




        //        Garrote::Time::start("normal clone");
        //        auto ptx1=mtrx.cloneMatrix();
        //        Garrote::Time::show("normal clone");
        //        delete ptx1;

        //        Garrote::Time::start("memcpy clone");
        //        auto ptx=mtrx.cloneMatrixwithmemcpy();
        //        Garrote::Time::show("memcpy clone");
        //        delete ptx;

        //        Garrote::Time::start("zero base");
        //        mtrx.zero();
        //        Garrote::Time::show("zero base");
        //        Garrote::Time::start("zero base");
        //        mtrx.zero();
        //        Garrote::Time::show("zero base");

        //        Garrote::Time::start("zero fill");
        //        mtrx.zero_fill();
        //        Garrote::Time::show("zero fill");
        //        Garrote::Time::start("zero fill");
        //        mtrx.zero_fill();
        //        Garrote::Time::show("zero fill");
        //        Garrote::Time::start("zero memset");
        //        mtrx.zero_memset();
        //        Garrote::Time::show("zero memset");
        //        Garrote::Time::start("zero memset");
        //        mtrx.zero_memset();
        //        Garrote::Time::show("zero memset");
        //        Garrote::Time::start("zero all");
        //        mtrx.all(0);
        //        Garrote::Time::show("zero all");
        //        Garrote::Time::start("zero all");
        //        mtrx.all(0);
        //        Garrote::Time::show("zero all");
        //        Garrote::Time::start("zero all+");
        //        mtrx.zero_all();
        //        Garrote::Time::show("zero all+");
        //        Garrote::Time::start("zero all+");
        //        mtrx.zero_all();
        //        Garrote::Time::show("zero all+");


        Matrix<double> A(5,5);

        A<<    17 ,   24   ,  1  ,   8 ,   15,
                23   ,  5   ,  7  ,  14  ,  16,
                4   ,   6   , 13   , 20  ,  22,
                10   ,  12   , 19   , 21  ,   3,
                11   ,  18   , 25    , 2   ,  9;

        std::cout<<A.toString()<<std::endl;
        auto Ai=A.inverse();
        std::cout<<Ai.toString()<<std::endl;

        std::cout<<(A*Ai).toString()<<std::endl;








    }


    void update(  int row,  int column,T value) {

        if((column<columns&&column>=0)&&(row<rows&&row>=0)){
            indexer[column][row]=value;
        }else if(row==-1 &&column==-1){
            all(value);
        }else if(row==-1){

            for(unsigned int i=0;i<rows;i++){
                indexer[column][i]=value;
            }

        }else if(column==-1){
            for(unsigned int i=0;i<columns;i++){
                indexer[i][row]=value;
            }
        }
    }




    inline void set(unsigned int row,unsigned int column,T value)const{

        if((column<columns)&&(row<rows)){
            indexer[column][row]=value;
        }
    }
    inline void setI(unsigned int row,unsigned int column,T value)const{

        if((column<rows)&&(row<columns)){
            indexer[row][column]=value;
        }
    }


    inline void unprotected_set(unsigned int row,unsigned int column,T value)const{
        indexer[column][row]=value;
    }

    inline T unprotected_get(unsigned int row,unsigned int column)const{
        return indexer[column][row];
    }


    bool setbool(unsigned int row,unsigned int column,T value)const{

        if((column>=columns)||(row>=rows)){
            return false;
        }else{
            matrix[column*rows+row]=value;
            return true;

        }
    }


    inline void unprotected_set_old(unsigned int row,unsigned int column,T value)const{
        matrix[column*rows+row]=value;
    }

    inline T unprotected_get_old(unsigned int row,unsigned int column)const{
        return   matrix[column*rows+row];
    }
    void set_old(unsigned int row,unsigned int column,T value)const{

        if((column<columns)&&(row<rows)){
            matrix[column*rows+row]=value;
        }
    }

    double get(unsigned int index_)const{
        if(index_>=count){
            return (T)0;
        }else{
            return matrix[index_];
        }
    }

    inline  double unprotected_get(unsigned int index_)const{
        return matrix[index_];
    }


    void set(unsigned int index_,T value)const{

        if(index_<count){
            matrix[index_]=value;
        }
    }
    inline void unprotected_set(unsigned int index_,T value)const{
        matrix[index_]=value;
    }



    Matrix<T>* clone() {

        Matrix<T> * newm=new Matrix<T>(rows,columns);
        //count=rows*columns;

        memcpy(newm->matrix,matrix,count*sizeof(T));
        //        for(unsigned int i=0;i<count;i++){
        //            newm->unprotected_set(i,unprotected_get(i));
        //        }
        return newm;
    }

    template <typename F>
    Matrix<F>* cloneCast() {

        Matrix<F> * newm=new Matrix<F>(rows,columns);
        //count=rows*columns;

        for(unsigned int i=0;i<count;i++){
            newm->matrix[i]=(F)matrix[i];
        }

        return newm;
    }


    T * cloneMatrix() const{

        T * newm=new  T [count];
        memcpy(newm,matrix,count*sizeof(T));
        return newm;

    }


    T * cloneMatrix_deprecated() const{

        T * newm=new  T [rows*columns];

        for(unsigned int i=0;i<count;i++){
            newm[i]=matrix[i];
        }

        return newm;

    }


    T * cloneMatrixwithmemcpy() const{

        T * newm=new  T [count];
        memcpy(newm,matrix,count*sizeof(T));
        return newm;

    }


    std::string toString(){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<std::setprecision(5)<<unprotected_get(i,f)<<" ";
            }
            out<<std::endl;
        }
        return out.str();
    }

    std::string toStringLine(){
        std::ostringstream out(std::stringstream::in | std::stringstream::out);
        char buffer[256];  // make sure this is big enough!!!
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){

                snprintf(buffer, sizeof(buffer), "%.5f", unprotected_get(i,f) );
                // out<<std::setprecision(9)<<get(i,f)<<" ";
                out<<buffer<<" ";
            }
            if(i==(rows-1)){

            }else{
                out<<"; ";
            }
        }
        return out.str();
    }


    std::string toStringFull(){
        std::ostringstream out(std::stringstream::in | std::stringstream::out);
        char buffer[256];  // make sure this is big enough!!!
        out<<rows<<" "<<columns<<" ";
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){

                snprintf(buffer, sizeof(buffer), "%.5f", unprotected_get(i,f) );
                out<<buffer<<" ";
            }
            if(i==(rows-1)){
            }else{
                out<<"; ";
            }
        }
        return out.str();
    }


    void fromStringFull(std::string str){

        std::cout<<str<<std::endl;

        String::StringTools::removeFromString(str,';');

        std::cout<<str<<std::endl;
        std::vector<std::string> vals=String::StringTools::breakIntoLines(str,' ');

        unsigned int rowsl=atoi(vals[0].c_str());
        unsigned int colsl=atoi(vals[1].c_str());

        reshape(rowsl,colsl);

        unsigned int indexL=2;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                T val=atof(vals[indexL].c_str());
                unprotected_set(i,f,val);
                indexL++;
            }
        }
    }

    std::string toStringArray(){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<unprotected_get(i,f)<<" ; ";
            }
            //out<<std::endl;
        }
        return out.str();
    }


    void toOutput(){
        //std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                std::cout<<unprotected_get(i,f)<<" ";
            }
            std::cout<<std::endl;
        }
    }

    unsigned int getRows()const{
        return rows;
    }
    unsigned int getColumns()const{
        return columns;
    }

    Matrix<T> transpose(){
        Matrix<T> output(columns,rows);
        //count=rows*columns;

        for (unsigned int i = 0; i < rows; i++)
        {
            for (unsigned int j = 0; j < columns; j++)
            {
                output.unprotected_set(j,i,unprotected_get(i,j));
                // b[j][i] = a[i][j];
            }
        }

        //        for(unsigned int i=0;i<count;i++){
        //            output.set(i,get(i));
        //        }

        return output;
    }



    double determinant()
    {
        double det = 0;
        //double **pd = m_pData;
        switch (rows)
        {
        case 2:
        {
            det = indexer[0][0] * indexer[1][1] - indexer[0][1] * indexer[1][0];
            return det;
        }
            break;
        case 3:
        {
            /***
                 a b c
                 d e f
                 g h i

                 a b c a b c
                 d e f d e f
                 g h i g h i

                 // det (A) = aei + bfg + cdh - afh - bdi - ceg.
                 ***/
            double a = indexer[0][0];
            double b = indexer[1][0];
            double c = indexer[2][0];
            double d = indexer[0][1];
            double e = indexer[1][1];
            double f = indexer[2][1];
            double g = indexer[0][2];
            double h = indexer[1][2];
            double i = indexer[2][2];
            det = (a * e * i + b * f * g + c * d * h);
            det = det - a * f * h;
            det = det - b * d * i;
            det = det - c * e * g;
            return det;
        }
            break;
        case 4:
        {
            Matrix<double> *temp[4];
            for (int i = 0; i < 4; i++)
                temp[i] = new Matrix<double>( 3, 3);
            for (int k = 0; k < 4; k++)
            {
                for (int i = 1; i < 4; i++)
                {
                    int j1 = 0;
                    for (int j = 0; j < 4; j++)
                    {
                        if (k == j)
                            continue;
                        temp[k]->set(i - 1,j1++,get(i,j));
                    }
                }
            }
            det = this->indexer[0][0] * temp[0]->determinant()
                    - this->indexer[1][0] * temp[1]->determinant()
                    + this->indexer[2][0] * temp[2]->determinant()
                    - this->indexer[3][0] * temp[3]->determinant();
            for (int i = 0; i < 4; i++)
                delete temp[i];
            return det;
        }
            break;
        case 5:
        {
            Matrix<double> *temp[5];
            for (int i = 0; i < 5; i++)
                temp[i] = new Matrix<double>( 4, 4);
            for (int k = 0; k < 5; k++)
            {
                for (int i = 1; i < 5; i++)
                {
                    int j1 = 0;
                    for (int j = 0; j < 5; j++)
                    {
                        if (k == j)
                            continue;
                        //                            temp[k]->m_pData[i - 1][j1++]
                        //                                    = this->m_pData[i][j];
                        temp[k]->set(i - 1,j1++,get(i,j));

                    }
                }
            }
            det = this->indexer[0][0] * temp[0]->determinant()
                    - this->indexer[1][0] * temp[1]->determinant()
                    + this->indexer[2][0] * temp[2]->determinant()
                    - this->indexer[3][0] * temp[3]->determinant()
                    + this->indexer[4][0] * temp[4]->determinant();

            for (int i = 0; i < 5; i++)
                delete temp[i];

            return det;
        }
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        default:
        {
            int DIM = rows;
            Matrix<double> **temp = new Matrix<double>*[DIM];
            for (int i = 0; i < DIM; i++)
                temp[i] = new Matrix<double>(DIM - 1, DIM - 1);
            for (int k = 0; k < DIM; k++)
            {
                for (int i = 1; i < DIM; i++)
                {
                    int j1 = 0;
                    for (int j = 0; j < DIM; j++)
                    {
                        if (k == j)
                            continue;
                        //                            temp[k]->m_pData[i - 1][j1++]
                        //                                    = this->m_pData[i][j];
                        temp[k]->set(i - 1,j1++,get(i,j));
                    }
                }
            }
            det = 0;
            for (int k = 0; k < DIM; k++)
            {
                if ((k % 2) == 0)
                    det = det + (this->indexer[k][0]
                            * temp[k]->determinant());
                else
                    det = det - (this->indexer[k][0]
                            * temp[k]->determinant());
            }
            for (int i = 0; i < DIM; i++)
                delete temp[i];
            delete[] temp;
            return det;
        }
            break;
        }
    }



    //    std::vector<double> eigen(){


    //    }

    Matrix<double> coFactor()
    {
        Matrix<double> cofactor( rows, columns);
        if (rows != columns)
            return cofactor;
        if (rows < 2)
            return cofactor;
        else if (rows == 2)
        {
            cofactor.indexer[0][0] = indexer[1][1];
            cofactor.indexer[1][0] = -indexer[0][1];
            cofactor.indexer[0][1] = -indexer[1][0];
            cofactor.indexer[1][1] = indexer[0][0];
            return cofactor;
        }
        else if (rows >= 3)
        {
            int DIM = rows;
            Matrix<double> ***temp = new Matrix<double>**[DIM];
            for (int i = 0; i < DIM; i++)
                temp[i] = new Matrix<double>*[DIM];
            for (int i = 0; i < DIM; i++)
                for (int j = 0; j < DIM; j++)
                    temp[i][j] = new Matrix<double>( DIM - 1, DIM - 1);
            for (int k1 = 0; k1 < DIM; k1++)
            {
                for (int k2 = 0; k2 < DIM; k2++)
                {
                    int i1 = 0;
                    for (int i = 0; i < DIM; i++)
                    {
                        int j1 = 0;
                        for (int j = 0; j < DIM; j++)
                        {
                            if (k1 == i || k2 == j)
                                continue;
                            //                            temp[k1][k2]->m_pData[i1][j1++]
                            //                                    = this->m_pData[i][j];
                            temp[k1][k2]->set(i1,j1++,get(i,j));
                        }
                        if (k1 != i)
                            i1++;
                    }
                }
            }
            bool flagPositive = true;
            for (int k1 = 0; k1 < DIM; k1++)
            {
                flagPositive = ((k1 % 2) == 0);
                for (int k2 = 0; k2 < DIM; k2++)
                {
                    if (flagPositive == true)
                    {
                        cofactor.indexer[k2][k1]
                                = temp[k1][k2]->determinant();
                        flagPositive = false;
                    }
                    else
                    {
                        cofactor.indexer[k2][k1]
                                = -temp[k1][k2]->determinant();
                        flagPositive = true;
                    }
                }
            }
            for (int i = 0; i < DIM; i++)
                for (int j = 0; j < DIM; j++)
                    delete temp[i][j];
            for (int i = 0; i < DIM; i++)
                delete[] temp[i];
            delete[] temp;
        }
        return cofactor;
    }
    Matrix<T> inverse(){
        Matrix<T> output(rows,columns);
        ///////
        ///////

        if (rows != columns){
            output.zero();
            return output;
        }

        double det = determinant();
        //        std::cout<<"det "<<det<<std::endl;
        Matrix<T> cofactor=coFactor();


        for (unsigned int i = 0; i < rows; i++)
        {
            for (unsigned int j = 0; j < columns; j++)
            {
                //                inv.m_pData[j][i] = cofactor.m_pData[i][j] / det;
                output.set(j,i,cofactor.get(i,j)/det);
            }
        }
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        ///////
        return output;
    }

    Matrix<T> &identity(){
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                if(i==f){
                    unprotected_set(i,f,(T)1);

                }else{
                    unprotected_set(i,f,(T)0);
                }
            }
        }
        return *this;
    }

    bool isOrthogonal(){

        Matrix<T> result=(*this)*this->transpose();

        Matrix<T> eye(result.getRows(),result.getColumns());

        eye.identity();

        return result==identity();

    }

    bool isSquare(){

        return rows==columns;
    }



    void transcribeOperationX2(const Matrix<T> &value){



        std::cout<<"START"<<std::endl;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<value.getColumns();f++){


                std::cout<<"out["<<ik(i,f)<<"]="<<std::endl;
                for( unsigned int k=0;k<value.getRows();k++){

                    if(fabs(value.getu(k,f))<0.001){
                        //std::cout<<"+A["<<ik(i,k)<<"]*B["<<value.ik(k,f)<<"]";

                    }else if(fabs(value.getu(k,f)-1.0)<0.001){
                        std::cout<<"+A["<<ik(i,k)<<"]";

                    }else{
                        std::cout<<"+A["<<ik(i,k)<<"]*B["<<value.ik(k,f)<<"]";
                    }
                    //                        val=val+matrix->ik(i,k)*value.matrix->ik(k,f);
                }
                std::cout<<";"<<std::endl;
                // result.set(i,f,val);
            }
        }
        std::cout<<"END"<<std::endl;


    }








    void transcribeOperationX(const Matrix<T> &value){



        std::cout<<"START"<<std::endl;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<value.getColumns();f++){


                std::cout<<"out["<<ik(i,f)<<"]="<<std::endl;
                for( unsigned int k=0;k<value.getRows();k++){

                    std::cout<<"+A["<<ik(i,k)<<"]*B["<<value.ik(k,f)<<"]";

                    //                        val=val+matrix->ik(i,k)*value.matrix->ik(k,f);
                }
                std::cout<<";"<<std::endl;
                // result.set(i,f,val);
            }
        }
        std::cout<<"END"<<std::endl;


    }





    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //



    Matrix<T> operator+(const Matrix<T> &valueal)const{

        //        if(valueal.getColumns()!=columns|| valueal.getRows()!=rows){
        //            throw "";
        //        }

        Matrix<T> output=*this;
        output.add(valueal);
        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){
        //                output.unprotected_set(i,f,unprotected_get(i,f)+valueal.unprotected_get(i,f));
        //            }
        //        }
        return output;
    }



    Matrix<T> operator+(const T &value)
    {
        Matrix<T> output=*this;

        output.add(value);
        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){

        //                output.unprotected_set(i,f,unprotected_get(i,f)+value);

        //            }
        //        }
        return output;
    }


    Matrix<T> operator-(const Matrix<T> &value){

        //        if(value.getColumns()!=columns|| value.getRows()!=rows){
        //            throw "";
        //        }

        Matrix<T> output=*this;
        output.take(value);

        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){

        //                output.unprotected_set(i,f,unprotected_get(i,f)-value.unprotected_get(i,f));
        //            }
        //        }
        return output;
    }
    Matrix<T>  operator-(const T &value){

        Matrix<T> output=*this;
        output.take(value);
        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){

        //                output.unprotected_set(i,f,unprotected_get(i,f)-value);
        //            }
        //        }
        return output;

    }

    Matrix<T> operator/(const Matrix<T> &value){

        return this*value.inverse();
    }

    Matrix<T> operator/(const T &value){

        Matrix<T> output=*this;

        output.divide(value);
        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){

        //                output.unprotected_set(i,f,unprotected_get(i,f)/value);
        //            }
        //        }
        return output;

    }




    Matrix<T> operator*(const Matrix<T> &value){

        if(value.getRows()!=columns){
            std::stringstream strd;
            strd<<" Can't multiply with different dimensions "<<rows<<"x"<<columns<<" vs "<<value.getRows()<<"x"<<value.getColumns()<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }

        Matrix<T> output(rows,value.getColumns());

        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<value.getColumns();f++){

                T val=(T)0;
                for( unsigned int k=0;k<value.getRows();k++){
                    val+=unprotected_get(i,k)*value.unprotected_get(k,f);
                }
                output.unprotected_set(i,f,val);
            }
        }
        return output;
    }

    template <typename F>
    Matrix<T> operator*(const Matrix<F> &value){

        if(value.getRows()!=columns){
            std::stringstream strd;
            strd<<" Can't multiply with different dimensions "<<rows<<"x"<<columns<<" vs "<<value.getRows()<<"x"<<value.getColumns()<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }

        Matrix<T> output(rows,value.getColumns());

        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<value.getColumns();f++){

                T val=(T)0;
                for( unsigned int k=0;k<value.getRows();k++){
                    val+=unprotected_get(i,k)*value.unprotected_get(k,f);
                }
                output.unprotected_set(i,f,val);
            }
        }
        return output;
    }



    template <typename Tf>
    Matrix<T> operator*(Tf point){


        if(getRows()==4 &&getColumns()==4){
            Matrix<T> output(getRows(),1);

            output[0]=matrix[0]*point.x + matrix[4]*point.y + matrix[8]*point.z + matrix[12];
            output[1]=matrix[1]*point.x + matrix[5]*point.y + matrix[9]*point.z + matrix[13];
            output[2]=matrix[2]*point.x + matrix[6]*point.y + matrix[10]*point.z + matrix[14];
            output[3]=1;
            return output;
        }
        Matrix<T> value(getRows(),1);

        value.set(0,0,point.x);
        value.set(1,0,point.y);

        if(getRows()>=3){
            value.set(2,0,point.z);
        }

        if(value.getRows()!=columns){
            std::stringstream strd;
            strd<<" Can't multiply with different dimensions "<<rows<<"x"<<columns<<" vs "<<value.getRows()<<"x"<<value.getColumns()<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }

        Matrix<T> output(rows,value.getColumns());

        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<value.getColumns();f++){

                T val=(T)0;
                for( unsigned int k=0;k<value.getRows();k++){
                    val+=unprotected_get(i,k)*value.unprotected_get(k,f);
                }
                output.unprotected_set(i,f,val);
            }
        }
        return output;
    }



    friend Matrix<T> operator*(double value, const Matrix<T> &p)
    {

        Matrix<T> output=p;

        output.multiply(value);


        //        for(unsigned int i=0;i<p.rows;i++){
        //            for(unsigned int f=0;f<p.columns;f++){

        //                output.unprotected_set(i,f,p.unprotected_get(i,f)*value);
        //            }
        //        }
        return output;
    }


    friend Matrix<T> operator+(double value, const Matrix<T> &p)
    {

        Matrix<T> output=p;

        output.add(value);

        //        for(unsigned int i=0;i<p.rows;i++){
        //            for(unsigned int f=0;f<p.columns;f++){

        //                output.unprotected_set(i,f,p.unprotected_get(i,f)+value);
        //            }
        //        }
        return output;
    }
    friend Matrix<T> operator-(double value, const Matrix<T> &p)
    {

        //        Matrix<T> output(p.rows,p.columns);

        //        for(unsigned int i=0;i<p.rows;i++){
        //            for(unsigned int f=0;f<p.columns;f++){

        //                output.unprotected_set(i,f,p.unprotected_get(i,f)-value);
        //            }
        //        }

        Matrix<T> output=p;

        output.take(value);
        return output;
    }





    friend Matrix<T> operator/(double value, const Matrix<T> &p)
    {

        Matrix<T> output=p;

        output.divide(value);
        return output;

    }




    Matrix<T> operator*(const T &value){

        Matrix<T> output=*this;
        output.multiply(value);
        return output;

    }



    Matrix<T> & operator*=(const Matrix<T> &value){
        this=this*value;
        return *this;
    }

    Matrix<T> & operator/=(const Matrix<T> &value){
        this=this/value;
        return *this;
    }
    Matrix<T> & operator/=(const T &value){

        divide(value);
        return *this;
    }




    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //




    inline void multiply(const T &value){
        for(unsigned int i =0;i<count;i++){
            matrix[i]*=value;
        }
    }
    inline void divide(const T &value){
        for(unsigned int i =0;i<count;i++){
            matrix[i]/=value;
        }
    }

    inline  void add(const T &value){
        for(unsigned int i =0;i<count;i++){
            matrix[i]+=value;
        }
    }
    inline  void take(const T &value){
        for(unsigned int i =0;i<count;i++){
            matrix[i]-=value;
        }
    }

    inline void pow(const T &value){
        for(unsigned int i=0;i<count;i++){
            matrix[i]=pow(matrix[i],value);
        }
    }


    inline  void add(const Matrix<T> &value){
        if(rows!=value.getRows() || columns!=value.getColumns()){

            std::stringstream strd;
            strd<<" Can't add with different dimensions"<<rows<<"x"<<columns<<" vs "<<value.getRows()<<"x"<<value.getColumns()<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }else{
            for(unsigned int i =0;i<count;i++){
                matrix[i]+=value.matrix[i];
            }
        }
    }
    inline  void take(const Matrix<T> &value){

        if(rows!=value.getRows() || columns!=value.getColumns()){

            std::stringstream strd;
            strd<<" Can't take with different dimensions"<<rows<<"x"<<columns<<" vs "<<value.getRows()<<"x"<<value.getColumns()<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }else{
            for(unsigned int i =0;i<count;i++){
                matrix[i]-=value.matrix[i];
            }
        }
    }




    Matrix<T> & operator*=(const T &value){
        multiply(value);
        return *this;
    }

    //T & operator[](const unsigned int row,const unsigned int column);

    Matrix<T> & operator^=(const T &value){
        pow(value);
        return *this;
    }

    Matrix<T> operator^(const T &value){

        Matrix<T> output=*this;
        output.pow(value);

        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){
        //                set(i,f,pow(unprotected_get(i,f),value));

        //            }
        //        }


        return output;

    }


    Matrix<T> & operator=(const T &value){

        dispose();
        matrix =new T[1];
        rows=1;
        columns=1;
        count=1;
        matrix[0]=value;
        indexer=new T*[1];
        indexer[0]=&matrix[0];


        return *this;
    }
    Matrix<T> & operator=(const Matrix<T> &value){


        if(rows!=value.getRows() || columns!=value.getColumns()){
            reshape(value.getRows(), value.getColumns());
        }
        //        for(unsigned int i=0;i<rows;i++){
        //            for(unsigned int f=0;f<columns;f++){
        //                //esta a usar o construtor
        //                set(i,f,value.unprotected_get(i,f));
        //            }
        //        }

        //        for(unsigned int i=0;i<count;i++){
        //            matrix[i]=value.matrix[i];
        //        }

        memcpy(matrix,value.matrix,count*sizeof(T));

        rows=value.getRows();
        columns=value.getColumns();

        return *this;
    }
    bool operator==(const Matrix<T> &other){
        if(this->columns!=other.columns){
            return false;
        }
        if(this->rows!=other.rows){
            return false;
        }


        for(unsigned int i=0;i<this->rows;i++){
            for(unsigned int g=0;g<this->columns;g++){

                if(get(i,g)!=other.get(i,g)){
                    return false;
                }
            }
        }

        return true;
    }
    bool operator!=(const Matrix<T> &other){
        return !(this->operator ==(other));
    }
    //    bool operator>(const Matrix<T> &other);
    //    bool operator<(const Matrix<T> &other);
    //    bool operator<=(const Matrix<T> &other);
    //    bool operator>=(const Matrix<T> &other);

    Matrix<T> & operator+=(const Matrix<T> &value){
        add(value);
        return *this;
    }
    Matrix<T> & operator+=(const T &value){
        add(value);
        return *this;
    }

    Matrix<T> & operator-=(const Matrix<T> &value){
        take(value);
        return *this;
    }
    Matrix<T> & operator-=(const T &value){
        take(value);
        return *this;
    }


    void append(std::initializer_list<T> l) {

        std::vector<T> v(l);

        if(l->size()<count){
            index=0;
            indexr=0;
            for(unsigned int i=0;i<v.size();i++){

                if(index>=columns){
                    index=0;
                    indexr+=1;
                }
                indexer[index][indexr]=v[i];
            }
        }else{

            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw MatrixException(strd.str());

        }

    }

    //void operator, (T value);
    T &operator()( unsigned int row, unsigned int column)const{

        if(row<rows && column<columns){
            return indexer[column][row];
            //            return matrix[column*rows+row];
        }else{
            std::stringstream strd;
            strd<<" Tried to access ("<<row<<","<<column<<")  from matrix "<<rows<<" x "<<columns<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }
    }

    T &operator()( unsigned int indexL)const{

        if(indexL<count){
            return matrix[indexL];
        }else{
            std::stringstream strd;
            strd<<" Tried to access ("<<indexL<<")  from matrix "<<rows<<" x "<<columns<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }
    }



    T &operator[]( unsigned int indexL){

        if(indexL<count){
            return matrix[indexL];
        }else{
            std::stringstream strd;
            strd<<" Tried to access ("<<indexL<<")  from matrix "<<rows<<" x "<<columns<<std::endl;
            std::cout<<strd.str();
            throw MatrixException(strd.str());
        }
    }







    Matrix<T> &operator << ( std::initializer_list<T> l){



        if(l->size()==count){
            std::vector<T> v(l);

            index=0;
            indexr=0;
            for(unsigned int i=0;i<v.size();i++){
                if(index>=columns){
                    index=0;
                    indexr+=1;
                }
                indexer[index][indexr]=v[i];
                index=index+1;

            }
        }else if(l->size()==columns){
            std::vector<T> v(l);
            index=0;
            indexr=0;
            for(unsigned int i=0;i<v.size();i++){
                if(index>=columns){
                    index=0;
                    indexr+=1;
                }
                indexer[index][indexr]=v[i];
                index=index+1;

            }


        }else{
            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw MatrixException(strd.str());
        }

        //val.clear();
        return *this;

    }


    Matrix<T> &operator << ( T val){


        index=0;
        indexr=0;
        matrix[0]=val;
        return *this;

    }


    Matrix<T> &operator, (std::initializer_list<T> l){

        if(l.size()!=columns){
            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw MatrixException(strd.str());

        }


        if((index+indexr*columns)>=count){
            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw MatrixException(strd.str());
        }
        std::vector<T> v(l);

        for(unsigned int i=0;i<v.size();i++){
            if(index>=columns){
                index=0;
                indexr+=1;
            }
            indexer[index][indexr]=v[i];
            index=index+1;

        }


        return *this;
    }

    Matrix<T> &operator, (T value){

        //std::cout<<"estou no operador ,"<<std::endl;

        index=index+1;

        if((index+indexr*columns)>=count){
            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw MatrixException(strd.str());
        }

        if(index>=columns){
            index=0;
            indexr+=1;
        }
        // int vf =columns*(index%rows)+(int)floor(((double)index/(double)rows));
        //matrix[rows*(index%columns)+(int)floor(((double)index/(double)columns))]=value;
        indexer[index][indexr]=value;

        //std::cout<<index<<"   "<<vf<<"   "<<columns*(index%rows)+(int)floor(((double)index/(double)rows)) <<"   "<<value<<"    "<<matrix[vf]<<std::endl;

        //trows_=0;


        return *this;
    }


    ~Matrix(){

        if((rows!=0)||(columns!=0)){
            rows=0;
            columns=0;
            //std::cout<<" Deleting size "<<rows<<"  "<<columns<<std::endl;
            delete [] matrix;
            delete [] indexer;


        }
    }

    //    T min(){

    //        T out=9999999;
    //        for(unsigned int i=0;i<count;i++){
    //            matrix[i]<out?out=matrix[i]: ;
    //        }

    //        return out;

    //    }
    //    T max(){

    //        T out=-9999999;
    //        for(unsigned int i=0;i<count;i++){
    //            matrix[i]>out?out=matrix[i]: ;
    //        }

    //        return out;

    //    }



    void normalize() {

        double mag=0.0;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                mag+= unprotected_get(i,f)*unprotected_get(i,f);
            }
        }
        //        if(mag<0){
        //            mag*=(T)(-1);
        //        }
        mag = sqrt(mag);

        if(mag<0.0001){
            mag=1;
        }
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                unprotected_set(i,f,unprotected_get(i,f)/mag);
            }
        }
    }
    //    void unit(){

    //        std::cout<<" 5x5 Matrix Zeros "<<std::endl;
    //        Matrix<double> test(5,5);
    //        test.zero();
    //        test.set(0,2,5.0);

    //        test.set(2,0,5.0);

    //        test.toOutput();


    //        std::cout<<" 5x5 Matrix Diagonal "<<std::endl;
    //        test.identity();
    //        test.toOutput();

    //        std::cout<<" 5x5 Matrix with ones + operator"<<std::endl;
    //        test.zero();
    //        test=test+1;
    //        test.toOutput();

    //        std::cout<<" 5x5 Matrix with 2 operator +"<<std::endl;
    //        test.zero();
    //        test=2+test;
    //        test.toOutput();

    //        std::cout<<" 5x5 Matrix ones with times 5 operator "<<std::endl;
    //        test.zero();
    //        test=(test+1)*5;
    //        test.toOutput();

    //        std::cout<<" 3x2 Matrix Input "<<std::endl;
    //        test.reshape(3,2);
    //        test<<1,1,2,1,1,1;
    //        //test=(test+1)*5;
    //        test.toOutput();

    //        std::cout<<" 3x2 Matrix Input "<<std::endl;
    //        test.reshape(3,2);
    //        test.zero();
    //        test<<1,2,3,4,5,6;
    //        test.toOutput();


    //        std::cout<<" 2x2 Matrix Input "<<std::endl;
    //        test.reshape(2,2);
    //        test<<1,2,3,4;
    //        Matrix<double> test2(2,2);
    //        test2<<1,2,3,4;
    //        (test*test2).toOutput();

    //        test=test*test2;
    //        test.toOutput();







    //        //test=(test+1)*5;





    //    }






};


template<class T,unsigned int R,unsigned int C>
class ExtendedMatrix: public Matrix<T>{

public:
    ExtendedMatrix(){
        this->rows=R;
        this->columns=C;
        this->matrix =new T [R*C];
        this->indexer=new T*[C];

        for(unsigned int i=0;i<C;i++){
            this->indexer[i]=&this->matrix[i*R];
        }



        for(unsigned int i=0;i<R*C;i++){

            this->matrix[i]=0;
        }
        this->count=R*C;
    }

    ExtendedMatrix(unsigned int i,unsigned int f){
        this->rows=std::max(R,i);
        this->columns=std::max(C,f);
        this->matrix =new T [this->rows*this->columns];

        this->indexer=new T*[C];

        for(unsigned int h=0;h<C;h++){
            this->indexer[h]=&this->matrix[h*R];
        }

        for(unsigned int h=0;h<R*C;h++){

            this->matrix[h]=0;
        }
        this->count=this->rows*this->columns;
    }

    ExtendedMatrix(const Matrix<T> &value){

        this->reshapeF(value.getRows(), value.getColumns());

        for(unsigned int i=0;i<this->rows;i++){
            for(unsigned int f=0;f<this->columns;f++){
                //esta a usar o construtor
                this->unprotected_set(i,f,this->value.unprotected_get(i,f));
            }
        }

    }


    ExtendedMatrix(T *data){
        this->rows=R;
        this->columns=C;
        this->matrix =new T [R*C];

        this->indexer=new T*[C];

        for(unsigned int i=0;i<C;i++){
            this->indexer[i]=&this->matrix[i*R];
        }

        for(unsigned int i=0;i<R*C;i++){

            this->matrix[i]=data[i];
        }
        this->count=R*C;
    }

    void resize(unsigned int r, unsigned int g){
        this->reshape(r,g);
    }

    ExtendedMatrix(const T *data){
        this->rows=R;
        this->columns=C;
        this->matrix =new T [R*C];

        this->indexer=new T*[C];

        for(unsigned int i=0;i<C;i++){
            this->indexer[i]=&this->matrix[i*R];
        }

        for(unsigned int i=0;i<R*C;i++){

            this->matrix[i]=data[i];
        }
        this->count=R*C;
    }

    void min(ExtendedMatrix< T, R, C> & var){

        for(unsigned int i=0;i<this->count;i++){
            this->matrix[i]=std::min(this->matrix[i],var.matrix[i]);

        }

    }
    void max(ExtendedMatrix< T, R, C> & var){

        for(unsigned int i=0;i<this->count;i++){
            this->matrix[i]=std::max(this->matrix[i],var.matrix[i]);

        }

    }


    void setOnes(){
        this->all(1);
    }


    T coeffRef(unsigned in){
        return this->get(in);
    }


    unsigned int rows_(){
        return this->rows;
    }
    unsigned int cols_(){
        return this->columns;
    }

    template<class F,int X, int Z>
    friend std::ostream& operator << (std::ostream& s, ExtendedMatrix<F,X,Z> &v);
};

template<class T,int R, int C>
std::ostream& operator << (std::ostream& s, ExtendedMatrix<T,R,C> &v)
{
    s << v.toString();
    return s;
}

//#define _COMMA ,
//#define _LEFTWING <
//#define _RIGHTWING <


//#define Matrix_LEFTWINGT_COMMAR_COMMAC_RIGHTWING ExtendedMatrix_LEFTWINGT_COMMAR_COMMAC_RIGHTWING

}}}

#endif // MATRIX_H




/*



#include <math.h>
#include <assert.h>


#include "Matrix.h"
#include "Spline.h"

FLOAT_T PMatrix::epsilon = 1e-20;

#ifndef SQ
#define SQ(x) ((x)*(x))
#endif


PMatrix::PMatrix(const bg::string& str)
{
  bg::string m = str.strip();
  bg::strlist l;
  std::vector<PVector> vecs;
  unsigned int i, j, r, c;

  rows = 0;
  columns = 0;
  data = NULL;

  if(!m.startsWith("("))
    BGTHROWC(bg::FormatError, "Matrix doesn't start with '('");
  if(!m.endsWith(")"))
    BGTHROWC(bg::FormatError, "Matrix doesn't end with ')'");
  m = m.substr(1, m.size()-2);

  l = m.split("),");
  if(!l.size())
    BGTHROWC(bg::FormatError, "0-row matrix?!");
  for(i=0; i<l.size()-1; i++)
    vecs.insert(vecs.end(), PVector(l[i].strip()+")"));
  vecs.insert(vecs.end(), PVector(l[l.size()-1].strip()));

  r = vecs.size();
  c = vecs[0].getNumberOfElements();

  for(i=1; i<vecs.size(); i++)
    if(vecs[i].getNumberOfElements() != c)
      BGTHROWC(bg::FormatError, "Non-uniform vector sizes");

  *this = PMatrix(r, c);
  for(i=0; i<r; i++)
    for(j=0; j<c; j++)
      (*this)[i][j] = vecs[i][j];
}


PMatrix&
PMatrix::operator=(const PMatrix& m)
{
  unsigned int row, col;
  if(this == &m) return *this; // catch 'm=m'

  // need to modify size?
  if(rows != m.rows || columns != m.columns)
    {
      // destroy ourselves...
      if(data)
    {
      for(row=0; row<rows; row++)
        delete data[row];
      delete data;
    }
      // ...and re-create.
      rows = m.rows;
      columns = m.columns;
      data = new FLOAT_T*[rows];
      for(row=0; row<rows; row++)
    data[row] = new FLOAT_T[columns];
    }
  // copy contents
  for(row=0; row<rows; row++)
    for(col=0; col<columns; col++)
      data[row][col] = m.data[row][col];
  return *this;
}

void
PMatrix::fill(const FLOAT_T value)
{
  for(unsigned int row=0; row<rows; row++)
    for(unsigned int col=0; col<columns; col++)
      data[row][col] = value;
}


 * Arithmetic operations


double
PMatrix::trace(void) const
{
  assert(rows == columns);
  double tr = 0;
  for(unsigned int i=0; i<rows; i++)
    tr += data[i][i];
  return tr;
}

void
PMatrix::getDiagonalMatrices(PMatrix& ll, PMatrix& ur)
  throw(InvalidMatrixError)
{
  if(rows != columns)
    BGTHROWC(InvalidMatrixError,
         "Cannot get diagonal matrices from a %dx%d matrix (must be NxN)!",
         rows, columns);
  std::vector<unsigned int> rowpermutations;
  unsigned int row;
  for(row=0; row<rows; row++)
    rowpermutations.insert(rowpermutations.end(), row);

  return getDiagonalMatrices(ll, ur, rowpermutations);
}

void
PMatrix::getDiagonalMatrices(PMatrix& ll, PMatrix& ur,
                 std::vector<unsigned int> rowpermutations)
  throw(InvalidMatrixError)
{
  if(rows != columns)
    BGTHROWC(InvalidMatrixError,
         "Cannot get diagonal matrices from a %dx%d matrix (must be NxN)!",
         rows, columns);
  if(rowpermutations.size() != rows)
    BGTHROWC(InvalidMatrixError,
         "%d-element row permutation vector for a "
         "%d-row matrix? You crazy?",
         rowpermutations.size(), rows);

  unsigned int row, column, index;

  if(ll.rows != rows && ll.columns != columns)
    ll = PMatrix(rows, columns);
  if(ur.rows != rows && ur.columns != columns)
    ur = PMatrix(rows, columns);
  for(index=0; index<rows; index++)
    {
      row = rowpermutations[index];
      for(column=0; column<columns; column++)
    {
      if(index == column)
        {
          ll.data[index][column] = 1.0;
          ur.data[index][column] = data[row][column];
        }
      else if(index < column)
        {
          ll.data[index][column] = 0.0;
          ur.data[index][column] = data[row][column];
        }
      else
        {
          ll.data[index][column] = data[row][column];
          ur.data[index][column] = 0.0;
        }
    }
    }
}

void
PMatrix::luDecompose(PMatrix& a,
             std::vector<unsigned int>& index,
             int* evenodd)
  throw(InvalidMatrixError)
{
  if(rows != columns)
    BGTHROWC(InvalidMatrixError,
         "Cannot LU-decompose a %dx%d matrix (must be NxN)!",
         rows, columns);

  int d;
  int n = rows;
  int i, imax, j, k;
  FLOAT_T big, dum, sum, temp;

#ifdef _MSC_VER
  FLOAT_T* vv = new FLOAT_T[n];
#else
  FLOAT_T vv[n]; // stores implicit scaling.
#endif

  FLOAT_T **matrix;

  // adapt storage if necessary
  if(index.size() != (unsigned)n)
    index = std::vector<unsigned int>(n);

  a = *this;
  matrix = a.data;

  // go.

  d = 1;

  for(i = 0; i < n; i++) {
    big = 0.0;
    for(j = 0; j < n; j++)
      if((temp = fabs(matrix[i][j])) > big) big = temp;
    if(big == 0.0) {
      matrix[i][i] = 1e-6;
      big = matrix[i][i];
    }
    vv[i] = 1.0 / big;
  }

  for(j = 0; j < n; j++) {

    for(i = 0; i < j; i++) {
      sum = matrix[i][j];
      for(k = 0; k < i; k++)
    sum -= matrix[i][k] * matrix[k][j];
      matrix[i][j] = sum;
    }

    big = 0.0;

    // BLOCK 2
    for(i = j; i < n; i++) {
      sum = matrix[i][j];
      for(k = 0; k < j; k++)
    sum -= matrix[i][k] * matrix[k][j];
      matrix[i][j] = sum;
      if((dum = vv[i] * fabs(sum)) >= big) {
    big = dum;
    imax = i;
      }
    }

    // BLOCK 3
    if(j != imax) {
      for(k = 0; k < n; k++) {
    dum = matrix[imax][k];
    matrix[imax][k] = matrix[j][k];
    matrix[j][k] = dum;
      }
      d = -d;
      vv[imax] = vv[j];
    }

    // BLOCK 4
    index[j] = imax;
    assert(matrix[j][j] != 0);

    // BLOCK 5
    if(j != n - 1) {
      dum = 1.0 / (matrix[j][j]);
      for(i = j + 1; i < n; i++)
    matrix[i][j] *= dum;
    }

  }

  if(evenodd) *evenodd = d;

#ifdef _MSC_VER
  delete[] vv;
#endif
}

PVector
PMatrix::luBacksubst(std::vector<unsigned int>& indx,
             PVector b)
  throw(InvalidMatrixError)
{
  if(rows != columns)
    BGTHROWC(InvalidMatrixError,
         "Cannot LU-backsubstitute a %dx%d matrix (must be NxN)!",
         rows, columns);
  if(indx.size() != rows)
    BGTHROWC(InvalidMatrixError,
         "Invalid size for rowpermutations vector (is %d, should be %d)!",
         indx.size(), rows);
  if(b.getNumberOfElements() != rows)
    BGTHROWC(InvalidMatrixError,
         "Invalid size for result vector b (is %d, should be %d)!",
         b.getNumberOfElements(), rows);

  int n = rows;
  int i, ii, ip, j;
  FLOAT_T sum;
  FLOAT_T **a = data;

  ii = 0;
  for(i=0; i < 4; i++)
    {
      ip = indx[i];
      sum = b[ip];
      b[ip] = b[i];
      if(ii >= 0)
    for(j=ii; j<=i-1; j++)
      sum -= a[i][j] * b[j];
      else if(sum)
    ii = i;

      b[i] = sum;
    }
  for(i=n-1; i>=0; i--)
    {
      sum = b[i];
      for(j=i+1; j<n; j++)
    sum -= a[i][j] * b[j];
      b[i] = sum / a[i][i];
    }

  return b;
}

PMatrix
PMatrix::inverse(void)
  throw(PMatrix::InvalidMatrixError)
{
  PMatrix retval(rows,rows), lu;
  unsigned int i, j, n;
  std::vector<unsigned int> rowpermutations;
  n = rows;
  PVector col(n);

  luDecompose(lu, rowpermutations, NULL);
  for(j=0; j<n; j++)
    {
      for(i=0; i<n; i++) col[i] = 0.0;
      col[j] = 1.0;
      col = lu.luBacksubst(rowpermutations, col);
      for(i=0; i<n; i++)
    retval[i][j] = col[i];
    }
  return retval;
}

PVector
PMatrix::leastSquares(PVector& values)
{
  PMatrix AT, ATA, ATAi;
  PVector c;

  AT = transpose();
  ATA = AT * (*this);
  ATAi = ATA.inverse();
  return ATAi * (AT * values);
}

void
PMatrix::qrDecomposition(PMatrix &R, bool &singular) const
{
  unsigned int i,j,k;
  float scale,sigma,sum,tau;

  assert(rows == columns);

#define SIGN(a, b) ((b) >= 0 ? fabs(a) : -fabs(a))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#define SQ(x) ((x)*(x))

  R = *this;

#ifdef _MSC_VER
    float* d = new float[rows];
    float* c = new float[rows];
#else
  float d[rows];
  float c[rows];
#endif

  singular = false;
  for (k=0; k<rows; k++)
    {
      scale=0.0;
      for (i=k; i<rows; i++)
    scale=MAX(scale,fabs(R[i][k]));
      if (scale == 0.0)
    { // Singular case.
      singular = true;
      c[k]=d[k]=0.0;
    }
      else
    { // Form Qk and Qk-A.
      for (i=k; i<rows; i++)
        R[i][k] /= scale;
      for (sum=0.0,i=k; i<rows; i++)
        sum += SQ(R[i][k]);
      sigma = SIGN(sqrt(sum),R[k][k]);
      R[k][k] += sigma;
      c[k]=sigma*R[k][k];
      d[k] = -scale*sigma;
      for (j=k+1; j<rows; j++)
        {
          for (sum=0.0,i=k; i<rows; i++)
        sum += R[i][k]*R[i][j];
          tau=sum/c[k];
          for (i=k; i<rows; i++)
        R[i][j] -= tau*R[i][k];
        }
    }
      }

  d[rows-1] = R[rows-1][rows-1]; if (d[rows-1] == 0.0) singular = true;

  for(i=0; i<rows; i++)
    for(j=0; j<columns; j++)
      if(i == j) R[i][j] = d[i];
      else if(i>j) R[i][j] = 0;

#ifdef _MSC_VER
  delete[] d;
  delete[] c;
#endif

  return;
}

void
PMatrix::rqDecompose(PMatrix &R, PMatrix &Q)
{
  // see Hartley/Zisserman, p. 552.
  // we use the sine, not the cosine equations, since the cosine has
  // the nasty habit of producing a rotation matrix of diag(-1, -1,
  // 1) and a rotated R matrix from matrices that actually have no
  // rotation at all.
  double s, angle;

  s = data[2][1]/sqrt(SQ(data[2][2])+SQ(data[2][1]));
  angle = asin(s);
  PMatrix Qx = makeGivensRotationX(angle);
  R = (*this)*Qx;

  s = R[2][0]/sqrt(SQ(R[2][2])+SQ(R[2][0]));
  angle = asin(s);
  PMatrix Qy = makeGivensRotationY(angle);
  R = R * Qy;

  s = R[1][0]/sqrt(SQ(R[1][1])+SQ(R[1][0]));
  angle = asin(s);
  PMatrix Qz = makeGivensRotationZ(angle);
  R = R * Qz;

  Q = Qx.transpose()*Qy.transpose()*Qz.transpose();
}

double pythag(double a, double b)
{
  double absa, absb; absa = fabs(a); absb = fabs(b);
  if(absa > absb) return absa*sqrt(1.0+SQ(absb/absa));
  else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQ(absa/absb)));
}

// from numerical recipes in c

void
PMatrix::singularValueDecomposition(PMatrix& U, PMatrix& W, PMatrix& V)
{
  int i, j;
  double **a, **v; double *w;
  a = new double*[rows+1];

  for(unsigned i=1; i<=rows; i++)
    {
      a[i] = new double[columns+1];
      for(unsigned j=1; j<=columns; j++)
    a[i][j] = (*this)[i-1][j-1];
    }
  w = new double[columns+1];
  v = new double*[columns+1];
  for(unsigned i=1; i<=columns; i++)
    {
      v[i] = new double[columns+1];
      for(unsigned j=1; j<=columns; j++)
    v[i][j] = 0;
    }

  int m = rows, n = columns;

  int flag, its, jj, k, l, nm;
  double anorm, c, f, g, h, s, scale, x, y, z;
  PVector rv1(n);
  g = scale = anorm = 0.0;

  for (i=1;i<=n;i++)
    {
      l=i+1;
      rv1[i]=scale*g;
      g=s=scale=0.0;
      if (i <= m)
    {
      for (k=i;k<=m;k++) scale += fabs(a[k][i]);
      if (scale) {
        for (k=i;k<=m;k++) {
          a[k][i] /= scale;
          s += a[k][i]*a[k][i];
        }
        f=a[i][i];
        g = -SIGN(sqrt(s),f);
        h=f*g-s; a[i][i]=f-g;
        for (j=l;j<=n;j++) {
        for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
        f=s/h;
        for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
          }
        for (k=i;k<=m;k++) a[k][i] *= scale;
      }
    }
      w[i]=scale *g;
      g=s=scale=0.0;
      if (i <= m && i != n) {
    for (k=l;k<=n;k++) scale += fabs(a[i][k]);
    if (scale) {
      for (k=l;k<=n;k++) {
        a[i][k] /= scale;
        s += a[i][k]*a[i][k];
      }
      f=a[i][l];
      g = -SIGN(sqrt(s),f);
      h=f*g-s;
      a[i][l]=f-g;
      for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
      for (j=l;j<=m;j++) {
        for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
        for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
      }
      for (k=l;k<=n;k++) a[i][k] *= scale;
    }
      }
      anorm=MAX(anorm,(fabs(w[i])+fabs(rv1[i])));
    }
  for (i=n;i>=1;i--) { // Accumulation of right-hand transformations.
    if (i < n) {
      if (g) {
    for (j=l;j<=n;j++) // Double division to avoid possible underflow.
      v[j][i]=(a[i][j]/a[i][l])/g;
    for (j=l;j<=n;j++) {
      for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
      for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
    }
      }
      for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
    }
    v[i][i]=1.0;
    g=rv1[i];
    l=i;
  }
  for (i=MIN(m,n);i>=1;i--) { // Accumulation of left-hand
                   // transformations.
    l=i+1;
    g=w[i];
    for (j=l;j<=n;j++) a[i][j]=0.0;
    if (g) {
      g=1.0/g;
      for (j=l;j<=n;j++) {
    for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
    f=(s/a[i][i])*g;
    for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
      }
      for (j=i;j<=m;j++) a[j][i] *= g;
    } else for (j=i;j<=m;j++) a[j][i]=0.0;
    ++a[i][i];
  }
  for (k=n;k>=1;k--) { // Diagonalization of the bidiagonal form:
    // Loop over singular values, and over allowed iterations.
    for (its=1;its<=90;its++) {
      flag=1;
      for (l=k;l>=1;l--) { // Test for splitting.
    nm=l-1; // Note that rv1[1] is always zero.
    if ((float)(fabs(rv1[l])+anorm) == anorm) {
      flag=0;
      break;
    }
    if ((float)(fabs(w[nm])+anorm) == anorm) break;
      }
      if (flag) {
    c=0.0; // Cancellation of rv1[l], if l>1.
    s=1.0;
    for (i=l;i<=k;i++) {
      f=s*rv1[i];
      rv1[i]=c*rv1[i];
      if ((float)(fabs(f)+anorm) == anorm) break;
      g=w[i];
      h=pythag(f,g);
      w[i]=h;
      h=1.0/h;
      c=g*h;
      s = -f*h;
      for (j=1;j<=m;j++) {
        y=a[j][nm];
        z=a[j][i];
        a[j][nm]=y*c+z*s;
        a[j][i]=z*c-y*s;
      }
    }
      }
      z=w[k];
      if (l == k) { // Convergence.
    if (z < 0.0) { // Singular value is made nonnegative.
      w[k] = -z;
      for (j=1;j<=n;j++) v[j][k] = -v[j][k];
    }
    break;
      }
      if (its == 30)
    BGTHROWC(bg::error, "no convergence in 30 svdcmp iterations");
      x=w[l]; // Shift from bottom 2-by-2 minor.
      nm=k-1;
      y=w[nm];
      g=rv1[nm];
      h=rv1[k];
      f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
      g=pythag(f,1.0);
      f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
      c=s=1.0; // Next QR transformation:
      for (j=l;j<=nm;j++) {
    i=j+1;
    g=rv1[i];
    y=w[i];
    h=s*g;
    g=c*g;
    z=pythag(f,h);
    rv1[j]=z;
    c=f/z;
    s=h/z;
    f=x*c+g*s;
    g = g*c-x*s;
    h=y*s; y*=c;
    for (jj=1;jj<=n;jj++) {
      x=v[jj][j];
      z=v[jj][i];
      v[jj][j]=x*c+z*s;
      v[jj][i]=z*c-x*s;
    }
    z=pythag(f,h);
    w[j]=z; // Rotation can be arbitrary if z=0.
    if (z) {
      z=1.0/z;
      c=f*z;
      s=h*z;
    }
    f=c*g+s*y;
    x=c*y-s*g;
    for (jj=1;jj<=m;jj++) {
      y=a[jj][j];
      z=a[jj][i];
      a[jj][j]=y*c+z*s;
      a[jj][i]=z*c-y*s;
    }
      }
      rv1[l]=0.0;
      rv1[k]=f;
      w[k]=x;
    }
  }
}


#if 0
void
PMatrix::singularValueDecompositionOrig(PMatrix& a, PMatrix& w, PMatrix& v)
{
  a = *this;
  w = PMatrix::makeUnity(columns);
  V = PMatrix::makeUnity(columns);

  int m = rows, n = columns;

  int flag, i, its, j, jj, k, l, nm;
  double anorm, c, f, g, h, s, scale, x, y, z;
  PVector rv1(n);
  g = scale = anorm = 0.0;

  for (i=1;i<=n;i++)
    {
      l=i+1;
      rv1[i]=scale*g;
      g=s=scale=0.0;
      if (i <= m)
    {
      for (k=i;k<=m;k++) scale += fabs(a[k][i]);
      if (scale) {
        for (k=i;k<=m;k++) {
          a[k][i] /= scale;
          s += a[k][i]*a[k][i];
        }
        f=a[i][i];
        g = -SIGN(sqrt(s),f);
        h=f*g-s; a[i][i]=f-g;
        for (j=l;j<=n;j++) {
        for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
        f=s/h;
        for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
          }
        for (k=i;k<=m;k++) a[k][i] *= scale;
      }
    }
      w[i]=scale *g;
      g=s=scale=0.0;
      if (i <= m && i != n) {
    for (k=l;k<=n;k++) scale += fabs(a[i][k]);
    if (scale) {
      for (k=l;k<=n;k++) {
        a[i][k] /= scale;
        s += a[i][k]*a[i][k];
      }
      f=a[i][l];
      g = -SIGN(sqrt(s),f);
      h=f*g-s;
      a[i][l]=f-g;
      for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
      for (j=l;j<=m;j++) {
        for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
        for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
      }
      for (k=l;k<=n;k++) a[i][k] *= scale;
    }
      }
      anorm=MAX(anorm,(fabs(w[i])+fabs(rv1[i])));
    }
  for (i=n;i>=1;i--) { // Accumulation of right-hand transformations.
    if (i < n) {
      if (g) {
    for (j=l;j<=n;j++) // Double division to avoid possible underflow.
      v[j][i]=(a[i][j]/a[i][l])/g;
    for (j=l;j<=n;j++) {
      for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
      for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
    }
      }
      for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
    }
    v[i][i]=1.0;
    g=rv1[i];
    l=i;
  }
  for (i=MIN(m,n);i>=1;i--) { // Accumulation of left-hand
                   // transformations.
    l=i+1;
    g=w[i];
    for (j=l;j<=n;j++) a[i][j]=0.0;
    if (g) {
      g=1.0/g;
      for (j=l;j<=n;j++) {
    for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
    f=(s/a[i][i])*g;
    for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
      }
      for (j=i;j<=m;j++) a[j][i] *= g;
    } else for (j=i;j<=m;j++) a[j][i]=0.0;
    ++a[i][i];
  }
  for (k=n;k>=1;k--) { // Diagonalization of the bidiagonal form:
    // Loop over singular values, and over allowed iterations.
    for (its=1;its<=30;its++) {
      flag=1;
      for (l=k;l>=1;l--) { // Test for splitting.
    nm=l-1; // Note that rv1[1] is always zero.
    if ((float)(fabs(rv1[l])+anorm) == anorm) {
      flag=0;
      break;
    }
    if ((float)(fabs(w[nm])+anorm) == anorm) break;
      }
      if (flag) {
    c=0.0; // Cancellation of rv1[l], if l>1.
    s=1.0;
    for (i=l;i<=k;i++) {
      f=s*rv1[i];
      rv1[i]=c*rv1[i];
      if ((float)(fabs(f)+anorm) == anorm) break;
      g=w[i];
      h=pythag(f,g);
      w[i]=h;
      h=1.0/h;
      c=g*h;
      s = -f*h;
      for (j=1;j<=m;j++) {
        y=a[j][nm];
        z=a[j][i];
        a[j][nm]=y*c+z*s;
        a[j][i]=z*c-y*s;
      }
    }
      }
      z=w[k];
      if (l == k) { // Convergence.
    if (z < 0.0) { // Singular value is made nonnegative.
      w[k] = -z;
      for (j=1;j<=n;j++) v[j][k] = -v[j][k];
    }
    break;
      }
      if (its == 30)
    BGTHROWC(bg::error, "no convergence in 30 svdcmp iterations");
      x=w[l]; // Shift from bottom 2-by-2 minor.
      nm=k-1;
      y=w[nm];
      g=rv1[nm];
      h=rv1[k];
      f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
      g=pythag(f,1.0);
      f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
      c=s=1.0; // Next QR transformation:
      for (j=l;j<=nm;j++) {
    i=j+1;
    g=rv1[i];
    y=w[i];
    h=s*g;
    g=c*g;
    z=pythag(f,h);
    rv1[j]=z;
    c=f/z;
    s=h/z;
    f=x*c+g*s;
    g = g*c-x*s;
    h=y*s; y*=c;
    for (jj=1;jj<=n;jj++) {
      x=v[jj][j];
      z=v[jj][i];
      v[jj][j]=x*c+z*s;
      v[jj][i]=z*c-x*s;
    }
    z=pythag(f,h);
    w[j]=z; // Rotation can be arbitrary if z=0.
    if (z) {
      z=1.0/z;
      c=f*z;
      s=h*z;
    }
    f=c*g+s*y;
    x=c*y-s*g;
    for (jj=1;jj<=m;jj++) {
      y=a[jj][j];
      z=a[jj][i];
      a[jj][j]=y*c+z*s;
      a[jj][i]=z*c-y*s;
    }
      }
      rv1[l]=0.0;
      rv1[k]=f;
      w[k]=x;
    }
  }
  free_vector(rv1,1,n);
}
#endif

PMatrix
PMatrix::getCofactorMatrix()
{
  PMatrix m(rows, columns);
  for(unsigned int i=0; i<rows; i++)
    for(unsigned int j=0; j<columns; j++)
        m[i][j] = pow(-1.0, (int) (i+j))*strikeOut(i, j).determinant();
  return m;
}

void
PMatrix::multiplyWithRotationMatrixAndTranspose(const unsigned int row,
                        const unsigned int col,
                        const FLOAT_T cosinus,
                        const FLOAT_T sinus) {
  if(rows != columns)
    BGTHROWC(bg::FormatError, "only implemented for symmetric matrices!");

  //J = I and
  //J[row][row] = J[col][col] = cosinus and
  //J[row][col] = sinus and J[col][row] = -sinus

  //computes: *this = J.transpose() * (*this) * J

  //first step: (*this) = (*this) * J
  this->multiplyWithRotationMatrix(row, col, cosinus, sinus);

  //second step: *this = J.transpose() * (*this) =>swap row and col
  FLOAT_T temp;
  for (unsigned int i = 0; i < rows; i++) {
    temp = (*this)[row][i];
    (*this)[row][i] = (*this)[row][i] * cosinus - (*this)[col][i] * sinus;
    (*this)[col][i] = temp * sinus + (*this)[col][i] * cosinus;
  }
}

void
PMatrix::multiplyWithRotationMatrix(const unsigned int row,
                    const unsigned int col,
                    const FLOAT_T cosinus,
                    const FLOAT_T sinus) {
  if(rows != columns)
    BGTHROWC(bg::FormatError, "only implemented for symmetric matrices!");

  //J = I and
  //J[row][row] = J[col][col] = cosinus and
  //J[row][col] = sinus and J[col][row] = -sinus

  //computes: *this = (*this) * J
  FLOAT_T temp;
  for (unsigned int i = 0; i < rows; i++) {
    temp = (*this)[i][row];
    (*this)[i][row] = (*this)[i][row] * cosinus - (*this)[i][col] * sinus;
    (*this)[i][col] = temp * sinus + (*this)[i][col] * cosinus;
  }
}

void
PMatrix::jacobiDecomposition(PMatrix& diag, PMatrix& orth){

  if(rows != columns)
    BGTHROWC(bg::FormatError, "Jacobian decomposition only implemented for symmetric and square matrices!");

  orth = makeUnity(rows);

  diag = *this;

  unsigned int maxRow, maxColumn;
  int iterations = 0;
  FLOAT_T MaxElement, alpha;

  const FLOAT_T cosPI_4 = cos(M_PI_4);
  const FLOAT_T sinPI_4 = sin(M_PI_4);
  FLOAT_T cosinus, sinus;

  do {
    iterations++;
    MaxElement = -1.0;
    for (unsigned int i = 0; i < rows; i++)
      for (unsigned int j = 0; j < columns; j++)
        if (i != j && fabs(diag[i][j]) > MaxElement) {
          maxRow = i;
          maxColumn = j;
          MaxElement = fabs(diag[i][j]);
        }
    //std::cout << "MaxEl: " << MaxElement << " @(" << maxRow+1 << ", " << maxColumn+1 << ")\n";

    // Special case: catch mpp == mqq (see Pardos Witz)
    if (fabs (diag[maxRow][maxRow] - diag[maxColumn][maxColumn]) < epsilon) {
      cosinus = cosPI_4;
      sinus = sinPI_4;
      //cerr << "m[maxRow][maxRow] == m[maxColumn][maxColumn] -> Kann Eigenwerte nicht berechnen!!!\n";
    } else {
      alpha =
        atan(2.0 * diag[maxRow][maxColumn] /
             (diag[maxColumn][maxColumn] -
              diag[maxRow][maxRow])) * 0.5;
      cosinus = cos(alpha);
      sinus = sin(alpha);
    }

    //diag = JT * diag * J;
    diag.multiplyWithRotationMatrixAndTranspose(maxRow, maxColumn,
                        cosinus, sinus);

    //orth = orth * J;
    orth.multiplyWithRotationMatrix(maxRow, maxColumn, cosinus, sinus);

  }
  while (MaxElement > 0.0001f && iterations < 20);
  //while (MaxElement > epsilon && iterations < 20);

  if (iterations >= 200)
    BGTHROWC(bg::error, "No convergence after 200 iterations in jacobiDecomposition");

  return;
}

PMatrix
PMatrix::getSubmatrix(const unsigned int r, const unsigned int c,
              const unsigned int r0,
              const unsigned int c0) const
{
  assert(r0+r <= rows && c0+c <= columns);
  PMatrix m(r, c);
  for(unsigned int row=0; row<r; row++)
    for(unsigned int column=0; column<c; column++)
      m[row][column] = data[r0+row][c0+column];
  return m;
}

PMatrix
PMatrix::strikeOut(unsigned int r, unsigned int c)
{
  assert(r<rows && c<columns);
  PMatrix m(rows-1, columns-1);
  for(unsigned int i=0, I=0; i<rows; i++, I++)
    {
      if(i==r) { I--; continue; }
      for(unsigned int j=0, J=0; j<columns; j++, J++)
    {
      if(j==c) { J--; continue; }
      m.data[I][J] = data[i][j];
    }
    }
  return m;
}


PVector
PMatrix::getColumnVector(const unsigned int c) const
{
  assert(c<columns);
  PVector v(rows, PVector::COLUMNVECTOR);
  for(unsigned int i=0; i<rows; i++)
    v[i] = data[i][c];
  return v;
}

PVector
PMatrix::getRowVector(const unsigned int r) const
{
  assert(r<rows);
  PVector v(columns, PVector::ROWVECTOR);
  for (unsigned int i=0; i<columns; i++)
    v[i] = data[r][i];
  return v;
}

bool
PMatrix::isEqual(const PMatrix& m) const
{
  if(m.rows != rows && m.columns != columns)
    return false;
  for(unsigned int row=0; row<rows; row++)
    for(unsigned int column=0; column<columns; column++)
      if(fabs(data[row][column]-m[row][column]) > epsilon)
    return false;
  return true;
}

bool
PMatrix::isPureHomRot(void) const
{
  return getSubmatrix(3, 1, 0, 3).isEqual(PMatrix(3,1));
}

bool
PMatrix::isPureHomTrans(void) const
{
  return getSubmatrix(3,3).isEqual(makeUnity(3));
}

void
PMatrix::getTranslation(FLOAT_T& x, FLOAT_T& y, FLOAT_T& z) const
{
  x = data[0][3];
  y = data[1][3];
  z = data[2][3];
}

void
PMatrix::setTranslation(const FLOAT_T x, const FLOAT_T y, const FLOAT_T z)
{
  data[0][3] = x;
  data[1][3] = y;
  data[2][3] = z;
}

PVector
PMatrix::getTranslation(void) const
{
  PVector v(rows);
  for(unsigned int i=0; i<rows; i++)
    v[i] = data[i][columns-1];
  return v;
}

void
PMatrix::getEulerAngles(FLOAT_T& z0, FLOAT_T& y1, FLOAT_T& z2) const
{
  if ((fabs(data[2][0]) < PMatrix::epsilon) &&
      (fabs(data[2][1]) < PMatrix::epsilon)) {

     * This means that sin beta is 0 -> beta = 0, 180, 30
     * -> cos beta = +/- 1
     * next, we set z2 to 0 degree

    y1 = acos(data[2][2]);
    z2 = 0;
    z0 = acos(data[0][0]);
  } else {
    z0 = atan2(data[1][2], data[0][2]);
    y1 = acos(data[2][2]);
    z2 = atan2(data[2][1], -data[2][0]);
  }
}

void
PMatrix::getRPYAngles(FLOAT_T& r, FLOAT_T& p, FLOAT_T& y) const{

  // singularity
  if (fabs(data[0][0]) < epsilon && fabs(data[1][0]) < epsilon){
    r = 0.0;
    p = atan2(-data[2][0], data[0][0]);
    y = atan2(-data[1][2], data[1][1]);
  }
  else{
    r = atan2(data[1][0], data[0][0]);
    FLOAT_T sp = sin(r);
    FLOAT_T cp = cos(r);
    p = atan2(-data[2][0], cp * data[0][0] + sp * data[1][0]);
    y = atan2(sp * data[0][2] - cp * data[1][2], cp*data[1][1] - sp*data[0][1]);
  }
  return;
}

bg::string
PMatrix::toString(const int precision, const int prefix) const
{
  bg::string str;
  bg::string fmtc, fmt;
  if(precision >= 0)
    {
      fmtc = bg::string("%%.%df, ", precision);
      fmt = bg::string("%%.%df", precision);
    }
  else
    {
      fmtc = bg::string("%%f, ");
      fmt = bg::string("%%f");
    }

  unsigned int r, c;
  str = "((";
  for(r=0; r<rows-1; r++)
    {
      for(c=0; c<columns-1; c++)
    str += bg::string(fmtc.c_str(), data[r][c]);
      str += bg::string(fmt.c_str(), data[r][c]);
      str += "),\n";
      for(int i=0; i<prefix; i++) str += " ";
      str += " (";
    }
  for(c=0; c<columns-1; c++)
    str += bg::string(fmtc.c_str(), data[r][c]);
  str += bg::string(fmt.c_str(), data[r][c]);
  str += "))";

  return str;
}

void
PMatrix::print(std::ostream& str, bg::string prompt) const
{
  bg::string matstr = toString(-1, prompt.size());
  str << prompt << matstr << std::endl;
}

PMatrix
PMatrix::makeUnity(const unsigned int n)
{
  PMatrix matrix(n, n);
  for(unsigned int i=0; i<n; i++)
    matrix[i][i] = 1;
  return matrix;
}

PMatrix
PMatrix::makeHomTrans(const FLOAT_T x, const FLOAT_T y, const FLOAT_T z)
{
  PMatrix m = makeUnity(4);
  m[0][3] = x;
  m[1][3] = y;
  m[2][3] = z;
  return m;
}

PMatrix
PMatrix::makeHomRotEuler(const FLOAT_T z0, const FLOAT_T y1,
             const FLOAT_T z2)
{
  PMatrix m(4, 4);
  FLOAT_T c0, s0, c1, s1, c2, s2;
  c0 = cos(z0); s0 = sin(z0);
  c1 = cos(y1); s1 = sin(y1);
  c2 = cos(z2); s2 = sin(z2);

//   das ist Roll-Pitch-Yaw
//  m[0][0] =  c0*c1;              // m00 = c1 * c2;
//  m[0][1] =  s0*c1;              // m01 = s1 * c2;
//  m[0][2] = -s1;                 // m02 = -s2;
//  m[1][0] = -s0*c2 + c0*s1*s2;   // m10 = -(s1 * c3)+(c1 * s2 * s3);
//  m[1][1] =  c0*c2 + s0*s1*s2;   // m11 = (c1*c3) + (s1 * s2 * s3);
//  m[1][2] =  c1*s2;              // m12 = c2 * s3;
//  m[2][0] =  s0*s2 + c0*s1*c2;   // m20 = (s1 * s3) + (c1 * s2 * c3);
//  m[2][1] = -c0*s2 + s0*s1*c2;   // m21 = -(c1 * s3) + (s1 * s2 * c3);
//  m[2][2] =  c1*c2;              // m22 = c2*c3;
//  m[3][3] =  1;
//

//   // Und das ist the one and only real y-Konvention!
//
// m[0][0] = -s2*s0 + c1*c0*c2;   // m00 = -(s3 * s1) + (c2 * c1 * c3);
// m[0][1] = -s0*c2 - c0*c1*s2;   // m01 = -(c3 * s1) - (c1 * c2 * s3);
// m[0][2] =  c0*s1;              // m02 =  (c1 * s2)
// m[1][0] =  c0*s2 + s0*c1*c2;   // m10 =  (c1 * s3) + (c2 * c2 * s1);
// m[1][1] =  c2*c0 - c1*s0*s2;   // m11 = (c3 * c1) - (c2 * s1 * s3);
// m[1][2] =  s0*s1;              // m12 = s1 * s2;
// m[2][0] = -s1*c2;              // m20 = -(s2 * c3);
// m[2][1] =  s1*s2;              // m21 = s2 * s3;
// m[2][2] =  c1;                 // m22 = c2;
// m[3][3] =  1;


 m[0][0] = -s2*s0 + c1*c0*c2;   // m00 = -(s3 * s1) + (c2 * c1 * c3);
 m[0][1] = -c0*c1*s2 - s0*c2;
 m[0][2] = c0*s1;

  m[1][0] = c1*s0*c2 + c0*s2;
  m[1][1] = -c1*s0*s2 + c0*c2;
  m[1][2] = s0*s1;

  m[2][0] = -s1*c2;
  m[2][1] = s1*s2;
  m[2][2] = c1;

  m[3][0] = m[3][1] = m[3][2] = 0;
  m[3][3] = 1;

  return m;
}

  Get quaternions rotation as matrix.
 *  See [Faugeras] for an explicit solution. See [Gamasutra] for an
 *  implementation with a minimum count of operations. Had not got the
 *  time to implement them, so here a very naive implementation, that
 *  is not optimized in any way.

// q0 is the angle, (q1, q2, q3) ^= (qx, qy, qz) the vector of the quaternion
PMatrix
PMatrix::makeHomRotQuat(FLOAT_T q0, FLOAT_T q1, FLOAT_T q2, FLOAT_T q3)
{

  return PMatrix::makeHomRotFromQuatFaugeras(q0, q1, q2, q3);

}

PMatrix
PMatrix::makeHomRotQuatTrans(const FLOAT_T q0, const FLOAT_T q1,
                 const FLOAT_T q2, const FLOAT_T q3,
                 const FLOAT_T x, const FLOAT_T y,
                 const FLOAT_T z)
{
  return PMatrix::makeHomRotFromQuatTransFaugeras(q0, q1, q2, q3, x, y, z);
}


PMatrix
PMatrix::makeHomRotFromQuatFaugeras(FLOAT_T q0, FLOAT_T q1, FLOAT_T q2, FLOAT_T q3)
{
  // normalizing
  double length = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= length;
  q1 /= length;
  q2 /= length;
  q3 /= length;

  PMatrix m(4, 4);

  FLOAT_T  ww =      q0 * q0;
  FLOAT_T  xx =      q1 * q1;
  FLOAT_T  yy =      q2 * q2;
  FLOAT_T  zz =      q3 * q3;
  FLOAT_T  WX = 2.0 *q0 * q1;
  FLOAT_T  WY = 2.0 *q0 * q2;
  FLOAT_T  WZ = 2.0 *q0 * q3;
  FLOAT_T  XY = 2.0 *q1 * q2;
  FLOAT_T  XZ = 2.0 *q1 * q3;
  FLOAT_T  YZ = 2.0 *q2 * q3;

  m[0][0] = ww+xx-yy-zz;
  m[0][1] = XY-WZ;
  m[0][2] = XZ+WY;
  m[1][0] = XY+WZ;
  m[1][1] = ww-xx+yy-zz;
  m[1][2] = YZ-WX;
  m[2][0] = XZ-WY;
  m[2][1] = YZ+WX;
  m[2][2] = ww-xx-yy+zz;
  m[3][3] = 1;

  return m;
}


PMatrix
PMatrix::makeHomRotFromQuatTransFaugeras(const FLOAT_T q0, const FLOAT_T q1,
                 const FLOAT_T q2, const FLOAT_T q3,
                 const FLOAT_T x, const FLOAT_T y,
                 const FLOAT_T z)
{
  PMatrix m = PMatrix::makeHomRotFromQuatFaugeras(q0, q1, q2, q3);

  m[0][3] = x;
  m[1][3] = y;
  m[2][3] = z;

  return m;
}

PMatrix
PMatrix::makeHomRotFromQuatHamilton(FLOAT_T q0, FLOAT_T q1,
                    FLOAT_T q2, FLOAT_T q3)
{

  // normalizing
  double length = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= length;
  q1 /= length;
  q2 /= length;
  q3 /= length;

  PMatrix m(4, 4);

  m[0][0] = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
  m[0][1] = 2.0 * (q0 * q1 - q2 * q3);
  m[0][2] = 2.0 * (q2 * q0 + q1 * q3);
  m[1][0] = 2.0 * (q0 * q1 + q2 * q3);
  m[1][1] = 1.0 - 2.0 * (q2 * q2 + q0 * q0);
  m[1][2] = 2.0 * (q1 * q2 - q0 * q3);
  m[2][0] = 2.0 * (q2 * q0 - q1 * q3);
  m[2][1] = 2.0 * (q1 * q2 + q0 * q3);
  m[2][2] = 1.0 - 2.0 * (q1 * q1 + q0 * q0);
  m[3][3] = 1.0;

  return m;

}

PMatrix
PMatrix::makeHomRotFromQuatTransHamilton(const FLOAT_T q0, const FLOAT_T q1,
                     const FLOAT_T q2, const FLOAT_T q3,
                     const FLOAT_T x, const FLOAT_T y,
                     const FLOAT_T z)
{
  PMatrix m = PMatrix::makeHomRotFromQuatHamilton(q0, q1, q2, q3);

  m[0][3] = x;
  m[1][3] = y;
  m[2][3] = z;

  return m;
}

PMatrix
PMatrix::makeHomRotFromVectors(const PVector& source, const PVector& dest)
{
  double cost;
  PVector s = source; s.normalize();
  PVector d = dest; d.normalize();

  // cosine of angle between vectors
  cost = s.dotProduct(d);

  PVector axis = s.crossProduct(d);
  axis.normalize();
  BGDBG(0, "Axis: %s\n", axis.toString().c_str());

  axis = axis * sqrt(0.5 * (1.0 - cost));

  double q0, q1 = axis[0], q2 = axis[1], q3 = axis[2];

  q0 = sqrt(0.5 * (1.0 + cost));

  BGDBG(0, "Quaternionen: %f %f %f %f\n", q0, q1, q2, q3);
  return makeHomRotQuat(q0, q1, q2, q3);
}

PMatrix
PMatrix::makeNURBSCoefficients(const PVector& u, const PVector& kvec,
                   const unsigned int l)
{
  PMatrix m(u.getNumberOfElements(), l);
  unsigned int order = kvec.getNumberOfElements() - l - 1;

  for(unsigned int i=0; i<u.getNumberOfElements(); i++)
    for(unsigned int j=0; j<l; j++)
      m[i][j] = PNURBS::basis(u[i], kvec, order, j);
  return m;
}

PMatrix
PMatrix::makeGivensRotationX(const double angle)
{
  PMatrix retval = makeUnity(3);
  retval[1][1] = cos(angle); retval[1][2] = -sin(angle);
  retval[2][1] = sin(angle); retval[2][2] = cos(angle);
  return retval;
}

PMatrix
PMatrix::makeGivensRotationY(const double angle)
{
  PMatrix retval = makeUnity(3);
  retval[0][0] = cos(angle); retval[0][2] = sin(angle);
  retval[2][0] = -sin(angle); retval[2][2] = cos(angle);
  return retval;
}

PMatrix
PMatrix::makeGivensRotationZ(const double angle)
{
  PMatrix retval = makeUnity(3);
  retval[0][0] = cos(angle); retval[0][1] = -sin(angle);
  retval[1][0] = sin(angle); retval[1][1] = cos(angle);
  return retval;
}

PMatrix
PMatrix::fromInventorMatrix(const InventorMat4x4& arr)
{
  unsigned int r, c;
  PMatrix matrix(4, 4);
  for(r=0; r<4; r++) for(c=0; c<4; c++)
    matrix[r][c] = arr[r][c];
  return matrix.transpose();
}

PMatrix
PMatrix::fromMatrix(const float** arr, const unsigned int m,
            const unsigned int n)
{
  unsigned int r, c;
  PMatrix matrix(m, n);
  for(r=0; r<m; r++)
    for(c=0; c<n; c++)
      matrix.data[r][c] = arr[r][c];
  return matrix;
}

PMatrix
PMatrix::fromMatrix(const double** arr, const unsigned int m,
            const unsigned int n)
{
    PMatrix matrix(m, n);
#if (FLOAT_T_VAL==BG_DOUBLE)
  unsigned int r;
  for(r=0; r<m; r++)
    memcpy(matrix.data[r], arr[r], sizeof(double)*n);
#else
  unsigned int r, c;
  for(r=0; r<m; r++)
    for(c=0; c<n; c++)
      matrix.data[r][c] = arr[r][c];
#endif
  return matrix;
}

PMatrix
PMatrix::fromMatrix(const float* arr, const unsigned int m,
            const unsigned int n)
{
#if (FLOAT_T_VAL==BG_FLOAT)
  unsigned int r;
  PMatrix matrix(m, n);
  for(r=0; r<m; r++)
    memcpy(matrix.data[r], &(arr[r*n]), sizeof(float)*n);
#else
  unsigned int r, c;
  PMatrix matrix(m, n);
  for(r=0; r<m; r++)
    for(c=0; c<n; c++)
      matrix.data[r][c] = arr[r*n+c];
#endif
  return matrix;
}


PMatrix
PMatrix::fromMatrix(const double* arr, const unsigned int m,
            const unsigned int n)
{
#if (FLOAT_T_VAL==BG_DOUBLE)
  unsigned int r;
  PMatrix matrix(m, n);
  for(r=0; r<m; r++)
    memcpy(matrix.data[r], &(arr[r*n]), sizeof(double)*n);
#else
  unsigned int r, c;
  PMatrix matrix(m, n);
  for(r=0; r<m; r++)
    for(c=0; c<n; c++)
      matrix.data[r][c] = arr[r*n+c];
#endif
  return matrix;
}


// this method is used by PApplication to read the
// transformation strings of the settings file.
// BE AWARE that the rotation is applied AFTER the translation!!!
// usually DO NOT USE!
PMatrix
PMatrix::stringToPMatrix(bg::string str)
{
  PMatrix trans;

  if (!(str.startsWith("(") && str.endsWith(")")))
    BGTHROWC(bg::error,
         "'%s' is not a valid position vector", str.c_str());

  str = str.substr(1, str.size() - 2);
  bg::strlist elements = str.split(",");

  BGDBGM("App", 4, "%d elements in vector '%s'!\n", elements.size(),
     str.c_str());

  if (elements.size() != 9 && elements.size() != 6 && elements.size() != 3)
    BGTHROWC(bg::error,
         "'%s' is not a valid position vector", str.c_str());

  double x, y, z;

  x = elements[0].strip().toDouble();
  y = elements[1].strip().toDouble();
  z = elements[2].strip().toDouble();

  trans = PMatrix::makeHomTrans(x, y, z);

  BGDBGM("App", 3, "Translating by %f %f %f\n", x, y, z);
  if (elements.size() >= 6) {

    double rz0, ry1, rz2;

    rz0 = elements[3].strip().toDouble();
    ry1 = elements[4].strip().toDouble();
    rz2 = elements[5].strip().toDouble();

    BGDBGM("App", 3, "Rotating by %f %f %f\n", rz0, ry1, rz2);
    PMatrix rot = PMatrix::makeHomRotEuler(rz0, ry1, rz2);
    trans = rot * trans;
    if (elements.size() == 9){
      PMatrix scale;
      scale[0][0] = elements[6].strip().toDouble();
      scale[1][1] = elements[7].strip().toDouble();
      scale[2][2] = elements[8].strip().toDouble();
      BGDBGM("App", 3, "Scaling by by %f %f %f\n",
         scale[0][0], scale[1][1], scale[2][2]);
      trans = trans * scale;
    }
  }

  return trans;
}


// FIXME: not tested yet, whether the notation of the matrix is correct (column/row first?!)
PMatrix
PMatrix::makeHomRotRPY(const FLOAT_T rx, const FLOAT_T ry,
               const FLOAT_T rz)
{
  // The order of rotations is first x, then y', then z''.
  double
    a = rx,
    b = ry,
    c = rz;

  PMatrix res = PMatrix::makeUnity(4);
  res[0][0] = cos(b)*cos(c);
  res[1][0] = -cos(b)*sin(c);
  res[2][0] = sin(b);
  res[0][1] = cos(a)*sin(c) + sin(a)*sin(b)*cos(c);
  res[1][1] = cos(a)*cos(c) - sin(a)*sin(b)*sin(c);
  res[2][1] = -sin(a)*cos(b);
  res[0][2] = sin(a)*sin(c) - cos(a)*sin(b)*cos(c);
  res[1][2] = sin(a)*cos(c) + cos(a)*sin(b)*sin(c);
  res[2][2] = cos(a)*cos(b);

  return res;
}

// FIXME: no unique pitch can be calculated with this method.
// maybe combining it with the matrix-to-rpy could help?
#if 0
#define SQUARE(x) ((x) * (x))
void
PMatrix::getRotationRPY(FLOAT_T& roll, FLOAT_T& pitch, FLOAT_T& yaw)
{
  FLOAT_T qw, qx, qy, qz;
  getQuaternion(qw, qx, qy, qz);

  yaw = atan( 2.0 * (qx * qy + qw * qz) / ( SQUARE(qw) + SQUARE(qx) - SQUARE(qy)- SQUARE(qz) ));
  pitch = asin( -2.0 * (qx * qz - qw * qy));
  roll = atan( 2.0 * (qw * qx+ qy * qz) / ( SQUARE(qw) - SQUARE(qx) - SQUARE(qy)+ SQUARE(qz) ));
}
#endif

// taken from www.euclideanspace.com
// FIXME: make sure the indices are correct (row/column first)!
void
PMatrix::getQuaternion(FLOAT_T& w, FLOAT_T& x, FLOAT_T& y, FLOAT_T& z) const
{
  FLOAT_T** a = data;
  float trace = a[0][0] + a[1][1] + a[2][2] + 1.0f;

  if( trace > PMatrix::epsilon) {
    float s = 0.5f / sqrtf(trace);
    w = 0.25f / s;
    x = ( a[2][1] - a[1][2] ) * s;
    y = ( a[0][2] - a[2][0] ) * s;
    z = ( a[1][0] - a[0][1] ) * s;
  } else {
    if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
      float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
      x = 0.25f * s;
      y = (a[0][1] + a[1][0] ) / s;
      z = (a[0][2] + a[2][0] ) / s;
      w = (a[1][2] - a[2][1] ) / s;

    } else if (a[1][1] > a[2][2]) {
      float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
      x = (a[0][1] + a[1][0] ) / s;
      y = 0.25f * s;
      z = (a[1][2] + a[2][1] ) / s;
      w = (a[0][2] - a[2][0] ) / s;

    } else {
      float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
      x = (a[0][2] + a[2][0] ) / s;
      y = (a[1][2] + a[2][1] ) / s;
      z = 0.25f * s;
      w = (a[0][1] - a[1][0] ) / s;
    }
  }
}

PVector
PMatrix::getQuaternion() const
{
  double qw, qx, qy, qz;
  getQuaternion(qw, qx, qy, qz);
  return PVector::makeHomVector(qw, qx, qy, qz);
}


#if Matrix_test
#include <stdio.h>
int main(int argc, char **argv)
{
  // This is a module-test block. You can put code here that tests
  // just the contents of this C file, and build it by saying
  //             make PMatrix_test
  // Then, run the resulting executable (PMatrix_test).
  // If it works as expected, the module is probably correct. ;-)

  fprintf(stderr, "Testing PMatrix\n");

  PMatrix u = PMatrix::makeUnity(4);
  PMatrix scale = u * 4.0;
  fprintf(stderr, "Unity: %s\n",
      u.toString(2, strlen("Unity: ")).c_str());
  fprintf(stderr, "Unity*4: %s\n",
      scale.toString(2, strlen("Unity*4: ")).c_str());

  PMatrix l(3, 2);
  l[0][0] = 1; l[0][1] = 2; l[1][0] = 3; l[1][1] = 4;
  l[2][0] = 5; l[2][1] = 6;
  PMatrix r(2, 3);
  r[0][0] = 1; r[0][1] = 2; r[0][2] = 3;
  r[1][0] = 4; r[1][1] = 5; r[1][2] = 6;

  fprintf(stderr, "L: %s\n", l.toString(-1, strlen("L: ")).c_str());
  fprintf(stderr, "R: %s\n", r.toString(-1, strlen("R: ")).c_str());

  PMatrix lr = l*r;

  fprintf(stderr, "L*R: %s\n", lr.toString(-1, strlen("L*R: ")).c_str());

  PMatrix rotZ90 = PMatrix::makeHomRotEuler(M_PI_2, 0, 0);
  fprintf(stderr, "rotZ90: %s\n", rotZ90.toString(-1, 8).c_str());
  if(rotZ90.isPureHomRot())
    fprintf(stderr, "is pure rot\n");
  else
    fprintf(stderr, "isn't pure rot\n");

  PMatrix trans = PMatrix::makeHomTrans(10, 20, 30);
  fprintf(stderr, "trans: %s\n", trans.toString(-1, 7).c_str());
  if(trans.isPureHomTrans())
    fprintf(stderr, "is pure trans\n");
  else
    fprintf(stderr, "isn't pure trans\n");

  PMatrix t = rotZ90*trans;
  fprintf(stderr, "t: %s\n", t.toString(-1, 3).c_str());
  if(t.isPureHomTrans())
    fprintf(stderr, "is pure trans\n");
   else
    fprintf(stderr, "isn't pure trans\n");

  FLOAT_T z0, y1, z2;
  t.getEulerAngles(z0, y1, z2);
  fprintf(stderr, "Euler angles: %f %f %f\n", z0, y1, z2);

  PMatrix inv = t.inverse();
  fprintf(stderr, "Inverse of t: %s\n", inv.toString(-1, 14).c_str());

  PMatrix prod = t*inv;
  fprintf(stderr, "Product of t*t^-1: %s\n",
      prod.toString(-1, 19).c_str());

  PMatrix ll(4, 4), ur(4, 4);
  prod.getDiagonalMatrices(ll, ur);
  fprintf(stderr, "ll: %s\nur: %s\n",
      ll.toString(2, 4).c_str(),
      ur.toString(2, 4).c_str());

  PMatrix xp = prod.transpose();
  fprintf(stderr, "Transposed: %s\n",
      xp.toString(-1, 12).c_str());

  xp.getDiagonalMatrices(ll, ur);
  fprintf(stderr, "ll: %s\nur: %s\n",
      ll.toString(2, 4).c_str(),
      ur.toString(2, 4).c_str());

  fprintf(stderr, "Permuted:\n");
  std::vector<unsigned int> p;
  p.insert(p.end(), 1);
  p.insert(p.end(), 0);
  p.insert(p.end(), 3);
  p.insert(p.end(), 2);
  xp.getDiagonalMatrices(ll, ur, p);
  fprintf(stderr, "ll: %s\nur: %s\n",
      ll.toString(2, 4).c_str(),
      ur.toString(2, 4).c_str());

#if 0
  SbMatrix imat, iimat;
  PMatrix pmat, pimat;

  printf("Matrix A (Inventor):\n");
  imat.setTransform(SbVec3f(10, 15, 20),
            SbRotation(M_PI/3, M_PI/5, M_PI/7, M_PI/11),
            SbVec3f(1, 1, 1));

  imat.print(stderr);
  pmat = PMatrix::fromInventorMatrix(imat);
  printf("Matrix A (Porki):\n");
  pmat.print(std::cerr);

  iimat = imat.inverse();
  printf("Matrix A^-1 (Inventor):\n");
  iimat.print(stderr);
  pimat = pmat.inverse();
  printf("Matrix A^-1 (Porki):\n");
  pimat.print(std::cerr);

  PMatrix unity = pmat*pimat;
  printf("Matrix B=A*A^-1 (Porki):\n");
  unity.print(std::cerr);

#endif

  PVector quat1 = PVector::makeHomVector(0.433700, 0.591600, -0.677900, 0.046500); // don't know much about it :-)
  PVector trans1 = PVector::makeHomVector(10.0,20.0,30.0);
  PMatrix quatMat = PMatrix::makeHomRotQuatTrans(quat1[0], quat1[1], quat1[2], quat1[3], trans1[0], trans1[1], trans1[2]);
  fprintf(stderr, "Matrix created from quaternion (%.2f, %.2f, %.2f, %.2f) and translation (%.2f, %.2f, %.2f): ",
      quat1[0], quat1[1], quat1[2], quat1[3],
      trans1[0], trans1[1], trans1[2]);
  quatMat.print(std::cerr);

  PMatrix dMat(4, 4);
  dMat[0][0] = 3; dMat[0][1] = 2; dMat[0][2] = -1; dMat[0][3] = 4;
  dMat[1][0] = 2; dMat[1][1] = 1; dMat[1][2] = 5; dMat[1][3] = 7;
  dMat[2][0] = 0; dMat[2][1] = 5; dMat[2][2] = 2; dMat[2][3] = -6;
  dMat[3][0] = -1; dMat[3][1] = 2; dMat[3][2] = 1; dMat[3][3] = 0;

  printf("Determinant: %.2f\n", dMat.determinant());

  PMatrix mm = PMatrix::makeHomRotRPY(0,M_PI,0);
  mm.print(std::cerr, "RPY(0, PI, 0):" );

  quatMat.print(std::cerr, "\ninitial matrix: ");
  double qw, qx, qy, qz;
  quatMat.getQuaternion(qw, qx, qy, qz);
  PMatrix nn = PMatrix::makeHomRotQuat(qw, qx, qy, qz);
  nn.print(std::cerr, "matrix after conversion to and from quaternions\n");
  PMatrix rr = quatMat * nn.inverse();
  rr.print(std::cerr, "first * second_inverted: ");

  for(unsigned int i=0, j=0; i<rr.getNumberOfRows(); i++, j++)
    rr.strikeOut(i, j).print(std::cerr,
                 bg::string("striking out %d,%d", i, j));

  return 0;
}




*/
