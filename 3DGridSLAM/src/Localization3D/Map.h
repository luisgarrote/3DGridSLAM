/**
*@file //TODO
*  @date 9 Oct 2013
*  @version 1.0
*  @brief //TODO.

This file is part of Luis Garrote Phd work.
Copyright (C) 2012-2016 Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt)
All rights reserved.

*
*  @author Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt)
*  @bug No know bugs.
*/
#ifndef MAP_H
#define MAP_H


#include <iostream>
#include <sstream>
#include <math.h>
#include <exception>
#include <string>     // std::string, toString
#include "Point.h"
//#include "Math/Gaussian.h"
//#include "Graphics/Models/Rectangle.h"
#include "Matrix.h"
#include "YAML.h"
#include "StringTools.h"

//#include "Theory/Robots/core/FootPrint.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include "PoseTools.h"


template<class T> //or typename??
class Map
{
private:

    unsigned int index;
    unsigned int rows;
    unsigned int columns;
    unsigned int count;

    T freeValue;
    T occupiedValue;
    std::string MapName;
    T *matrix;
    T **indexer;

    Point start;
    T base;




  double fx;
  double fy;

    void dispose(){
        if(matrix!=NULL){
            delete [] matrix;
            delete [] indexer;
        }
        rows=0;
        columns=0;
    }
    void disposePointer(){
        if(matrix!=NULL){
            delete [] matrix;
            delete [] indexer;
        }

    }

public:


    double columns_2;
    double rows_2;
    double fcolumns_2;
    double frows_2;
    double columns_25;
    double rows_25;
    double cellsizeX_2;
    double cellsizeY_2;
    double scellsizeX_2;
    double scellsizeY_2;

    double cyfrows_2;
    double cxfcolumns_2;

    double cyrows_2;
    double cxcolumns_2;

    double cellsizeX;
    double cellsizeY;






    int xmin=0;
    int ymin=0;
    int xmax=0;
    int ymax=0;

    void setCenter(Point &start_){
        start=start_;
        columns_2=(columns)/2.0;
        rows_2=(rows)/2.0;
        columns_25=(columns)/2.0 -0.5;
        rows_25=(rows)/2.0 -0.5;
        fcolumns_2=floor((columns)/2.0);
        frows_2=floor((rows)/2.0);

        cellsizeX_2=cellsizeX/2.0;
        cellsizeY_2=cellsizeY/2.0;
        scellsizeX_2=-start.x+cellsizeX/2.0;
        scellsizeY_2=-start.y+cellsizeY/2.0;
        cyfrows_2=-frows_2*cellsizeY+start.y;
        cxfcolumns_2=-fcolumns_2*cellsizeX+start.x;

        cyrows_2=-rows_2*cellsizeY+start.y;
        cxcolumns_2=-columns_2*cellsizeX+start.x;

      fx=scellsizeX_2/cellsizeX+ columns_2-0.5;
      fy=scellsizeY_2/cellsizeY+ rows_2-0.5;

    }
    Point getCenter(){
        return start;
    }
    // #########################################################################################################################
    // constructors and destructor
    // #########################################################################################################################

    Map(){

        freeValue=0.499999;
        occupiedValue=0.5;
        matrix=NULL;
        indexer=NULL;
        rows=0;
        columns=0;
        count=0;

    }

    Map(unsigned int rows_, unsigned int columns_){

        rows=rows_;
        columns=columns_;
        matrix =new T [rows_*columns_];
        indexer=new T*[rows_];
        count=columns*rows;

        for(unsigned int i=0;i<rows_;i++){
            indexer[i]=&matrix[i*columns_];
        }
        zero();

        columns_2=(columns)/2.0;
        rows_2=(rows)/2.0;
        columns_25=(columns)/2.0 -0.5;
        rows_25=(rows)/2.0 -0.5;
        fcolumns_2=floor((columns)/2.0);
        frows_2=floor((rows)/2.0);

        cellsizeX_2=cellsizeX/2.0;
        cellsizeY_2=cellsizeY/2.0;
        scellsizeX_2=-start.x+cellsizeX/2.0;
        scellsizeY_2=-start.y+cellsizeY/2.0;
        cyfrows_2=-frows_2*cellsizeY+start.y;
        cxfcolumns_2=-fcolumns_2*cellsizeX+start.x;

        cyrows_2=-rows_2*cellsizeY+start.y;
        cxcolumns_2=-columns_2*cellsizeX+start.x;

      fx=scellsizeX_2/cellsizeX+ columns_2-0.5;
      fy=scellsizeY_2/cellsizeY+ rows_2-0.5;

        //        base=T(0);
    }
    Map(unsigned int rows_, unsigned int columns_,T vl){

        rows=rows_;
        columns=columns_;
        matrix =new T [rows_*columns_];
        indexer=new T*[rows_];
        count=columns*rows;

        for(unsigned int i=0;i<rows_;i++){
            indexer[i]=&matrix[i*columns_];
        }
        all(vl);
        base=vl;

        columns_2=(columns)/2.0;
        rows_2=(rows)/2.0;
        columns_25=(columns)/2.0 -0.5;
        rows_25=(rows)/2.0 -0.5;
        fcolumns_2=floor((columns)/2.0);
        frows_2=floor((rows)/2.0);

        cellsizeX_2=cellsizeX/2.0;
        cellsizeY_2=cellsizeY/2.0;
        scellsizeX_2=-start.x+cellsizeX/2.0;
        scellsizeY_2=-start.y+cellsizeY/2.0;
        cyfrows_2=-frows_2*cellsizeY+start.y;
        cxfcolumns_2=-fcolumns_2*cellsizeX+start.x;

        cyrows_2=-rows_2*cellsizeY+start.y;
        cxcolumns_2=-columns_2*cellsizeX+start.x;

      fx=scellsizeX_2/cellsizeX+ columns_2-0.5;
      fy=scellsizeY_2/cellsizeY+ rows_2-0.5;
    }
    Map(const Map<T> &value){

        rows=value.getRows();
        columns=value.getColumns();
        matrix =new T [rows*columns];
        indexer=new T*[rows];
        count=columns*rows;
        base=value.base;


        columns_2=value.columns_2;
        rows_2=value.rows_2;
        columns_25=value.columns_25;
        rows_25=value.rows_25;
        fcolumns_2=value.fcolumns_2;
        frows_2=value.frows_2;

        cellsizeX_2=value.cellsizeX_2;
        cellsizeY_2=value.cellsizeY_2;
        scellsizeX_2=value.scellsizeX_2;
        scellsizeY_2=value.scellsizeY_2;

        cyfrows_2=value.cyfrows_2;
        cxfcolumns_2=value.cxfcolumns_2;


        cyrows_2=value.cyrows_2;
        cxcolumns_2=value.cxcolumns_2;

      fx=value.fx;
      fy=value.fy;

        for(unsigned int i=0;i<rows;i++){
            indexer[i]=&matrix[i*columns];
        }
        zero();

        for(unsigned int i=0;i<columns;i++){
            for(unsigned int f=0;f<rows;f++){
                //esta a usar o construtor
                setIndexXY(i,f,value.getIndexXY(i,f));
            }
        }

        start=value.start;
        cellsizeX=value.cellsizeX;
        cellsizeY=value.cellsizeY;
        freeValue=value.freeValue;
        occupiedValue=value.occupiedValue;
        MapName=value.MapName;
    }

    std::pair<double,double> getMapCoords(double px,double py){
        std::pair<double,double> out;
        out.first=  (px/cellsizeX)+scellsizeX_2/cellsizeX+ columns_2-0.5;
        out.second=  (py/cellsizeY)+scellsizeY_2/cellsizeY+ rows_2-0.5;
        return out;
    }


    std::pair<double,double> getWorldCoords(double px,double py){

        std::pair<double,double> out;
        out.first=  cellsizeX*px+cxcolumns_2;//cellsizeX*-(columns_2))+start.x;
        out.second=  cellsizeY*py+cyrows_2;//cellsizeY*-(rows_2))+start.y;
        return out;

    }


    ~Map(){

        if(rows!=0 && columns!=0){
            delete [] matrix;
            delete [] indexer;
            rows=0;
            columns=0;
        }
    }

    // #########################################################################################################################
    // access handlers
    // #########################################################################################################################

    T &operator()( unsigned int row, unsigned int column)const{

        if(row<rows && column<columns){
            return indexer[row][column];
        }else{
            std::stringstream strd;
            strd<<" Tried to access ("<<row<<","<<column<<")  from matrix "<<rows<<" x "<<columns<<std::endl;
            throw strd.str();
        }
    }

    Map<T> &operator << ( T val){

        index=0;
        zero();
        matrix[index]=val;

        if(index>=count){
            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw strd.str();
        }

        return *this;
    }

    Map<T> &operator, (T value){
        index=index+1;

        if(index>=count){
            std::stringstream strd;
            strd<<" The input stream is not for a "<<rows<<" x "<<columns<< " Matrix "<<std::endl;
            throw strd.str();
        }
        matrix[rows*(index%columns)+(int)floor(((double)index/(double)columns))]=value;

        return *this;
    }

    inline bool validIndex(unsigned int x,unsigned int y){
        return (x<rows) && (y<columns);
    }

    inline T getIndexXY(unsigned int column,unsigned int row)const{

        if((column>=columns)||(row>=rows)){
            return occupiedValue;
        }else{
            return indexer[row][column];
        }

    }
    inline T getIndexXY2(unsigned int column,unsigned int row)const{

        if((column>=columns)||(row>=rows)){
            return 0;
        }else{
            return indexer[row][column];
        }

    }

    T getXY(double X,double Y){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        return getIndexXY(k,n);

    }

    T getXY(double X,double Y,int &x,int &y){
        x= convertToXindex(X);
        y= convertToYindex(Y);

        if((x>=columns)||(y>=rows) || (x<0)||(y<0) ){
            x=-1;
            y=-1;
            return occupiedValue;
        }else{
            return indexer[y][x];
        }
    }


    bool setIndexXY(unsigned int column,unsigned int row,T value)const{
        if((column>=columns)||(row>=rows)){
            return false;
        }else{
            indexer[row][column]=value;
            return true;
        }
    }



    bool addIndexXY(unsigned int column,unsigned int row,T value)const{
        if((column>=columns)||(row>=rows)){
            return false;
        }else{

            if(indexer[row][column]>(std::numeric_limits<T>::max()-value)){

            }else{
                indexer[row][column]=indexer[row][column]+value;

            }

            //            indexer[row][column]=std::max(std::min((indexer[row][column]+value),std::numeric_limits<T>::max()),std::numeric_limits<T>::min());
            return true;
        }
    }

    bool takeIndexXY(unsigned int column,unsigned int row,T value)const{
        if((column>=columns)||(row>=rows)){
            return false;
        }else{
            if(indexer[row][column]<value){

            }else{
                indexer[row][column]=indexer[row][column]-value;

            }
            //            indexer[row][column]=std::max(std::min((indexer[row][column]-value),std::numeric_limits<T>::max()),std::numeric_limits<T>::min());
            return true;
        }
    }
    void setMinIndexXY(unsigned int column,unsigned int row,T  value)const{
        if((column<columns)&&(row<rows)){

            if(indexer[row][column]>value){
                indexer[row][column]=value;
            }

        }
        //        if((column>=columns)||(row>=rows)){
        //            return false;
        //        }else{
        //            indexer[row][column]=std::min(indexer[row][column],value);
        //            return true;
        //        }
    }


    void unprotected_setMaxIndexXY(unsigned int column,unsigned int row,T &value)const{
        if(indexer[row][column]<value){
            indexer[row][column]=value;
        }
    }
    void setMaxIndexXY(unsigned int column,unsigned int row,T value)const{
        if((column<columns)&&(row<rows)){

            if(indexer[row][column]<value){
                indexer[row][column]=value;
            }
            //            indexer[row][column]=std::max(indexer[row][column],value);
        }
    }

    bool setXY(double X,double Y,T value){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        return setIndexXY(k,n,value);
    }
    void setXYMin(double X,double Y,T value){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
//        std::cout<<k<<" "<<n<<" "<<columns<<" "<<rows<<" "<<std::endl;

        setMinIndexXY(k,n,value);
    }


    void setXYMax(double X,double Y,T value){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        setMaxIndexXY(k,n,value);
    }

    bool addXY(double X,double Y,T value){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        return setIndexXY(k,n,value+getIndexXY(k,n));
    }


    void maxXY(double X,double Y,T &value){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        setMaxIndexXY(k,n,value);
    }

    bool minXY(double X,double Y,T value){
        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        return setIndexXY(k,n,std::min(value,getIndexXY(k,n)));
    }

    T* getPointerXY(double X,double Y){

        unsigned int n= convertToXindex(X);
        unsigned int k= convertToYindex(Y);

        if((n>=columns)||(k>=rows)){
            return NULL;
        }else{
            return &indexer[k][n];
        }

    }

    T* getPointerIndexXY(unsigned int n,unsigned int k){
        if((n>=columns)||(k>=rows)){
            return NULL;
        }else{
            return &indexer[k][n];
        }
    }

    inline T unprotected_getIndexXY(unsigned int column,unsigned int row)const{
        return indexer[row][column];
    }

    inline void  unprotected_setIndexXY(unsigned int column,unsigned int row,T  value)const{
        indexer[row][column]=value;
    }

    T* unprotected_getPointerIndexXY(unsigned int n,unsigned int k){
        return &indexer[k][n];
    }

    //matrix like access

    T unprotected_get(unsigned int row,unsigned int column){
        return indexer[row][column];
    }

    void unprotected_set(unsigned int row,unsigned int column,T value){
        indexer[row][column]=value;
    }

    T get(unsigned int indexL)const{

        if(indexL>=count){
            return (T)0;
        }else{
            return matrix[indexL];
        }
    }

    //    T getLineTotalXY(Point p1,Point p2){

    //        unsigned int x0= convertToXindex(p1.x);
    //        unsigned int y0= convertToYindex(p1.y);
    //        unsigned int x1= convertToXindex(p2.x);
    //        unsigned int y1= convertToYindex(p2.y);


    //        return getLineTotal(x0,y0,x1,y1);

    //    }



    template <typename fx>
    std::vector<T> getLineCosts(fx p1,fx p2){

        unsigned int x0= convertToXindex(p1.x);
        unsigned int y0= convertToYindex(p1.y);
        unsigned int x1= convertToXindex(p2.x);
        unsigned int y1= convertToYindex(p2.y);


        return getLine<T>(x0,y0,x1,y1);
    }


    template <typename fx,typename F>
    T getLineTotalXY(fx p1,fx p2){

        unsigned int x0= convertToXindex(p1.x);
        unsigned int y0= convertToYindex(p1.y);
        unsigned int x1= convertToXindex(p2.x);
        unsigned int y1= convertToYindex(p2.y);


        return getLineTotal<F>(x0,y0,x1,y1);
    }

    template <typename fx>
    T getLineMinXY(fx p1,fx p2){

        unsigned int x0= convertToXindex(p1.x);
        unsigned int y0= convertToYindex(p1.y);
        unsigned int x1= convertToXindex(p2.x);
        unsigned int y1= convertToYindex(p2.y);


        return getLineMin(x0,y0,x1,y1);


    }

    T getLineMin(  int x0,   int y0,  int x1,  int y1){

        T out=std::numeric_limits<T>::max();
        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return out;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return out;
        }

        int mx=std::max(dx,dy);
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;


                out=std::min( unprotected_get(y0,x0),out);  // Alterar
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;


                out=std::min( unprotected_get(y0,x0),out);  // Alterar

            }
        }

        return out;
    }



    template<typename F>
    F getLineTotal(  int x0,   int y0,  int x1,  int y1){

        F out=0;
        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return std::numeric_limits<F>::max()/2.0;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return std::numeric_limits<F>::max()/2.0;
        }

        int mx=std::max(dx,dy);
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;


                out+= unprotected_get(y0,x0);  // Alterar
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;


                out+= unprotected_get(y0,x0);  // Alterar

            }
        }

        return out;
    }

    template<typename F>
    std::vector<F> getLine(  int x0,   int y0,  int x1,  int y1){

        std::vector<F> out;
        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return out;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return out;
        }

        int mx=std::max(dx,dy);
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;

                out.emplace_back(unprotected_get(y0,x0));  // Alterar
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;


                out.emplace_back(unprotected_get(y0,x0));  // Alterar

            }
        }

        return out;
    }

    void set(unsigned int indexL,T value)const{

        if(indexL<count){
            matrix[indexL]=value;
        }
    }

    T unprotected_get(unsigned int indexL)const{

        if(indexL>=count){
            return (T)0;
        }else{
            return matrix[indexL];
        }
    }

    void unprotected_set(unsigned int indexL,T value)const{

        if(indexL<count){
            matrix[indexL]=value;
        }
    }


    bool cloneIndexXY(unsigned int column,unsigned int row,T value,bool justsegments=true)const{

        if((column>=columns)||(row>=rows)){
            return false;
        }else{



            indexer[row][column]=value->clone(justsegments);

            //            matrix[row*rows+column]=value;

            return true;
        }

    }

    // #########################################################################################################################
    // Occupied and free handles
    // #########################################################################################################################



    bool isOccupiedXY(double X,double Y){

        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);

        return isOccupiedIndexXY(k,n);
    }

    bool isOccupiedIndexXY(unsigned int x,unsigned int y){

        return getIndexXY(x,y)>=occupiedValue;
    }
    inline bool unprotected_isOccupiedIndexXY(unsigned int x,unsigned int y){

        return unprotected_getIndexXY(x,y)>=occupiedValue;
    }

    bool isOccupiedIndexXY(std::pair<unsigned int,unsigned int> pa){

        return getIndexXY(pa.first,pa.second)>=occupiedValue;
    }
 
    //    bool isOccupiedFast(FootPrint ft){

    //        bool occupied=false;


    //        Point midle1=(ft.X1+ft.X2)/2.0;
    //        Point midle2=(ft.X3+ft.X4)/2.0;


    //        std::vector<std::pair<int,int>> line1;
    //        LineAlgorithms::getSoftBresenham(convertToXindex(ft.X1.x),convertToYindex(ft.X1.y),convertToXindex(ft.X3.x),convertToYindex(ft.X3.y),line1);


    //        for(unsigned int g=0;g<line1.size();g++){
    //            if(isOccupiedIndexXY(line1[g].first,line1[g].second)){
    //                return true;
    //            }
    //        }

    //        std::vector<std::pair<int,int>> line2;
    //        LineAlgorithms::getSoftBresenham(convertToXindex(ft.X2.x),convertToYindex(ft.X2.y),convertToXindex(ft.X4.x),convertToYindex(ft.X4.y),line2);

    //        for(unsigned int g=0;g<line2.size();g++){
    //            if(isOccupiedIndexXY(line2[g].first,line2[g].second)){
    //                return true;
    //            }
    //        }


    //        std::vector<std::pair<int,int>> line3;
    //        LineAlgorithms::getSoftBresenham(convertToXindex(midle1.x),convertToYindex(midle1.y),convertToXindex(midle2.x),convertToYindex(midle2.y),line3);

    //        for(unsigned int g=0;g<line3.size();g++){
    //            if(isOccupiedIndexXY(line3[g].first,line3[g].second)){
    //                return true;
    //            }
    //        }


    //        return occupied;
    //    }


    template<typename PointC>
    void getObstacleAngles(PointC pc,double radius,std::vector<double> &angle){

        int x=convertToXindex(pc.x);
        int y=convertToYindex(pc.y);

        int t=radius/cellsizeX;

        for( int l=x-t;l<x+t;l++){
            for( int g=y-t;g<y+t;g++){

                if(isOccupiedIndexXY(l,g)){
                    angle.push_back(PoseTools::angleBetween(x,y,l,g));
                }
            }
        }
    }

    template<typename PointC>
    void getObstacleAnglesDistance(PointC pc,double radius,std::vector<std::pair<double,double> > &angle){

        int x=convertToXindex(pc.x);
        int y=convertToYindex(pc.y);

        int t=radius/cellsizeX;

        for( int l=x-t;l<x+t;l++){
            for( int g=y-t;g<y+t;g++){

                if(isOccupiedIndexXY(l,g)){
                    angle.emplace_back(PoseTools::angleBetween(x,y,l,g),PoseTools::distance2D(x,y,l,g)/cellsizeX);
                }
            }
        }
    }


    




    template<typename XXX>
    std::vector<XXX> getPoints(double X,double Y,double th,double R){

        std::vector<XXX> out;

        int x=convertToXindex(X);
        int y=convertToYindex(Y);

        int windowx=R/cellsizeX;
        int windowy=R/cellsizeY;

        XXX aux;

        for(int i=(x-windowx);i<=(x+windowx);i++){
            for(int f=(y-windowy);f<=(y+windowy);f++){


                if(getIndexXY(i,f)>=th){
                    aux.x=convertToXFromIndex(i);
                    aux.y=convertToYFromIndex(f);
                    out.push_back(aux);
                }
            }
        }


        return out;
    }


 

    bool isOccupied(std::vector<geometry_msgs::Pose> &pts){

        for(unsigned int i=0;i<pts.size();i++){
            if(isOccupiedXY(pts[i].position.x,pts[i].position.y)){
                return true;
            }
        }

        return false;
    }

    bool isOccupied(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2){

        // std::vector<std::pair<int,int>> line1;
        // LineAlgorithms::getSoftBresenham(convertToXindex(p1.position.x),convertToYindex(p1.position.y),convertToXindex(p2.position.x),convertToYindex(p2.position.y),line1);

        // for(unsigned int g=0;g<line1.size();g++){
        //     if(isOccupiedIndexXY(line1[g].first,line1[g].second)){
        //         return true;
        //     }
        // }
        // return false;

        return isOccupiedLine(convertToXindex(p1.position.x),convertToYindex(p1.position.y),convertToXindex(p2.position.x),convertToYindex(p2.position.y));
    }

    bool isOccupiedIndexXYSquare(unsigned int x,unsigned int y,double distanceX,double distanceY){



        int indexX=round(distanceX/cellsizeX);
        int indexY=round(distanceY/cellsizeY);


        for( int l=x-indexX;l<x+indexX;l++){
            for( int g=y-indexY;g<y+indexY;g++){

                if(isOccupiedIndexXY(l,g)){

                    return true;
                }

            }
        }

        return false;



    }


    bool isOccupiedXYSquare(double X,double Y,double distanceX,double distanceY){

        unsigned int x=convertToXindex(X);
        unsigned int y=convertToYindex(Y);

        int indexX=round(distanceX/cellsizeX);
        int indexY=round(distanceY/cellsizeY);


        for( int l=x-indexX;l<x+indexX;l++){
            for( int g=y-indexY;g<y+indexY;g++){

                if(isOccupiedIndexXY(l,g)){

                    return true;
                }

            }
        }

        return false;



    }


    bool isOccupiedXYSquare(double X,double Y,double Theta,double distanceX,double distanceY,double distance  ) {


        unsigned int k=convertToXindex(X);
        unsigned int n=convertToYindex(Y);

        int indexX=round(distance/cellsizeX);
        int indexY=round(distance/cellsizeY);

        std::vector<Point> Points;
        Points.push_back(Point(convertToXindex(X+distanceX*cos(Theta+M_PI/2.0)+distanceY*cos(Theta)),convertToYindex(Y+distanceX*sin(Theta+M_PI/2.0)+distanceY*sin(Theta)),0));
        Points.push_back(Point(convertToXindex(X+distanceX*cos(Theta+M_PI/2.0)-distanceY*cos(Theta)),convertToYindex(Y+distanceX*sin(Theta+M_PI/2.0)-distanceY*sin(Theta)),0));
        Points.push_back(Point(convertToXindex(X+distanceX*cos(Theta-M_PI/2.0)-distanceY*cos(Theta)),convertToYindex(Y+distanceX*sin(Theta-M_PI/2.0)-distanceY*sin(Theta)),0));
        Points.push_back(Point(convertToXindex(X+distanceX*cos(Theta-M_PI/2.0)+distanceY*cos(Theta)),convertToYindex(Y+distanceX*sin(Theta-M_PI/2.0)+distanceY*sin(Theta)),0));
        Points.push_back(Point(convertToXindex(X+distanceX*cos(Theta+M_PI/2.0)+distanceY*cos(Theta)),convertToYindex(Y+distanceX*sin(Theta+M_PI/2.0)+distanceY*sin(Theta)),0));

        for(long int l=k-indexX;l<k+indexX;l++){
            for(long int g=n-indexY;g<n+indexY;g++){

                if(isOccupiedIndexXY(l,g)){

                    Point test(l,g,0);

                    unsigned int i;
                    int j, c = 0;
                    for (i = 0, j = Points.size()-1; i < Points.size(); j = i++) {
                        if ( ((Points[i].y>test.y) != (Points[j].y>test.y)) &&
                             (test.x < (Points[j].x-Points[i].x) * (test.y-Points[i].y) / (Points[j].y-Points[i].y) + Points[i].x) )
                            c = !c;
                    }

                    if(c==1){
                        return true;
                    }
                }
            }

        }

        return false;

    }

    bool isOccupiedXYSquare2(double X,double Y,double Theta,double distanceX,double distanceY,double distance  ) {


        unsigned int k=convertToXindex(X);
        unsigned int n=convertToYindex(Y);


        int indexX=round(distance/cellsizeX);
        int indexY=round(distance/cellsizeY);

        distanceY=distanceY/2.0;


        std::vector<Point> Points2;
        Points2.push_back(Point( (X+distanceY*cos(Theta+M_PI/2.0)), (Y+distanceY*sin(Theta+M_PI/2.0)),0));
        Points2.push_back(Point( (X+distanceY*cos(Theta+M_PI/2.0))+distanceX*cos(Theta), (Y+distanceY*sin(Theta+M_PI/2.0)+distanceX*sin(Theta)),0));
        Points2.push_back(Point( (X+distanceY*cos(Theta-M_PI/2.0))+distanceX*cos(Theta), (Y+distanceY*sin(Theta-M_PI/2.0)+distanceX*sin(Theta)),0));
        Points2.push_back(Point( (X+distanceY*cos(Theta-M_PI/2.0)), (Y+distanceY*sin(Theta-M_PI/2.0)),0));
        Points2.push_back(Point( (X+distanceY*cos(Theta+M_PI/2.0)), (Y+distanceY*sin(Theta+M_PI/2.0)),0));


        //        for(unsigned int l=std::max((k-indexX),(unsigned int)  0);l<(k+indexX);l++){
        //            for(unsigned int g=std::max((n-indexY),(unsigned int) 0);g<(n+indexY);g++){
        for(long int l=k-indexX;l<k+indexX;l++){
            for(long int g=n-indexY;g<n+indexY;g++){

                if(isOccupiedIndexXY(l,g)){
                    Point test(convertToXFromIndex( l),convertToYFromIndex( g),0);

                    unsigned int i;
                    int j, c = 0;
                    for (i = 0, j = Points2.size()-1; i < Points2.size(); j = i++) {
                        if ( ((Points2[i].y>test.y) != (Points2[j].y>test.y)) &&
                             (test.x < (Points2[j].x-Points2[i].x) * (test.y-Points2[i].y) / (Points2[j].y-Points2[i].y) + Points2[i].x) )
                            c = !c;
                    }
                    if(c==1){


                        return true;
                    }
                }
            }

        }

        return false;

    }

    bool isOffMap(unsigned int column,unsigned int row){

        if((column>=columns)||(row>=rows)){
            return true;
        }

        return false;

    }



 
    void getBresenhamLine( int x0,   int y0,  int x1,  int y1,std::vector<std::pair<int,int> > &output){

        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return;
        }

        int mx=std::max(dx,dy);
        output.reserve(output.size()+2*mx);
        output.emplace_back(x0,y0);  // Alterar
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;

                output.emplace_back(x0,y0);  // Alterar
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;

                output.emplace_back(x0,y0);  // Alterar

            }
        }

        return;

    }

 

    void setOccupiedXY(double X,double Y){
        setXY(X,Y,occupiedValue);
    }
    void setOccupied(unsigned int  X,unsigned int Y){
        setIndexXY(X,Y,occupiedValue);
    }

    void setFreeXY(double X,double Y){
        setXY(X,Y,freeValue);
    }
    void setFree(unsigned int  X,unsigned int Y){
        setIndexXY(X,Y,freeValue);
    }

    inline void unprotected_setFree(unsigned int X,unsigned int Y){
        //setIndexXY(X,Y,freeValue);
        indexer[Y][X]=freeValue;
    }
    inline void unprotected_setOccupied(unsigned int X,unsigned int Y){
        //setIndexXY(X,Y,freeValue);
        indexer[Y][X]=occupiedValue;
    }


    void setFreeLine_FullLine(  int x0,   int y0,  int x1,  int y1){

        int sx,sy,err,dx,dy,e2;

        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        while(true){
            unprotected_setFree(x0,y0);
            if ((x0 == x1) && (y0 == y1)){
                //exit loop
                return;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
            }
            if ((x0 == x1) && (y0 == y1)){
                unprotected_setFree(x0,y0);
            }
            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
            }
        }

        return;
    }




    void setFreeLineXY(  double x0,   double y0,   double x1,   double y1){

        int x0_=convertToXindex(x0);
        int y0_=convertToYindex(y0);
        int x1_=convertToXindex(x1);
        int y1_=convertToYindex(y1);

        setFreeLine(x0_,y0_,x1_,y1_);
    }


    void setLineXY(  double x0,   double y0,   double x1,   double y1, T value){

        int x0_=convertToXindex(x0);
        int y0_=convertToYindex(y0);
        int x1_=convertToXindex(x1);
        int y1_=convertToYindex(y1);

        setLineIndexXY(x0_,y0_,x1_,y1_,value);
    }

    void setMaxLineXY(  double x0,   double y0,   double x1,   double y1, T value){

        int x0_=convertToXindex(x0);
        int y0_=convertToYindex(y0);
        int x1_=convertToXindex(x1);
        int y1_=convertToYindex(y1);

        setMaxLineIndexXY(x0_,y0_,x1_,y1_,value);
    }





    void setFreeLine(  int x0,   int y0,   int x1,   int y1){

        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return;
        }

        int mx=std::max(dx,dy)-1;
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;


                unprotected_set(y0,x0,freeValue);  // Alterar
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;


                unprotected_set(y0,x0,freeValue);  // Alterar

            }
        }

        return;
    }

    void setFreeLine(  int x0,   int y0,   int x1,   int y1,double dz){

        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return;
        }

        int mx=std::max(dx,dy)-1;
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;


                unprotected_set(y0,x0,dz);  // Alterar
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;


                unprotected_set(y0,x0,dz);  // Alterar

            }
        }

        return;
    }

    //    void setFreeLine(  int x0,   int y0,   int x1,   int y1){

    //        int sx,sy,err,dx,dy,e2;
    //        dx = abs((int)x1-(int)x0);
    //        dy = abs((int)y1-(int)y0);
    //        if (x0 < x1) sx = 1; else sx = -1;
    //        if (y0 < y1) sy = 1; else sy = -1;
    //        err = dx-dy;

    //        //         std::cout<<"##### FreeLine"<<std::endl;

    //        while(true){
    //            //             std::cout<<"->  x: "<<x0<<"  y: "<<y0<<std::endl;
    //            //            unprotected_setFree(x0,y0);
    //            setFree(x0,y0);

    //            if ((x0 == x1) && (y0 == y1)){
    //                //exit loop
    //                return;
    //            }
    //            e2 = 2*err;
    //            if (e2 > -dy){
    //                err = err - dy;
    //                x0 = x0 + sx;
    //            }
    //            if ((x0 == x1) && (y0 == y1)){
    //                // unprotected_setFree(x0,y0);
    //            }
    //            if (e2 <  dx){
    //                err = err + dx;
    //                y0 = y0 + sy;
    //            }
    //        }

    //        return;
    //    }

    void setOccupiedLine(  int x0,   int y0,   int x1,   int y1){


        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        while(true){
            unprotected_setOccupied(x0,y0);
            if ((x0 == x1) && (y0 == y1)){
                //  unprotected_setOccupied(x0,y0);
                //exit loop
                return;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
            }
            if ((x0 == x1) && (y0 == y1)){
                unprotected_setOccupied(x0,y0);
            }
            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
            }
        }

        return;
    }


    void setMaxLineIndexXY(  int x0,   int y0,   int x1,   int y1,T value){


        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return;
        }
        setMaxIndexXY(x0,y0,value);

        while(true){
            if ((x0 == x1) && (y0 == y1)){
                return;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
                setMaxIndexXY(x0,y0,value);

            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
                setMaxIndexXY(x0,y0,value);
            }
        }

        return;
    }







    //    void getLineIndexXY2(  int x0,   int y0,   int x1,   int y1,std::map<int,std::pair<int,int>> &data){


    //        int sx,sy,err,dx,dy,e2;
    //        dx = abs((int)x1-(int)x0);
    //        dy = abs((int)y1-(int)y0);
    //        if (x0 < x1) sx = 1; else sx = -1;
    //        if (y0 < y1) sy = 1; else sy = -1;
    //        err = dx-dy;


    //        int ct=0;
    //        int mx=std::max(dx,dy);

    //        while(ct<mx){

    //            if ((x0 == x1) && (y0 == y1)){
    //                return;
    //            }
    //            e2 = 2*err;
    //            if (e2 > -dy){
    //                err = err - dy;
    //                x0 = x0 + sx;
    //                if ((x0 == x1) && (y0 == y1)){
    //                    return;
    //                }
    //                data[x0*count+y0]=std::pair<int,int>(x0,y0);


    //             }

    //            if (e2 <  dx){
    //                err = err + dx;
    //                y0 = y0 + sy;
    //                if ((x0 == x1) && (y0 == y1)){
    //                    return;
    //                }
    //                data[x0*count+y0]=std::pair<int,int>(x0,y0);




    //            }

    //            ct++;
    //        }

    //        return;
    //    }







    void setLineIndexXY2(  int x0,   int y0,   int x1,   int y1,T value,int countx,bool freeit=true){


        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;


        if(freeit){
            takeIndexXY(x0,y0,value);
        }else{
            addIndexXY(x0,y0,value);
        }

        int ct=0;
        while(ct<countx){

            if ((x0 == x1) && (y0 == y1)){
                return;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
                if ((x0 == x1) && (y0 == y1)){
                    return;
                }
                if(freeit){
                    takeIndexXY(x0,y0,value);
                }else{
                    addIndexXY(x0,y0,value);
                }

                ct++;
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
                if ((x0 == x1) && (y0 == y1)){
                    return;
                }
                if(freeit){
                    takeIndexXY(x0,y0,value);
                }else{
                    addIndexXY(x0,y0,value);
                }

                ct++;

            }
        }

        return;
    }



    void setLineIndexXYBuffer(  int x0,   int y0,   int x1,   int y1,Map<T> *value){

        int sx,sy,err,dx,dy,e2;
        dx = abs(x1-x0);
        dy = abs(y1-y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;
        //        if ((x0 == x1) && (y0 == y1)){
        //            return;
        //        }

        if ((x1 < 0) || (x1 >= columns) || (y1 < 0) || (y1 >= rows)) {
            return;
        }
        if ((x0 < 0) || (x0 >= columns) || (y0 < 0) || (y0 >= rows)) {
            return;
        }

        int mx=std::max(dx,dy);
        for(int i=0;i<mx;i++){
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;


                value->setIndexXY(x0,y0,111);
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;


                value->setIndexXY(x0,y0,111);

            }
        }
    }
    //        int sx,sy,err,dx,dy,e2;
    //        dx = abs((int)x1-(int)x0);
    //        dy = abs((int)y1-(int)y0);
    //        if (x0 < x1) sx = 1; else sx = -1;
    //        if (y0 < y1) sy = 1; else sy = -1;
    //        err = dx-dy;

    //        value->setIndexXY(x0,y0,111);

    //        while(true){

    //            e2 = 2*err;
    //            if (e2 > -dy){
    //                err = err - dy;
    //                x0 = x0 + sx;
    //                if ((x0 == x1) && (y0 == y1)){
    //                    return;
    //                }

    //            }

    //            if (e2 <  dx){
    //                err = err + dx;
    //                y0 = y0 + sy;
    //                if ((x0 == x1) && (y0 == y1)){
    //                    return;
    //                }
    //                value->setIndexXY(x0,y0,111);
    //            }
    //        }

    //        return;
    //    }

    void setLineIndexXY(  int x0,   int y0,   int x1,   int y1,T value){


        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        setIndexXY(x0,y0,value);

        while(true){
            if ((x0 == x1) && (y0 == y1)){
                return;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
                setIndexXY(x0,y0,value);

            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
                setIndexXY(x0,y0,value);
            }
        }

        return;
    }

    std::pair<int,int> transitions(  int x0,   int y0,   int x1,   int y1,T value){

        std::pair<int,int> out(0,0);
        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        T v;
        v=getIndexXY(x0,y0);
        if(v!=value){
            out.first++;
        }else{
            out.second++;
        }

        while(true){

            if ((x0 == x1) && (y0 == y1)){
                //  unprotected_setOccupied(x0,y0);
                //exit loop
                return out;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
                v=getIndexXY(x0,y0);
                if(v!=value){
                    out.first++;
                }else{
                    out.second++;
                }
            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
                v=getIndexXY(x0,y0);
                if(v!=value){
                    out.first++;
                }else{
                    out.second++;
                }
            }
        }

        return out;
    }

    std::pair<int,int> transitions2(  int x0,   int y0,   int x1,   int y1,T value){

        std::pair<int,int> out(0,0);
        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        T v;
        v=getIndexXY(x0,y0);
        if(v!=value){
            out.first++;
        }else{
            out.second++;
        }

        while(true){

            if ((x0 == x1) && (y0 == y1)){
                //  unprotected_setOccupied(x0,y0);
                //exit loop
                return out;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
                v=getIndexXY(x0,y0);
                if(v!=value){
                    out.first++;
                }else{
                    out.second++;
                }


            }

            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
                v=getIndexXY(x0,y0);
                if(v!=value){
                    out.first++;
                }else{
                    out.second++;
                }
            }

            if(out.first!=0 && out.second!=0){
                return out;
            }
        }

        return out;
    }




    void setOccupiedUltraPixel(unsigned int  X,unsigned int Y){

        setIndexXY(X-1,Y-1,occupiedValue);
        setIndexXY(X-1,Y,occupiedValue);
        setIndexXY(X-1,Y+1,occupiedValue);
        setIndexXY(X,Y-1,occupiedValue);
        setIndexXY(X,Y,occupiedValue);
        setIndexXY(X,Y+1,occupiedValue);
        setIndexXY(X+1,Y-1,occupiedValue);
        setIndexXY(X+1,Y,occupiedValue);
        setIndexXY(X+1,Y+1,occupiedValue);

    }

    void setFreeUltraPixel(unsigned int  X,unsigned int Y){

        setIndexXY(X-1,Y-1,freeValue);
        setIndexXY(X-1,Y,freeValue);
        setIndexXY(X-1,Y+1,freeValue);
        setIndexXY(X,Y-1,freeValue);
        setIndexXY(X,Y,freeValue);
        setIndexXY(X,Y+1,freeValue);
        setIndexXY(X+1,Y-1,freeValue);
        setIndexXY(X+1,Y,freeValue);
        setIndexXY(X+1,Y+1,freeValue);

    }

    void setOccupiedUltraPixelXY(double  x,double y){

        unsigned int X=convertToXindex(x);
        unsigned int Y=convertToYindex(y);
        setIndexXY(X-1,Y-1,occupiedValue);
        setIndexXY(X-1,Y,occupiedValue);
        setIndexXY(X-1,Y+1,occupiedValue);
        setIndexXY(X,Y-1,occupiedValue);
        setIndexXY(X,Y,occupiedValue);
        setIndexXY(X,Y+1,occupiedValue);
        setIndexXY(X+1,Y-1,occupiedValue);
        setIndexXY(X+1,Y,occupiedValue);
        setIndexXY(X+1,Y+1,occupiedValue);
    }

    void setFreeUltraPixelXY(double x,double y){

        unsigned int X=convertToXindex(x);
        unsigned int Y=convertToYindex(y);

        setIndexXY(X-1,Y-1,freeValue);
        setIndexXY(X-1,Y,freeValue);
        setIndexXY(X-1,Y+1,freeValue);
        setIndexXY(X,Y-1,freeValue);
        setIndexXY(X,Y,freeValue);
        setIndexXY(X,Y+1,freeValue);
        setIndexXY(X+1,Y-1,freeValue);
        setIndexXY(X+1,Y,freeValue);
        setIndexXY(X+1,Y+1,freeValue);
    }



    bool isOccupiedLine(unsigned int x0,unsigned  int y0,unsigned  int x1,unsigned  int y1,std::pair<unsigned int,unsigned int> &node){


        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        while(true){
            if(unprotected_isOccupiedIndexXY(x0,y0)){
                node.first=x0;
                node.second=y0;
                return true;
            }
            if ((x0 == x1) && (y0 == y1)){
                //exit loop
                return false;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
            }
            if ((x0 == x1) && (y0 == y1)){
                if(unprotected_isOccupiedIndexXY(x0,y0)){
                    node.first=x0;
                    node.second=y0;
                    return true;
                }
            }
            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
            }
        }


        return false;
    }


    bool isOccupiedLine(unsigned int x0,unsigned  int y0,unsigned  int x1,unsigned  int y1){


        int sx,sy,err,dx,dy,e2;
        dx = abs((int)x1-(int)x0);
        dy = abs((int)y1-(int)y0);
        if (x0 < x1) sx = 1; else sx = -1;
        if (y0 < y1) sy = 1; else sy = -1;
        err = dx-dy;

        while(true){
            if(isOccupiedIndexXY(x0,y0)){
                return true;
            }
            if ((x0 == x1) && (y0 == y1)){
                //exit loop
                return false;
            }
            e2 = 2*err;
            if (e2 > -dy){
                err = err - dy;
                x0 = x0 + sx;
            }
            if ((x0 == x1) && (y0 == y1)){
                if(isOccupiedIndexXY(x0,y0)){
                    return true;
                }
            }
            if (e2 <  dx){
                err = err + dx;
                y0 = y0 + sy;
            }
        }


        return false;
    }



    bool isOccupiedLine(geometry_msgs::Pose a,geometry_msgs::Pose b){

        //        int startX=convertToXindex(a.position.x);
        //        int startY=convertToYindex(a.position.y);
        //        int endX=convertToXindex(b.position.x);
        //        int endY=convertToYindex(b.position.y);

        //        std::vector<std::pair<int,int> > da;
        //        LineAlgorithms::getSoftBresenham(startX,startY,endX,endY,da);

        //        int si=da.size();
        //        for( int h=0;h<si-1;h++){
        //            if(isOccupiedIndexXY(da[h].first,da[h].second)){
        //                return true;
        //            }
        //        }
        //        return false;

        return isOccupiedLine(convertToXindex(a.position.x),convertToYindex(a.position.y),convertToXindex(b.position.x),convertToYindex(b.position.y));
    }

 




    bool neighborObstacle(unsigned int Xn,unsigned int Yn,bool temp){
        //UNUSED(temp);

        int Xmove[]={0,1,0,0,1,1,-1,-1,-1};
        int Ymove[]={0,0,1,-1,1,-1,0,1,-1};

        //+ ... ...
        //... ... ...
        //... ... ...
        //... ... -
        if(validIndex(Xn-1,Yn-1) && validIndex(Xn+1,Yn+1)){

            for(int i=0;i<8;i++){
                if((unprotected_getIndexXY( Xn+Xmove[i], Yn+Ymove[i])>=occupiedValue)){
                    return true;
                }
            }
        }else{
            for(int i=0;i<8;i++){
                if((getIndexXY( Xn+Xmove[i], Yn+Ymove[i])>=occupiedValue)){
                    return true;
                }
            }
        }

        return false;
        //        int Xmove[]={0,1,0,0,1,1,-1,-1,-1};
        //        int Ymove[]={0,0,1,-1,1,-1,0,1,-1};

        //        for(int i=0;i<8;i++){
        //            if((getIndexXY( X+Xmove[i], Y+Ymove[i])>=occupiedValue)){
        //                return true;
        //            }
        //        }

        //        return false;
    }

    bool neighborObstacle(double X,double Y){


        int Xn=convertToXindex(X);
        int Yn=convertToYindex(Y);
        int Xmove[]={0,1,0,0,1,1,-1,-1,-1};
        int Ymove[]={0,0,1,-1,1,-1,0,1,-1};

        //+ ... ...
        //... ... ...
        //... ... ...
        //... ... -
        if(validIndex(Xn-1,Yn-1) && validIndex(Xn+1,Yn+1)){

            for(int i=0;i<8;i++){
                if((unprotected_getIndexXY( Xn+Xmove[i], Yn+Ymove[i])>=occupiedValue)){
                    return true;
                }
            }
        }else{
            for(int i=0;i<8;i++){
                if((getIndexXY( Xn+Xmove[i], Yn+Ymove[i])>=occupiedValue)){
                    return true;
                }
            }
        }

        return false;
    }


    std::vector<double> getWindow(int X,int Y,int var){
     std::vector<double> out;


        for(unsigned int i=(X- var);i<=(X+ var);i++){
            for(unsigned int f=(Y- var);f<=(Y+ var);f++){

                out.push_back(getIndexXY(i,f));
            }
        }


        return out;
    }

    void setInflatedObstacleXY(double Xg,double Yg,double distance){

        unsigned int X=convertToXindex(Xg);
        unsigned int Y=convertToYindex(Yg);
        unsigned int Xvar=round(distance/cellsizeX);
        unsigned int Yvar=round(distance/cellsizeY);

        for(unsigned int i=(X-Xvar);i<=(X+Xvar);i++){
            for(unsigned int f=(Y-Yvar);f<=(Y+Yvar);f++){

                setIndexXY(i,f,occupiedValue);
            }
        }
    }

    void setInflatedObstacle(unsigned int X,unsigned int Y,double distance){

        unsigned int Xvar=round(distance/cellsizeX);
        unsigned int Yvar=round(distance/cellsizeY);

        for(unsigned int i=(X-Xvar);i<=(X+Xvar);i++){
            for(unsigned int f=(Y-Yvar);f<=(Y+Yvar);f++){

                setIndexXY(i,f,occupiedValue);
            }
        }
    }

    void setInflatedObstacle(Point center,double distance){

        unsigned int X=convertToXindex(center.x);
        unsigned int Y=convertToYindex(center.y);

        unsigned int Xvar=round(distance/cellsizeX);
        unsigned int Yvar=round(distance/cellsizeY);

        for(unsigned int i=(X-Xvar);i<=(X+Xvar);i++){
            for(unsigned int f=(Y-Yvar);f<=(Y+Yvar);f++){
                setIndexXY(i,f,occupiedValue);
            }
        }
    }


    // #########################################################################################################################
    // cell operations
    // #########################################################################################################################


    void incrementXY(double  X,double Y){

        unsigned int x=convertToXindex(X);
        unsigned int y=convertToYindex(Y);
        increment(x,y);
    }

    void decrementXY(double X,double Y){

        unsigned int x=convertToXindex(X);
        unsigned int y=convertToYindex(Y);
        decrement(x,y);
    }

    void increment(unsigned int  X,unsigned int Y){
        setIndexXY(X,Y,getIndexXY(X,Y)+1);
    }

    void decrement(unsigned int  X,unsigned int Y){
        setIndexXY(X,Y,getIndexXY(X,Y)-1);
    }

    void decrement(T value){
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int j=0;j<columns;j++){
                unprotected_set(j,i,unprotected_get(j,i)-value);
            }
        }
    }

    // #########################################################################################################################
    // converters  XY -> ij  and  ij ->XY
    // #########################################################################################################################







    std::pair<double,double> getMapCoords2(double px,double py){
        std::pair<double,double> out;
        out.first=  ((px+scellsizeX_2)/cellsizeX)+ (columns_25);
        out.second=  ((py+scellsizeY_2)/cellsizeY)+ (rows_25);
        return out;
    }



//    inline int convertToXindex(double X){
//        return floor((X+scellsizeX_2)/cellsizeX)+fcolumns_2;
//    }

//    inline int convertToYindex(double Y){
//        return floor((Y+scellsizeY_2)/cellsizeY)+frows_2;
//    }

    inline int convertToXindexDebug(double X){
        std::cout<<"in "<<X<<std::endl;
        std::cout<<"-start.x+cellsizeX/2.0 "<<-start.x+cellsizeX/2.0<<std::endl;

        std::cout<<"fcolumns_2 "<<fcolumns_2<<std::endl;
        std::cout<<"scellsizeX_2 "<<scellsizeX_2<<std::endl;
        std::cout<<"out "<<floor((X+scellsizeX_2)/cellsizeX)+fcolumns_2<<std::endl;

        return -1;
    }

    inline int convertToYindexDebug(double Y){
        std::cout<<"in "<<Y<<std::endl;
        std::cout<<"-start.y+cellsizeY/2.0 "<<-start.y+cellsizeY/2.0<<std::endl;

        std::cout<<"frows_2 "<<frows_2<<std::endl;
        std::cout<<"scellsizeY_2 "<<scellsizeY_2<<std::endl;
        std::cout<<"out  "<<floor((Y+scellsizeY_2)/cellsizeY)+frows_2<<std::endl;

        return -1;
    }

    inline int convertToXindexZero(double X){
        return floor((X+cellsizeX_2)/cellsizeX)+fcolumns_2;
    }

    inline int convertToYindexZero(double Y){
        return floor((Y+cellsizeY_2)/cellsizeY)+frows_2;
    }




    inline  int convertToXindex(double X){
          return floor(((X-start.x+cellsizeX/2.0)/cellsizeX)+fcolumns_2);
     }
    inline int convertToYindex(double Y){
          return floor(((Y-start.y+cellsizeY/2.0)/cellsizeY)+frows_2);

     }





    inline double convertToXFromIndex(int i){
         return cellsizeX*((double)i-fcolumns_2)+start.x;
//        return cellsizeX*i+cxfcolumns_2;

    }




    inline double convertToYFromIndex(int f){
         return cellsizeY*((double)f-frows_2)+start.y;
//        return cellsizeY*f+cyfrows_2;

    }


    //    inline double convertToXFromIndex2(int i){
    //        return cellsizeX*(i+0.5-floor(columns/2.0))+start.x;
    //    }

    //    inline double convertToYFromIndex2(int f){
    //        return cellsizeY*(f+0.5-floor(rows/2.0))+start.y;
    //    }

    // #########################################################################################################################
    // gets and sets
    // #########################################################################################################################


    unsigned int getRows()const{
        return rows;
    }

    unsigned int getColumns()const{
        return columns;
    }

    inline double getXcellsize(){
        return cellsizeX;
    }

    inline double getYcellsize(){
        return cellsizeY;
    }

    unsigned int sizeY() const{
        return rows;
    }

    unsigned int sizeX() const{
        return columns;
    }

    void setCellSizes(double cellsizeX_,double cellsizeY_,Point start_){
        cellsizeX=cellsizeX_;
        cellsizeY=cellsizeY_;
        start=start_;

        columns_2=(columns)/2.0;
        rows_2=(rows)/2.0;
        columns_25=(columns)/2.0 -0.5;
        rows_25=(rows)/2.0 -0.5;
        fcolumns_2=floor((columns)/2.0);
        frows_2=floor((rows)/2.0);

        cellsizeX_2=cellsizeX/2.0;
        cellsizeY_2=cellsizeY/2.0;
        scellsizeX_2=-start.x+cellsizeX/2.0;
        scellsizeY_2=-start.y+cellsizeY/2.0;

        cyfrows_2=-frows_2*cellsizeY+start.y;
        cxfcolumns_2=-fcolumns_2*cellsizeX+start.x;

        cyrows_2=-rows_2*cellsizeY+start.y;
        cxcolumns_2=-columns_2*cellsizeX+start.x;

      fx=scellsizeX_2/cellsizeX+ columns_2-0.5;
      fy=scellsizeY_2/cellsizeY+ rows_2-0.5;
    }

    inline void setName(std::string MapName_){
        MapName = MapName_;
    }

    std::string getName( ){
        return MapName;
    }

    T getOccValue(){
        return occupiedValue;
    }

    T getFreeValue(){
        return freeValue;
    }

    Point getStart(){
        return start;
    }

    void setOccupiedAndFree(T occ_,T free_){
        freeValue=free_;
        occupiedValue=occ_;
    }

    // #########################################################################################################################
    // constructors and destructor
    // #########################################################################################################################

    void clearMap(){

        zero();
    }

    void zero(){
        memset(matrix,0,(count)*sizeof(T));
    }

    void pattern(){

        for(unsigned int i=0;i<rows;i++){

            if(i%2==0){
                continue;
            }

            for(unsigned int j=0;j<columns;j++){

                unprotected_set(j,i,255);

            }
        }
    }

    void all(T val){
        std::fill(matrix, matrix + (count), val);
    }

    void random(){

        srand(5555);

        for(int i=0;i<rows;i++){
            for(int f=0;f<columns;f++){

                T data=(T)rand()/(T)RAND_MAX;
                unprotected_setIndexXY(i,f,data);
            }
        }
    }

    void reshapeF(unsigned int rows_, unsigned int columns_){

        dispose();
        rows=rows_;
        columns=columns_;

        matrix =new T [rows_*columns_];
        indexer=new T*[columns_];
        count=columns*rows;

        for(unsigned int i=0;i<columns_;i++){
            indexer[i]=&matrix[i*rows_];
        }

    }

    void reshape(unsigned int rows_, unsigned int columns_){

        dispose();
        rows=rows_;
        columns=columns_;

        matrix =new T [rows_*columns_];
        indexer=new T*[columns_];
        count=columns*rows;

        for(unsigned int i=0;i<columns_;i++){
            indexer[i]=&matrix[i*rows_];
        }
        zero();
    }


    // #########################################################################################################################
    // Opengl connection
    // #########################################################################################################################

    T * toGrayMap(){
        return matrix;
    }

    unsigned char * getTextureMap(){

        unsigned char *map;

        map= new unsigned char[count];

        for(unsigned int i=0;i<count;i++){
            map[i]=round(matrix[i]*255.0);
        }
        return map;
    }

    // #########################################################################################################################
    // from ROS to ROS
    // #########################################################################################################################

    void fromROSOccupancyGrid(nav_msgs::OccupancyGrid *occupancy){

        index=0;
        reshapeF(occupancy->info.height,occupancy->info.width);
        cellsizeX=occupancy->info.resolution;
        cellsizeY=occupancy->info.resolution;
        for(unsigned int i=0;i<count;i++){
            matrix[i]=occupancy->data.data()[i];
        }
        start.x=occupancy->info.origin.position.x+((double)getColumns()+1.0)*cellsizeX_2;
        start.y=occupancy->info.origin.position.y+((double)getRows()+1.0)*cellsizeY_2;

    }

    void toROSOccupancyGrid(nav_msgs::OccupancyGrid *occupancy){

        occupancy->info.height=rows;
        occupancy->info.width=columns;
        occupancy->info.resolution=cellsizeX;
        //Clock a;
        //a.start();
        occupancy->data.assign(matrix,matrix+count);
        //std::cout<<a.elapsed()<<std::endl;

        occupancy->info.origin.position.x=start.x-((double)getColumns()+1.0)*cellsizeX_2;
        occupancy->info.origin.position.y=start.y-((double)getRows()+1.0)*cellsizeY_2;
        occupancy->info.origin.position.z=0;

    }

    void toROSOccupancyGrid(nav_msgs::OccupancyGrid *occupancy,Point newcenter,int rows_,int cols_,int vmin,int  vmax){

        occupancy->info.height=rows_;
        occupancy->info.width=cols_;
        occupancy->info.resolution=cellsizeX;
        //Clock a;
        //a.start();

        unsigned int x=convertToXindex(newcenter.x);
        unsigned int y=convertToYindex(newcenter.y);



#warning "this can be a big error here"

        for(unsigned int j=(y-rows_/2.0);j<(y+rows_/2.0);j++){
            for(unsigned int i=(x-cols_/2.0);i<(x+cols_/2.0);i++){

                occupancy->data.push_back(this->isOccupiedIndexXY(i,j)?vmax:(this->getIndexXY(i,j)>freeValue?50:vmin));
            }
        }
        //        occupancy->data.assign(matrix,matrix+count);
        //std::cout<<a.elapsed()<<std::endl;


        occupancy->info.origin.position.x=0-((double)getColumns()+1.0)*cellsizeX_2;
        occupancy->info.origin.position.y=0-((double)getRows()+1.0)*cellsizeY_2;
        occupancy->info.origin.position.z=0;

    }

     
  
    // #########################################################################################################################
    // to String
    // #########################################################################################################################




    template <typename Td>
    std::string toStringModified(){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<(Td)unprotected_get(i,f)<<" ";
            }
            out<<std::endl;
        }
        return out.str();
    }

    std::string toString(){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<unprotected_get(i,f)<<" ";
            }
            out<<std::endl;
        }
        return out.str();
    }

    std::string toString(T th,T vl=0){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<(unprotected_get(i,f)>th?vl:0)<<" ";
            }

            if(i<rows-1){
                out<<";"<<std::endl;
            }else{

            out<<std::endl;
            }
        }
        return out.str();
    }

    std::string toString2(){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<(unprotected_get(i,f))<<" ";
            }

            if(i<rows-1){
                out<<";"<<std::endl;
            }else{

            out<<std::endl;
            }
        }
        return out.str();
    }

    void toOutput(){
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                std::cout<<unprotected_get(i,f)<<" ";
            }
            std::cout<<std::endl;
        }
    }

    // #########################################################################################################################
    // Methods
    // #########################################################################################################################



 
    geometry_msgs::Pose getMinimunIntersectionToObstacle(geometry_msgs::Pose start_,geometry_msgs::Pose end_){

        int startX=convertToXindex(start_.position.x);
        int startY=convertToYindex(start_.position.y);
        int endX=convertToXindex(end_.position.x);
        int endY=convertToYindex(end_.position.y);


        std::pair<unsigned int,unsigned int> pt;
        geometry_msgs::Pose out;
        if(isOccupiedLine(startX,startY,endX,endY,pt)){

            out.position.x= convertToXFromIndex(pt.first);
            out.position.y= convertToYFromIndex(pt.second);
            return out;
        }

        out=end_;
        out.position.z=-1;


        //        std::vector<std::pair<int,int> > result;
        //        LineAlgorithms::getSoftBresenham(startX,startY,endX,endY,result);
        //        geometry_msgs::Pose out;

        //        for(unsigned int i=0;i<result.size();i++){

        //            if(isOccupiedIndexXY(result[i])){
        //                out.position.x= convertToXFromIndex(result[i].first);
        //                out.position.y= convertToYFromIndex(result[i].second);
        //                return out;
        //            }
        //        }
        //        out=end_;
        //        out.position.z=-1;
        return out;
    }




    Point computeAreaField(double X,double Y,double Theta,double distance){
        //        UNUSED(Theta);

        //http://books.google.pt/books?id=C-ekP4m2y7gC&pg=PA85&lpg=PA85&dq=repulsive+force+mobile&source=bl&ots=dFfbAxfrP8&sig=-bcrF17Xrkcu9vft7CKlpIcSSqE&hl=pt-PT&sa=X&ei=v0KCUv3LHKOI7Abf_oDACw&ved=0CGgQ6AEwCDgK#v=onepage&q=repulsive%20force%20mobile&f=false


        unsigned int k=convertToXindex(X);
        unsigned int n=convertToYindex(Y);

        int indexX=round(distance/cellsizeX);
        int indexY=round(distance/cellsizeY);

        Point out; // set point label to -1.

        for(unsigned int l=k-indexX;l<k+indexX;l++){
            for(unsigned int g=n-indexY;g<n+indexY;g++){

                if(isOccupiedIndexXY(l,g)){

                    double X1=convertToXFromIndex(l);
                    double Y1=convertToXFromIndex(g);


                    double d=sqrt((X1-X)*(X1-X)+(Y1-Y)*(Y1-Y));
                    out.x=out.x+ (1000.0/(d*d))*((X1-X)/d);
                    out.y=out.y+ (1000.0/(d*d))*((Y1-Y)/d);
                }
            }
        }

        return out;

    }

  
    void mapLine(Point a,Point b,T value){

        setLine(convertToXindex(a.x),convertToYindex(a.y),convertToXindex(b.x),convertToYindex(b.y),value);

        //        int startX=convertToXindex(a.x);
        //        int startY=convertToYindex(a.y);
        //        int endX=convertToXindex(b.x);
        //        int endY=convertToYindex(b.y);

        //        std::vector<std::pair<int,int> > da;
        //        LineAlgorithms::getSoftBresenham(startX,startY,endX,endY,da);

        //        int si=da.size();
        //        for( int h=0;h<si-1;h++){
        //            setOccupied(da[h].first,da[h].second);
        //        }
    }


    bool  doesTransition(Point a,Point b,T value){
        auto o= transitions(convertToXindex(a.x),convertToYindex(a.y),convertToXindex(b.x),convertToYindex(b.y),value);

        if(o.first!=0 && o.second!=0 ){
            return true;
        }
        return false;
    }
    void mapLine(Point a,Point b){

        setOccupiedLine(convertToXindex(a.x),convertToYindex(a.y),convertToXindex(b.x),convertToYindex(b.y));

        //        int startX=convertToXindex(a.x);
        //        int startY=convertToYindex(a.y);
        //        int endX=convertToXindex(b.x);
        //        int endY=convertToYindex(b.y);

        //        std::vector<std::pair<int,int> > da;
        //        LineAlgorithms::getSoftBresenham(startX,startY,endX,endY,da);

        //        int si=da.size();
        //        for( int h=0;h<si-1;h++){
        //            setOccupied(da[h].first,da[h].second);
        //        }
    }

    void mapLaserLikePolygon(Transform *tf, sensor_msgs::LaserScan *scan){


        //Transform laserworldtf=TransformServer::instance()->get(id)->transformationFromBase();

        Transform laserworldtf=*tf;
        Point base=laserworldtf*Point(0,0,0);
        double roll,pitch,yaw;

        laserworldtf.getRPY(roll,pitch,yaw);

        int startX=convertToXindex(base.x);
        int startY=convertToYindex(base.y);
        int endX,endY;

        double range=scan->angle_min;
        unsigned int i=0;
        //std::vector<std::pair<int,int> > da;
        std::vector<std::pair<int,int> > occ;

        while(range<scan->angle_max+scan->angle_increment){

            Point pt=Point(base.x+scan->ranges[i]*cos(yaw+range),base.y+scan->ranges[i]*sin(yaw+range),0);

            endX=convertToXindex(pt.x);
            endY=convertToYindex(pt.y);
            setFreeLine(startX,startY,endX,endY);
            //            da.clear();
            //            LineAlgorithms::getSoftBresenham(startX,startY,convertToXindex(pt.x),convertToYindex(pt.y),da);

            //            int si=da.size();
            //            for( int h=0;h<si-1;h++){
            //                setFree(da[h].first,da[h].second);
            //            }

            //            if(si>0 && scan->ranges[i]!=scan->range_max ){
            //                occ.push_back(da[si-1]);
            //            }

            if(scan->ranges[i]<scan->range_max ){
                occ.emplace_back(endX,endY);
            }

            range=range+scan->angle_increment;
            i=i+1;

        }

        for(i=0;i<occ.size();i++){
            setOccupied(occ[i].first,occ[i].second);
        }

    }

    void mapLaserLikePolygon(Transform &tf, sensor_msgs::Range *scan){



        Point pt=Point(scan->range,0,0);
        pt=tf*pt;
        Point base=tf*Point(0,0,0);
        int startX=convertToXindex(base.x);
        int startY=convertToYindex(base.y);
        int endX=convertToXindex(pt.x);
        int endY=convertToYindex(pt.y);

        //        std::vector<std::pair<int,int> > da;
        //        LineAlgorithms::getSoftBresenham(startX,startY,endX,endY,da);


        //        int si=da.size();
        //        for( int h=0;h<si-1;h++){
        //            setFree(da[h].first,da[h].second);
        //        }

        //        if(si>0 && scan->range!=scan->max_range){

        //            setOccupied(da[(si-1)].first,da[(si-1)].second);
        //        }


        setFreeLine(startX,startY,endX,endY);
        if(scan->range<=scan->max_range){
            setOccupied(endX,endY);
        }



    }

    void mapLaserLikePolygonOccupied(Transform &tf, sensor_msgs::Range *scan){



        Point pt=Point(scan->range,0,0);
        pt=tf*pt;

        setOccupied(convertToXindex(pt.x),convertToYindex(pt.y));

    }

    void mapLaserLikePolygonOccupied(Transform &tf, sensor_msgs::PointCloud *scan){



        for(unsigned int i=0;i<scan->points.size();i++){
            Point pt=Point(scan->points[i].x,scan->points[i].y,scan->points[i].z);
            pt=tf*pt;

            setOccupied(convertToXindex(pt.x),convertToYindex(pt.y));
        }



    }

    void mapLaserLikePolygonOccupiedUltraPixel(Transform &tf, sensor_msgs::PointCloud *scan){


        for(unsigned int i=0;i<scan->points.size();i++){
            Point pt=Point(scan->points[i].x,scan->points[i].y,scan->points[i].z);
            pt=tf*pt;

            setOccupiedUltraPixel(convertToXindex(pt.x),convertToYindex(pt.y));
        }



    }



    void MapOnlyOcc( Transform &tf,const sensor_msgs::PointCloud &scan, Map<T> *buffer){


        auto tfpoint=tf.point();
        unsigned int startX=convertToXindex(tfpoint.x);
        unsigned int startY=convertToYindex(tfpoint.y);
        unsigned int endX,endY;


        Point point;
        xmin=startX-1;
        ymin=startY-1;
        xmax=startX+1;
        ymax=startY+1;


        std::vector<std::pair<int,int>> occ;
        occ.reserve(scan.points.size());

        for(unsigned int i=0;i<scan.points.size();i++){

            point = tf * Point(scan.points[i].x, scan.points[i].y, scan.points[i].z);

            endX=convertToXindex(point.x);
            endY=convertToYindex(point.y);


            if(endX<xmin){
                xmin=endX-1;
            }

            if(endX>xmax){
                xmax=endX+1;
            }

            if(endY<ymin){
                ymin=endY-1;
            }

            if(endY>ymax){
                ymax=endY+1;
            }


            setLineIndexXYBuffer(startX,startY,endX,endY, buffer);
            occ.emplace_back(endX,endY);


        }



        if(xmin<0){
            xmin=0;

        }
        if(xmax>=columns){
            xmax=columns-1;
        }

        if(ymin<0){
            ymin=0;
        }
        if(ymax>=rows){
            ymax=rows-1;

        }

        for(unsigned int i=xmin;i<=xmax;i++){
            for(unsigned int j=ymin;j<=ymax;j++){

                if(buffer->getIndexXY(i,j)==111){
                    buffer->setIndexXY(i,j,0);
                    takeIndexXY(i,j,20);
                }

            }
        }


        for(unsigned int i=0;i<occ.size();i++){
            setOccupied(occ[i].first,occ[i].second);
        }

        //        auto tfpoint=tf.point();
        //        unsigned int startX=convertToXindex(tfpoint.x);
        //        unsigned int startY=convertToYindex(tfpoint.y);
        //        unsigned int endX,endY;


        //        Point point;
        //        xmin=startX-1;
        //        ymin=startY-1;
        //        xmax=startX+1;
        //        ymax=startY+1;

        //        for(unsigned int i=0;i<scan.points.size();i++){



        //            point = tf * Point(scan.points[i].x, scan.points[i].y, scan.points[i].z);

        //            endX=convertToXindex(point.x);
        //            endY=convertToYindex(point.y);


        //            if(endX<xmin){
        //                xmin=endX-1;
        //            }

        //            if(endX>xmax){
        //                xmax=endX+1;
        //            }

        //            if(endY<ymin){
        //                ymin=endY-1;
        //            }

        //            if(endY>ymax){
        //                ymax=endY+1;
        //            }

        //            setOccupied(endX,endY);
        //        }

    }



    void LogMapLaserLikePolygon3( Transform &tf,const sensor_msgs::PointCloud &scan){



        auto tfpoint=tf.point();
        unsigned int startX=convertToXindex(tfpoint.x);
        unsigned int startY=convertToYindex(tfpoint.y);
        unsigned int endX,endY;


        Point point;
        xmin=startX-1;
        ymin=startY-1;
        xmax=startX+1;
        ymax=startY+1;


        std::vector<std::pair<int,int>> occ;
        occ.reserve(scan.points.size());

        for(unsigned int i=0;i<scan.points.size();i++){



            point = tf * Point(scan.points[i].x, scan.points[i].y, scan.points[i].z);

            endX=convertToXindex(point.x);
            endY=convertToYindex(point.y);


            if(endX<xmin){
                xmin=endX-1;
            }

            if(endX>xmax){
                xmax=endX+1;
            }

            if(endY<ymin){
                ymin=endY-1;
            }

            if(endY>ymax){
                ymax=endY+1;
            }


            setFreeLine(startX,startY,endX,endY);
            occ.emplace_back(endX,endY);


        }
        for(unsigned int i=0;i<occ.size();i++){
            setOccupied(occ[i].first,occ[i].second);

        }




    }




    void mapLaserLikePolygonOccupiedFull(Transform &tf, sensor_msgs::PointCloud *scan){


        std::vector<std::pair<int,int> > out;


        Point base=tf.point();
        int x=convertToXindex(base.x);
        int y=convertToYindex(base.y);
        int xl;
        int yl;
        for(unsigned int i=0;i<scan->points.size();i++){
            Point pt=Point(scan->points[i].x,scan->points[i].y,scan->points[i].z);
            pt=tf*pt;
            xl=convertToXindex(pt.x);
            yl=convertToYindex(pt.y);

            setFreeLine(x,y,xl,yl);
            out.emplace_back(xl,yl);
        }


        for(unsigned int i=0;i<out.size();i++){

            setOccupied(out[i].first,out[i].second);

        }



    }

    void mapLaserLikePolygonOccupiedUltraPixelFull(Transform &tf, sensor_msgs::PointCloud *scan){
        std::vector<std::pair<int,int> > out;


        Point base=tf.point();
        int x=convertToXindex(base.x);
        int y=convertToYindex(base.y);
        int xl;
        int yl;
        for(unsigned int i=0;i<scan->points.size();i++){
            Point pt=Point(scan->points[i].x,scan->points[i].y,scan->points[i].z);
            pt=tf*pt;
            xl=convertToXindex(pt.x);
            yl=convertToYindex(pt.y);

            setFreeLine(x,y,xl,yl);
            out.emplace_back(xl,yl);
        }

        for(unsigned int i=0;i<out.size();i++){

            setOccupiedUltraPixel(out[i].first,out[i].second);

        }




    }
    //    void mapLaserLikePolygonOccupied(Transform &tf, sensor_msgs::PointCloud *scan){


    //        for(unsigned int i=0;i<scan->points.size();i++){
    //            Point pt=Point(scan->points[i].x,scan->points[i].y,scan->points[i].z);
    //            pt=tf*pt;

    //            setOccupied(convertToXindex(pt.x),convertToYindex(pt.y));
    //        }



    //    }

    void mapLaserLikePolygon(Point *base, std::vector<FastPoint<unsigned int> *> *polygon){

        int startX=convertToXindex(base->x);
        int startY=convertToYindex(base->y);

        unsigned int size=polygon->size();
        for(unsigned int i=0;i<size;i++){

            setFreeLine(startX,startY,polygon->operator [](i)->x,polygon->operator [](i)->y);
            setOccupied(polygon->operator [](i)->x,polygon->operator [](i)->y);

        }


        //        std::vector<std::pair<int,int> > da;
        //        unsigned int size=polygon->size();
        //        for(unsigned int i=0;i<size;i++){

        //            LineAlgorithms::getSoftBresenham(startX,startY,polygon->operator [](i)->x,polygon->operator [](i)->y,da);

        //            int si=da.size();
        //            for( int h=0;h<si-1;h++){
        //                setFree(da[h].first,da[h].second);
        //            }

        //        }


        //        for(unsigned int i=0;i<size;i++){
        //            if(polygon->operator [](i)->getZ()>0){
        //                setOccupied(polygon->operator [](i)->x,polygon->operator [](i)->y);
        //            }
        //        }



    }




    //    void mapLaserLikePolygon( std::vector<FastPoint<double> > &polygon,T occ,T fre){

    //        int startX=convertToXindex(0);
    //        int startY=convertToYindex(0);

    //        unsigned int size=polygon.size();
    //        for(unsigned int i=0;i<size;i++){

    //            setLine(startX,startY,convertToXindex(polygon.operator [](i).x),convertToYindex(polygon.operator [](i).y),fre);
    //            setXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
    //        }




    //    }


    void mapLaserLikePolygon( std::vector<FastPoint<double> > &polygon,T occ,T fre){


        unsigned int size=polygon.size();
        for(unsigned int i=0;i<size;i++){

            setLineXY(0,0,convertToXindex(polygon.operator [](i).x),convertToYindex(polygon.operator [](i).y),fre);
            setXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
        }


    }


    void mapLaserLikePolygon( Transform &tf,std::vector<FastPoint<double>> &polygon2,T occ,T fre){



        auto polygon = tf*polygon2;

        unsigned int size=polygon.size();
        for(unsigned int i=0;i<size;i++){
            setLineXY(tf.x(),tf.y(), (polygon.operator [](i).x), (polygon.operator [](i).y),fre);
            setXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
        }
    }

    template <typename XYZ>
    void mapLaserLikePolygonMAX( Transform tf,std::vector<XYZ>  polygon2,T occ,T fre,double zth){


        if(polygon2.size()==0){return;}
        auto polygon = tf*polygon2;

        //        std::cout<<tf.point().toString()<<std::endl;

        unsigned int size=polygon.size();
        for(unsigned int i=0;i<size;i++){
            //std::cout<<"DATA "<<__LINE__<<std::endl;

//            if( (polygon.operator [](i).z)>zth){
//                setMaxLineXY(tf.x(),tf.y(), (polygon.operator [](i).x), (polygon.operator [](i).y),fre);

                maxXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
//            }

        }
    }

    template <typename XYZ>
        void mapLaserLikePolygon( std::vector<XYZ > &polygon,T occ,T fre){


           unsigned int size=polygon.size();
           for(unsigned int i=0;i<size;i++){
               //std::cout<<"DATA "<<__LINE__<<std::endl;

               setLineXY(0,0,convertToXindex(polygon.operator [](i).x),convertToYindex(polygon.operator [](i).y),fre);
               //std::cout<<"DATA "<<__LINE__<<std::endl;

               setXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
           }


       }

    void mapLaserLikePolygonMAX( Transform &tf,  std::vector<FastPoint<double>>  polygon2,T occ,T fre){



        auto polygon = tf*polygon2;

        unsigned int size=polygon.size();
        for(unsigned int i=0;i<size;i++){
            setMaxLineXY(tf.x(),tf.y(), (polygon.operator [](i).x), (polygon.operator [](i).y),fre);
            maxXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
        }
    }


    void mapLaserLikePolygon( Point base,std::vector<FastPoint<double>> &polygon ,T free,T occ){

        unsigned int size=polygon.size();
        for(unsigned int i=0;i<size;i++){
            setLineXY(base.x,base.y, (polygon.operator [](i).x), (polygon.operator [](i).y),free);
            setXY(polygon.operator [](i).x,polygon.operator [](i).y,occ);
        }
    }



    void mapLaserLikePolygon( Transform &tf,std::vector<FastPoint<double>> &polygon2){

        int startX=convertToXindex(0);
        int startY=convertToYindex(0);

        auto polygon = tf*polygon2;

        unsigned int size=polygon.size();
        for(unsigned int i=0;i<size;i++){
            setFreeLineXY(startX,startY,convertToXindex(polygon.operator [](i).x),convertToYindex(polygon.operator [](i).y));
            setOccupiedXY(polygon.operator [](i).x,polygon.operator [](i).y);
        }
    }




    std::pair<double,double> objectSegmentList(geometry_msgs::Pose base,geometry_msgs::Pose &poses,double window ){

        std::pair<double,double> out;
        std::pair<int,int> outcnt;

        //left
        out.first=9999;
        //right
        out.second=9999;
        outcnt.first=0;
        outcnt.second=0;

        int x=convertToXindex(base.position.x);
        int y=convertToYindex(base.position.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;


        double Yaw=PoseTools::getYaw(base);

        double yleft=base.position.y+10.0*sin(Yaw+M_PI/2.0);
        double yright=base.position.y+10.0*sin(Yaw-M_PI/2.0);


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    double rx=convertToXFromIndex(i);
                    double ry=convertToYFromIndex(f);


                    bool isleft= sqrt((rx-poses.position.x)*(rx-poses.position.x)+(ry-yleft)*(ry-yleft))<sqrt((rx-poses.position.x)*(rx-poses.position.x)+(ry-yright)*(ry-yright));
                    if(isleft){

                        geometry_msgs::Pose p;
                        p.position.x=rx;
                        p.position.y=ry;

                        out.first=std::min(out.first,PoseTools::distance2D(p,poses));

                    }else{


                        geometry_msgs::Pose p;
                        p.position.x=rx;
                        p.position.y=ry;

                        out.second=std::min(out.second,PoseTools::distance2D(p,poses));


                    }


                }

            }
        }
        return out;

    }
















 






















































    std::pair<double,double> objectSegmentList(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &poses,double window ){

        std::pair<double,double> out;
        std::pair<int,int> outcnt;

        //left
        out.first=9999;
        //right
        out.second=9999;
        outcnt.first=0;
        outcnt.second=0;

        int x=convertToXindex(base.position.x);
        int y=convertToYindex(base.position.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;


        double Yaw=PoseTools::getYaw(base);

        double yleft=base.position.y+10.0*sin(Yaw+M_PI/2.0);
        double yright=base.position.y+10.0*sin(Yaw-M_PI/2.0);


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    double rx=convertToXFromIndex(i);
                    double ry=convertToYFromIndex(f);

                    bool isleft= sqrt((rx-poses[poses.size()-1].position.x)*(rx-poses[poses.size()-1].position.x)+(ry-yleft)*(ry-yleft))<sqrt((rx-poses[poses.size()-1].position.x)*(rx-poses[poses.size()-1].position.x)+(ry-yright)*(ry-yright));
                    if(isleft){
                        geometry_msgs::Pose p;
                        p.position.x=rx;
                        p.position.y=ry;
                        out.first=std::min(out.first,PoseTools::min3p(p,poses));
                    }else{
                        geometry_msgs::Pose p;
                        p.position.x=rx;
                        p.position.y=ry;
                        out.second=std::min(out.second,PoseTools::min3p(p,poses));
                    }
                }

            }
        }

        return out;
    }
    double minobjectdistance(Point base,double window ){


        double outfirst=9999;

        int x=convertToXindex(base.x);
        int y=convertToYindex(base.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;
        //        geometry_msgs::Pose p;


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    //                    double rx=convertToXFromIndex(i);
                    //                    double ry=convertToYFromIndex(f);

                    //                    p.position.x=rx;
                    //                    p.position.y=ry;

                    outfirst=std::min(outfirst, (double)((i-x)*(i-x)+(f-y)*(f-y)));
                    //                    outfirst=std::min(outfirst,PoseTools::distance(p,base));

                }

            }
        }

        outfirst=sqrt(outfirst)*cellsizeX;



        return outfirst;

    }
    double minobjectdistance(geometry_msgs::Pose base,double window ){


        double outfirst=9999;

        int x=convertToXindex(base.position.x);
        int y=convertToYindex(base.position.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;
        //        geometry_msgs::Pose p;


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    //                    double rx=convertToXFromIndex(i);
                    //                    double ry=convertToYFromIndex(f);

                    //                    p.position.x=rx;
                    //                    p.position.y=ry;

                    outfirst=std::min(outfirst,sqrt((i-x)*(i-x)+(f-y)*(f-y)));
                    //                    outfirst=std::min(outfirst,PoseTools::distance(p,base));

                }

            }
        }

        outfirst=outfirst*cellsizeX;



        return outfirst;

    }
    template <typename Tg>
    double nearestObstacle(Tg point,double dist){

        //        double outfirst=0.01;


        int x=convertToXindex(point.x);
        int y=convertToYindex(point.y);

        int windowx=dist/cellsizeX;
        int windowy=dist/cellsizeY;


        for(int i=0;i<windowx;i++){
            for(int f=0;f<windowy;f++){
                if(isOccupiedIndexXY(x+i,y+f)){
                    return std::max((double)f*cellsizeY,(double)i*cellsizeX);
                }
                if(isOccupiedIndexXY(x-i,y-f)){
                    return std::max((double)f*cellsizeY,(double)i*cellsizeX);
                }
            }
        }


        return dist;


    }
    double nearestObstacle(double xx,double yy,double dist){

        //        double outfirst=0.01;


        int x=convertToXindex(xx);
        int y=convertToYindex(yy);

        int windowx=dist/cellsizeX;
        int windowy=dist/cellsizeY;
        double dx=1.5*windowx*windowx;


        for(int i=std::max(x-windowx,0);i<=std::min(x+windowx,(int)(columns-1));i++){
            for(int f=std::max(y-windowy,0);f<=std::min(y+windowy,(int)(rows-1));f++){
                if(isOccupiedIndexXY(i,f)){
                    double d=(i-x)*(i-x)+(f-y)*(f-y);
                    if(d<dx){
                        dx=d;
                    }
            }}
        }


        return sqrt(dx)*cellsizeX;


    }
    double nearestObstacle(double xx,double yy,double dist,T cut){

        //        double outfirst=0.01;


        int x=convertToXindex(xx);
        int y=convertToYindex(yy);

        int windowx=dist/cellsizeX;
        int windowy=dist/cellsizeY;
        double dx=1.5*windowx*windowx;


        for(int i=std::max(x-windowx,0);i<=std::min(x+windowx,(int)(columns-1));i++){
            for(int f=std::max(y-windowy,0);f<=std::min(y+windowy,(int)(rows-1));f++){
                if(getIndexXY2(i,f)>=cut){
                    double d=(i-x)*(i-x)+(f-y)*(f-y);
        if(d<dx){
                        dx=d;
                    }
            }}
        }


        return sqrt(dx)*cellsizeX;


    }

    std::pair<double,double> nearestObstacleCenterPoint(double dist){

        //        double outfirst=0.01;

        std::pair<double,double> out;
        out.first=999;
        out.second=999;
        double outfirst=9999;


        int x=columns/2.0;
        int y=rows/2.0;

        int windowx=dist/cellsizeX;
        int windowy=dist/cellsizeY;



        for(int i=x-windowx;i<=x+windowx;i++){
            for(int f=y-windowy;f<=y+windowy;f++){
                if(isOccupiedIndexXY(i,f)){
                    double d=(i-x)*(i-x)+(f-y)*(f-y);
                    if(d<outfirst){
                        outfirst= d;
                        out.first=fabs(i-x)*cellsizeX;
                        out.second=fabs(f-y)*cellsizeY;

                    }
                }

            }
        }





        return out;

    }

    double nearestObstacleCenter(double dist){

        //        double outfirst=0.01;

        double outfirst=9999;

        int x=columns/2.0;
        int y=rows/2.0;

        int windowx=dist/cellsizeX;
        int windowy=dist/cellsizeY;



        for(int i=x-windowx;i<=x+windowx;i++){
            for(int f=y-windowy;f<=y+windowy;f++){
                if(isOccupiedIndexXY(i,f)){
                    outfirst=std::min(outfirst,sqrt((i-x)*(i-x)+(f-y)*(f-y)));
                }

            }
        }


        outfirst=outfirst*cellsizeX;



        return outfirst;

    }






    double meanobjectdistance(geometry_msgs::Pose base,double window ){


        double outfirst=9999;
        bool start__=true;

        int x=convertToXindex(base.position.x);
        int y=convertToYindex(base.position.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    double rx=convertToXFromIndex(i);
                    double ry=convertToYFromIndex(f);

                    geometry_msgs::Pose p;
                    p.position.x=rx;
                    p.position.y=ry;

                    if(start__==true){
                        outfirst=PoseTools::distance(p,base);
                        start__=false;
                    }else{
                        outfirst=(outfirst+PoseTools::distance(p,base))/2.0;
                    }



                }

            }
        }





        return outfirst;

    }

    double minobjectdistance(geometry_msgs::Pose base,geometry_msgs::Pose &poses,double window ){


        double outfirst=9999;


        int x=convertToXindex(base.position.x);
        int y=convertToYindex(base.position.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    double rx=convertToXFromIndex(i);
                    double ry=convertToYFromIndex(f);


                    geometry_msgs::Pose p;
                    p.position.x=rx;
                    p.position.y=ry;

                    outfirst=std::min(outfirst,PoseTools::distance2D(p,poses));

                }

            }
        }

        return outfirst;

    }

    double minobjectdistance(geometry_msgs::Pose base,std::vector<geometry_msgs::Pose> &poses,double window ){


        double outfirst=9999;


        int x=convertToXindex(base.position.x);
        int y=convertToYindex(base.position.y);

        int windowx=window/cellsizeX;
        int windowy=window/cellsizeY;


        for(int i=x-windowx;i<x+windowx;i++){
            for(int f=y-windowy;f<y+windowy;f++){

                if(isOccupiedIndexXY(i,f)){

                    double rx=convertToXFromIndex(i);
                    double ry=convertToYFromIndex(f);



                    geometry_msgs::Pose p;
                    p.position.x=rx;
                    p.position.y=ry;

                    outfirst=std::min(outfirst,PoseTools::min3p2D(p,poses));




                }

            }
        }





        return outfirst;

    }
 

    void translate(geometry_msgs::Pose &pose){

        if(start.x==pose.position.x && start.y==pose.position.y){
            return;
        }


        Map<T> old=*this;

        start.x=pose.position.x;
        start.y=pose.position.y;


        zero();

        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){

                double x=old.convertToXFromIndex(i);
                double y=old.convertToYFromIndex(f);
                setXY(x,y,old.getIndexXY(x,y));
            }
        }

    }


    void translate(double dx,double dy){

        int mdx=dx>=0?std::floor(dx/cellsizeX):std::ceil(dx/cellsizeX);
        int mdy=dy>=0?std::floor(dy/cellsizeY):std::ceil(dy/cellsizeY);

        if(mdx==0 && mdy==0){
            return ;
        }

        T *matrix_new;
        T **indexer_new;


        matrix_new =new T [rows*columns];
        indexer_new=new T*[columns];

        for(unsigned int i=0;i<columns;i++){
            indexer_new[i]=&matrix_new[i*rows];
        }
        memset(matrix_new,base,(count)*sizeof(T));


        unsigned int leftx=std::max(0,mdx);
        unsigned int rightx=std::min(columns,columns+mdx);

        unsigned int lefty=std::max(0,mdy);
        unsigned int righty=std::min(rows,rows+mdy);


        //        std::cout<<mdx<<"  "<<mdy<<std::endl;
        //        std::cout<<leftx<<"  "<<rightx<<"  "<<lefty<<"  "<<righty<<std::endl;

        for(unsigned int i=leftx;i<rightx;i++){
            for(unsigned int j=lefty;j<righty;j++){
                indexer_new[j-mdy][i-mdx]=indexer[j][i];
            }
        }

        for(unsigned int i=0;i<columns;i++){
            for(unsigned int j=0;j<rows;j++){
                indexer[j][i]=indexer_new[j][i];
            }
        }



        delete [] matrix_new;
        delete [] indexer_new;
    }


    void decay(T delta=10){
        for(unsigned int i=0;i<columns;i++){
            for(unsigned int j=0;j<rows;j++){
                indexer[j][i]=indexer[j][i]-delta;
            }
        }
    }
    void decay2(T delta=10){
        for(unsigned int i=0;i<columns;i++){
            for(unsigned int j=0;j<rows;j++){
                if(indexer[j][i]>delta){
                    indexer[j][i]=indexer[j][i]-delta;
                }
            }
        }
    }


    void rotateAndTranslate(double dx,double dy,double theta){

        translate(dx,dy);
        Transform tf;
        tf.applyRotationZ(-theta);
        rotateAndTranslate(tf);

        //        tf.applyTranslation(dx,dy,0);
        //        tf=tf.inverse();
        //        rotateAndTranslate(tf);
    }


    void rotateAndTranslate_old(double dx,double dy,double theta){
        Transform tf;
        tf.applyRotationZ(theta);
        tf.applyTranslation(dx,dy,0);
        tf=tf.inverse();
        rotateAndTranslate(tf);
    }




    void rotateAndTranslate(Transform &tf){


        T *matrix_new;
        T **indexer_new;


        matrix_new =new T [rows*columns];
        indexer_new=new T*[columns];

        for(unsigned int i=0;i<columns;i++){
            indexer_new[i]=&matrix_new[i*rows];
        }
        memset(matrix_new,0,(count)*sizeof(T));



        Point point;

        for(unsigned int i=0;i<columns;i++){
            for(unsigned int f=0;f<rows;f++){

                if(unprotected_isOccupiedIndexXY(i,f)){


                    point=getPointXY(i,f);
                    point=tf*point;

                    unsigned int k= convertToXindex(point.x);
                    unsigned int n= convertToYindex(point.y);
                    if((k>=columns)||(n>=rows)){
                    }else{
                        indexer_new[n][k]=unprotected_getIndexXY(i,f);
                    }

                }
            }
        }

        disposePointer();
        matrix=matrix_new;
        indexer=indexer_new;
    }



    inline Point getPointXY(unsigned int i,unsigned int j){

        return Point(convertToXFromIndex(i),convertToYFromIndex(j),0);

    }





};



typedef Map<unsigned char> OccupancyGridMap;

#endif // MAP_H
