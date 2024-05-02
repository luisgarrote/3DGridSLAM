#ifndef GMAP3D_H
#define GMAP3D_H



#ifndef Point_h
#define Point_h
class Point{
public:
    double x,y,z;
    Point(){
        x=0;
        y=0;
        z=0;
    }
};

#endif

#include <vector>
#include <functional>
#include <map>
#include <iostream>
#include <iomanip>

#include "YAML.h"
#include "PoseTools.h"
#include "ransac.h"


class Voxel3D{

public:

    double x=0;
    double y=0;
    double z=0;

    unsigned char count=0;

    Voxel3D(){

    }

    bool isValid(){
        return count>0;
    }

    std::string toString(){
        return "["+$("{}").format(x)+
                ","+$("{}").format(y)+
                ","+$("{}").format(z)+"];";
    }


    void toOpenGL(float xsize,float ysize,float *data,unsigned int index1,unsigned int *indexes,unsigned int index2,unsigned int ct,unsigned int *indexes1,unsigned int index3, bool line,double cellsize=0.05){

        float x_=xsize/2.0;
        float y_=ysize/2.0;
        float z_=cellsize/2.0;
        float Z=z;



        data[index1+0]=x-x_; //a
        data[index1+1]=y-y_;
        data[index1+2]=Z+z_;


        data[index1+3]=x+x_; //d
        data[index1+4]=y-y_;
        data[index1+5]=Z+z_;

        data[index1+6]=x+x_; //c
        data[index1+7]=y+y_;
        data[index1+8]=Z+z_;

        data[index1+9]=x-x_; //b
        data[index1+10]=y+y_;
        data[index1+11]=Z+z_;
        ///////////////

        data[index1+12]=x-x_;  //e
        data[index1+13]=y-y_;
        data[index1+14]=Z-z_;


        data[index1+15]=x+x_;  //f
        data[index1+16]=y-y_;
        data[index1+17]=Z-z_;

        data[index1+18]=x+x_;  //g
        data[index1+19]=y+y_;
        data[index1+20]=Z-z_;

        data[index1+21]=x-x_;  //h
        data[index1+22]=y+y_;
        data[index1+23]=Z-z_;


        //      3   2
        //    0 7 1 6
        //    4   5


        indexes[index2+0]=ct+0;
        indexes[index2+1]=ct+4;
        indexes[index2+2]=ct+1;

        indexes[index2+3]=ct+1;
        indexes[index2+4]=ct+4;
        indexes[index2+5]=ct+5;

        indexes[index2+6]=ct+1;
        indexes[index2+7]=ct+5;
        indexes[index2+8]=ct+6;

        indexes[index2+9]=ct+1;
        indexes[index2+10]=ct+2;
        indexes[index2+11]=ct+6;


        //      3   2
        //    0 7 1 6
        //    4   5

        indexes[index2+12]=ct+2;
        indexes[index2+13]=ct+7;
        indexes[index2+14]=ct+3;

        indexes[index2+15]=ct+2;
        indexes[index2+16]=ct+7;
        indexes[index2+17]=ct+6;

        indexes[index2+18]=ct+3;
        indexes[index2+19]=ct+0;
        indexes[index2+20]=ct+4;

        indexes[index2+21]=ct+3;
        indexes[index2+22]=ct+7;
        indexes[index2+23]=ct+4;





        indexes[index2+24]=ct+0;
        indexes[index2+25]=ct+1;
        indexes[index2+26]=ct+2;

        indexes[index2+27]=ct+0;
        indexes[index2+28]=ct+3;
        indexes[index2+29]=ct+2;

        indexes[index2+30]=ct+7;
        indexes[index2+31]=ct+4;
        indexes[index2+32]=ct+5;

        indexes[index2+33]=ct+7;
        indexes[index2+34]=ct+6;
        indexes[index2+35]=ct+5;




        if(line){

            indexes1[index3+0]=ct+0;
            indexes1[index3+1]=ct+1;
            indexes1[index3+2]=ct+1;
            indexes1[index3+3]=ct+2;
            indexes1[index3+4]=ct+2;
            indexes1[index3+5]=ct+3;
            indexes1[index3+6]=ct+3;
            indexes1[index3+7]=ct+0;


            //      3   2
            //    0 7 1 6
            //    4   5
            indexes1[index3+8]=ct+4;
            indexes1[index3+9]=ct+5;
            indexes1[index3+10]=ct+5;
            indexes1[index3+11]=ct+6;
            indexes1[index3+12]=ct+6;
            indexes1[index3+13]=ct+7;
            indexes1[index3+14]=ct+7;
            indexes1[index3+15]=ct+4;

            indexes1[index3+16]=ct+0;
            indexes1[index3+17]=ct+4;
            indexes1[index3+18]=ct+1;
            indexes1[index3+19]=ct+5;
            indexes1[index3+20]=ct+2;
            indexes1[index3+21]=ct+6;
            indexes1[index3+22]=ct+3;
            indexes1[index3+23]=ct+7;
        }




    }


};


template<class T> //or typename??
class Grid3D{


private:


    T *matrix;

    RANSACModelPlane<std::vector<Point>,Plane<Point>,Point> model;

    Point start;

    void dispose(){
        delete [] matrix;
        rows=0;
        columns=0;
        height=0;
    }

public:


    bool wasupdated=false;

    void setUpdated(bool f){
        wasupdated=f;
    }

    bool updated(){
        return wasupdated;
    }

    unsigned int rows;
    unsigned int columns;
    unsigned int height;

    double cellsizeX;
    double cellsizeY;
    double cellsizeZ;
    std::pair<unsigned int,unsigned int> limx;
    std::pair<unsigned int,unsigned int> limy;
    std::pair<unsigned int,unsigned int> limz;

    Grid3D(unsigned int rows_, unsigned int columns_, unsigned int height_){

        rows=rows_;
        columns=columns_;
        height=height_;
        matrix =new T [rows_*columns_*height_];
        all(0);
        limx.first=columns_-1;
        limy.first=rows_-1;
        limz.first=height_-1;

        limx.second=0;
        limy.second=0;
        limz.second=0;
    }

    Point getStart(){
        return start;
    }


    void setStart(Point pt){
        start=pt;
    }


    void reset(){

        all(0);
        limx.first=columns-1;
        limy.first=rows-1;
        limz.first=height-1;

        limx.second=0;
        limy.second=0;
        limz.second=0;
    }


    Voxel3D nearest(Voxel3D p,double th,double win){


        //lets put default stuf
        return nearest(p.x,p.y,p.z,6,1).second;
    }




    std::pair<bool,Voxel3D> nearest(double X,double Y,double Z,int win, T ctx=1,int winz=3){ 

        std::pair<bool,Voxel3D> out;
        out.first=false;
        out.second.count=0;

        int z= convertToZindex(Z);
        int y= convertToYindex(Y);
        int x= convertToXindex(X);


        //        int xo;
        //        int yo;
        //        int zo;


        int distance=9999;
        int d;

        int ct=0;
        //        int k=z;
        winz=win;
        for(int k=(z-winz);k<=(z+winz);k++){
            for(int i=(y-win);i<=(y+win);i++){
                for(  int f=(x-win);f<=(x+win);f++){


                    int c=get (f,i,k);
                    if(c>=ctx){
                        d=(i-y)*(i-y)+(f-x)*(f-x)+(k-z)*(k-z);
                        ct++;

                        if(d<distance){
                            distance=d;
                            out.first=true;
                            out.second.count=c;


                            if(k==0){
                            out.second.z=Z;

                            }else{
                            out.second.z=convertToZFromIndex(k);

                            }
                            if(i==0){
                                                            out.second.y=Y;

                            }else{
                                                            out.second.y=convertToYFromIndex(i);

                            }
                            if(f==0){
                                                            out.second.x=X;

                            }else{
                                                            out.second.x=convertToXFromIndex(f);

                            }
                            //                            xo=f;
                            //                            yo=i;
                            //                            zo=k;

                        }
                    }
                }
            }
        }



        //        if(distance==0){
        //            bool valid=isTemplateValid(xo,yo,zo,win);
        //            if(!valid){
        //                out.first=false;
        //                out.second.count=110;

        //            }
        //        }




        return out;
    }


    //    bool isTemplateValid(int x,int y,int z,int win, T ctx=1){



    //        std::vector<Point> ptx;
    //        for(int k=(z-win);k<=(z+win);k++){
    //            for(int i=(y-win);i<=(y+win);i++){
    //                for( int f=(x-win);f<=(x+win);f++){

    //                    if(get(f,i,k)>=ctx){
    //                        ptx.push_back(Point(f,i,k));
    //                    }
    //                }
    //            }
    //        }



    //        if(ptx.size()<=3){
    //            return true;
    //        }

    //        RANSAC<std::vector<Point>,Plane<Point> ,Point>::fit(ptx,&(model),win*win*win,3,0.25,5);

    //        if(model.inliers.size()==(ptx.size()-3)){
    //            return false;
    //        }


    //        return true;
    //    }




    void reshape(unsigned int rows_, unsigned int columns_, unsigned int height_){

        dispose();

        rows=rows_;
        columns=columns_;
        height=height_;
        matrix =new T [rows_*columns_*height_];
        all(0);

    }



    void setSizes(double cellsizeX_,double cellsizeY_,double cellsizeZ_){
        cellsizeX=cellsizeX_;
        cellsizeY=cellsizeY_;
        cellsizeZ=cellsizeZ_;


    }

    void all(double v){
        //        for(unsigned int i=0;i<rows*columns*height;i++){
        //            matrix[i]=(T)v;
        //        }

        std::fill(matrix, matrix + (rows*columns*height), v);
    }


    inline double getXcellsize(){
        return cellsizeX;
    }
    inline double getYcellsize(){
        return cellsizeY;
    }
    inline double getZcellsize(){
        return cellsizeZ;
    }


    T &operator()(  int r,   int c, int h ){

        return get(r,c,h);
    }

    T &operator()(  std::pair<int,int> d2, int h )const{

        return get(d2.first,d2.second,h);
    }

    T &get(unsigned int n,unsigned int k,unsigned int j){


        if((n>=columns)||(k>=rows)||(j>=height)){
            return matrix[0];
        }else{
            return matrix[(k*rows+n)+columns*rows*j];
        }

    }

    unsigned int getIndex(unsigned int n,unsigned int k,unsigned int j){

        if((n>=columns)||(k>=rows)||(j>=height)){
            return (T)0;
        }else{
            return (k*rows+n)+columns*rows*j;
        }

    }
    void set(unsigned int n,unsigned int k,unsigned int j,T value){

        if((n>=columns)||(k>=rows) ||(j>=height)){
        }else{
            limx.first=std::min(limx.first,n);
            limy.first=std::min(limy.first,k);
            limz.first=std::min(limz.first,j);


            limx.second=std::max(limx.second,n);
            limy.second=std::max(limy.second,k);
            limz.second=std::max(limz.second,j);

            matrix[(k*rows+n)+columns*rows*j]=value;  //n*rows+k
        }
    }

    void addLineXYZ(double x1, double y1, double z1, double x2,double y2, double z2,T val){
        unsigned int z_1= convertToZindex(z1);
        unsigned int y_1= convertToYindex(y1);
        unsigned int x_1= convertToXindex(x1);

        unsigned int z_2= convertToZindex(z2);
        unsigned int y_2= convertToYindex(y2);
        unsigned int x_2= convertToXindex(x2);

        if((x_1>=columns)||(y_1>=rows) ||(z_1>=height) ||(x_2>=columns)||(y_2>=rows) ||(z_2>=height)){
        }else{
            addLine(x_1,y_1,z_1,x_2,y_2,z_2,val);
        }
    }

    void addLine(int x1, int y1, int z1, const int x2, const int y2, const int z2,T val){

        int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
        int point[3];

        point[0] = x1;
        point[1] = y1;
        point[2] = z1;
        dx = x2 - x1;
        dy = y2 - y1;
        dz = z2 - z1;
        x_inc = (dx < 0) ? -1 : 1;
        l = abs(dx);
        y_inc = (dy < 0) ? -1 : 1;
        m = abs(dy);
        z_inc = (dz < 0) ? -1 : 1;
        n = abs(dz);
        dx2 = l << 1;
        dy2 = m << 1;
        dz2 = n << 1;

        if ((l >= m) && (l >= n)) {
            err_1 = dy2 - l;
            err_2 = dz2 - l;
            for (i = 0; i < l-1; i++) {
                add(point[0], point[1], point[2],val);
                if (err_1 > 0) {
                    point[1] += y_inc;
                    err_1 -= dx2;
                }
                if (err_2 > 0) {
                    point[2] += z_inc;
                    err_2 -= dx2;
                }
                err_1 += dy2;
                err_2 += dz2;
                point[0] += x_inc;
            }
        } else if ((m >= l) && (m >= n)) {
            err_1 = dx2 - m;
            err_2 = dz2 - m;
            for (i = 0; i < m-1; i++) {
                add(point[0], point[1], point[2],val);
                if (err_1 > 0) {
                    point[0] += x_inc;
                    err_1 -= dy2;
                }
                if (err_2 > 0) {
                    point[2] += z_inc;
                    err_2 -= dy2;
                }
                err_1 += dx2;
                err_2 += dz2;
                point[1] += y_inc;
            }
        } else {
            err_1 = dy2 - n;
            err_2 = dx2 - n;
            for (i = 0; i < n-1; i++) {
                add(point[0], point[1], point[2],val);
                if (err_1 > 0) {
                    point[1] += y_inc;
                    err_1 -= dz2;
                }
                if (err_2 > 0) {
                    point[0] += x_inc;
                    err_2 -= dz2;
                }
                err_1 += dy2;
                err_2 += dx2;
                point[2] += z_inc;
            }
        }
        add(point[0], point[1], point[2],val);
    }


    void add(unsigned int n,unsigned int k,unsigned int j,T value){

        if((n>=columns)||(k>=rows) ||(j>=height)){
        }else{
            limx.first=std::min(limx.first,n);
            limy.first=std::min(limy.first,k);
            limz.first=std::min(limz.first,j);


            limx.second=std::max(limx.second,n);
            limy.second=std::max(limy.second,k);
            limz.second=std::max(limz.second,j);


            T var=matrix[(k*rows+n)+columns*rows*j];
            if((value+(int)var)>std::numeric_limits<T>::max()){
                matrix[(k*rows+n)+columns*rows*j]=std::numeric_limits<T>::max();

            }else if((value+(int)var)<std::numeric_limits<T>::lowest()){
                matrix[(k*rows+n)+columns*rows*j]=std::numeric_limits<T>::lowest();

            }else{
                matrix[(k*rows+n)+columns*rows*j]=var+value;
            }

        }
    }
    T getXYZ(double X,double Y,double Z){


        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        unsigned int j= convertToZindex(Z);


        return get(k,n,j);


    }
    void setXYZ(double X,double Y,double Z,T val){

        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        unsigned int j= convertToZindex(Z);

        limx.first=std::min(limx.first,k);
        limy.first=std::min(limy.first,n);
        limz.first=std::min(limz.first,j);


        limx.second=std::max(limx.second,k);
        limy.second=std::max(limy.second,n);
        limz.second=std::max(limz.second,j);


        set(k,n,j,val);

    }



    void setXY(double X,double Y,T val){

        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);

        limx.first=std::min(limx.first,k);
        limy.first=std::min(limy.first,n);
        limz.first=std::min(limz.first,(unsigned int)0);


        limx.second=std::max(limx.second,k);
        limy.second=std::max(limy.second,n);
        limz.second=std::max(limz.second,this->height-1);


        for(unsigned int i=0;i<this->height;i++){
            set(k,n,i,val);
        }

    }




    void addXYZ(double X,double Y,double Z,T val){

        unsigned int k= convertToXindex(X);
        unsigned int n= convertToYindex(Y);
        unsigned int j= convertToZindex(Z);



        add(k,n,j,val);

    }

    unsigned int getXsize(){
        return columns;

    }
    unsigned int getYsize(){
        return rows;

    }
    unsigned int getZsize(){
        return height;
    }




    //World to index
    inline int convertToXindex(double X){
        return floor(((X-start.X()+cellsizeX/2.0)/cellsizeX)+floor((columns)/2.0));
    }

    inline int convertToYindex(double Y){
        return floor(((Y-start.Y()+cellsizeY/2.0)/cellsizeY)+floor((rows)/2.0));

    }
    inline int convertToZindex(double Z){
        return floor(((Z-start.Z()+cellsizeZ/2.0)/cellsizeZ)+floor((height)/2.0));

    }

    //index to World

    inline double convertToXFromIndex(int i){
        return cellsizeX*(i-floor(columns/2.0))+start.X();
    }

    inline double convertToYFromIndex(int f){
        return cellsizeY*(f-floor(rows/2.0))+start.Y();
    }

    inline double convertToZFromIndex(int f){
        return cellsizeZ*(f-floor(height/2.0))+start.Z();
    }



    /////////////////////////////////////////////////////////////////////////



    void toOutput(){
        for(unsigned int k=0;k<height;k++){
            std::cout<<"Level :"<< k<<std::endl;

            for(unsigned int i=0;i<rows;i++){
                for(unsigned int f=0;f<columns;f++){
                    std::cout<<get(i,f,k)<<" ";
                }
                std::cout<<std::endl;
            }
        }
    }



    double error(Grid3D<T> *th){

        double out=0;
        Point p;
        T a,b;
        int count=0;

        if(height!=th->height || rows!=th->rows ||columns!=th->columns ){
            return 999999;
        }
        for(unsigned int k=0;k<height;k++){
            for(unsigned int i=0;i<rows;i++){
                for(unsigned int f=0;f<columns;f++){

                    a=get(i,f,k);
                    b=th->get(i,f,k);

                    if(a==1 && b==1){
                        out+=1.0;
                        count++;
                    }else if(a==1 || b==1){
                        count++;
                    }

                }
            }
        }

        return out/(double)count;
    }





    double error(Grid3D<T> *th,int window){

        double out=0;
        Point p;
        T a,b;
        int count=0;

        if(height!=th->height || rows!=th->rows ||columns!=th->columns ){
            return 999999;
        }
        for(  int k=0;k<height;k++){
            for(  int i=0;i<rows;i++){
                for(  int f=0;f<columns;f++){


                    double Fx=0;
                    int Gx=0;

                    for( int kx=(k-window);kx<=(k-window);kx++){
                        for( int ix=(i-window);ix<=(i-window);ix++){
                            for( int fx=(f-window);fx<=(f-window);fx++){
                                a=get(ix,fx,kx);
                                b=th->get(ix,fx,kx);

                                if(a==1 && b==1){
                                    Fx=1.0;
                                    Gx=1;
                                    //out+=1.0;
                                    //count++;
                                }else if(a==1 || b==1){
                                    Gx=1;
                                }

                            }
                        }
                    }
                    out+=Fx;
                    count=count+Gx;
                }
            }
        }

        return out/(double)count;
    }




    std::vector<Voxel3D> getDataFree(T th=0 ){

        std::vector<Voxel3D> out;
        Voxel3D p;



        for(unsigned int k=limz.first;k<=limz.second;k++){

            for(unsigned int i=limy.first;i<=limy.second;i++){
                for(unsigned int f=limx.first;f<=limx.second;f++){

                    int c=get(f,i,k);
                    if(c<=th){

                        p.x=convertToXFromIndex(f);
                        p.y=convertToYFromIndex(i);
                        p.z=convertToZFromIndex(k);
                        p.count=c;

                        out.push_back(p);
                    }

                }
            }
        }


        return out;
    }





    std::vector<Voxel3D> getData2(T th=0 ){

        std::vector<Voxel3D> out;
        Voxel3D p;



        for(unsigned int k=limz.first;k<=limz.second;k++){

            for(unsigned int i=limy.first;i<=limy.second;i++){
                for(unsigned int f=limx.first;f<=limx.second;f++){

                    int c=get(f,i,k);
                    if(c>th){

                        p.x=convertToXFromIndex(f);
                        p.y=convertToYFromIndex(i);
                        p.z=convertToZFromIndex(k);
                        p.count=c;

                        out.push_back(p);
                    }

                }
            }
        }


        return out;
    }



    std::vector<Point> getData(T th=0){

        std::vector<Point> out;
        Point p;

        for(unsigned int k=0;k<height;k++){

            for(unsigned int i=0;i<rows;i++){
                for(unsigned int f=0;f<columns;f++){


                    if(get(f,i,k)>th){

                        p.x=convertToXFromIndex(f);
                        p.y=convertToYFromIndex(i);
                        p.z=convertToZFromIndex(k);

                        out.push_back(p);
                    }

                }
            }
        }


        return out;
    }

    void setLayerValue(int index,double d){

        for(unsigned int f=0;f<columns;f++){

            for(unsigned int i=0;i<rows;i++){


                set(f,i,index,d);
            }

        }
    }


    void setLayerValue(int index,std::function<double(double,double)> fn){


        for(unsigned int f=0;f<columns;f++){

            for(unsigned int i=0;i<rows;i++){
                double x=convertToXFromIndex(f);
                double y=convertToYFromIndex(i);



                set(f,i,index,fn(x,y));
            }

        }
    }


    std::map<int,double> weightedHistogramMAX(std::vector<std::pair<int,int> > &dt,std::vector<int> &zz,std::vector<double> &w){

        std::map<int,double> out;
        for(unsigned int j=0;j<zz.size();j++){
            out[zz[j]]=0;
        }

        for(unsigned int i=0;i<dt.size();i++){

            int maxID=-1;
            double value=-9999999;
            for(unsigned int j=0;j<zz.size();j++){

                double d=get(dt[i].first,dt[i].second,zz[j]);
                if(d>value){
                    value=d;
                    maxID=zz[j];
                }
            }


            if(maxID>=0){
                out[maxID]+=w[i];
            }
        }



        return out;

    }


    std::map<int,double> weightedHistogramMAX2(std::vector<std::pair<int,int> > &dt,std::vector<int> &zz,std::vector<double> &w,int idle){

        std::map<int,double> out;
        for(unsigned int j=0;j<zz.size();j++){
            out[zz[j]]=0;
        }
        out[idle]=0;
        for(unsigned int i=0;i<dt.size();i++){

            int maxID=-1;
            double value=-9999999;
            for(unsigned int j=0;j<zz.size();j++){

                double d=get(dt[i].first,dt[i].second,zz[j]);
                if(d>value){
                    value=d;
                    maxID=zz[j];
                }
            }


            if(maxID>=0 && fabs(value)>0.001){
                out[maxID]+=w[i];
            }else{
                out[idle]+=w[i]; //assume idle and let the sistem

            }
        }



        return out;

    }


    std::string toString(int h){
        std::stringstream out;
        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                out<<std::setprecision(5)<<get(i,f,h)<<" ";
            }
            out<<std::endl;
        }
        return out.str();
    }


    std::string toString(){
        std::stringstream out;

        for(unsigned int i=0;i<height;i++){
            out<<"Level :"<< i<<std::endl;
            out<<toString(i)<<std::endl;
        }


        return out.str();
    }
    std::map<int,int> histogramMAX(std::vector<std::pair<int,int> > &dt,std::vector<int> &zz){

        std::map<int,int> out;
        for(unsigned int j=0;j<zz.size();j++){
            out[zz[j]]=0;
        }

        for(unsigned int i=0;i<dt.size();i++){

            int maxID=-1;
            double value=-9999999;
            for(unsigned int j=0;j<zz.size();j++){

                double d=get(dt[i].first,dt[i].second,zz[j]);
                if(d>value){
                    value=d;
                    maxID=zz[j];
                }
            }


            if(maxID>=0){
                out[maxID]++;
            }
        }



        return out;

    }




    T  max(std::pair<int,int>  & data,int z){
        T maxd=-999999;
        if(z==-1){

            for(unsigned int k=0;k<height;k++){
                maxd=std::max(maxd,get(data.first,data.second,k));
            }

        }

        return  maxd;
    }


    T  max(std::vector<std::pair<int,int> > & data,int z){
        T maxd=-999999;
        if(z==-1){

            for(unsigned int i=0;i<data.size();i++){
                for(unsigned int k=0;k<height;k++){
                    maxd=std::max(maxd,get(data[i].first,data[i].second,k));
                }
            }

        }else{
            for(unsigned int i=0;i<data.size();i++){
                maxd=std::max(maxd,get(data[i].first,data[i].second,z));
            }
        }

        return  maxd;
    }

    std::vector<Point> getData(std::function<bool(T)> fu){

        std::vector<Point> out;
        Point p;

        for(unsigned int k=0;k<height;k++){

            for(unsigned int i=0;i<rows;i++){
                for(unsigned int f=0;f<columns;f++){


                    if(fu(get(i,f,k))){

                        p.x=convertToXFromIndex(f);
                        p.y=convertToYFromIndex(i);
                        p.z=convertToZFromIndex(k);

                        out.push_back(p);
                    }

                }
            }
        }


        return out;
    }



    template <typename Tx>
    std::vector<Tx> getData2( ){

        std::vector<Tx> out;
        Tx p;

        for(unsigned int k=0;k<height;k++){

            for(unsigned int i=0;i<rows;i++){
                for(unsigned int f=0;f<columns;f++){


                    if(get(f,i,k)>0){

                        p.x=convertToXFromIndex(f);
                        p.y=convertToYFromIndex(i);
                        p.minz=convertToZFromIndex(k)-cellsizeZ/2.0;
                        p.maxz=convertToZFromIndex(k)+cellsizeZ/2.0;

                        out.push_back(p);
                    }

                }
            }
        }


        return out;
    }


    ~Grid3D(){

        if(rows!=0 && columns!=0){
            delete [] matrix;

            rows=0;
            columns=0;
        }
    }



    bool load(std::string filename){

        YAML yaml;
        if(yaml.load(filename)){
            cellsizeX=yaml.get<double>("resolutionX",0.05);
            cellsizeY=yaml.get<double>("resolutionY",0.05);
            cellsizeZ=yaml.get<double>("resolutionZ",0.05);

            int rows1=yaml.get<double>("rows",2000);
            int columns1=yaml.get<double>("columns",2000);
            int height1=yaml.get<double>("height",160);

            if(rows1!=rows || columns1!=columns  || height1!=height ){
                return false;
            }

            start=PointTools::fromString<Point>(yaml.get("start"));



            all(0);


            for(unsigned int i=0;i<rows;i++){
                for(unsigned int f=0;f<columns;f++){

                    std::string data=yaml.get2("data"+std::to_string(i)+"_"+std::to_string(f));


                    if (typeid(T) == typeid(unsigned char) || typeid(T) == typeid( char)){
                        auto numbers=$(data).numbers<int>();


                        if(numbers.size()==height){

                            //                            std::cout<<data<<std::endl;
                            //                            std::cout<<$("{}").format(numbers)<<std::endl;

                            for(unsigned int k=0;k<height;k++){
                                set(f,i,k,numbers[k]);
                            }
                        }else {
                            //                        std::cout<<"Invalid height "<<numbers.size()<<" "<<height<<std::endl;
                        }

                    }else{

                        auto numbers=$(data).numbers<T>();


                        if(numbers.size()==height){

                            //                        std::cout<<data<<std::endl;
                            //                        std::cout<<$("{}").format(numbers)<<std::endl;

                            for(unsigned int k=0;k<height;k++){
                                set(f,i,k,numbers[k]);
                            }
                        }else {
                            //                        std::cout<<"Invalid height "<<numbers.size()<<" "<<height<<std::endl;
                        }
                    }

                }
            }
            return true;
        }

        return false;


    }

    bool save(std::string filename,T th=0){

        YAML yaml;
        yaml.addElement("resolutionX",std::to_string(cellsizeX));
        yaml.addElement("resolutionY",std::to_string(cellsizeY));
        yaml.addElement("resolutionZ",std::to_string(cellsizeZ));
        yaml.addElement("start",(PointTools::toString(start)));
        yaml.addElement("rows",std::to_string(rows));
        yaml.addElement("columns",std::to_string(columns));
        yaml.addElement("height",std::to_string(height));


        std::stringstream fd;

        for(unsigned int i=0;i<rows;i++){
            for(unsigned int f=0;f<columns;f++){
                fd.str("");
                fd.clear();
                bool hasdata=false;
                for(unsigned int k=0;k<height;k++){

                    T d=get(f,i,k);
                    if(d>th){
                        hasdata=true;
                    }
                    fd<<(int)d<<";";
                }
                if(hasdata){
                    yaml.addElement("data"+std::to_string(i)+"_"+std::to_string(f),fd.str());
                }

            }
        }

        return yaml.save(filename);

    }



};

//Local
#endif // MAP3D_H
