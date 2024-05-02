#ifndef ANGULARHISTOGRAM_H
#define ANGULARHISTOGRAM_H

#include <vector>
#include "stringformat.h"
#include <cmath>

#ifndef M_PI
# define M_PI           3.14159265358979323846
#endif


template <typename T>
class AngularHistogramCaching{


   std::vector<T> *count;
    double step;
    int datapoints;
    unsigned int countsize;
    double bias=0;


public:

    ~AngularHistogramCaching(){
        delete [] count;
    }


    AngularHistogramCaching(double step_=(1.0472),double bias_=0.5236){  //0.39269 //0.15707963267
        step=step_;
        bias=bias_;
        datapoints=0;

        unsigned int steps=std::round(2.0*M_PI/step);
        countsize=steps;

        count=new  std::vector<T>[steps+1];

        //then use floor
        for(unsigned int i=0;i<(steps+1);i++){
            count[i]=std::vector<T>();

        }

    }


    void reset(){
        for(unsigned int i=0;i<countsize;i++){
            count[i]=0;
        }
        datapoints=0;
    }

    void fill(T val){
        for(unsigned int i=0;i<countsize;i++){
            count[i]=val;
        }
    }


    void add(double value,T val){

        datapoints++;
       unsigned int i=std::floor(std::fabs((value+M_PI+bias)/step));


        if(i<countsize){
            count[i].push_back(val);
        }else{

            // do stuff later?
        }

    }

    double getStep(){
        return step;
    }

    int getDatapoints() const
    {
        return datapoints;
    }


    std::vector<T> getData(const std::function<bool (std::vector<T>&, T&)> getBest){
        std::vector<T> out;


        for(unsigned int i=0;i<(countsize+1);i++){

            T t;
            if(getBest(count[i],t)){
                out.push_back(t);
             }
        }





        return out;
    }









};





template <typename T>
class AngularHistogram{


    //T *count;

    std::vector<T> count;
    double step;
    int datapoints;
    unsigned int countsize;
    double bias=0;


public:

    ~AngularHistogram(){
//        delete [] count;
    }


    AngularHistogram(double step_=(1.0472),double bias_=0.5236){  //0.39269 //0.15707963267
        step=step_;
        bias=bias_;
        datapoints=0;

        unsigned int steps=std::round(2.0*M_PI/step);
        countsize=steps+1;

        count=std::vector<T>(steps+1);

        //then use floor
        for(unsigned int i=0;i<(steps+1);i++){
            count[i]=(0);

        }

    }


    void reset(){
        for(unsigned int i=0;i<countsize;i++){
            count[i]=0;
        }
        datapoints=0;
    }

    void fill(T val){
        for(unsigned int i=0;i<countsize;i++){
            count[i]=val;
        }
    }

    void add(double value){

        datapoints++;
       unsigned int i=std::floor(std::fabs((value+M_PI+bias)/step));


        if(i<countsize){
            count[i]=count[i]+1;
        }else{

            // do stuff later?
        }

    }

    T get(double value){

        unsigned int i=std::floor(std::fabs((value+M_PI+bias)/step));


        if(i<countsize){
           return count[i];
        }else{
            return 1;
            // do stuff later?
        }

    }
    void add(double value,T val){

        datapoints++;
       unsigned int i=std::floor(std::fabs((value+M_PI+bias)/step));


        if(i<countsize){
            count[i]=std::min(count[i],val);
        }else{

            // do stuff later?
        }

    }

    double getStep(){
        return step;
    }

    int getDatapoints() const
    {
        return datapoints;
    }


    std::vector<double> normalizeAndBinarize(){
        std::vector<double> out;



        double maxv=(*std::max_element(count,count+countsize));
        for(unsigned int i=0;i<countsize;i++){
            auto vr=(double)count[i]/(double)maxv;
            out.push_back(vr<0.75?0:1); // why 0.75? because i said so :)
        }

        return out;
    }

    std::vector<double> binarize(){
        std::vector<double> out;

         for(unsigned int i=0;i<countsize;i++){
            if(count[i]>0){
                out.push_back(1);
            }else{
                out.push_back(0);
            }
        }
        return out;
    }

    std::vector<double> normalized(){
        std::vector<double> out;



        double maxv=(*std::max_element(count,count+countsize));
        for(unsigned int i=0;i<countsize;i++){
            out.push_back((double)count[i]/(double)maxv);
        }

        return out;
    }

    std::vector<double> middlelabels(){

        std::vector<double> out;

        for(unsigned int i=0;i<countsize;i++){
            out.push_back(-M_PI+((double)i)*step+step/2.0 -bias);
        }


        return out;

    }


    std::vector<double> labels(){
        std::vector<double> out;

        for(unsigned int i=0;i<countsize;i++){
            out.push_back(-M_PI+((double)i)*step-bias);
        }

        return out;
    }



    std::string toString(){

        std::stringstream str;
        //|       ||       ||       ||       |
        //|   0   ||       ||       ||       |
        //|  xxx  ||       ||       ||       |

        for(unsigned int i=0;i<countsize;i++){

            str<<"|"<<$("{:=9}").format(" ") <<"|";

        }

        str<<std::endl;

        for(unsigned int i=0;i<countsize;i++){
            str<<"|"<<$("{:=9}").format(count[i]) <<"|";
        }

        str<<std::endl;


        for(unsigned int i=0;i<countsize;i++){
            str<<"|"<<$("{:=9}").format(-M_PI+((double)i)*step+step/2.0 -bias) <<"|";
        }

        str<<std::endl;

        return str.str();

    }




};

#endif // ANGULARHISTOGRAM_H
