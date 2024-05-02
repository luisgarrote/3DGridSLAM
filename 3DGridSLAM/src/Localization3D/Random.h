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
#ifndef RANDOM_H
#define RANDOM_H

#include <cstdlib>
#include <time.h>
#include <math.h>
#include <iostream>
//#include "MersenneTwister.h"
#include <vector>
#include <random>
class Random
{
public:
    //    MersenneTwister mt;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
    Random(unsigned int min,unsigned int max): distribution(0.0,1.0){
        maxRandom=max;
        minRandom=min;
        seed=time(NULL);
        srand (seed);

        generator.seed(seed+rand());

     }


    Random(): distribution(0.0,1.0){
        maxRandom=100;
        minRandom=-100;
        seed=time(NULL);
        srand (seed);
        generator.seed(seed+rand());


    }

    //pede um valor aleatorio
    unsigned int getRandom()
    {
        return rand() % (maxRandom-minRandom) + minRandom;
    }


    double getRandomFromInterval(double max_,double min_){

        // Take 1 draw from Uniform(0,1)
        double res=((max_-min_)*( distribution(generator))+min_);
        //        std::cout<<max_<<"  "<<min_<<"  "<<res<<std::endl;
        return  res;

    }
    template<typename T>
    inline double getRandom(){
        return (T)((maxRandom-minRandom)*( distribution(generator))+minRandom);
    }

    template<typename T>
    void draw(std::vector<T> &data){
        for(unsigned int i=0;i<data.size();i++){

            data[i]=getRandom<T>();
        }
    }

    template<typename T>
    T drawFrom(std::vector<T> &data){

        if(data.size()==0){
            return 0;
        }else if(data.size()==1){
            return data[0];
        }else{
            std::uniform_int_distribution<int> distr (0,data.size()-1);

            int index=distr(generator);

            return data[index];
        }

    }
    static double generateRandomFromInterval(double max_,double min_){

        //int seed=time(NULL);
        //srand (seed);

        //std::cout<<"Seed :: "<<seed<<std::endl;

        //        std::cout<<"max "<<max_<<std::endl;
        //        std::cout<<"min "<<min_<<std::endl;

        double ra=rand();
        //        std::cout<<"rand() "<<ra<<std::endl;

        //        std::cout<<"RAND_MAX "<<RAND_MAX<<std::endl;
        //        std::cout<<"res"<<(max_-min_)*(((double)ra/(double)RAND_MAX))+min_<<std::endl;

        return (max_-min_)*(((double)ra/(double)RAND_MAX))+min_;


    }


    static void setSeed(int seedval=time(NULL)){

        srand (seedval);

    }
    unsigned int maxRandom;
    unsigned int minRandom;
private:
    unsigned int seed;


};

#endif // RANDOM_H
