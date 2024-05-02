/** 
*  @file //TODO
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

#ifndef EVENT_H
#define EVENT_H


#include "Thread.h"
#include "Clock.h"
#include <typeinfo>




 
#define _(x) std::cout << #x << std::endl;

template < typename T>
class Event;

template < typename T>

class EventData{

public:
    double h;
    T Context;
    Event<T> * EventHandle;
    void (*CallBack)(T);



    //    EventData(){

    //        h=-1;
    //        EventHandle=NULL;
    //        CallBack=NULL;
    //    }

    EventData(T C,Event<T>* e,double h_){

        h=h_;
        Context=C;
        EventHandle=e;
        CallBack=e->CallBack;
    }


 


};

 


#include <utility>
class FunctionEvent{

    std::thread t1;

public:
    template<class Function, class... Args>
    explicit FunctionEvent(Function&& f, Args&&... args){

        t1=std::thread(f,std::forward<Args>(args)...);
        t1.detach();
    }


    void stop(){
        std::cout<<"stop FunctionEvent"<<std::endl;

        if(t1.joinable())
            t1.join();
    }


    ~FunctionEvent(){

        std::cout<<"Killing FunctionEvent"<<std::endl;

        //
    }


};


template < typename T>
class Event{

    bool wasdetached=false;

    int status;
    int traded;
    double h;
    Thread Main;
    Clock clock;
    std::string debugname;
    T *dummy=NULL;

public:
    void (*CallBack)(T);

    Event( ){

        status=-1;
        CallBack=NULL;
        h=-1;
    }

    ~Event(){
        if(status!=-1){

            status=-1;
            if(!wasdetached){
                std::cout<<"delete Event"<<std::endl;

                Main.join();
            }

        }
        //Main.detach();
    }

    static void EventHandler(EventData<T> data){

        data.EventHandle->clock.start();
        while( data.EventHandle->status!=-1){

//            try {
                if( data.EventHandle->status==0){
                    data.EventHandle->traded=0;
                    data.EventHandle->CallBack(data.Context);
                }else{
                    data.EventHandle->traded=-1;
                }

                double datatemp=data.h;
                while(datatemp>1.0){
                    data.EventHandle->clock.delayr((long double)1000000000.0);
                    datatemp=datatemp-1.0;
                }
                data.EventHandle->clock.delayr((long double)datatemp*(long double)1000000000.0);
//            }catch(...)
//            {
//                std::exception_ptr p = std::current_exception();
//                std::cout <<(p. : "null") << std::endl;
//            }

            // std::cout<<typeid(data.EventHandle->dummy).name()<<" On EventHandler ... CallBack "<<(long double)data.h*( long double)1000000000.0<<std::endl;
        }


    }
    static void oneShotEventHandler(EventData<T> data){

        data.EventHandle->clock.start();
        double datatemp=data.h;
        while(datatemp>1.0){
            data.EventHandle->clock.delayr((long double)1000000000.0);
            datatemp=datatemp-1.0;
        }
        data.EventHandle->clock.delayr((long double)datatemp*(long double)1000000000.0);
        //        data.EventHandle->clock.delayr((long double)data.h*(long double)1000000000.0);
        //std::cout<<typeid(data.EventHandle->dummy).name()<<" On oneshot ... CallBack "<<(long double)data.h*( long double)1000000000.0<<std::endl;

        data.CallBack(data.Context);
        data.EventHandle->status=-1;

    }


    void forcestart(double tf,void (*CallBack_)(T),T data){

        if(status!=-1){

            stop();
        }
        CallBack=CallBack_;
        h=tf;
        status=0;
        Main.execute<EventData<T> >(EventHandler,EventData<T>(data,this,h));
        //std::cout<<typeid(dummy).name()<<" starting event!!!!!!!!!!  "<<h<<std::endl;
    }


    void start(double tf,void (*CallBack_)(T),T data){



        if(status==1){
            //is paused

            unpause();
            return;
        }


        if(status!=-1 && tf!=h){

            stop();

        }else if(status!=-1){

            std::cout<<"Event is already online"<<std::endl;

            return;
        }



        CallBack=CallBack_;
        h=tf;
        status=0;


        Main.execute<EventData<T> >(EventHandler,EventData<T>(data,this,h));

    }

    void startOneShootEvent(double tf,void (*CallBack_)(T),T data){

        //        std::cout<<"doing a one shot "<< debugname<<std::endl;
        //not implemented?
        if(status!=-1){
            stop();
        }
        CallBack=CallBack_;
        h=tf;
        status=0;
        EventData<T> ev(data,this,h);
        Main.execute<EventData<T> >(oneShotEventHandler,ev);
        Main.detach();


    }

    void pause(){
        if(status==-1){
            return;
        }
        status=1;

    }

    void waitTillPause(){
        if(status==-1){
            return;
        }
        status=1;

        Clock ck;
        while(traded!=-1){
            ck.delayr(1000000);
        }


    }

    void unpause(){
        if(status==-1){
            return;
        }
        if(status==1) status=0;

    }
    void start(){
        if(status==-1){
            return;
        }
        if(status==1) status=0;

    }


    void killAfterLoop(){
        if(status!=1){
            status=1;
        }
    }
    void stop(){

        if(status!=-1){
            status=-1;
            std::cout<<"stop Event"<<std::endl;
            std::cout<<debugname<<" "<<typeid(dummy).name()<<"  deleted event!!!!!!!!!!    "<<h<<" s"<<std::endl;

            Main.join();
            //kill it
        }
    }




    std::string getName() const
    {
        return debugname;
    }

    void setName(const std::string &value)
    {
        debugname = value+"  "+ typeid(dummy).name();
    }


};


#endif // EVENTS_H

