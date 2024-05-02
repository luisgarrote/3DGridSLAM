#ifndef THREAD_H
#define THREAD_H

#if  __cplusplus > 199711L

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <utility>
#include <iostream>

#else
#include <thread>

#endif


//#warning "this thread class only exists to be used in systems without c++14";

class Thread{

    std::thread *start;
    bool status;

    bool pointer;
    bool detached;


public:




    Thread() {
        start=NULL;
        status=false;
        pointer=false;
        detached=false;
    }

    //    Thread(void (*t)(T),T temp){
    //        start=new std::thread(t,temp);
    //    }
    //    Thread(void (*t)(T)){
    //        start=new std::thread(t);
    //    }


    template < typename T>
    void execute(void (*t)(T),T temp){
//        std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(detached==true){
            delete start;
            start=NULL;
            status=false;
            detached=false;
        }
//        std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(status==false){
            status=true;

//           std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;

            if(start!=NULL){
                if(!start->joinable())
                    start->detach();
                else{

                    std::cout<<"thread execute"<<std::endl;

                    start->join();

                }

                delete start;
                start=NULL;
            }
//            std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            start=new std::thread(t,temp);
        }
//        std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
    }


    template < typename T>
    void execute(void (*t)(T)){
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(detached==true){
            delete start;
            start=NULL;
            status=false;
            detached=false;
            //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        }
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(status==false){
            status=true;
            //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            if(start!=NULL){
                delete start;
                start=NULL;

            }
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            start=new std::thread(t);

        }
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
    }

    template < typename T,typename D>
    void execute(void (*t)(T,D)){
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(detached==true){
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            delete start;
            start=NULL;
            status=false;
            detached=false;
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;

        }
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(status==false){
            status=true;

            if(start!=NULL){
                delete start;
            }
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            start=new std::thread(t);
        }
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
    }


    void execute(void (*t)()){
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(detached==true){
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            delete start;
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            start=NULL;
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            status=false;
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            detached=false;
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;

        }
        // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        if(status==false){
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            status=true;
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            if(start!=NULL){
                // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
                delete start;
                // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
                start=NULL;
                // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            }
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
            start=new std::thread(t);
            // //std::cout<<__LINE__<<"   "<<__FILE__<<std::endl;
        }
    }




    void join(){
        std::cout<<"join Thread"<<std::endl;

        if(status==true){
            status=false;
            if(start!=NULL){

                if(start->joinable()){
                    start->join();
                    start=NULL;
                }
            }
        }
    }


    void dispose(){

        if(status==false){
            if(start!=NULL){

                // if(detached==false){
                if(start->joinable()){
                    std::cout<<"thread dispose"<<std::endl;

                    start->join();
                }
                //}
                delete start;
                start=NULL;

            }
        }
    }



    void detach(){
        if(start!=NULL){

            if(start->joinable()){
                start->detach();
                detached=true;
            }
        }

    }

    ~Thread(){

        if(detached==false){
            if(start!=NULL){
                std::cout<<"thread kill"<<std::endl;

                join();
                start=NULL;

            }

            if(pointer==true){
                pointer=false;
                delete this;
            }
        }
        delete start;

    }

    void* operator new(size_t sz)
    {
        //cerr << "new " << sz << " bytes\n";
        Thread* data =(Thread*)::operator new(sz);
        //assim sei que é um ponteiro e posso efectuar o suicide :)
        data->pointer=true;
        return data;
    }

    //    void operator delete(void* p)
    //    {

    //        ::operator delete(p);
    //    }
};



#endif // THREAD_H
