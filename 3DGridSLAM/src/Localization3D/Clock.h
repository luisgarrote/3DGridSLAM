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
#ifndef CLOCK_H
#define CLOCK_H

#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#ifdef __WIN32

#include <winsock2.h>
#include <stdint.h>
#ifndef INT64_C
#define INT64_C(x) (x ## LL)
#endif
/* Number of 100ns-seconds between the beginning of the Windows epoch
 * (Jan. 1, 1601) and the Unix epoch (Jan. 1, 1970)
 */
#define DELTA_EPOCH_IN_100NS    INT64_C(116444736000000000)

#define MAX_SLEEP_IN_MS         4294967294UL

#define POW10_2     (100)
#define POW10_3     (1000)
#define POW10_4     (10000)
#define POW10_6     (1000000)
#define POW10_7     (10000000)
#define POW10_9     (1000000000)

#endif

//http://www.cplusplus.com/reference/thread/this_thread/sleep_for/
//TODO

class Clock
{

#ifdef __MACH__
#include <sys/time.h>
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 0
#endif

    //clock_gettime is not implemented on OSX
    static  int clock_gettime(int /*clk_id*/, struct timespec* t) {
        struct timeval now;
        int rv = gettimeofday(&now, NULL);
        if (rv) return rv;
        t->tv_sec  = now.tv_sec;
        t->tv_nsec = now.tv_usec * 1000;
        return 0;
    }
#endif

#ifdef __WIN32

    static int clock_gettime_v2(int, struct timespec *spec)      //C-file part
    {  __int64 wintime; GetSystemTimeAsFileTime((FILETIME*)&wintime);
       wintime      -=116444736000000000LL;  //1jan1601 to 1jan1970
       spec->tv_sec  =wintime / 10000000LL;           //seconds
       spec->tv_nsec =wintime % 10000000LL *100;      //nano-seconds
       return 0;
    }
    int nanosleep_2(const struct timespec *request, struct timespec *remain)
    {
        unsigned long ms, rc = 0;
        unsigned __int64 u64, want, real;

        union {
            unsigned __int64 ns100;
            FILETIME ft;
        }  _start, _end;

        if (request->tv_sec < 0 || request->tv_nsec < 0 || request->tv_nsec >= POW10_9) {
            errno = EINVAL;
            return -1;
        }

        if (remain != NULL) GetSystemTimeAsFileTime(&_start.ft);

        want = u64 = request->tv_sec * POW10_3 + request->tv_nsec / POW10_6;
        while (u64 > 0 && rc == 0) {
            if (u64 >= MAX_SLEEP_IN_MS) ms = MAX_SLEEP_IN_MS;
            else ms = (unsigned long) u64;

            u64 -= ms;
            rc =0;
            Sleep(ms);
        }

        if (rc != 0) { /* WAIT_IO_COMPLETION */
            if (remain != NULL) {
                GetSystemTimeAsFileTime(&_end.ft);
                real = (_end.ns100 - _start.ns100) / POW10_4;

                if (real >= want) u64 = 0;
                else u64 = want - real;

                remain->tv_sec = u64 / POW10_3;
                remain->tv_nsec = (long) (u64 % POW10_3) * POW10_6;
            }

            errno = EINTR;
            return -1;
        }

        return 0;
    }
#endif
private:
    struct timespec init;
    struct timespec end;
    double initTime;
public:
    Clock(){

#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &init );
#else
        clock_gettime(CLOCK_MONOTONIC, &init );

#endif
        initTime= init.tv_sec*1000.0 + init.tv_nsec/1000000.0;

    }


    static void sleep(unsigned long int ns){
        Clock c;
        c.start();
        c.delay(ns);
    }

    void start(){

#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &init );
#else
        clock_gettime(CLOCK_MONOTONIC, &init );

#endif
        //initTime= init.tv_sec*1000.0 + init.tv_nsec/1000000.0;
        tic();

    }
    double elapsed(){


#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &end );
#else
        clock_gettime(CLOCK_MONOTONIC, &end );

#endif
//        clock_gettime(CLOCK_MONOTONIC, &end );

        double dSeconds = (end.tv_sec - init.tv_sec)*1000.0;

        double dNanoSeconds = (double)( end.tv_nsec - init.tv_nsec ) / 1000000.0;

        return (dSeconds + dNanoSeconds);

    }



    void delay(unsigned long int ns){

        end=init;

        end.tv_nsec+=ns;

        while (end.tv_nsec >= 1000000000) {
            end.tv_nsec -= 1000000000;
            end.tv_sec++;
        }
#if (defined __MACH__ || defined __WIN32)
        //nanosleep(&end, NULL);
        //   std::cout<<" end delay  "<<std::endl;


        struct timespec temp;
//        clock_gettime(CLOCK_MONOTONIC, &temp );
#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &temp );
#else
        clock_gettime(CLOCK_MONOTONIC, &temp );

#endif

        long int ns1=end.tv_nsec-temp.tv_nsec;
        long int s=end.tv_sec-temp.tv_sec;




        //        std::cout<<" tmp tv_nsec  "<<temp.tv_nsec<<std::endl;

        //        std::cout<<" tmp tv_sec  "<<temp.tv_sec<<std::endl;

        if(ns1<0 && s<0){

        }else if(s<0){

        }else{

            if(ns1<0){
                ns1=1000000000+ns1;
                s=s-1;
            }
            temp.tv_nsec=ns1;
            temp.tv_sec=s;
//            std::cout<<ns<<" xxnss "<<ns1<<" s "<<s<<std::endl;
            if(temp.tv_sec<0){
//                std::cout <<temp.tv_sec<<"  "<<ns<< "\n";
                temp.tv_sec=0;
                temp.tv_nsec=0;
            }
//            auto start = std::chrono::high_resolution_clock::now();
//             std::this_thread::sleep_for (std::chrono::nanoseconds(temp.tv_nsec));
//             if(temp.tv_sec>0){
//             std::this_thread::sleep_for (std::chrono::seconds(temp.tv_sec));
//             }
#ifdef __WIN32
             SleepEx (temp.tv_nsec/1000000+temp.tv_sec*1000,true);
#else
              nanosleep(&temp, NULL);
#endif
//            auto end = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<double, std::nano> elapsed = end-start;
//            std::cout <<ns<< " Waited2 " << (int)elapsed.count() << " nano\n";

            //            nanosleep(&temp, NULL);
        }



#else
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&end, NULL);
#endif
    }




    void delayr(unsigned long int ns){


        //                std::cout<<" Start delay on MAC "<<std::endl;
        //                std::cout<<" ns = "<<ns<<std::endl;

        //                std::cout<<" init tv_nsec  "<<init.tv_nsec<<std::endl;

        //                std::cout<<" init tv_sec  "<<init.tv_sec<<std::endl;


        end=init;

        end.tv_nsec+=ns;

        while (end.tv_nsec >= 1000000000) {
            end.tv_nsec -= 1000000000;
            end.tv_sec++;
        }



#if (defined __MACH__ || defined __WIN32)
        //        std::cout<<" start delay  "<<std::endl;

        //nanosleep(&end, NULL);
        //   std::cout<<" end delay  "<<std::endl;


        struct timespec temp;
#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &temp );
#else
        clock_gettime(CLOCK_MONOTONIC, &temp );

#endif
        long int ns1=end.tv_nsec-temp.tv_nsec;
        long int s=end.tv_sec-temp.tv_sec;


//if(ns>1000000000){
//         std::cout<<"goal  "<<ns<<" ns  "<<ns1<< " s  "<<s <<std::endl;
//}

        if(ns1<0 && s<0){

        }else if(s<0){

        }else{

            if(ns1<0){
                 ns1=1000000000+ns1;
//                ns1=std::fabs(ns1);

                s=s-1;
            }
            temp.tv_nsec=ns1;
            temp.tv_sec=s;

            if(temp.tv_sec<0){
//                  std::cout <<ns<<" "<<temp.tv_sec<<"  "<<temp.tv_nsec<<"  "<<(end.tv_nsec-temp.tv_nsec)<<"  "<<(end.tv_sec-temp.tv_sec)<< "\n";

                temp.tv_sec=0;
                temp.tv_nsec=0;
            }

//            std::cout<<ns<<" nss "<<ns1<<" s "<<s<<std::endl;

            //             std::cout<<" nss "<<ns1<<" s "<<s<<std::endl;
            //            nanosleep(&temp, NULL);

#ifdef __WIN32
            SleepEx (temp.tv_nsec/1000000+temp.tv_sec*1000,true);
#else
              nanosleep(&temp, NULL);
#endif
//            auto start = std::chrono::high_resolution_clock::now();
//             std::this_thread::sleep_for (std::chrono::nanoseconds(temp.tv_nsec));
//             if(temp.tv_sec>0){
//             std::this_thread::sleep_for (std::chrono::seconds(temp.tv_sec));
//             }
//            auto end = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<double, std::nano> elapsed = end-start;
//            std::cout <<ns<< " Waited " << (int)elapsed.count() << " nano\n";

        }


#else

        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&end, NULL);

#endif
        init=end;


    }



    static double sinceEpochMS(){

        struct timespec temp;
#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &temp );
#else
        clock_gettime(CLOCK_MONOTONIC, &temp );

#endif
        return temp.tv_sec*1000.0 + temp.tv_nsec/1000000.0;

    }


    static double sinceEpochS(){

        struct timespec temp;
#ifdef __WIN32
        clock_gettime_v2(CLOCK_MONOTONIC, &temp );
#else
        clock_gettime(CLOCK_MONOTONIC, &temp );

#endif
        return (double)temp.tv_sec + temp.tv_nsec/1000000000.0;

    }


    void tic()
    {
        struct timeval first;
        struct timezone tzp;
        gettimeofday(&first, &tzp);

        initTime= first.tv_sec*1000.0 + first.tv_usec/1000.0;


    }

    double toc()
    {
        struct timeval first;
        struct timezone tzp;
        gettimeofday(&first, &tzp);

        double endTime= first.tv_sec*1000.0 + first.tv_usec/1000.0;
        return endTime-initTime;

    }


    static double getTimestamp(){

        struct timeval first;
        struct timezone tzp;
        double timestamp;

        gettimeofday(&first, &tzp);
        timestamp = first.tv_sec*1000.0 + first.tv_usec/1000.0;
        return timestamp;

    }

    static std::string getTimestampString(){

        struct timeval first;
        struct timezone tzp;
        double timestamp;

        gettimeofday(&first, &tzp);
        timestamp = first.tv_sec*1000.0 + first.tv_usec/1000.0;
        std::stringstream ss;
        ss.precision(24);
        ss<<timestamp;
        return ss.str();

    }

    static void printDate(){

        time_t t = time(NULL);
        tm* timePtr = localtime(&t);

        std::cout << "seconds = " << timePtr->tm_sec << std::endl;
        std::cout << "minutes = " << timePtr->tm_min << std::endl;
        std::cout << "hours = " << timePtr->tm_hour << std::endl;
        std::cout << "day of month = " << timePtr->tm_mday << std::endl;
        std::cout << "month of year = " << timePtr->tm_mon << std::endl;
        std::cout << "year = " << 1900+timePtr->tm_year << std::endl;
        std::cout << "weekday = " << timePtr->tm_wday << std::endl;
        std::cout << "day of year = " << timePtr->tm_yday << std::endl;
        std::cout << "daylight savings = " << timePtr->tm_isdst << std::endl;
    }

    static std::string date(std::string yearSeparator,std::string clockseparator){

        time_t t = time(NULL);
        tm* timedata = localtime(&t);

        std::stringstream cd;

        cd  <<1900+timedata->tm_year<<yearSeparator;
        cd << timedata->tm_mon<<yearSeparator;
        cd << timedata->tm_mday<<"_";
        cd << timedata->tm_hour<<clockseparator;
        cd << timedata->tm_min<<clockseparator;
        cd << timedata->tm_sec;

        return cd.str();
    }

    static double getGlobalTimestamp(){

        struct timeval first;
        struct timezone tzp;
        gettimeofday(&first, &tzp);
        return first.tv_sec*1000.0 + first.tv_usec/1000.0;

    }



    void VectorTest(){


        //        cout << "~~ Vector access speed test ~~" << endl << endl;
        //        cout << "~ Initialization ~" << endl;
        //        //long long t;
        //        int a;
        //        vector <int> test (0);
        //        for (int i = 0; i < 100000000; i++)
        //        {
        //            test.push_back(i);
        //        }
        //        cout << "~ Initialization complete ~" << endl << endl;


        //        cout << "     iterator test: ";
        //        tic();
        //        for (vector<int>::iterator it = test.begin(); it < test.end(); it++)
        //        {
        //            a = *it;
        //        }
        //        cout << toc() << endl;



        //        cout << "Optimised iterator: ";
        //        tic();
        //        vector<int>::iterator endofv = test.end();
        //        for (vector<int>::iterator it = test.begin(); it < endofv; it++)
        //        {
        //            a = *it;
        //        }
        //        cout << toc()<< endl;



        //        cout << "                At: ";
        //        tic();
        //        for (unsigned int i = 0; i < test.size(); i++)
        //        {
        //            a = test.at(i);
        //        }
        //        cout << toc() << endl;



        //        cout << "      Optimised at: ";
        //        tic();
        //        int endof = test.size();
        //        for (int i = 0; i < endof; i++)
        //        {
        //            a = test.at(i);
        //        }
        //        cout << toc() << endl;



        //        cout << "             Index: ";
        //        tic();
        //        for (unsigned int i = 0; i < test.size(); i++)
        //        {
        //            a = test[i];
        //        }
        //        cout << toc()<< endl;



        //        cout << "   Optimised Index: ";
        //        tic();
        //        int endofvec = test.size();
        //        for (int i = 0; i < endofvec; i++)
        //        {
        //            a = test[i];
        //        }
        //        cout << toc() << endl;

    }




};

#endif // CLOCK_H
