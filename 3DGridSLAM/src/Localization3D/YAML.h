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
#ifndef YAML_H
#define YAML_H


#include <iostream>
#include <unordered_map>
#include <map>
#include <fstream>
#include <sstream>
#include "StringTools.h"

//#include "Logging/FakeLog.h"
//namespace _YAML_H {
//static bool state=Garrote::FakeLog::instance()->registerLine(std::string(__FILE__)+" file was compiled on "+std::string(__TIME__));
//}


class YAML{

    //#warning Simple YAML Code to Load ROS Data from yaml files
public:

    std::unordered_map<std::string,std::string> data;


    YAML(){

    }


    ~YAML(){

        clear();
    }

    double getAsDouble(std::string key){

        if(data.find(key)!=data.end()){
            return atof(data[key].c_str());
        }else{
            std::cout<<" DID NOT FIND "<<key<<std::endl;
            return  0;
        }

    }
    int getAsInt(std::string key){

        if(data.find(key)!=data.end()){
            return atoi(data[key].c_str());
        }else{
            std::cout<<" DID NOT FIND "<<key<<std::endl;
            return  0;
        }

    }
    std::string get(std::string key){

        if(data.find(key)!=data.end()){
            return data[key];
        }else{
            std::cout<<" DID NOT FIND "<<key<<std::endl;
            return  "";
        }

    }
    std::string get2(std::string key){

        if(data.find(key)!=data.end()){
            return data[key];
        }else{
             return  "";
        }

    }

    template <typename T>
    T get (std::string key){

        if(data.find(key)!=data.end()){
            //return ;
            std::stringstream  myString(data[key]);
            T value;
            myString >> value;
            return value;

        }else{
            std::cout<<" DID NOT FIND "<<key<<std::endl;
            return 0;
        }
    }

    bool exists(std::string key){

        if(data.find(key)!=data.end()){
            return true;
        }else{
            return  false;
        }

    }

    void clear(){
        data.clear();
    }

    void addElement(std::string Name,std::string Value){
        data[Name]=Value;
    }



    bool save(std::string filename){
        std::fstream fd;
        fd.open(filename.c_str(), std::fstream::out| std::ios_base::trunc );

        if(fd.is_open()){

            for(std::unordered_map<std::string,std::string>::iterator it = data.begin(); it != data.end(); ++it) {

                // std::cout<<it->first<<": "<<it->second<<std::endl;
                fd<<it->first<<": "<<it->second<<std::endl;

            }

            fd.close();
            return true;
        }else{
            std::cout<<"Could not create file "<<filename<<std::endl;
            return false;
        }



    }


    std::map<std::string,std::string>   getVariablesMAP(){

        std::map<std::string,std::string>   out;
        for(std::unordered_map<std::string,std::string>::iterator it = data.begin(); it != data.end(); ++it) {

            out[it->first]=it->second;
        }
        return out;

    }


    std::unordered_map<std::string,std::string>  getVariables(){
        return data;
    }



    bool fileExists(std::string filename){
        std::cout<<"Loading "<<filename<<std::endl;
        std::fstream fd;

        fd.open(filename.c_str(), std::fstream::in );

        if(fd.is_open()){
            fd.close();
            return true;
        }
        return false;
    }

    std::string get(std::string key,std::string defaultValue){

        if(data.find(key)!=data.end()){
            return data[key];
        }else{
            std::cout<<" DID NOT FIND "<<key<<" Using :: "<<defaultValue<<std::endl;
            return  defaultValue;
        }

    }

    bool get(std::string key,bool defaultValue){

        if(data.find(key)!=data.end()){

            std::string d=data[key];
            std::transform(d.begin(), d.end(), d.begin(), ::tolower);

            if(d=="yes" || d=="y" || d=="true" || d=="t"){
                return true;
            }else{
                return false;
            }

        }else{
            std::cout<<" DID NOT FIND "<<key<<" Using :: "<<defaultValue<<std::endl;
            return  defaultValue;
        }

    }

    template <typename T>
    T get (std::string key, T defaultValue){

        if(data.find(key)!=data.end()){
            //return ;
            std::stringstream  str;
            str<<(data[key]);
            T value;
            str >> value;
            return value;

        }else{
            std::cout<<" DID NOT FIND "<<key<<" Using :: "<<defaultValue<<std::endl;
            return defaultValue;
        }
    }
   std::string get (std::string key, char *defaultValue){


        if(data.find(key)!=data.end()){
                      return data[key];

        }else{
            std::string df(defaultValue);
            std::cout<<" DID NOT FIND "<<key<<" Using :: "<<df<<std::endl;
            return df;
        }
    }

    bool load(std::string filename){

        std::cout<<"Loading "<<filename<<std::endl;
        std::fstream fd;
        fd.open(filename.c_str(), std::fstream::in );

        clear();


        if(fd.is_open()){



            data.clear();

            std::string buffer;

            while(!fd.eof()){

                getline(fd,buffer);

                auto index=buffer.length();
                if(index==0){
                    continue;
                }




                if(buffer[0]=='#'){

                    std::cout<<buffer<<std::endl;

                }else if(( index=buffer.find(": "))!=std::string::npos){

                    std::string variable,variable2;
                    variable=buffer.substr(0,index);
                    if(buffer[index+1]==' '){
                        variable2=buffer.substr(index+2);
                    }else{
                        variable2=buffer.substr(index+1);
                    }

                    Garrote::String::StringTools::removeFromString(variable,'\n');
                    Garrote::String::StringTools::removeFromString(variable,'\r');
                    Garrote::String::StringTools::removeFromString(variable,'\t');

                    Garrote::String::StringTools::removeFromString(variable2,'\n');
                    Garrote::String::StringTools::removeFromString(variable2,'\r');
                    Garrote::String::StringTools::removeFromString(variable2,'\t');

                    //                    std::cout<<"YAMLLOADER v1 ->#"<<variable<<"#v2 -> "<<variable2<<std::endl;
                    data[variable]=variable2;
                }else{

                    if(!buffer.empty()){
                        data["raw"]=buffer;
                        //                        std::cout<<"YAMLLOADER v1 ->#"<<"raw"<<"#v2 ->#"<<buffer<<"#"<<std::endl;
                    }
                }



                //                if(( index=buffer.find(": "))!=std::string::npos){

                //                    std::string variable,variable2;

                //                    variable=buffer.substr(0,index);
                //                    variable2=buffer.substr(index+2);
                //                 Garrote::String::StringTools::removeFromString(variable,'\n');
                //                 Garrote::String::StringTools::removeFromString(variable,'\r');
                //                 Garrote::String::StringTools::removeFromString(variable,'\t');

                //                 Garrote::String::StringTools::removeFromString(variable2,'\n');
                //                 Garrote::String::StringTools::removeFromString(variable2,'\r');
                //                 Garrote::String::StringTools::removeFromString(variable2,'\t');

                ////                 std::cout<<" v1 ->#"<<variable<<"#v2 ->#"<<variable2<<"#"<<std::endl;

                //                    //                    std::string::iterator end_pos = std::remove(variable.begin(), variable.end(), ' ');
                //                    //                    variable.erase(end_pos, variable.end());

                //                    //                    end_pos = std::remove(variable2.begin(), variable2.end(), ' ');
                //                    //                    variable2.erase(end_pos, variable2.end());



                //                    data[variable]=variable2;



                //                }


            }
            fd.close();



            if(data.empty()){
                return false;
            }else{
                return true;
            }



        }else{
            std::cout<<"Failed to Load "<<filename<<std::endl;

            //fd.close();
            return false;
        }

    }
    bool load(std::string filename,std::string separator){

        std::cout<<"Loading "<<filename<<std::endl;
        std::fstream fd;
        fd.open(filename.c_str(), std::fstream::in );

        clear();


        if(fd.is_open()){



            data.clear();

            std::string buffer;

            while(!fd.eof()){

                getline(fd,buffer);

//                unsigned long int index;

//                if(buffer.length()==0){
//                    continue;
//                }
                auto index=buffer.length();
                if(index==0){
                    continue;
                }
                if(buffer[0]=='#'){

                    std::cout<<buffer<<std::endl;

                }else if(( index=buffer.find(separator))!=std::string::npos){

                    std::string variable,variable2;
                    variable=buffer.substr(0,index);
                    if(buffer[index+1]==' '){
                        variable2=buffer.substr(index+2);
                    }else{
                        variable2=buffer.substr(index+1);
                    }

                    Garrote::String::StringTools::removeFromString(variable,'\n');
                    Garrote::String::StringTools::removeFromString(variable,'\r');
                    Garrote::String::StringTools::removeFromString(variable,'\t');

                    Garrote::String::StringTools::removeFromString(variable2,'\n');
                    Garrote::String::StringTools::removeFromString(variable2,'\r');
                    Garrote::String::StringTools::removeFromString(variable2,'\t');

                    //                    std::cout<<"YAMLLOADER v1 ->#"<<variable<<"#v2 -> "<<variable2<<std::endl;
                    data[variable]=variable2;
                }else{

                    if(!buffer.empty()){
                        data["raw"]=buffer;
                        //                        std::cout<<"YAMLLOADER v1 ->#"<<"raw"<<"#v2 ->#"<<buffer<<"#"<<std::endl;
                    }
                }



                //                if(( index=buffer.find(": "))!=std::string::npos){

                //                    std::string variable,variable2;

                //                    variable=buffer.substr(0,index);
                //                    variable2=buffer.substr(index+2);
                //                 Garrote::String::StringTools::removeFromString(variable,'\n');
                //                 Garrote::String::StringTools::removeFromString(variable,'\r');
                //                 Garrote::String::StringTools::removeFromString(variable,'\t');

                //                 Garrote::String::StringTools::removeFromString(variable2,'\n');
                //                 Garrote::String::StringTools::removeFromString(variable2,'\r');
                //                 Garrote::String::StringTools::removeFromString(variable2,'\t');

                ////                 std::cout<<" v1 ->#"<<variable<<"#v2 ->#"<<variable2<<"#"<<std::endl;

                //                    //                    std::string::iterator end_pos = std::remove(variable.begin(), variable.end(), ' ');
                //                    //                    variable.erase(end_pos, variable.end());

                //                    //                    end_pos = std::remove(variable2.begin(), variable2.end(), ' ');
                //                    //                    variable2.erase(end_pos, variable2.end());



                //                    data[variable]=variable2;



                //                }


            }
            fd.close();



            if(data.empty()){
                return false;
            }else{
                return true;
            }



        }else{

            //fd.close();
            return false;
        }

    }





    static std::map<std::string,std::string> parseText(std::string &data){

        std::stringstream str;
        std::map<std::string,std::string> out;
        str<<data;

        std::string buffer;
        while(!str.eof()){

            std::getline(str,buffer);

             auto index=buffer.length();
            if(index==0){
                continue;
            }
            if(( index=buffer.find(": "))!=std::string::npos){

                std::string variable,variable2;
                variable=buffer.substr(0,index);
                if(buffer[index+1]==' '){
                    variable2=buffer.substr(index+2);
                }else{
                    variable2=buffer.substr(index+1);
                }
                //  std::cout<<"parseText v1 ->#"<<variable<<"#v2 -> "<<variable2<<std::endl;
                out[variable]=variable2;
            }else{

                if(!buffer.empty()){
                    out["raw"]=buffer;
                    //      std::cout<<"parseText v1 ->#"<<"raw"<<"#v2 ->#"<<buffer<<"#"<<std::endl;
                }
            }

        }


        return out;


    }






};

#endif // YAML_H
