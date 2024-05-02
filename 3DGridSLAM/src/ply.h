#ifndef PLY_H
#define PLY_H

#include <map>
#include <fstream>
#include <vector>
class PLY{


    //    std::map<std::string,std::map<std::string,std::string>> properties;
    //    std::map<std::string,std::map<std::string,std::string>> element;
    std::map<std::string,std::string> properties;
    std::map<std::string,std::string> element;


public:

    std::vector<std::vector<double>> data;


    template<typename T>
    bool save(std::string filename,std::vector<T> &points){


        data.clear();
        properties.clear();
        element.clear();

        std::fstream fd;
        fd.open(filename.c_str(), std::fstream::out );


//        properties["x"]="float";
//        properties["y"]="float";
//        properties["z"]="float";




        if(fd.is_open()){


//            for(unsigned int i=0;i<points.size();i++){
//                std::vector<double> out;
//                out.push_back(points[i].x);
//                out.push_back(points[i].y);
//                out.push_back(points[i].z);
//                data.push_back(out);
//            }


            fd<<"ply"<<std::endl;
            fd<<"format ascii 1.0"<<std::endl;
            fd<<"element vertex "<<std::to_string(points.size())<<std::endl;
            fd<<"property float x"<<std::endl;
            fd<<"property float y"<<std::endl;
            fd<<"property float z"<<std::endl;
            fd<<"end_header"<<std::endl;

            for(unsigned int i=0;i<points.size();i++){
                fd<<std::to_string(points[i].x)<<" "<<std::to_string(points[i].y)<<" "<<std::to_string(points[i].z)<<std::endl;
            }


            fd.close();
            return true;
        }
        return false;
    }

    bool load(std::string filename){

        std::fstream fd;
        fd.open(filename.c_str(), std::fstream::in );



        if(fd.is_open()){


            std::string buffer;

            getline(fd,buffer);


            if(buffer!="ply"){
                return false;
            }

            bool  header=true;

            while(!fd.eof()){

                getline(fd,buffer);


                if(buffer.length()==0){
                    continue;
                }


                if(header){


                    if(buffer.find("property")!=buffer.npos){

                        auto v=$(buffer).split(" ");

                        if(v.size()==3){
                            properties[v[0]+"::"+v[2]]=v[1];
                        }

                    }
                    if(buffer.find("element")!=buffer.npos){

                        auto v=$(buffer).split(" ");

                        if(v.size()==3){
                            element[v[1]]=v[2];
                        }

                    }

                    if(buffer.find("end_header")!=buffer.npos){
                        header=false;
                    }
                }else{
                    auto v=$(buffer).numbers<double>();
                    if(v.size()==properties.size()){
                        data.push_back(v);
                    }
                }



            }


            fd.close();
            return true;
        }else {
            return  true;
        }

        return false;

    }

};
#endif // PLY_H
