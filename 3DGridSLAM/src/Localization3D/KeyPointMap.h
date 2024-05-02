#ifndef KEYPOINTMAP_H
#define KEYPOINTMAP_H


#include "KdTree.h"


#include <mutex>



template <typename T>
class KeyPointMap{

std::mutex mtx; 

public:

    KdTree<T> tree;
    KeyPointMap():tree(3){



    }

    void clear(){
        tree.clear();
    }

    T nearest(T &p,bool &valid){
        mtx.lock();
        T out;
        auto res=tree.getNearestNode(&p);
        if(res==NULL){

            valid=false;
        }else{

            valid=true;

            out.x=res->x;
            out.y=res->y;
            out.z=res->z;
        }
        mtx.unlock();
        return out;

    }

    void map(T &p,double radius){
        mtx.lock();

        auto res=tree.getNearestNode(&p);

        if(res!=NULL){

            if(sqrt((p.x-res->x)*(p.x-res->x)+(p.y-res->y)*(p.y-res->y)+(p.z-res->z)*(p.z-res->z))<radius){
                        mtx.unlock();

                return;
            }
        }else{
            //return;
        }

        T *val=new T();
        val->x=p.x;
        val->y=p.y;
        val->z=p.z;
        tree.push_back(val);
                mtx.unlock();

    }

    void map(T &p){
                mtx.lock();

        T *val=new T();
        val->x=p.x;
        val->y=p.y;
        val->z=p.z;
        tree.push_back(val);
                mtx.unlock();

    }


    bool save(std::string filename){

           std::fstream fd;
    
    fd.open(filename+".ply", std::fstream::out );
    
    if(fd.is_open())
    {
        
        fd<<"ply"<<std::endl;
        fd<<"format ascii 1.0"<<std::endl;
        fd<<"element vertex "<<std::to_string(tree.points.size())<<std::endl;
        fd<<"property float x"<<std::endl;
        fd<<"property float y"<<std::endl;
        fd<<"property float z"<<std::endl;
        fd<<"end_header"<<std::endl;
        
        for (int i=0;i<tree.points.size();i++)
        {
            
            fd<<std::to_string(tree.points[i].x)<<" "<<std::to_string(tree.points[i].y)<<" "<<std::to_string(tree.points[i].z)<<std::endl;
        }
        
        fd.close();
        std::cout<<"Map Saved"<<std::endl;
    }else{
        return false;
    }

    return true;
    
    }


};

#endif // KEYPOINTMAP_H
