#ifndef SPHERECONTAINER_H
#define SPHERECONTAINER_H
#include <iostream>
#include <vector>
#include <map>
#include "Map.h"

 


template< typename P>
class PointData{

public:

    P minpoint;
    double max=-99999;
    double min=99999;

void update(P p,double r){
        //points.push_back(p);
        max=std::max(max,r);


        //points.push_back(out);
        max=std::max(max,r);
        if(r<min){
            min=r;
            minpoint=p;
        }   

 }

template< typename X>
    void update(X p,double r){
        P out;
        out.x=p.x;
        out.y=p.y;
        out.z=p.z;

        //points.push_back(out);
        max=std::max(max,r);
        if(r<min){
            min=r;
            minpoint=out;
        }
        
    }

};

template< typename P>
class PointVectorToSphereContainer{

    double dphi;
    double dth;

    P center;


    double step=0.1;
public:

    std::pair<double,double> r;
    std::pair<double,double> phi;
    std::pair<double,double> t;

    Map<PointData<P>> *view=NULL;

    ~PointVectorToSphereContainer(){

        delete view;
    }

    PointVectorToSphereContainer(double dphi_,double dth_){
        dphi=dphi_;
        dth=dth_;

    }

    void set(double rmin,double rmax,double phimin,double phimax,double thmin,double thmax,double step_){

        r.first=rmin;
        r.second=rmax;

        t.first=thmin;
        t.second=thmax;

        phi.first=phimin;
        phi.second=phimax;

        step=step_;
        PointData<P> defaultCELL;
        view=new Map<PointData<P>>(((phi.second-phi.first)/dphi)+1,((t.second-t.first)/dth)+1,defaultCELL);

        Point x((t.second+t.first)/2.0,(phi.second+phi.first)/2.0,0);
        view->setCellSizes(dth,dphi,x);

        //std::cout<<"Size: "<<view->cellsizeX<<" "<<view->cellsizeY<<" "<<view->getRows()<<" "<<view->getColumns()<<std::endl;


    }

    float angleBetween(Point p1, Point p2)
    {
        float result = atan2(p2.y-p1.y,p2.x-p1.x) * (180.0/3.141592653589793238463);
        if (result < 0)
        {
            result=result+360;
        }
        return result;
    }

template< typename X>
    void project(X &points){

        //Point p1,p2;
        //p1.x=0;
        //p1.y=0;

        //float angle2=0;

        for(unsigned int i=0;i<points.size();i++){
            //p2.x=points[i].x;
            //p2.y=points[i].y;
            //p2.z=points[i].z;
            //angle2=angleBetween(p1,p2);angle2>=195 || angle2<=165

            
            double rg=sqrt(points[i].x*points[i].x+points[i].y*points[i].y+points[i].z*points[i].z);
            double phig=acos(points[i].z/rg);
            double thg=atan2(points[i].y,points[i].x);

            auto pv=view->getPointerXY(thg,phig);
            if(pv!=NULL)
                pv->update(points[i],rg-step);
            
        }

    }

     void project(const std::vector<P> &points){

        Point p1,p2;
        p1.x=0;
        p1.y=0;

        float angle2=0;

        for(unsigned int i=0;i<points.size();i++){
            p2.x=points[i].x;
            p2.y=points[i].y;
            p2.z=points[i].z;
            angle2=angleBetween(p1,p2);

            if(angle2>=195 || angle2<=165){
                double rg=sqrt(points[i].x*points[i].x+points[i].y*points[i].y+points[i].z*points[i].z);
                double phig=acos(points[i].z/rg);
                double thg=atan2(points[i].y,points[i].x);
                
                auto pv=view->getPointerXY(thg,phig);
                if(pv!=NULL)
                    pv->update(points[i],rg-step);
            }
        }

    }

    void project(Transform center_,const std::vector<P> &points){
        //        center=center_;
        P pt;
        
        Point p1,p2;
        p1.x=0;
        p1.y=0;

        auto tf=center_.inverse();

        float angle2=0;

        for(unsigned int i=0;i<points.size();i++){
            p2.x=points[i].x;
            p2.y=points[i].y;
            p2.z=points[i].z;
            angle2=angleBetween(p1,p2);

            if(angle2>=195 || angle2<=165){
                pt=tf*points[i];


                double rg=sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
                double phig=acos(pt.z/rg);
                double thg=atan2(pt.y,pt.x);

                auto pv=view->getPointerXY(thg,phig);
                if(pv!=NULL)
                    pv->update(points[i],rg-step);
            }
        }

    }



    std::vector<P> getFeatureAreas(double delta){

        std::vector<P> out;
        P pout;
        
        for(unsigned int i=0;i<view->getColumns();i++){
            for(unsigned int j=0;j<view->getRows();j++){

                auto a=view->getIndexXY(i,j);

                if(a.max<90.0){

                    double tdelta=a.max-a.min;

                    if(tdelta>delta){
                        //https://stackoverflow.com/questions/201718/concatenating-two-stdvectors
                        //out.insert(
                        //     out.end(),
                        //     std::make_move_iterator(a.points.begin()),
                        //     std::make_move_iterator(a.points.end())
                        //   );
                        //double thg=view->convertToXFromIndex(i);
                        //double phig=view->convertToYFromIndex(j);
                        //double rg=a.min;

                        //pout.x=rg*cos(thg)*sin(phig);
                        //pout.y=rg*sin(thg)*sin(phig);
                        //pout.z=rg*cos(phig);
                        out.push_back(a.minpoint);

                  
                    }
                }
            }
        }
        return out;
    }



};




 


#endif // SPHERECONTAINER_H
