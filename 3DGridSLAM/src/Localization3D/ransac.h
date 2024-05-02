#ifndef RANSAC_H
#define RANSAC_H

#include "Random.h"
#include <vector>
/*
Given:
    data – a set of observed data points
    model – a model that can be fitted to data points
    n – the minimum number of data values required to fit the model
    k – the maximum number of iterations allowed in the algorithm
    t – a threshold value for determining when a data point fits a model
    d – the number of close data values required to assert that a model fits well to data

Return:
    bestfit – model parameters which best fit the data (or nul if no good model is found)

iterations = 0
bestfit = nul
besterr = something really large
while iterations < k {
    maybeinliers = n randomly selected values from data
    maybemodel = model parameters fitted to maybeinliers
    alsoinliers = empty set
    for every point in data not in maybeinliers {
        if point fits maybemodel with an error smaller than t
             add point to alsoinliers
    }
    if the number of elements in alsoinliers is > d {
        % this implies that we may have found a good model
        % now test how good it is
        bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
        thiserr = a measure of how well model fits these points
        if thiserr < besterr {
            bestfit = bettermodel
            besterr = thiserr
        }
    }
    increment iterations
}
return bestfit
*/

template <typename PointType>
class Plane{

public:

    double coefs[4];
    float pmax[3];
    float pmin[3];
    bool valid=false;

    double norm;

    bool minmax=false;
    void setValid(bool valid_)
    {
        valid=valid_;
    }
    bool isValid()
    {
        return valid;
    }
    Plane(){
        coefs[0]=0;
        coefs[1]=0;
        coefs[2]=0;
        coefs[3]=0;

        pmax[0]=0;
        pmax[1]=0;
        pmax[2]=0;

        pmin[0]=0;
        pmin[1]=0;
        pmin[2]=0;


    }
    Plane(const PointType &p1,const PointType &p2,const PointType &p3){
        valid=true;
        minmax=false;
        double dx1=p2.x-p1.x;
        double dy1=p2.y-p1.y;
        double dz1=p2.z-p1.z;
        double dx2=p3.x-p1.x;
        double dy2=p3.y-p1.y;
        double dz2=p3.z-p1.z;
        coefs[0]=dy1*dz2-dy2*dz1;
        coefs[1]=dz1*dx2-dz2*dx1;
        coefs[2]=dx1*dy2-dx2*dy1;

        if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/
        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;
        coefs[3]=-coefs[0]*p1.x-coefs[1]*p1.y-coefs[2]*p1.z;
        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);

    }

    bool set(const PointType &p1,const PointType &p2,const PointType &p3){
        valid=true;
        minmax=false;
        double dx1=p2.x-p1.x;
        double dy1=p2.y-p1.y;
        double dz1=p2.z-p1.z;
        double dx2=p3.x-p1.x;
        double dy2=p3.y-p1.y;
        double dz2=p3.z-p1.z;
        coefs[0]=dy1*dz2-dy2*dz1;
        coefs[1]=dz1*dx2-dz2*dx1;
        coefs[2]=dx1*dy2-dx2*dy1;


        if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/
        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;
        coefs[3]=-coefs[0]*p1.x-coefs[1]*p1.y-coefs[2]*p1.z;


        //        pmax[0]=std::max(p1.x,std::max(p2.x,p3.x));
        //        pmax[1]=std::max(p1.y,std::max(p2.y,p3.y));
        //        pmax[2]=std::max(p1.z,std::max(p2.z,p3.z));


        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        //        pmin[0]=std::min(p1.x,std::min(p2.x,p3.x));
        //        pmin[1]=std::min(p1.y,std::min(p2.y,p3.y));
        //        pmin[2]=std::min(p1.z,std::min(p2.z,p3.z));
        return valid;
    }

    bool set(double a,double b,double c,double d){
        valid=true;






        if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/
        double norm=sqrt(a*a+b*b+c*c);
        coefs[0]=a/norm;
        coefs[1]=b/norm;
        coefs[2]=c/norm;
        coefs[3]=d;

        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);

        //        pmax[0]=std::max(p1.x,std::max(p2.x,p3.x));
        //        pmax[1]=std::max(p1.y,std::max(p2.y,p3.y));
        //        pmax[2]=std::max(p1.z,std::max(p2.z,p3.z));


        //        pmin[0]=std::min(p1.x,std::min(p2.x,p3.x));
        //        pmin[1]=std::min(p1.y,std::min(p2.y,p3.y));
        //        pmin[2]=std::min(p1.z,std::min(p2.z,p3.z));
        return valid;
    }

    Plane(std::vector<PointType> &data, std::vector<unsigned int> &indexes){
        valid=true;
        minmax=false;
        // from https://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
        double meanx=0;
        double meany=0;
        double meanz=0;

        double x_2=0;
        double y_2=0;
        double y_=0;
        double x_=0;
        double xy=0;
        double z_=0;
        double xz=0;
        double yz=0;

        double a,b,c,d,e,f,g,h,i,x,y,z;

        //        for(auto i=0;i<indexes.size();i++){
        //            meanx+=data[indexes[i]].x;
        //            meany+=data[indexes[i]].y;
        //            meanz+=data[indexes[i]].z;
        //        }

        //        pmax[0]=data[indexes[0]].x;
        //        pmax[1]=data[indexes[0]].y;
        //        pmax[2]=data[indexes[0]].z;


        //        pmin[0]=data[indexes[0]].x;
        //        pmin[1]=data[indexes[0]].y;
        //        pmin[2]=data[indexes[0]].z;

        for(auto ii=0;ii<indexes.size();ii++){
            meanx+=data[indexes[ii]].x;
            meany+=data[indexes[ii]].y;
            meanz+=data[indexes[ii]].z;
        }

        //        pmax[0]=data[indexes[0]].x;
        //        pmax[1]=data[indexes[0]].y;
        //        pmax[2]=data[indexes[0]].z;

        
        meanx=meanx/(double)indexes.size();
        meany=meany/(double)indexes.size();
        meanz=meanz/(double)indexes.size();

        //        pmin[0]=data[indexes[0]].x;
        //        pmin[1]=data[indexes[0]].y;
        //        pmin[2]=data[indexes[0]].z;
        for(auto ii=0;ii<indexes.size();ii++){

            x_2+=(data[indexes[ii]].x-meanx)*(data[indexes[ii]].x-meanx);
            y_2+=(data[indexes[ii]].y-meany)*(data[indexes[ii]].y-meany);
            z_+=(data[indexes[ii]].z-meanz);
            y_+=(data[indexes[ii]].y-meany);
            x_+=(data[indexes[ii]].x-meanx);
            xy+=(data[indexes[ii]].x-meanx)*(data[indexes[ii]].y-meany);
            xz+=(data[indexes[ii]].x-meanx)*(data[indexes[ii]].z-meanz);
            yz+=(data[indexes[ii]].z-meanz)*(data[indexes[ii]].y-meany);

            //            pmax[0]=std::max(data[indexes[i]].x,pmax[0]);
            //            pmax[1]=std::max(data[indexes[i]].y,pmax[1]);
            //            pmax[2]=std::max(data[indexes[i]].z,pmax[2]);


            //            pmin[0]=std::min(data[indexes[i]].x,pmin[0]);
            //            pmin[1]=std::min(data[indexes[i]].y,pmin[1]);
            //            pmin[2]=std::min(data[indexes[i]].z,pmin[2]);
        }


        //        for(auto i=0;i<indexes.size();i++){

        //            x_2+=(data[indexes[i]].x-meanx)*(data[indexes[i]].x-meanx);
        //            y_2+=(data[indexes[i]].y-meany)*(data[indexes[i]].y-meany);
        //            z_+=(data[indexes[i]].z-meanz);
        //            y_+=(data[indexes[i]].y-meany);
        //            x_+=(data[indexes[i]].x-meanx);
        //            xy+=(data[indexes[i]].x-meanx)*(data[indexes[i]].y-meany);
        //            xz+=(data[indexes[i]].x-meanx)*(data[indexes[i]].z-meanz);
        //            yz+=(data[indexes[i]].z-meanz)*(data[indexes[i]].y-meany);

        ////            pmax[0]=std::max(data[indexes[i]].x,pmax[0]);
        ////            pmax[1]=std::max(data[indexes[i]].y,pmax[1]);
        ////            pmax[2]=std::max(data[indexes[i]].z,pmax[2]);


        ////            pmin[0]=std::min(data[indexes[i]].x,pmin[0]);
        ////            pmin[1]=std::min(data[indexes[i]].y,pmin[1]);
        ////            pmin[2]=std::min(data[indexes[i]].z,pmin[2]);

        //        }



        //  from matlab using ...
        //syms a b c d e f g h i x y z;
        //A=[a b c; d e f; g h i];
        //B= [x ;y; z]
        //inv(A)*B

        a=x_2;
        b=xy;
        c=x_;
        d=xy;
        e=y_2;
        f=y_;
        g=x_;
        h=y_;
        i=indexes.size();

        x=xz;
        y=yz;
        z=z_;

        coefs[0]=(z*(b*f - c*e))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (y*(b*i - c*h))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) + (x*(e*i - f*h))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        coefs[1]=(y*(a*i - c*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (z*(a*f - c*d))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (x*(d*i - f*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        coefs[2]=(z*(a*e - b*d))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (y*(a*h - b*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) + (x*(d*h - e*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);


        if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/


        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;

        coefs[3]=-(coefs[0]*data[indexes[0]].x+coefs[1]*data[indexes[0]].y+coefs[2]*data[indexes[0]].z);


        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);

    }



    //http://www.songho.ca/math/plane/plane.html
    bool set(std::vector<PointType> &data, std::vector<unsigned int> &indexes){
        valid=true;
        minmax=false;
        // from https://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
        double meanx=0;
        double meany=0;
        double meanz=0;

        double x_2=0;
        double y_2=0;
        double y_=0;
        double x_=0;
        double xy=0;
        double z_=0;
        double xz=0;
        double yz=0;

        double a,b,c,d,e,f,g,h,i,x,y,z;

        for(unsigned int ii=0;ii<indexes.size();ii++){
            meanx+=data[indexes[ii]].x;
            meany+=data[indexes[ii]].y;
            meanz+=data[indexes[ii]].z;
        }

        //        pmax[0]=data[indexes[0]].x;
        //        pmax[1]=data[indexes[0]].y;
        //        pmax[2]=data[indexes[0]].z;


        meanx=meanx/(double)indexes.size();
        meany=meany/(double)indexes.size();
        meanz=meanz/(double)indexes.size();

        //        pmin[0]=data[indexes[0]].x;
        //        pmin[1]=data[indexes[0]].y;
        //        pmin[2]=data[indexes[0]].z;
        for(unsigned int ii=0;ii<indexes.size();ii++){

            x_2+=(data[indexes[ii]].x-meanx)*(data[indexes[ii]].x-meanx);
            y_2+=(data[indexes[ii]].y-meany)*(data[indexes[ii]].y-meany);
            z_+=(data[indexes[ii]].z-meanz);
            y_+=(data[indexes[ii]].y-meany);
            x_+=(data[indexes[ii]].x-meanx);
            xy+=(data[indexes[ii]].x-meanx)*(data[indexes[ii]].y-meany);
            xz+=(data[indexes[ii]].x-meanx)*(data[indexes[ii]].z-meanz);
            yz+=(data[indexes[ii]].z-meanz)*(data[indexes[ii]].y-meany);

            //            pmax[0]=std::max(data[indexes[i]].x,pmax[0]);
            //            pmax[1]=std::max(data[indexes[i]].y,pmax[1]);
            //            pmax[2]=std::max(data[indexes[i]].z,pmax[2]);


            //            pmin[0]=std::min(data[indexes[i]].x,pmin[0]);
            //            pmin[1]=std::min(data[indexes[i]].y,pmin[1]);
            //            pmin[2]=std::min(data[indexes[i]].z,pmin[2]);
        }



        //  from matlab using ...
        //syms a b c d e f g h i x y z;
        //A=[a b c; d e f; g h i];
        //B= [x ;y; z]
        //inv(A)*B

        a=x_2;
        b=xy;
        c=x_;
        d=xy;
        e=y_2;
        f=y_;
        g=x_;
        h=y_;
        i=indexes.size();

        x=xz;
        y=yz;
        z=z_;

        coefs[0]=(z*(b*f - c*e))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (y*(b*i - c*h))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) + (x*(e*i - f*h))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        coefs[1]=(y*(a*i - c*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (z*(a*f - c*d))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (x*(d*i - f*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        coefs[2]=(z*(a*e - b*d))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (y*(a*h - b*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) + (x*(d*h - e*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/


        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;
        coefs[3]=-(coefs[0]*data[indexes[0]].x+coefs[1]*data[indexes[0]].y+coefs[2]*data[indexes[0]].z);

        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);

        return valid;
    }




    bool set(std::vector<PointType> &data,double th){
        valid=true;
        minmax=true;
        // from https://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf

        //https://math.stackexchange.com/questions/1657030/fit-plane-to-3d-data-using-least-squares


        double meanx=0;
        double meany=0;
        double meanz=0;




        for(unsigned int ii=0;ii<data.size();ii++){
            meanx+=data[ii].x;
            meany+=data[ii].y;
            meanz+=data[ii].z;
        }

        pmax[0]=data[0].x;
        pmax[1]=data[0].y;
        pmax[2]=data[0].z;

        meanx=meanx/(double)data.size();
        meany=meany/(double)data.size();
        meanz=meanz/(double)data.size();


        pmin[0]=data[0].x;
        pmin[1]=data[0].y;
        pmin[2]=data[0].z;

        double l00=0;
        double l11=0;
        double l01=0;

        double r0=0;
        double r1=0;



        for(unsigned int ii=0;ii<data.size();ii++){

            l00+=(data[ii].x-meanx)*(data[ii].x-meanx);
            l11+=(data[ii].y-meany)*(data[ii].y-meany);
            l01+=(data[ii].x-meanx)*(data[ii].y-meany);

            r0+=(data[ii].z-meanz)*(data[ii].x-meanx);
            r1+=(data[ii].z-meanz)*(data[ii].y-meany);

            pmax[0]=std::max((float)data[ii].x,pmax[0]);
            pmax[1]=std::max((float)data[ii].y,pmax[1]);
            pmax[2]=std::max((float)data[ii].z,pmax[2]);


            pmin[0]=std::min((float)data[ii].x,pmin[0]);
            pmin[1]=std::min((float)data[ii].y,pmin[1]);
            pmin[2]=std::min((float)data[ii].z,pmin[2]);
        }


        //Cz=Ax+By+D
        //to...
        //Cz-Ax-By-D=0



        //Cz=Ax+By+D :

        coefs[0]= (l11*r0-l01*r1)/(l00*l11-l01*l01);
        coefs[1]= (l00*r1-l01*r0)/(l00*l11-l01*l01);
        //if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/
        coefs[2]=1;
        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;
        coefs[3]=-coefs[0]*meanx-coefs[1]*meany+coefs[2]*meanz;

        //Cz-Ax-By-D=0 :

        coefs[0]= -coefs[0];
        coefs[1]= -coefs[1];
//        coefs[2]=1;
        coefs[3]=-coefs[3];


        int nonIn=0;
        for(unsigned int ii=0;ii<data.size();ii++){

             if(distance(data[ii])>0.7*th){
                 nonIn++;
             }
        }
        double cut=(double)nonIn/(double)data.size();

        if(cut>0.1){

            valid=false;
        }



        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);


        return valid;
    }


    template <typename PointType2>
    bool set2(std::vector<PointType2> &data,double th){
        valid=true;
        minmax=true;
        // from https://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf

        //https://math.stackexchange.com/questions/1657030/fit-plane-to-3d-data-using-least-squares


        double meanx=0;
        double meany=0;
        double meanz=1;




        for(unsigned int ii=0;ii<data.size();ii++){
            meanx+=data[ii].x;
            meany+=data[ii].y;
        }

        pmax[0]=data[0].x;
        pmax[1]=data[0].y;

        meanx=meanx/(double)data.size();
        meany=meany/(double)data.size();
        meanz=0.5;


        pmin[0]=data[0].x;
        pmin[1]=data[0].y;

        double l00=0;
        double l11=0;
        double l01=0;

        double r0=0;
        double r1=0;



        for(unsigned int ii=0;ii<data.size();ii++){

            l00+=(data[ii].x-meanx)*(data[ii].x-meanx);
            l11+=(data[ii].y-meany)*(data[ii].y-meany);
            l01+=(data[ii].x-meanx)*(data[ii].y-meany);

//            r0+=(data[ii].z-meanz)*(data[ii].x-meanx);
//            r1+=(data[ii].z-meanz)*(data[ii].y-meany);

            r0+=(0.0-meanz)*(data[ii].x-meanx)*0.5;
            r1+=(0.0-meanz)*(data[ii].y-meany)*0.5;
            r0+=(2.0-meanz)*(data[ii].x-meanx)*0.5;
            r1+=(2.0-meanz)*(data[ii].y-meany)*0.5;

            pmax[0]=std::max((float)data[ii].x,pmax[0]);
            pmax[1]=std::max((float)data[ii].y,pmax[1]);


            pmin[0]=std::min((float)data[ii].x,pmin[0]);
            pmin[1]=std::min((float)data[ii].y,pmin[1]);
         }
        pmax[2]=2;
        pmin[2]=0;


        //Cz=Ax+By+D
        //to...
        //Cz-Ax-By-D=0



        //Cz=Ax+By+D :

        coefs[0]= (l11*r0-l01*r1)/(l00*l11-l01*l01);
        coefs[1]= (l00*r1-l01*r0)/(l00*l11-l01*l01);
        //if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/
        coefs[2]=1;
        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;
        coefs[3]=-coefs[0]*meanx-coefs[1]*meany+coefs[2]*meanz;

        //Cz-Ax-By-D=0 :

        coefs[0]= -coefs[0];
        coefs[1]= -coefs[1];
//        coefs[2]=1;
        coefs[3]=-coefs[3];


        int nonIn=0;
        for(unsigned int ii=0;ii<data.size();ii++){

             if(distance(data[ii])>0.7*th){
                 nonIn++;
             }
        }
        double cut=(double)nonIn/(double)data.size();

        if(cut>0.1){

            valid=false;
        }



        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);


        return valid;
    }




    bool set_old(std::vector<PointType> &data){
        valid=true;
        minmax=true;
        // from https://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
        double meanx=0;
        double meany=0;
        double meanz=0;

        double x_2=0;
        double y_2=0;
        double y_=0;
        double x_=0;
        double xy=0;
        double z_=0;
        double xz=0;
        double yz=0;

        double a,b,c,d,e,f,g,h,i,x,y,z;

        for(unsigned int ii=0;ii<data.size();ii++){
            meanx+=data[ii].x;
            meany+=data[ii].y;
            meanz+=data[ii].z;
        }

        pmax[0]=data[0].x;
        pmax[1]=data[0].y;
        pmax[2]=data[0].z;

        meanx=meanx/(double)data.size();
        meany=meany/(double)data.size();
        meanz=meanz/(double)data.size();


        pmin[0]=data[0].x;
        pmin[1]=data[0].y;
        pmin[2]=data[0].z;
        for(unsigned int ii=0;ii<data.size();ii++){

            x_2+=(data[ii].x-meanx)*(data[ii].x-meanx);
            y_2+=(data[ii].y-meany)*(data[ii].y-meany);
            z_+=(data[ii].z-meanz);
            y_+=(data[ii].y-meany);
            x_+=(data[ii].x-meanx);
            xy+=(data[ii].x-meanx)*(data[ii].y-meany);
            xz+=(data[ii].x-meanx)*(data[ii].z-meanz);
            yz+=(data[ii].z-meanz)*(data[ii].y-meany);

            pmax[0]=std::max(data[ii].x,pmax[0]);
            pmax[1]=std::max(data[ii].y,pmax[1]);
            pmax[2]=std::max(data[i].z,pmax[2]);


            pmin[0]=std::min(data[ii].x,pmin[0]);
            pmin[1]=std::min(data[ii].y,pmin[1]);
            pmin[2]=std::min(data[ii].z,pmin[2]);
        }



        //  from matlab using ...
        //syms a b c d e f g h i x y z;
        //A=[a b c; d e f; g h i];
        //B= [x ;y; z]
        //inv(A)*B

        a=x_2;
        b=xy;
        c=x_;
        d=xy;
        e=y_2;
        f=y_;
        g=x_;
        h=y_;
        i=data.size();

        x=xz;
        y=yz;
        z=z_;

        coefs[0]=(z*(b*f - c*e))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (y*(b*i - c*h))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) + (x*(e*i - f*h))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        coefs[1]=(y*(a*i - c*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (z*(a*f - c*d))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (x*(d*i - f*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        coefs[2]=(z*(a*e - b*d))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) - (y*(a*h - b*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g) + (x*(d*h - e*g))/(a*e*i - a*f*h - b*d*i + b*f*g + c*d*h - c*e*g);
        if (std::abs(coefs[0])<0.001&&std::abs(coefs[1])<0.001&&std::abs(coefs[2])<0.001) valid=false;/*throw std::logic_error("Points are linearly dependent");*/


        double norm=sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        coefs[0]=coefs[0]/norm;
        coefs[1]=coefs[1]/norm;
        coefs[2]=coefs[2]/norm;
        coefs[3]=-(coefs[0]*data[0].x+coefs[1]*data[0].y+coefs[2]*data[0].z);


        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
        return valid;
    }









    inline Plane(double A,double B,double C,double D)	{
        coefs[0]=A;
        coefs[1]=B;
        coefs[2]=C;
        coefs[3]=D;
    }

    inline double evaluatePoint(const PointType &point){
        return coefs[0]*point.x+coefs[1]*point.y+coefs[2]*point.z+coefs[3];
    }

    bool contains(const PointType &point){
        return distance(point)<0.001;
    }
    double distance(const PointType &point){
        return std::fabs(evaluatePoint(point))/std::sqrt(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
    }

    //http://mrpt.sourcearchive.com/documentation/1:0.8.1-1ubuntu2/lightweight__geom__data_8cpp-source.html
    double dot_(PointType point){
        return coefs[0]*point.x+coefs[1]*point.y+coefs[2]*point.z+coefs[3];
    }

    double dotNormal(PointType point){
        return coefs[0]*point.x+coefs[1]*point.y+coefs[2]*point.z;
    }




    void forceComputeNorm(){
        norm=(coefs[0]*coefs[0]+coefs[1]*coefs[1]+coefs[2]*coefs[2]);
    }
    template <typename PT>
    PT getProjectionPoint(PT point){
        //http://www.9math.com/book/projection-point-plane

        PT out;

        double t0=-(coefs[0]*point.x+coefs[1]*point.y+coefs[2]*point.z+coefs[3])/norm;

        out.x=point.x+coefs[0]*t0;
        out.y=point.y+coefs[1]*t0;
        out.z=point.z+coefs[2]*t0;


        return out;
    }


    void getNormalVector(double (&vec)[3]){
        vec[0]=coefs[0];
        vec[1]=coefs[1];
        vec[2]=coefs[2];
    }


    template <typename PT>
    void setNormalVector(double a,double b,double c,PT p1){
        coefs[0]=a;
        coefs[1]=b;
        coefs[2]=c;
        coefs[3]=-coefs[0]*p1.x-coefs[1]*p1.y-coefs[2]*p1.z;

    }

    geometry_msgs::Point32 center(){
        geometry_msgs::Point32 out;
        geometry_msgs::Point   pmin_;
        pmin_.x=pmin[0];
        pmin_.y=pmin[1];
        pmin_.z=pmin[2];

        geometry_msgs::Point   pmax_;

        pmax_.x=pmax[0];
        pmax_.y=pmax[1];
        pmax_.z=pmax[2];
        if(fabs(coefs[1])>fabs(coefs[2]) && fabs(coefs[1])>fabs(coefs[0]) ){
            geometry_msgs::Point   p1;
            p1.x = pmin_.x;
            p1.y = -(coefs[0]*pmin_.x+coefs[2]*pmin_.z+coefs[3])/coefs[1];
            p1.z = pmin_.z;
            geometry_msgs::Point   p2;
            p2.x = pmin_.x;
            p2.y = -(coefs[0]*pmin_.x+coefs[2]*pmax_.z+coefs[3])/coefs[1];
            p2.z = pmax_.z;
            geometry_msgs::Point   p3;
            p3.x = pmax_.x;
            p3.y = -(coefs[0]*pmax_.x+coefs[2]*pmax_.z+coefs[3])/coefs[1];
            p3.z = pmax_.z;
            geometry_msgs::Point   p4;
            p4.x = pmax_.x;
            p4.y = -(coefs[0]*pmax_.x+coefs[2]*pmin_.z+coefs[3])/coefs[1];
            p4.z = pmin_.z;



            out.x=(p1.x+p2.x+p3.x+p4.x)/4.0;
            out.y=(p1.y+p2.y+p3.y+p4.y)/4.0;
            out.z=(p1.z+p2.z+p3.z+p4.z)/4.0;

        }else if(fabs(coefs[2])>fabs(coefs[1]) && fabs(coefs[2])>fabs(coefs[0])){

            geometry_msgs::Point p1;
            p1.x = pmin_.x;
            p1.y = pmin_.y;
            p1.z = -(coefs[0]*pmin_.x+coefs[1]*pmin_.y+coefs[3])/coefs[2];
            geometry_msgs::Point p2;
            p2.x = pmin_.x;
            p2.y = pmax_.y;
            p2.z = -(coefs[0]*pmin_.x+coefs[1]*pmax_.y+coefs[3])/coefs[2];
            geometry_msgs::Point p3;
            p3.x = pmax_.x;
            p3.y = pmax_.y;
            p3.z = -(coefs[0]*pmax_.x+coefs[1]*pmax_.y+coefs[3])/coefs[2];
            geometry_msgs::Point p4;
            p4.x = pmax_.x;
            p4.y = pmin_.y;
            p4.z = -(coefs[0]*pmax_.x+coefs[1]*pmin_.y+coefs[3])/coefs[2];


            out.x=(p1.x+p2.x+p3.x+p4.x)/4.0;
            out.y=(p1.y+p2.y+p3.y+p4.y)/4.0;
            out.z=(p1.z+p2.z+p3.z+p4.z)/4.0;

        }else{

            geometry_msgs::Point p1;
            p1.z = pmin_.z;
            p1.y = pmin_.y;
            p1.x = -(coefs[2]*pmin_.z+coefs[1]*pmin_.y+coefs[3])/coefs[0];
            geometry_msgs::Point p2;
            p2.z = pmin_.z;
            p2.y = pmax_.y;
            p2.x = -(coefs[2]*pmin_.z+coefs[1]*pmax_.y+coefs[3])/coefs[0];
            geometry_msgs::Point p3;
            p3.z = pmax_.z;
            p3.y = pmax_.y;
            p3.x = -(coefs[2]*pmax_.z+coefs[1]*pmax_.y+coefs[3])/coefs[0];
            geometry_msgs::Point p4;
            p4.z = pmax_.z;
            p4.y = pmin_.y;
            p4.x = -(coefs[2]*pmax_.z+coefs[1]*pmin_.y+coefs[3])/coefs[0];

            out.x=(p1.x+p2.x+p3.x+p4.x)/4.0;
            out.y=(p1.y+p2.y+p3.y+p4.y)/4.0;
            out.z=(p1.z+p2.z+p3.z+p4.z)/4.0;
        }

        return out;

    }

    geometry_msgs::Point32 normal(){
        geometry_msgs::Point32 out;
        out.x=coefs[0];
        out.y=coefs[1];
        out.z=coefs[2];
        return out;
    }


    template <typename pointtype>
    std::vector<pointtype> planePoints(){


        std::vector<pointtype> out;


        if(valid==true && minmax==true){


            pointtype   pmin_;
            pmin_.x=pmin[0];
            pmin_.y=pmin[1];
            pmin_.z=pmin[2];

            pointtype   pmax_;

            pmax_.x=pmax[0];
            pmax_.y=pmax[1];
            pmax_.z=pmax[2];

//ver isto da equacao , z=xxxx vs z+xxxx=0;

            if(fabs(coefs[1])>fabs(coefs[2]) && fabs(coefs[1])>fabs(coefs[0]) ){
                pointtype   p1;
                p1.x = pmin_.x;
                p1.y = -(coefs[0]*pmin_.x+coefs[2]*pmin_.z+coefs[3])/coefs[1];
                p1.z = pmin_.z;
                pointtype   p2;
                p2.x = pmin_.x;
                p2.y = -(coefs[0]*pmin_.x+coefs[2]*pmax_.z+coefs[3])/coefs[1];
                p2.z = pmax_.z;
                pointtype   p3;
                p3.x = pmax_.x;
                p3.y = -(coefs[0]*pmax_.x+coefs[2]*pmax_.z+coefs[3])/coefs[1];
                p3.z = pmax_.z;
                pointtype   p4;
                p4.x = pmax_.x;
                p4.y = -(coefs[0]*pmax_.x+coefs[2]*pmin_.z+coefs[3])/coefs[1];
                p4.z = pmin_.z;



                out.push_back(p1);
                out.push_back(p2);
                out.push_back(p3);
                out.push_back(p3);
                out.push_back(p4);
                out.push_back(p1);
                //                Pen  p(Garrote::Graphics::Colors::Green(),1);
                //                p.drawPlan(Point::convert<geometry_msgs::Point>(p1),Point::convert<geometry_msgs::Point>(p2 ),Point::convert<geometry_msgs::Point>(p3 ),Point::convert<geometry_msgs::Point>(p4  ));


            }else if(fabs(coefs[2])>fabs(coefs[1]) && fabs(coefs[2])>fabs(coefs[0])){

                pointtype p1;
                p1.x = pmin_.x;
                p1.y = pmin_.y;
                p1.z = -(coefs[0]*pmin_.x+coefs[1]*pmin_.y+coefs[3])/coefs[2];
                pointtype p2;
                p2.x = pmin_.x;
                p2.y = pmax_.y;
                p2.z = -(coefs[0]*pmin_.x+coefs[1]*pmax_.y+coefs[3])/coefs[2];
                pointtype p3;
                p3.x = pmax_.x;
                p3.y = pmax_.y;
                p3.z = -(coefs[0]*pmax_.x+coefs[1]*pmax_.y+coefs[3])/coefs[2];
                pointtype p4;
                p4.x = pmax_.x;
                p4.y = pmin_.y;
                p4.z = -(coefs[0]*pmax_.x+coefs[1]*pmin_.y+coefs[3])/coefs[2];


                out.push_back(p1);
                out.push_back(p2);
                out.push_back(p3);
                out.push_back(p3);
                out.push_back(p4);
                out.push_back(p1);
                //                Pen  p(Garrote::Graphics::Colors::Green(),1);
                //                p.drawPlan(Point::convert<geometry_msgs::Point>(p1),Point::convert<geometry_msgs::Point>(p2 ),Point::convert<geometry_msgs::Point>(p3 ),Point::convert<geometry_msgs::Point>(p4  ));
            }else{

                pointtype p1;
                p1.z = pmin_.z;
                p1.y = pmin_.y;
                p1.x = -(coefs[2]*pmin_.z+coefs[1]*pmin_.y+coefs[3])/coefs[0];
                pointtype p2;
                p2.z = pmin_.z;
                p2.y = pmax_.y;
                p2.x = -(coefs[2]*pmin_.z+coefs[1]*pmax_.y+coefs[3])/coefs[0];
                pointtype p3;
                p3.z = pmax_.z;
                p3.y = pmax_.y;
                p3.x = -(coefs[2]*pmax_.z+coefs[1]*pmax_.y+coefs[3])/coefs[0];
                pointtype p4;
                p4.z = pmax_.z;
                p4.y = pmin_.y;
                p4.x = -(coefs[2]*pmax_.z+coefs[1]*pmin_.y+coefs[3])/coefs[0];


                out.push_back(p1);
                out.push_back(p2);
                out.push_back(p3);
                out.push_back(p3);
                out.push_back(p4);
                out.push_back(p1);
                //                Pen  p(Garrote::Graphics::Colors::Green(),1);
                //                p.drawPlan(Point::convert<geometry_msgs::Point>(p1),Point::convert<geometry_msgs::Point>(p2 ),Point::convert<geometry_msgs::Point>(p3 ),Point::convert<geometry_msgs::Point>(p4  ));
            }


        }


        return out;
    }



    void paint(){

        if(valid==true && minmax==true){


            geometry_msgs::Point   pmin_;
            pmin_.x=pmin[0];
            pmin_.y=pmin[1];
            pmin_.z=pmin[2];

            geometry_msgs::Point   pmax_;

            pmax_.x=pmax[0];
            pmax_.y=pmax[1];
            pmax_.z=pmax[2];



            if(fabs(coefs[1])>fabs(coefs[2]) && fabs(coefs[1])>fabs(coefs[0]) ){
                geometry_msgs::Point   p1;
                p1.x = pmin_.x;
                p1.y = -(coefs[0]*pmin_.x+coefs[2]*pmin_.z+coefs[3])/coefs[1];
                p1.z = pmin_.z;
                geometry_msgs::Point   p2;
                p2.x = pmin_.x;
                p2.y = -(coefs[0]*pmin_.x+coefs[2]*pmax_.z+coefs[3])/coefs[1];
                p2.z = pmax_.z;
                geometry_msgs::Point   p3;
                p3.x = pmax_.x;
                p3.y = -(coefs[0]*pmax_.x+coefs[2]*pmax_.z+coefs[3])/coefs[1];
                p3.z = pmax_.z;
                geometry_msgs::Point   p4;
                p4.x = pmax_.x;
                p4.y = -(coefs[0]*pmax_.x+coefs[2]*pmin_.z+coefs[3])/coefs[1];
                p4.z = pmin_.z;




               // Pen  p(Garrote::Graphics::Colors::Green(),1);
               // p.drawPlan(Point::convert<geometry_msgs::Point>(p1),Point::convert<geometry_msgs::Point>(p2 ),Point::convert<geometry_msgs::Point>(p3 ),Point::convert<geometry_msgs::Point>(p4  ));


            }else if(fabs(coefs[2])>fabs(coefs[1]) && fabs(coefs[2])>fabs(coefs[0])){

                geometry_msgs::Point p1;
                p1.x = pmin_.x;
                p1.y = pmin_.y;
                p1.z = -(coefs[0]*pmin_.x+coefs[1]*pmin_.y+coefs[3])/coefs[2];
                geometry_msgs::Point p2;
                p2.x = pmin_.x;
                p2.y = pmax_.y;
                p2.z = -(coefs[0]*pmin_.x+coefs[1]*pmax_.y+coefs[3])/coefs[2];
                geometry_msgs::Point p3;
                p3.x = pmax_.x;
                p3.y = pmax_.y;
                p3.z = -(coefs[0]*pmax_.x+coefs[1]*pmax_.y+coefs[3])/coefs[2];
                geometry_msgs::Point p4;
                p4.x = pmax_.x;
                p4.y = pmin_.y;
                p4.z = -(coefs[0]*pmax_.x+coefs[1]*pmin_.y+coefs[3])/coefs[2];



                //Pen  p(Garrote::Graphics::Colors::Green(),1);
                //p.drawPlan(Point::convert<geometry_msgs::Point>(p1),Point::convert<geometry_msgs::Point>(p2 ),Point::convert<geometry_msgs::Point>(p3 ),Point::convert<geometry_msgs::Point>(p4  ));
            }else{

                geometry_msgs::Point p1;
                p1.z = pmin_.z;
                p1.y = pmin_.y;
                p1.x = -(coefs[2]*pmin_.z+coefs[1]*pmin_.y+coefs[3])/coefs[0];
                geometry_msgs::Point p2;
                p2.z = pmin_.z;
                p2.y = pmax_.y;
                p2.x = -(coefs[2]*pmin_.z+coefs[1]*pmax_.y+coefs[3])/coefs[0];
                geometry_msgs::Point p3;
                p3.z = pmax_.z;
                p3.y = pmax_.y;
                p3.x = -(coefs[2]*pmax_.z+coefs[1]*pmax_.y+coefs[3])/coefs[0];
                geometry_msgs::Point p4;
                p4.z = pmax_.z;
                p4.y = pmin_.y;
                p4.x = -(coefs[2]*pmax_.z+coefs[1]*pmin_.y+coefs[3])/coefs[0];



                //Pen  p(Garrote::Graphics::Colors::Green(),1);
                //p.drawPlan(Point::convert<geometry_msgs::Point>(p1),Point::convert<geometry_msgs::Point>(p2 ),Point::convert<geometry_msgs::Point>(p3 ),Point::convert<geometry_msgs::Point>(p4  ));
            }



            //            if(fabs(coefs[2])<0.001){

            //                Point p1(pmin[0],-(coefs[0]*pmin[0]+coefs[2]*pmin[2]+coefs[3])/coefs[1],pmin[2]);
            //                Point p2(pmin[0],-(coefs[0]*pmin[0]+coefs[2]*pmax[2]+coefs[3])/coefs[1],pmax[2]);
            //                Point p3(pmax[0],-(coefs[0]*pmax[0]+coefs[2]*pmax[2]+coefs[3])/coefs[1],pmax[2]);
            //                Point p4(pmax[0],-(coefs[0]*pmax[0]+coefs[2]*pmin[2]+coefs[3])/coefs[1],pmin[2]);

            //                Pen  p(Garrote::Graphics::Colors::Red(),1);
            //                p.drawPlan(p1,p2,p3,p4);

            //            }else{
            //                Point p1(pmin[0],pmin[1],-(coefs[0]*pmin[0]+coefs[1]*pmin[1]+coefs[3])/coefs[2]);
            //                Point p2(pmin[0],pmax[1],-(coefs[0]*pmin[0]+coefs[1]*pmax[1]+coefs[3])/coefs[2]);
            //                Point p3(pmax[0],pmax[1],-(coefs[0]*pmax[0]+coefs[1]*pmax[1]+coefs[3])/coefs[2]);
            //                Point p4(pmax[0],pmin[1],-(coefs[0]*pmax[0]+coefs[1]*pmin[1]+coefs[3])/coefs[2]);
            //                Pen  p(Garrote::Graphics::Colors::Green(),1);
            //                p.drawPlan(p1,p2,p3,p4);
            //            }
        }
    }
    std::string toString(){
        std::stringstream str;
        str<<coefs[0]<<" "<<coefs[1]<<" "<<coefs[2]<<" "<<coefs[3];
        return str.str();
    }
    void unit(){

    }

};


//class Line{
//    //    Point point1;
//    //    Point point2;

//public:

//        double distance(const TPoint3D &point) const;

//};

template <typename T,typename D,typename F>
class RANSACModel{


public:
    double score=9999999;
    unsigned int numpoints=1;
    D model;
    std::vector<unsigned int> inliers;

    RANSACModel(){

    }
    virtual ~RANSACModel(){

    }

    virtual  unsigned int eval(T &data,double dist)=0;
    virtual  unsigned int eval(T &data,double dist,std::vector<unsigned int> &sample,std::vector<unsigned int> &inliers)=0;
    virtual  bool isValid(T &data,std::vector<unsigned int> &indexes)=0;
    virtual  bool fit(T &data,std::vector<unsigned int> &indexes)=0;
    virtual void boundaries(T &data,double dist)=0;
    virtual RANSACModel<T,D,F> *clone()=0;
    virtual std::string toString()=0;

    virtual  F nearestInlier(F p,T &data)=0;

    virtual  F nearestInlier(T &data)=0;
};



template <typename T,typename D,typename F>
class RANSACModelPlane : public RANSACModel<T,D,F>{
public:


    RANSACModelPlane() : RANSACModel<T,D,F>(){

        this->numpoints=3;
    }
    virtual std::string toString(){
        return $("{} {} {} {}  [{},{}] [{},{}] [{},{}]").format(this->model.coefs[0],this->model.coefs[1],this->model.coefs[2],this->model.coefs[3],this->model.pmin[0],this->model.pmax[0],this->model.pmin[1],this->model.pmax[1],this->model.pmin[2],this->model.pmax[2]);
    }

    virtual  unsigned int eval(T &data,double dist,std::vector<unsigned int> &sample,std::vector<unsigned int> &inliers){

        unsigned int ct=0;
        if(this->model.valid){
            this->score=0;
            for(unsigned int i=0;i<data.size();i++){
                //if(sample[0]!=i && sample[1]!=i && sample[2]!=i){
                double d=this->model.distance(data[i]);
                this->score+=d*d;
                if(d<dist){
                    inliers[ct]=i;
                    ct++;
                }
                // }
            }

            this->score=std::sqrt(this->score/(double)data.size()); //rms

        }else{
            this-> score=9999999;
        }


        return ct;
    }


    virtual  unsigned int eval(T &data,double dist){

        unsigned int ct=0;
        if(this->model.valid){
            this->score=0;
            for(unsigned int i=0;i<data.size();i++){
                //if(sample[0]!=i && sample[1]!=i && sample[2]!=i){
                double d=this->model.distance(data[i]);
                this->score+=d*d;
                (d<dist)?ct++:ct;

                // }
            }

            this->score=std::sqrt(this->score/(double)data.size()); //rms

        }else{
            this-> score=9999999;
        }


        return ct;
    }


    virtual  F nearestInlier(F p,T &data){

        F out;
        double mind=9999999;
        unsigned int i=0;
        if(this->model.valid ){ //&& data.size()==this->inliers.size()
            for(unsigned int f=0;f<this->inliers.size();f++){
                i=this->inliers[f];
                double d=sqrt((data[i].x-p.x)*(data[i].x-p.x)+(data[i].y-p.y)*(data[i].y-p.y)+(data[i].z-p.z)*(data[i].z-p.z));
                if(d<mind){
                    mind=d;
                    out=data[i];
                }
            }
        }
        return out;
    }

    virtual  F nearestInlier(T &data){

        F out;
        double mind=9999999;
        if(this->model.valid){
            for(unsigned int i=0;i<data.size();i++){
                double d=this->model.distance(data[i]);
                if(d<mind){
                    mind=d;
                    out=data[i];
                }
            }
        }
        return out;
    }

    virtual bool valid(){

        return this->model.valid;
    }
    virtual  bool isValid(T &data,std::vector<unsigned int> &indexes){
        return true;
    }
    virtual  bool fit(T &data,std::vector<unsigned int> &indexes){

        if(indexes.size()!=3){


            //            // Compute the segment values (in 3d) between p1 and p0
            //               p1p0 = p1 - p0;
            //              // Compute the segment values (in 3d) between p2 and p0
            //               p2p0 = p2 - p0;

            //              // Avoid some crashes by checking for collinearity here
            //              Eigen::Array4f dy1dy2 = p1p0 / p2p0;
            //              if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) )          // Check for collinearity
            //                return (false);

            return this->model.set(data,indexes);
        }else{
            //            T p0=data[indexes[0]];
            //            T p1=data[indexes[1]];
            //            T p2=data[indexes[2]];


            //Dx1 := x2 - x1;
            // Dy1 := y2 - y1;
            // Dz1 := z2 - z1;

            // Dx2 := x3 - x1;
            // Dy2 := y3 - y1;
            // Dz2 := z3 - z1;

            // {perform a 3d cross product}
            // Cx := Dy1 * Dz2 - Dy2 * Dz1;
            // Cy := Dx2 * Dz1 - Dx1 * Dz2;
            // Cz := Dx1 * Dy2 - Dx2 * Dy1;

            // Result := IsEqual(Cx * Cx + Cy * Cy + Cz * Cz, 0.0);

            return this->model.set(data[indexes[0]],data[indexes[1]],data[indexes[2]]);

        }
        return true;
    }

    virtual void boundaries(T &data,double dist){

        if(this->inliers.size()>0){

            this->model.pmax[0]=data[this->inliers[0]].x;
            this->model.pmax[1]=data[this->inliers[0]].y;
            this->model.pmax[2]=data[this->inliers[0]].z;


            this->model.pmin[0]=data[this->inliers[0]].x;
            this->model.pmin[1]=data[this->inliers[0]].y;
            this->model.pmin[2]=data[this->inliers[0]].z;

            for(unsigned int i=1;i<this->inliers.size();i++){


                this->model.pmax[0]=std::max((float)data[this->inliers[i]].x,this->model.pmax[0]);
                this->model.pmax[1]=std::max((float)data[this->inliers[i]].y,this->model.pmax[1]);
                this->model.pmax[2]=std::max((float)data[this->inliers[i]].z,this->model.pmax[2]);


                this->model.pmin[0]=std::min((float)data[this->inliers[i]].x,this->model.pmin[0]);
                this->model.pmin[1]=std::min((float)data[this->inliers[i]].y,this->model.pmin[1]);
                this->model.pmin[2]=std::min((float)data[this->inliers[i]].z,this->model.pmin[2]);



            }




            this->model.minmax=true;
        }else if(data.size()>0){

            if(this->model.valid){

                int xc=0;

                for(unsigned int i=0;i<data.size();i++){
                    //if(sample[0]!=i && sample[1]!=i && sample[2]!=i){
                    double d=this->model.distance(data[i]);
                    if(d<=dist){
                        this->inliers.push_back(i);

                        if(xc==0){
                            this->model.pmax[0]=data[i].x;
                            this->model.pmax[1]=data[i].y;
                            this->model.pmax[2]=data[i].z;


                            this->model.pmin[0]=data[i].x;
                            this->model.pmin[1]=data[i].y;
                            this->model.pmin[2]=data[i].z;

                            xc=1;
                        }else{
                            this->model.pmax[0]=std::max((float)data[i].x,this->model.pmax[0]);
                            this->model.pmax[1]=std::max((float)data[i].y,this->model.pmax[1]);
                            this->model.pmax[2]=std::max((float)data[i].z,this->model.pmax[2]);


                            this->model.pmin[0]=std::min((float)data[i].x,this->model.pmin[0]);
                            this->model.pmin[1]=std::min((float)data[i].y,this->model.pmin[1]);
                            this->model.pmin[2]=std::min((float)data[i].z,this->model.pmin[2]);
                        }

                    }

                }

            }else{

            }

            this->model.minmax=true;

        }
    }
    virtual RANSACModel<T,D,F> *clone(){

        RANSACModel<T,D,F> *ptr=new RANSACModelPlane<T,D,F>();
        ptr->model=this->model;
        ptr->score=this->score;
        return ptr;
    }




};


//class RANSACModelPlane : public RANSACModel<T,Plane>{
//public:
//    virtual  double eval(T &data,double dist,std::vector<unsigned int> &sample,std::vector<unsigned int> &inliers)=0;
//    virtual  bool isValid(T &data,std::vector<unsigned int> &indexes){
//        return true;
//    }
//    virtual  bool fit(T &data,std::vector<unsigned int> &indexes)=0;
//};



template <typename T,typename D,typename F>
class RANSAC{

    /*

    data – a set of observed data points
    model – a model that can be fitted to data points
    n – the minimum number of data values required to fit the model
    k – the maximum number of iterations allowed in the algorithm
    t – a threshold value for determining when a data point fits a model
    d – the number of close data values required to assert that a model fits well to data

     */
public:

    static void fit(T &data,RANSACModel<T,D,F> *model,unsigned int n,unsigned int k,double t,unsigned int d){



        RANSACModel<T,D,F> *bestfit=model->clone();
        Random random(0,data.size());
        unsigned int iterations = 0;
        std::vector<unsigned int> sampledindexes( n );
        //        std::vector<unsigned int> inlierindexes(data.size());

        while (iterations < k) {
            //            Garrote::Time::start($("{} iterations").format(iterations));

            //            inlierindexes.clear();
            random.draw(sampledindexes);
            unsigned int counted=0;
            if(model->isValid(data,sampledindexes)){
                model->fit(data,sampledindexes);
                //                counted= model->eval(data,t,sampledindexes,inlierindexes);
                counted= model->eval(data,t);

                //                for(auto i=0;i<sampledindexes.size();i++){
                //                    inlierindexes.erase(std::remove(inlierindexes.begin(), inlierindexes.end(), sampledindexes[i]), inlierindexes.end());
                //                }
            }


            if(counted>d){

                //                inlierindexes.insert(inlierindexes.end(),sampledindexes.begin(), sampledindexes.end());
                //                model->fit(data,inlierindexes);


                if(bestfit->score>model->score){

                    bestfit->model=model->model;
                    bestfit->score=model->score;
                    //                    bestfit->inliers.clear();
                    //                    bestfit->inliers=(inlierindexes);
                    //                    bestfit->inliers.erase(bestfit->inliers.begin()+counted,bestfit->inliers.end());
                    //                    //.insert(bestfit->inliers.begin(),inlierindexes.begin(),inlierindexes.begin()+counted);


                }
            }

            //            Garrote::Time::show($("{} iterations").format(iterations));

            iterations++;
        }

        model->model= bestfit->model;
        model->score=bestfit->score;
        model->inliers=std::move(bestfit->inliers);
        model->boundaries(data,t);
        //        std::cout<<model->toString()<<std::endl;
        delete bestfit;

    }


    static void fit(T &data,RANSACModel<T,D,F> *model,unsigned int max_iterations_,double threshold_){



        RANSACModel<T,D,F> *bestfit=model->clone();
        Random random(0,data.size());
        //        unsigned int iterations = 0;
        double probability_ =(0.99);
        std::vector<unsigned int> sampledindexes( model->numpoints );

        unsigned int iterations_ = 0;
        int n_best_inliers_count = -INT_MAX;
        double k = 1.0;
        //        std::vector<int> selection;
        double log_probability  = log (1.0 - probability_);
        double one_over_indices = 1.0 / static_cast<double> (data.size());//sac_model_->getIndices ()->size ());
        int n_inliers_count = 0;
        unsigned skipped_count = 0;
        const unsigned max_skip = max_iterations_ * 10;

        while (iterations_ < k && skipped_count < max_skip)
        {

            random.draw(sampledindexes);
            if(! model->fit(data,sampledindexes)){
                ++skipped_count;
                continue;
            }
            n_inliers_count= model->eval(data,threshold_);

            if(n_inliers_count>n_best_inliers_count){
                n_best_inliers_count = n_inliers_count;

                //                inlierindexes.insert(inlierindexes.end(),sampledindexes.begin(), sampledindexes.end());
                //                model->fit(data,inlierindexes);



                bestfit->model=model->model;
                bestfit->score=model->score;
                //                    bestfit->inliers.clear();
                //                    bestfit->inliers=(inlierindexes);
                //                    bestfit->inliers.erase(bestfit->inliers.begin()+counted,bestfit->inliers.end());
                //                    //.insert(bestfit->inliers.begin(),inlierindexes.begin(),inlierindexes.begin()+counted);
                double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
                double p_no_outliers = 1.0 - pow (w, static_cast<double> (model->numpoints));
                p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
                p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
                k = log_probability / log (p_no_outliers);

            }

            //            Garrote::Time::show($("{} iterations").format(iterations));

            iterations_++;

            if (iterations_ > max_iterations_)
            {

                break;
            }
        }

        model->model= bestfit->model;
        model->score=bestfit->score;
        model->inliers=std::move(bestfit->inliers);
        model->boundaries(data,threshold_);
        //        std::cout<<model->toString()<<std::endl;
        delete bestfit;

    }

//    static void fitX(T &data,RANSACModel<T,D,F> *model,unsigned int max_iterations_,double threshold_){



//        RANSACModel<T,D,F> *bestfit=model->clone();
//        Random random(0,data.size());
//        //        unsigned int iterations = 0;
//        double probability_ =(0.99);
//        std::vector<unsigned int> sampledindexes( model->numpoints );

//        unsigned int iterations_ = 0;
//        int n_best_inliers_count = -INT_MAX;
//        double k = 1.0;
//        //        std::vector<int> selection;
//        double log_probability  = log (1.0 - probability_);
//        double one_over_indices = 1.0 / static_cast<double> (data.size());//sac_model_->getIndices ()->size ());
//        int n_inliers_count = 0;
//        unsigned skipped_count = 0;
//        const unsigned max_skip = max_iterations_ * 10;

//        while (iterations_ < k && skipped_count < max_skip)
//        {

//            random.draw(sampledindexes);
//            if(! model->fit(data,sampledindexes)){
//                ++skipped_count;
//                continue;
//            }
//            n_inliers_count= model->eval(data,threshold_);

//            if(n_inliers_count>n_best_inliers_count){
//                n_best_inliers_count = n_inliers_count;

//                //                inlierindexes.insert(inlierindexes.end(),sampledindexes.begin(), sampledindexes.end());
//                //                model->fit(data,inlierindexes);



//                bestfit->model=model->model;
//                bestfit->score=model->score;
//                //                    bestfit->inliers.clear();
//                //                    bestfit->inliers=(inlierindexes);
//                //                    bestfit->inliers.erase(bestfit->inliers.begin()+counted,bestfit->inliers.end());
//                //                    //.insert(bestfit->inliers.begin(),inlierindexes.begin(),inlierindexes.begin()+counted);
//                double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
//                double p_no_outliers = 1.0 - pow (w, static_cast<double> (model->numpoints));
//                p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
//                p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
//                k = log_probability / log (p_no_outliers);

//            }

//            //            Garrote::Time::show($("{} iterations").format(iterations));

//            iterations_++;

//            if (iterations_ > max_iterations_)
//            {

//                break;
//            }
//        }

//        model->model= bestfit->model;
//        model->score=bestfit->score;
//        model->inliers=std::move(bestfit->inliers);
//        model->boundaries(data,threshold_);
//        //        std::cout<<model->toString()<<std::endl;
//        delete bestfit;

//    }




};




#endif // RANSAC_H


