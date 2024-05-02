#ifndef LOCALIZATION3D_H
#define LOCALIZATION3D_H



#include "SequentialMonteCarlo.h"
#include <geometry_msgs/Pose.h>
#include "NumericArray.h"
#include "PoseTools.h"
#include "Transform.h"
#include "Grid3D.h" //novo
#include "Map.h"
#include "AngularHistogram.h"
#include <random>
#include <atomic>
#include <mutex>
#include <random>
#include "Gaussian.h"
#include "KeyPointMap.h"
#include "spherecontainer.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <omp.h>

namespace pcl
{
template<>
struct SIFTKeypointFieldSelector<PointXYZ>
{
    inline float
    operator () (const PointXYZ &p) const
    {
        return p.y;
    }
};
}

namespace Garrote {
namespace Localization3D {
namespace Map3D {




class LevenbergMarquardtGrid3D{


    Eigen::Matrix3d H;
    Eigen::Matrix3d diag;
    Eigen::Vector3d dTr;




    // http://fourier.eng.hmc.edu/e176/lectures/NM/node36.html
public:
    double lamb;
    bool gaussnewton=false;
    bool enhancedMode=false;
    bool extraMode=false;

    double factor1=0.1; //0.1
    double factor2=0.4; //0.4 0.05

    double alpha1=10;  //10 1 teste2 1 (teste 3 1)

    double alpha2=10;  //10 1 teste2 1 (teste 3 1)
    double alpha3=10;  //10 1 teste2 1 (teste 3 1)

    double cutout=5.0;  //5.0
    double sigmaX=0.05; //0.05

    double error=999; //999
    double lasterror=999; //999



    LevenbergMarquardtGrid3D(){

        H = Eigen::Matrix3d::Zero();
        diag=Eigen::Matrix3d::Zero();
        dTr =Eigen::Vector3d::Zero();
        lamb=50;

    }

    void reset(){
        H = Eigen::Matrix3d::Zero();
        dTr = Eigen::Vector3d::Zero();
    }
    double clamp(double f,double m){

        if(f>m){
            return m;
        }
        if(f<-m){
            return -m;
        }
        return f;
    }



    double minabs(double val){

        //std::cout<<val<<std::endl;

        double v1=fabs(val);
        double v2=fabs(val+2.0*M_PI);
        double v3=fabs(val-2.0*M_PI);

        if(v1<=v2 && v1<=v3){
            return val;
        }
        if(v2<=v1 && v2<=v3){
            return val+2.0*M_PI;
        }
        return val-2.0*M_PI;

    }

template <typename TT>
    bool iterate(NumericArray<double,3> &estimate,std::vector<TT> &data,Grid3D<char> *map,int size){
        std::array<float, 4> td;
        Transform tf;
        tf.applyRotationZ(estimate[2]);
        tf.applyTranslation(estimate[0],estimate[1],0);

        H = Eigen::Matrix3d::Zero();
        diag=Eigen::Matrix3d::Zero();
        dTr =Eigen::Vector3d::Zero();


        //Idea
        //https://en.wikipedia.org/wiki/Normal_distribution#Definition
        //https://en.wikipedia.org/wiki/Kernel_density_estimation

        //        count=0;

        double yaw= estimate[2];

      
        double sinRot = sin(yaw);
        double cosRot = cos(yaw);
        //        error=0;

        TT pi;
        std::vector<double> trig;
        trig.push_back(sinRot);
        trig.push_back(cosRot);
        error=0;

        for(unsigned int i=0;i<data.size();i++){


            Point p=tf.point(data[i].x,data[i].y,data[i].z);

            pi=data[i];
            pi.x=p.x;
            pi.y=p.y;
            pi.z=p.z;
            //std::cout<<"x "<<p.x<<"y "<<p.y<<"z "<<pi.z<<std::endl;






                auto o= map-> nearest(pi.x,pi.y,pi.z,size, size,10); //aqui 3, 1,10


                if(o.first){

                    auto b=o.second;
                    double x=data[i].x;
                    double y=data[i].y;
                    double tx=estimate[0];
                    double ty=estimate[1];

                    float dx=(p.x-b.x);
                    float dy=(p.y-b.y);

                    



                    //std::cout<<(int)b.count<<" bx "<<b.x<<"by "<<b.y<<"bz "<<b.z<<std::endl;

                    


                    td[0]=  2.0*dx;
                    td[1]=   2.0*dy;
                    td[2]=  minabs(2.0*((x*trig[1] - y*trig[0])*(ty - b.y + y*trig[1] + x*trig[0]) - (y*trig[1] + x*trig[0])*(tx - b.x + x*trig[1] - y*trig[0])));
                    td[3]=   sqrt(dx*dx + dy*dy);



                    error+=td[3];



                    float funVal = td[3];
                    // right in page 157  "A Flexible and Scalable SLAM System with Full 3D Motion Estimation"

                    dTr[0] += td[0] * funVal;
                    dTr[1] += td[1] * funVal;
                    dTr[2] += td[2] * funVal;

                    H(0, 0) += td[0]*td[0];
                    H(1, 1) += td[1]*td[1];
                    H(2, 2) += td[2]*td[2];

                    H(0, 1) += td[0] * td[1];
                    H(0, 2) += td[0] * td[2];
                    H(1, 2) += td[1] * td[2];
                }else{
                    error+=1;

                }


        }

        H(1, 0) = H(0, 1);
        H(2, 0) = H(0, 2);
        H(2, 1) = H(1, 2);
        //        std::cout<<"->"<<fabs(H(0, 0)) <<" "<<fabs(H(1, 1)) <<" "<<std::endl;

        if ((fabs(H(0, 0)) > 0.00000001f) && (fabs(H(1, 1)) > 0.00000001f)) {




            diag(0,0)=H(0,0);
            diag(1,1)=H(1,1);
            diag(2,2)=H(2,2);



            Eigen::Vector3d  searchDir;

            if(!gaussnewton){
                searchDir= -((H+lamb*diag).inverse() * dTr);
            }else{
                searchDir= -((H).inverse() * dTr);
            }


            if(error>lasterror){
                lamb=lamb*1.01;
            }else{
                lamb=lamb*0.99;
            }


            lasterror=error;

            double a=(alpha1*searchDir[0]);
            double b=(alpha2*searchDir[1]);
            double c=(alpha3*searchDir[2]);

            //std::cout<<a<<" "<<b<<" "<<c<<std::endl;
            estimate[0]+=a;
            estimate[1]+=b;
            estimate[2]+=c;

            if(sqrt(a*a+b*b+c*c)<0.00001){
                return false;
            }

            return true;
        }
        return false;
    }


template <typename TT>
    bool iterate(NumericArray<double,3> &estimate,std::vector<TT> &data,Grid3D<char> *map,int size,double d){
        std::array<float, 4> td;
        Transform tf;
        tf.applyRotationZ(estimate[2]);
        tf.applyTranslation(estimate[0],estimate[1],0);

        H = Eigen::Matrix3d::Zero();
        diag=Eigen::Matrix3d::Zero();
        dTr =Eigen::Vector3d::Zero();


        //Idea
        //https://en.wikipedia.org/wiki/Normal_distribution#Definition
        //https://en.wikipedia.org/wiki/Kernel_density_estimation

        //        count=0;

        double yaw= estimate[2];

      
        double sinRot = sin(yaw);
        double cosRot = cos(yaw);
        //        error=0;

        TT pi;
        std::vector<double> trig;
        trig.push_back(sinRot);
        trig.push_back(cosRot);
        error=0;

        double squared=d*d;

        for(unsigned int i=0;i<data.size();i++){


            if((data[i].x*data[i].x+data[i].y*data[i].y+data[i].z*data[i].z)>squared){
              continue;
               }


            Point p=tf.point(data[i].x,data[i].y,data[i].z);

            pi=data[i];
            pi.x=p.x;
            pi.y=p.y;
            pi.z=p.z;






                auto o= map-> nearest(pi.x,pi.y,pi.z,size, size,size); //aqui 3, 1,10


                if(o.first){

                    auto b=o.second;
                    double x=data[i].x;
                    double y=data[i].y;
                    double tx=estimate[0];
                    double ty=estimate[1];

                    float dx=(p.x-b.x);
                    float dy=(p.y-b.y);


                    if(fabs(dx)<map->cellsizeX/2.0){
                      dx=0;

                     }
                    

                     if(fabs(dy)<map->cellsizeY/2.0){
                      dy=0;

                     }

                    //std::cout<<(int)b.count<<" bx "<<b.x<<"by "<<b.y<<"bz "<<b.z<<std::endl;

                    


                    td[0]=  2.0*dx;
                    td[1]=   2.0*dy;
                    td[2]=  minabs(2.0*((x*trig[1] - y*trig[0])*(ty - b.y + y*trig[1] + x*trig[0]) - (y*trig[1] + x*trig[0])*(tx - b.x + x*trig[1] - y*trig[0])));
                    td[3]=   sqrt(dx*dx + dy*dy);



                    error+=td[3];



                    float funVal = td[3];
                    // right in page 157  "A Flexible and Scalable SLAM System with Full 3D Motion Estimation"

                    dTr[0] += td[0] * funVal;
                    dTr[1] += td[1] * funVal;
                    dTr[2] += td[2] * funVal;

                    H(0, 0) += td[0]*td[0];
                    H(1, 1) += td[1]*td[1];
                    H(2, 2) += td[2]*td[2];

                    H(0, 1) += td[0] * td[1];
                    H(0, 2) += td[0] * td[2];
                    H(1, 2) += td[1] * td[2];
                }else{
                    error+=1;

                }


        }

        H(1, 0) = H(0, 1);
        H(2, 0) = H(0, 2);
        H(2, 1) = H(1, 2);
        //        std::cout<<"->"<<fabs(H(0, 0)) <<" "<<fabs(H(1, 1)) <<" "<<std::endl;

        if ((fabs(H(0, 0)) > 0.00000001f) && (fabs(H(1, 1)) > 0.00000001f)) {




            diag(0,0)=H(0,0);
            diag(1,1)=H(1,1);
            diag(2,2)=H(2,2);



            Eigen::Vector3d  searchDir;

            if(!gaussnewton){
                searchDir= -((H+lamb*diag).inverse() * dTr);
            }else{
                searchDir= -((H).inverse() * dTr);
            }


            if(error>lasterror){
                lamb=lamb*1.01;
            }else{
                lamb=lamb*0.99;
            }


            lasterror=error;

            double a=(alpha1*searchDir[0]);
            double b=(alpha2*searchDir[1]);
            double c=(alpha3*searchDir[2]);

            //std::cout<<a<<" "<<b<<" "<<c<<std::endl;
            estimate[0]+=a;
            estimate[1]+=b;
            estimate[2]+=c;

            if(sqrt(a*a+b*b+c*c)<0.00001){
                return false;
            }

            return true;
        }
        return false;
    }




public:


};




class KullbackLeiblerDivergence{

public:
    int maxParticles=5555;
    int minParticles=600;
    double zeta=0.99;
    double epsilon=0.01;


    std::pair<int,int> xx;
    std::pair<int,int> yy;

    KullbackLeiblerDivergence(){
        xx.first=99999;
        xx.second=0;
        yy.first=99999;
        yy.second=0;
    }


    Map<AngularHistogram<unsigned char> *> *map;
    // we need to create bins for 3 dimentions..
    // if we allocate a map and a ROI... it should be faster than Kd-tree
    

    bool inEmptyBin(geometry_msgs::Pose &ps){
        int x;
        int y;
        AngularHistogram<unsigned char> * ph=map->getXY(ps.position.x,ps.position.y,x,y);
        if(ph==0 && (x>=0 && y>=0)){

            ph=new AngularHistogram<unsigned char> (0.1308995,0.1308995/2.0);       //0.261799 0.261799/2
            map->setXY(ps.position.x,ps.position.y,ph);
            double yaw=PoseTools::getYaw(ps);
            ph->add(yaw);
            map->unprotected_setIndexXY(x,y,ph);

            xx.first=std::min(x,xx.first);
            xx.second=std::max(x,xx.second);

            yy.first=std::min(y,yy.first);
            yy.second=std::max(y,yy.second);

            return true;
        }

        if(ph==0){
             return true;
        }

        double yaw=PoseTools::getYaw(ps);
        bool test= ph->get(yaw)==0;

        if(test){
         ph->add(yaw);
         }  

         if((x>=0 && y>=0)){
            xx.first=std::min(x,xx.first);
            xx.second=std::max(x,xx.second);

            yy.first=std::min(y,yy.first);
            yy.second=std::max(y,yy.second);
            }  
         return test;      
    }

    void set(geometry_msgs::Pose &ps){
       // AngularHistogram<unsigned char> * ph=map->getXY(ps.position.x,ps.position.y);
       // if(ph==0){
            //            ph=new AngularHistogram<unsigned char> (0.06544975,0.032724875);
       //     ph=new AngularHistogram<unsigned char> (0.1308995,0.1308995/2.0);       //0.261799 0.261799/2
       //     map->setXY(ps.position.x,ps.position.y,ph);
       // }
       // double yaw=PoseTools::getYaw(ps);
       // ph->add(yaw);

    }


    void reset(){



       //std::cout<<xx.first<<" "<<xx.second<<" "<<yy.first<<" "<<yy.second<<" "<<std::endl;
        for(unsigned int i=xx.first;i<=xx.second;i++){
            for(unsigned int j=yy.first;j<=yy.second;j++){

                auto g=map->unprotected_getIndexXY(i,j);

                if(g!=0){
                    g->reset();
                }
            }

        }


 
        /* for(unsigned int i=0;i<map->getColumns();i++){
            for(unsigned int j=0;j<map->getRows();j++){

                auto g=map->unprotected_getIndexXY(i,j);

                if(g!=0){
                    g->reset();
                }
            }

        }
        */

 
        xx.first=99999;
        xx.second=0;
        yy.first=99999;
        yy.second=0;

    }


    ~KullbackLeiblerDivergence(){

        for(unsigned int i=0;i<map->getColumns();i++){
            for(unsigned int j=0;j<map->getRows();j++){

                auto g=map->unprotected_getIndexXY(i,j);

                if(g!=0){
                    delete g;
                }
            }

        }

    }

};

template <typename T>
class SamplingScheme{

    std::vector<Particle<T>> *p_set;
    std::uniform_real_distribution<double> uniform;
    std::default_random_engine gen;
    std::vector<double> dist;
   



public:

    SamplingScheme():uniform(0.0,1.0){




    }

    void prepare(std::vector<Particle<T>> *pc){
        p_set=pc;
        double wc=0;
        dist.resize(pc->size(),0);


        for(unsigned int i=0;i<pc->size();i++){

            //            if(pc->operator [](i).w>0.0){
            wc+=pc->operator[](i).w;
            dist[i]=(wc);
            //            }
        }
    }

    //https://github.com/mjl/particle_filter_demo/blob/master/particle_filter.py
    //https://arxiv.org/pdf/1202.6163.pdf
    //    http://www.crest.fr/ckfinder/userfiles/files/Pageperso/chopin/AOS234.pdf
    // https://arxiv.org/pdf/1202.6163.pdf
    //https://xianblog.wordpress.com/tag/stratified-resampling/
    //http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1521264
    //https://pdfs.semanticscholar.org/e694/5926fa11d2f45dd1eadecb8d5d3b76d2d869.pdf
    //https://arxiv.org/pdf/cs/0507025.pdf
    //http://sait.cie.put.poznan.pl/38/SAIT_38_02.pdf
    //http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.81.8370&rep=rep1&type=pdf
    Particle<T> &pick(){


        // Multinomial Resampling

        double disp=uniform(gen)+0.001;


        int index=0;
        for(unsigned int i=0;i<dist.size();i++){

            if(dist[i]>disp){
                index=i;
                break;
            }
        }

        if(index<0){
            index=0;
        }

        return p_set->operator[](index);

    }
};

class SensorConfigs{

public:
    bool use=true;
    bool isGlobal=false;
    bool updated=false;
    double paramMinDistanceDiffForLocUpdate=0.1;//M_PI/6.0;
    double paramMinAngleDiffForLocUpdate=0.2;
};

class SensorHandle{

public:

    Particle< geometry_msgs::Pose>  empty; // use as static, just an helper
    std::unordered_map<std::string, std::pair<SensorConfigs,Particle<geometry_msgs::Pose> > >  sensorhandle;

    std::mutex mtx;

    void localmotionmodelupdate(Particle<geometry_msgs::Pose>&p,NumericArray<double,3>&d){
        //    lets say as in u=[x -x-1];
        // TAble 5.6 we receive sigr1 sigt sigr2

        double sig_rot1=d[0];
        double sig_trans=d[1];
        double sig_rot2=d[2];


        double theta=PoseTools::getYaw(p.state);

        p.state.position.x+=(sig_trans*cos(theta+sig_rot1));
        p.state.position.y+=(sig_trans*sin(theta+sig_rot1));
        PoseTools::updateYaw(p.state,theta+sig_rot2+sig_rot1);//+Or*distribution_robotOr->operator ()(sample));

    }

    static NumericArray<double,3> motionmodel(geometry_msgs::Pose backupPose, geometry_msgs::Pose lastScanMatchPose){
        NumericArray<double,3>  lastdiff;
        lastdiff[0]=atan2(lastScanMatchPose.position.y-backupPose.position.y,lastScanMatchPose.position.x-backupPose.position.x)-PoseTools::getYaw(backupPose);
        lastdiff[1]=std::sqrt((lastScanMatchPose.position.y-backupPose.position.y)*(lastScanMatchPose.position.y-backupPose.position.y)+((lastScanMatchPose.position.x-backupPose.position.x)*(lastScanMatchPose.position.x-backupPose.position.x)));


        if(lastdiff[1]<0.00001){ // to avoid overexploration 0.1 (0.001 standart) 0.00001
            lastdiff[0]=0;
        }

        lastdiff[2]=PoseTools::getYaw(lastScanMatchPose)-PoseTools::getYaw(backupPose)-lastdiff[0];

        return lastdiff;
    }


    bool isGlobal(std::string & key){

        if(sensorhandle.find(key)==sensorhandle.end()){
            return false;
        }
        return sensorhandle[key].first.isGlobal;
    }

    void accumulateSensor(std::string key,geometry_msgs::Pose backupPose, geometry_msgs::Pose lastScanMatchPose){

mtx.lock();
        if(isGlobal(key)){
            sensorhandle[key].second.state=lastScanMatchPose;
            sensorhandle[key].first.updated=true;
        }else{
            NumericArray<double,3> val= motionmodel(backupPose,lastScanMatchPose);
            accumulate(key,val);
        }
        mtx.unlock();


    }

    void accumulate(std::string key,geometry_msgs::Pose backupPose, geometry_msgs::Pose lastScanMatchPose){
        //         NumericArray<double,3> val= motionmodel(backupPose,lastScanMatchPose);
        //         accumulate(key,val);
 
        if(isGlobal(key)){
            sensorhandle[key].second.state=lastScanMatchPose;
            sensorhandle[key].first.updated=true;
        }else{
            NumericArray<double,3> val= motionmodel(backupPose,lastScanMatchPose);
            accumulate(key,val);
        }

 
    }

    void accumulate(std::string key,NumericArray<double,3> temp){

 
        if(sensorhandle.find(key)==sensorhandle.end()){
            sensorhandle[key]=std::pair<SensorConfigs,Particle<geometry_msgs::Pose> > (SensorConfigs(),Particle<geometry_msgs::Pose>());
        }
        localmotionmodelupdate(sensorhandle[key].second,temp);
 
    }

    std::pair<bool,NumericArray<double,3> > sensorSelection(){

        std::pair<bool,NumericArray<double,3> > out;

        out.first=false;

        double count=0;

mtx.lock();

        for(std::unordered_map<std::string, std::pair<SensorConfigs,Particle<geometry_msgs::Pose> > >::iterator it = sensorhandle.begin(); it != sensorhandle.end(); ++it) {

            if(it->second.first.use){

                if(PoseTools::distance(it->second.second.state, empty.state)>it->second.first.paramMinDistanceDiffForLocUpdate || std::fabs(PoseTools::diffYaw(it->second.second.state, empty.state))> it->second.first.paramMinAngleDiffForLocUpdate){

                    out.first=true;

                    out.second+=motionmodel(empty.state,it->second.second.state);
                    it->second.second.reset();
                    count+=1.0;

                }
            }

        }

        if(out.first==true){
            out.second/=count;
        }
mtx.unlock();

        return out;
    }


    void configure(std::string key,bool use,double d,double t,bool isGlobal=false){
        if(sensorhandle.find(key)!=sensorhandle.end()){
            sensorhandle[key].first.use=use;
            sensorhandle[key].first.paramMinDistanceDiffForLocUpdate=d;
            sensorhandle[key].first.paramMinAngleDiffForLocUpdate=t;
            sensorhandle[key].first.isGlobal=isGlobal;
            sensorhandle[key].first.updated=false;


        }else{
            sensorhandle[key]=std::pair<SensorConfigs,Particle<geometry_msgs::Pose> > (SensorConfigs(),Particle<geometry_msgs::Pose>());
            sensorhandle[key].first.use=use;
            sensorhandle[key].first.paramMinDistanceDiffForLocUpdate=d;
            sensorhandle[key].first.paramMinAngleDiffForLocUpdate=t;
            sensorhandle[key].first.isGlobal=isGlobal;
            sensorhandle[key].first.updated=false;
        }
    }

    std::unordered_map<std::string,NumericArray<double,3> > getValidSensorSelection(){

        std::unordered_map<std::string,NumericArray<double,3> > out;
mtx.lock();

        for(std::unordered_map<std::string, std::pair<SensorConfigs,Particle<geometry_msgs::Pose> > >::iterator it = sensorhandle.begin(); it != sensorhandle.end(); ++it) {

            if(it->second.first.use){

                if(PoseTools::distance(it->second.second.state, empty.state)>it->second.first.paramMinDistanceDiffForLocUpdate || std::fabs(PoseTools::diffYaw(it->second.second.state, empty.state))> it->second.first.paramMinAngleDiffForLocUpdate){

                    if(it->second.first.isGlobal && it->second.first.updated==true){
                        out[it->first]=NumericArray<double,3>();
                        out[it->first][0]=it->second.second.state.position.x;
                        out[it->first][1]=it->second.second.state.position.y;
                        out[it->first][2]=PoseTools::getYaw(it->second.second.state);
                        it->second.second.reset();
                        it->second.first.updated=false;

                    }else if (!(it->second.first.isGlobal)){
                        out[it->first]=motionmodel(empty.state,it->second.second.state);
                        //std::cout<<"Measure : "<<out[it->first][0]<<" "<<out[it->first][1]<<" "<<out[it->first][2]<<std::endl;
                        it->second.second.reset();
                    }
                }
            }
        }

        if(out.size()>=1){
            for(std::unordered_map<std::string, std::pair<SensorConfigs,Particle<geometry_msgs::Pose> > >::iterator it = sensorhandle.begin(); it != sensorhandle.end(); ++it) {

                if(it->second.first.use){

                    if(out.find(it->first)==out.end()){

                        if(it->second.first.isGlobal && it->second.first.updated==true){
                            out[it->first]=NumericArray<double,3>();
                            out[it->first][0]=it->second.second.state.position.x;
                            out[it->first][1]=it->second.second.state.position.y;
                            out[it->first][2]=PoseTools::getYaw(it->second.second.state);
                            it->second.second.reset();

                            it->second.first.updated=false;
                        }else if (!(it->second.first.isGlobal)){
                            out[it->first]=motionmodel(empty.state,it->second.second.state);
                            it->second.second.reset();
                        }
                    }
                }
            }

        }
        mtx.unlock();

        return out;
    }
};


//class DynamicCellsDetector{

//    Grid3D<char> *free;
//    Grid3D<char> *occ;

//public:

//    DynamicCellsDetector(int rows,int cols,int h,double cellsize){
//        free=new Grid3D<char>( rows,cols,h);
//        occ=new Grid3D<char>( rows,cols,h);



//        Point p;
//        p.x=0;
//        p.y=0;
//        p.z=160*0.05/2.0;
//        free->setSizes(cellsize,cellsize,cellsize);
//        occ->setSizes(cellsize,cellsize,cellsize);


//        occ->setStart(p);
//        free->setStart(p);

//    }

//    void move(geometry_msgs::Pose  moved){
//        auto p=free->getStart();

//        int x=std::round((p.x-moved.position.x)/free->cellsizeX);
//        int y=std::round((p.y-moved.position.y)/free->cellsizeX);

//        free->translate(x,y);
//        occ->translate(x,y);

//        free->setStart(Point(moved.position.x,moved.position.y,moved.position.z));
//        occ->setStart(Point(moved.position.x,moved.position.y,moved.position.z));


//    }

//    void map(std::vector<Voxel3D> &voxels,double dist=5){

//        int iterationfx=rand();

//        auto p=free->getStart();
//        int x=free->convertToXindex(p.x);
//        int y=free->convertToYindex(p.y);

//        for(unsigned int i=0;i<voxels.size();i++){

//            if(sqrt((p.x-voxels[i].x)*(p.x-voxels[i].x)+(p.y-voxels[i].y)*(p.y-voxels[i].y))>dist){
//                continue;
//            }

//            std::vector<std::pair<int,int> > vec;
//            free->getLine(x,y,free->convertToXindex(voxels[i].x),free->convertToYindex(voxels[i].y),vec);
//            LineSegment up(Point(p.x,p.y,p.z),Point(voxels[i].x,voxels[i].y,voxels[i].zmax));
//            LineSegment down(Point(p.x,p.y,p.z),Point(voxels[i].x,voxels[i].y,voxels[i].zmin));

//            for(  int j=5;j<((int)vec.size()-1);j++){

//                auto cell=free->getIndexXY(vec[j].first,vec[j].second);
//                double xl=free->convertToXFromIndex(vec[j].first); // x no mundo
//                double yl=free->convertToYFromIndex(vec[j].second); // y no mundo
//                double hmax=up.heightfromIntersectionWithVerticalLine(xl,yl,LineSegment::MaxValue); //zmin
//                double hmin=down.heightfromIntersectionWithVerticalLine(xl,yl,LineSegment::MinValue); //zmax

//                if(cell!=NULL){
//                    if(cell->iteration!=iterationfx){
//                        cell->count++;
//                        cell->iteration=iterationfx;
//                    }
//                }else{
//                    auto cellx=new HeightCell(hmin,hmax);
//                    cellx->iteration=iterationfx;
//                    free->addIndexXY(vec[j].first,vec[j].second,cellx);
//                }
//            }
//        }
//        for(unsigned int i=0;i<voxels.size();i++){

//            auto cellf= free->getXY(voxels[i].x,voxels[i].y);

//            if(cellf!=NULL){
//                if(cellf->iteration==iterationfx){
//                    if(cellf->count==1){
//                        cellf->count=0;
//                    }
//                }
//            }

//            auto cellx= occ->getXY(voxels[i].x,voxels[i].y);
//            if(cellx!=NULL){
//                cellx->count++; //??
//            }else{
//                occ->addXY(voxels[i].x,voxels[i].y,new HeightCell(voxels[i].zmin,voxels[i].zmax));
//            }
//        }
//    }

//    std::pair<std::vector<Voxel3D>,std::vector<Voxel3D>> getVoxels(std::vector<Voxel3D> &data){
//        // get moving and static voxels
//        std::pair<std::vector<Voxel3D>,std::vector<Voxel3D>> out;


//        //http://ppniv12.irccyn.ec-nantes.fr/paper/5Qadeer.pdf
//        //Fast classification of static and dynamic environment for Bayesian Occupancy Filter (BOF)
//        //Motion detection:

//        for(unsigned int i=0;i<data.size();i++){

//            int x=occ->convertToXindex(data[i].x);
//            int y=occ->convertToYindex(data[i].y);


//            auto cellocc=occ->getIndexXY(x,y);
//            auto cellfree=free->getIndexXY(x,y);

//            if(cellocc!=NULL && cellfree!=NULL){

//                if(cellfree->count>2.0*cellocc->count){
//                    out.first.push_back(data[i]);
//                }else{
//                    out.second.push_back(data[i]);
//                }

//            }else if(cellfree!=NULL){
//                out.first.push_back(data[i]);
//            }else{
//                out.second.push_back(data[i]);
//            }
//        }


//        return out;
//    }

//};






class ParticleFilterLocalization{

    Particle< geometry_msgs::Pose>  empty; // use as static, just an helper


    std::atomic<bool> pseudolock;
    std::mutex mtx;
    std::default_random_engine gen;




    //    std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<double> Xdis;
    std::uniform_real_distribution<double> Ydis;
    std::uniform_real_distribution<double> Tdis;

    std::uniform_real_distribution<double> augmented;

    std::pair<double,double> pmin;
    std::pair<double,double> pmax;
    normal_distribution<double,false> *distributionX;
    normal_distribution<double,false> *distributionY;
    normal_distribution<double,false> *d_cellX;
    normal_distribution<double,false> *d_cellY;
    normal_distribution<double,false> *GPSdistributionX;
    normal_distribution<double,false> *GPSdistributionY;
    normal_distribution<double,false> *GPSdistributionTheta;

    //    normal_distribution<double,true> *GPSradius;
    std::normal_distribution<double> *distribution_robotTr;
    std::normal_distribution<double> *distribution_robotOr;

    std::default_random_engine sample;

    Grid3D<char> *map=NULL;

    double tr;
    double Or;

    double alpha_slow=0.05;
    double alpha_fast=0.5;

    double w_slow=0.000001;
    double w_fast=0.000001;

    int type=1;

    // Variáveis que são utilizadas para gerar uma partícula apartír da leitura de GPS
    NumericArray<double,3> last_pf;
    // Variável que guarda os dados da ultima leitura do GPS
    NumericArray<double,2> last_GPS;

    bool init=false;
    //    double paramMinDistanceDiffForLocUpdate=0.2;
    //    double paramMinAngleDiffForLocUpdate=M_PI/6.0;
    //  int resample_interval=1;

    // CI area vector
    std::vector<double> Area{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // CI angular cone vector
    std::vector<double> Ang_cone{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Area threshold variable
    double AreaCI_threshold = 0.01;
    // Angle threshold variable
    double AngleCI_threshold = 0.12217304764;

    SamplingScheme<geometry_msgs::Pose> ssch;

    std::string name="ParticleFilter";

    bool usingGPS=false;


public:
    //    LaserFeatureExtraction lfe;
    KeyPointMap<geometry_msgs::Point32> keypoints;

    SequentialMonteCarlo<geometry_msgs::Pose,NumericArray<double,3>,std::vector<Voxel3D> > *smc;


    //    DynamicCellsDetector dcd;

    bool useFeatures=false;


    // Variáveis que são substituidas com a entrada do cpp
    float alphaGPS = 0.9;
    float thdGPS = 0.45;
    int op_GPS = 1;
    // motion model
    // alphas anteriores: 0.15
    double alpha1=0.04;
    double alpha2=0.04;
    double alpha3=0.04;
    double alpha4=0.04;
    // classe KLD
    KullbackLeiblerDivergence kldHandle;

    // Variable to check if map is updated or not
    bool update_flag = true;
    // Transform
    Transform tf;


    SensorHandle sensorhandle;

                                      
    sensor_msgs::PointCloud pointcloud_keypoints;
    //keypointSift parameters
    const float min_scale=0.05 ;      //               parameters (t1:0.1  ok   t2:0.02 ok t3:0.05  ok    t4:0.01 )
    const int n_octaves=5;          //                 parameters (t1:6   ok   t2:5   ok  t3:5      ok   t4:3    )
    const int n_scales_per_octave=3;//                 parameters (t1:10 ok   t2:3  ok   t3:3      ok   t4:4    )
    const float min_contrast=0.06; //                 parameters (t1:0.05 ok   t2:0.03 ok  t3:0.06  ok   t4:0.001)
























    bool usePCLFeatures=false;
    bool useKeypointsmap=false;


































    bool toProcess=false;
    geometry_msgs::Pose pose_;
    std::vector<Voxel3D> data_;
    sensor_msgs::PointCloud keypoints_;

    void setPoseData(geometry_msgs::Pose &pose){
       lastPose=pose;
       setlast=true;
   }

    void setPoseDataFeatures(geometry_msgs::Pose &pose,std::vector<Voxel3D> &data,sensor_msgs::PointCloud &keypointsx){
      pose_=pose;
      data_=data;
      keypoints_=keypointsx;
      toProcess=true;
    }


    ParticleFilterLocalization(Grid3D<char> *map_,Transform &tf_,int particles): pseudolock(false),augmented(0,1),Xdis(-2,5),Ydis(-4,4),Tdis(-M_PI,M_PI){
        tf=tf_;
        std::function<void (Particle<geometry_msgs::Pose>&)> sampleStage_=std::bind(&ParticleFilterLocalization::sampleStage,this,std::placeholders::_1);
        std::function<void (Particle<geometry_msgs::Pose>&,NumericArray<double,3>&)> predictStage_=std::bind(&ParticleFilterLocalization::predictStage,this,std::placeholders::_1,std::placeholders::_2);
        std::function<void (Particle<geometry_msgs::Pose>&,std::vector<Voxel3D> &)> updateStage_=std::bind(&ParticleFilterLocalization::updateStage,this,std::placeholders::_1,std::placeholders::_2);
        std::function<void (std::vector<Particle<geometry_msgs::Pose>> &)> resampleStage_=std::bind(&ParticleFilterLocalization::resampleStage,this,std::placeholders::_1);
        std::function<void (std::vector<Particle<geometry_msgs::Pose>> &,NumericArray<double,3>&,std::vector<Voxel3D> &)> fullStage_=std::bind(&ParticleFilterLocalization::fullImplementation,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
        std::function<void (std::vector<Particle<geometry_msgs::Pose>> &,std::unordered_map<std::string,NumericArray<double,3> >&,std::vector<Voxel3D> &)> fullStage2_=std::bind(&ParticleFilterLocalization::fullImplementation3,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

        smc=new SequentialMonteCarlo<geometry_msgs::Pose,NumericArray<double,3>,std::vector<Voxel3D>  > (10,sampleStage_,predictStage_,updateStage_,resampleStage_,fullStage_,fullStage2_);


        distributionX=new normal_distribution<double,false>(0,0.35,true);//0.05 0.35 teste2 0.35 teste3 0.05
        distributionY=new normal_distribution<double,false>(0,0.35,true);//0.05 0.35 teste2 0.35 teste3 0.05
        d_cellX = new normal_distribution<double,false>(0,0.1);
        d_cellY = new normal_distribution<double,false>(0,0.1);
        GPSdistributionX=new normal_distribution<double,false>(0,0.05);
        GPSdistributionY=new normal_distribution<double,false>(0,0.05);
        GPSdistributionTheta=new normal_distribution<double,false>(0,0.2); // 0.0175 éra o valor que estava
        distribution_robotTr=new std::normal_distribution<double>(0,0.05);
        distribution_robotOr=new std::normal_distribution<double>(0,0.01);


        //        GPSradius=new normal_distribution<double,true>(0,0.5);


        pmin.first=-1;
        pmin.second=-1;

        pmax.first=1;
        pmax.second=1;

        map=map_;

        tr=1.0;
        Or=1.0;
        kldHandle.maxParticles=particles;

        smc->particles.resize(kldHandle.maxParticles);

        kldHandle.map=new Map<AngularHistogram<unsigned char> *> (map->rows,map->columns,0);
        kldHandle.map->setCellSizes(map->cellsizeX,map->cellsizeY,Point());




    }

    std::string getName(){
        return name;
    }

    // GUI variable registration
    void registration(std::unordered_map<std::string,double> &NamedConstants){

        NamedConstants.operator[](name+"::alpha_slow")=alpha_slow;
        NamedConstants.operator[](name+"::alpha_fast")=alpha_fast;
        NamedConstants.operator[](name+"::type")=type;

        NamedConstants.operator[](name+"::alpha1")=alpha1;
        NamedConstants.operator[](name+"::alpha2")=alpha2;
        NamedConstants.operator[](name+"::alpha3")=alpha3;
        NamedConstants.operator[](name+"::alpha4")=alpha4;

        std::cout<<"Registration call"<<std::endl;

    }
    // GUI variable update
    void update(std::string key, double &value){

        if(key.find(name+"::alpha_slow")!=key.npos){
            alpha_slow=value;
        }
        if(key.find(name+"::alpha_fast")!=key.npos){
            alpha_fast=value;
        }
        if(key.find(name+"::alpha1")!=key.npos){
            alpha1=value;
        }
        if(key.find(name+"::alpha2")!=key.npos){
            alpha2=value;
        }
        if(key.find(name+"::alpha3")!=key.npos){
            alpha3=value;
        }
        if(key.find(name+"::alpha4")!=key.npos){
            alpha4=value;
        }
        if(key.find(name+"::type")!=key.npos){

            if(type!=value && value>0){

                smc->numParticles=kldHandle.maxParticles-1;
                smc->sample();
            }
            type=value;

        }

        std::cout<<"Update call"<<std::endl;
    }










    // Função que gera uma partícula com base no GPS
    void GPSPart(Particle< geometry_msgs::Pose>& p, double GPS_X, double GPS_Y){

        double X = GPSdistributionX -> sample();
        double Y = GPSdistributionY -> sample();
        double theta = last_pf[2] - GPSdistributionTheta -> sample();
        p.state.position.x = GPS_X - X;
        p.state.position.y = GPS_Y - Y;
        PoseTools::updateYaw(p.state,theta);

    }


    // new mode?
    void run(std::vector<Voxel3D>  &measure){

        std::pair<bool,NumericArray<double,3> > temp1 = sensorhandle.sensorSelection();

        if(temp1.first){

            // pseudolock.store(true,std::memory_order_seq_cst);
            // mtx.lock();


            if(type==0){
                smc->prediction(temp1.second);
                init=true;
                smc->update(measure);
                smc->resampling();
            }else{
                smc->fullImplementation(temp1.second,measure);
            }
            // mtx.unlock();
            // pseudolock.store(false,std::memory_order_seq_cst);
        }
    }

    void parseLaser(std::vector<Voxel3D> &measure){

        //lfe.extractFeatures(measure);
        std::cout<<__LINE__<<"  "<<__FUNCTION__<<"  "<<__FILE__<<std::endl;
    }

    //Funcao alterada
    bool run2KLD(std::vector<Voxel3D> &measure){
        auto temp1 = sensorhandle.getValidSensorSelection();

        if(temp1.size()!=0){

            // pseudolock.store(true,std::memory_order_seq_cst);
            // mtx.lock();
            //std::cout<<"processing data"<<std::endl;
            fullImplementation3(smc->particles,temp1,measure);

            return true;
            // mtx.unlock();
            // pseudolock.store(false,std::memory_order_seq_cst);
        }else{
            return false;
        }
    }

    bool run2KLD(std::vector<geometry_msgs::Point32>  measure){
        auto temp1 = sensorhandle.getValidSensorSelection();

        if(temp1.size()!=0){

            // pseudolock.store(true,std::memory_order_seq_cst);
            // mtx.lock();
            //std::cout<<"processing data"<<std::endl;
            fullImplementation3_v2(smc->particles,temp1,measure);
            return true;
            // mtx.unlock();
            // pseudolock.store(false,std::memory_order_seq_cst);
        }else{
            return false;
        }
    }



double tic(int mode=0) {
    static std::chrono::_V2::system_clock::time_point t_start2;
    
    if (mode==0)
        t_start2 = std::chrono::high_resolution_clock::now();
    else {
        auto t_end = std::chrono::high_resolution_clock::now();
        if (mode==1)
        std::cout << "L3D:: Elapsed time is " << (t_end-t_start2).count()*1E-9 << " seconds\n";

    return (t_end-t_start2).count()*1E-9;
      
    }
    return 0;
}
double toc(int id=1) { return tic(id); }


    bool run2KLD(const sensor_msgs::PointCloud2ConstPtr& laser){
        auto temp1 = sensorhandle.getValidSensorSelection(); //???

        if(temp1.size()!=0){



            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*laser,pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloudf(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud2);

            if(usePCLFeatures){

            
            double d_x, d_y, d_z, dist=0;

            for (int i=0;i<temp_cloud2->size();i++)              //localmap, data e verificação
            {
                d_x=temp_cloud2->points[i].x*temp_cloud2->points[i].x;
                d_y=temp_cloud2->points[i].y*temp_cloud2->points[i].y;
                d_z=temp_cloud2->points[i].z*temp_cloud2->points[i].z;
                dist=sqrt(d_x+d_y+d_z);
                //std::cout<<"dist: "<<dist<<std::endl;
                if(temp_cloud2->points[i].z<=1.75 && temp_cloud2->points[i].z>-1.06 && dist<=10)
                {
                    temp_cloud->push_back(temp_cloud2->points[i]);
                }

            }


            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(temp_cloud);
            sor.setLeafSize(0.15,0.15,0.15); //0.01 t1 t3 0.02 t3 0.03 t3 0.04 t3 0.05 t3 0.06 t3 0.08 t3 0.1 t3 0.15 t3
            sor.filter(*temp_cloudf);


            pcl::PointCloud<pcl::PointXYZ>::Ptr p_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
            sift.setSearchMethod(tree);
            sift.setScales(min_scale, n_octaves, n_scales_per_octave);
            sift.setMinimumContrast(min_contrast);
            sift.setInputCloud(temp_cloudf);
            sift.compute(result);

            pcl::copyPointCloud(result, *p_keypoints);


            pointcloud_keypoints.points.clear();
            geometry_msgs::Point32 point;
            for (int i=0;i<p_keypoints->size();i++)
            {
                point.x=p_keypoints->points[i].x;
                point.y=p_keypoints->points[i].y;
                point.z=p_keypoints->points[i].z;

            pointcloud_keypoints.points.push_back(point);
            }



}else{
//tic();


double rmax= 120;
double rmin= 0.0;
double phimin= 1.6;
double phimax= 1.78;
double thmax= 3.10;
double thmin= -3.10;
double deltath= 0.05;
double deltaphi= 0.02;



            PointVectorToSphereContainer<geometry_msgs::Point32>  DistanceBasedFeatureExtraction(deltaphi,deltath);
            DistanceBasedFeatureExtraction.set(rmin,rmax,phimin,phimax,thmin,thmax,0);

            DistanceBasedFeatureExtraction.project(temp_cloud2->points);

            pointcloud_keypoints.points=DistanceBasedFeatureExtraction.getFeatureAreas(0.2); //0.2

//toc();
            std::cout<<pointcloud_keypoints.points.size()<<std::endl;


            }










            // pseudolock.store(true,std::memory_order_seq_cst);
            // mtx.lock();
            //std::cout<<"processing data"<<std::endl;
            fullImplementation3_v2(smc->particles,temp1,pointcloud_keypoints.points); //points
            return true;
            // mtx.unlock();
            // pseudolock.store(false,std::memory_order_seq_cst);
        }else{
            return false;
        }
    }



    bool run2KLDv2(NumericArray<double,3> &dt,std::vector<Voxel3D> &measure){
        

        auto temp1 = sensorhandle.getValidSensorSelection();

        if(temp1.size()!=0){

 
            //std::cout<<"processing data"<<std::endl;
            temp1["laserodom"]=dt;
            fullImplementation3(smc->particles,temp1,measure);

            return true;
            // mtx.unlock();
            // pseudolock.store(false,std::memory_order_seq_cst);
        }else{
            return false;
        }



  
    }

    bool run2KLD(NumericArray<double,3> &dt,std::vector<Voxel3D> &measure){
        std::unordered_map<std::string,NumericArray<double,3> > temp1;
        temp1["odom_pf"]=dt;
        fullImplementation3(smc->particles,temp1,measure);

        return true;

    }

    bool run3KLD(std::vector<Voxel3D> &measure,geometry_msgs::Pose po){
        auto temp1 = sensorhandle.getValidSensorSelection();

        if(temp1.size()!=0){

            // pseudolock.store(true,std::memory_order_seq_cst);
            // mtx.lock();
            fullImplementation4(smc->particles,temp1,measure,po);

            return true;
            // mtx.unlock();
            // pseudolock.store(false,std::memory_order_seq_cst);
        }else{
            return false;
        }
    }
    void mapUpdate(geometry_msgs::Pose p,std::vector<Voxel3D> &voxels,double dist){

        //        int iterationfx=rand();


        //        auto p=px;
        //        int x=map->convertToXindex(p.position.x);
        //        int y=map->convertToYindex(p.position.y);
        dist=dist*dist;


        for(unsigned int i=0;i<voxels.size();i++){
            if(((p.position.x-voxels[i].x)*(p.position.x-voxels[i].x)+(p.position.y-voxels[i].y)*(p.position.y-voxels[i].y))>dist){
                continue;
            }
            map->addLineXYZ(p.position.x,p.position.y,p.position.z,voxels[i].x,voxels[i].y,voxels[i].z,-1);
        }

        for(unsigned int i=0;i<voxels.size();i++){
            if(((p.position.x-voxels[i].x)*(p.position.x-voxels[i].x)+(p.position.y-voxels[i].y)*(p.position.y-voxels[i].y))>dist){
                continue;
            }
            map->addXYZ(voxels[i].x,voxels[i].y,voxels[i].z,2);
        }

        //        for(unsigned int i=0;i<voxels.size();i++){


        //            if(sqrt((p.position.x-voxels[i].x)*(p.position.x-voxels[i].x)+(p.position.y-voxels[i].y)*(p.position.y-voxels[i].y))>dist){
        //                continue;
        //            }

        //            std::vector<std::pair<int,int> > vec;
        //            map->getLine(x,y,map->convertToXindex(voxels[i].x),map->convertToYindex(voxels[i].y),vec);
        //            LineSegment up(Point(p.position.x,p.position.y,p.position.z),Point(voxels[i].x,voxels[i].y,voxels[i].zmax));
        //            LineSegment down(Point(p.position.x,p.position.y,p.position.z),Point(voxels[i].x,voxels[i].y,voxels[i].zmin));

        //            for(int j=5;j<((int)vec.size());j++){

        //                auto cell= map->getIndexXY(vec[j].first,vec[j].second);
        //                double xl=map->convertToXFromIndex(vec[j].first); // x no mundo
        //                double yl=map->convertToYFromIndex(vec[j].second); // y no mundo
        //                double hmax=up.heightfromIntersectionWithVerticalLine(xl,yl,LineSegment::MaxValue); //zmin
        //                double hmin=down.heightfromIntersectionWithVerticalLine(xl,yl,LineSegment::MinValue); //zmax

        //                if(cell!=NULL){
        //                    bool ovl=cell->overlaps(hmin,hmax);

        //                    if(ovl==true && cell->iteration!=iterationfx){ // intersecao

        //                        cell->iteration=iterationfx;
        //                        cell->occ-=10;
        //                        cell->occ=std::max(cell->occ,(char)0);
        //                        if(cell->occ<50){
        //                            if(cell->isInside(hmin) && cell->isInside(hmax)){

        //                                double val=cell->getIOU(hmin,hmax);
        //                                if(val>0.75){
        //                                    HeightCell * value=NULL;
        //                                    map->addIndexXY(vec[j].first,vec[j].second,value);
        //                                }else{

        //                                }
        //                            }else if(cell->isInside(hmin) && !cell->isInside(hmax)){
        //                                cell->max=hmin;

        //                            }else if(!cell->isInside(hmin) && cell->isInside(hmax)){
        //                                cell->min=hmax;

        //                            }else if(!cell->isInside(hmin) && !cell->isInside(hmax)){
        //                                HeightCell * value=NULL;
        //                                map->addIndexXY(vec[j].first,vec[j].second,value);
        //                            }

        //                        }

        //                    }
        //                    else{ // nao ha intersecao


        //                    }

        //                }else{
        //                }
        //            }

        //        }



    }




 void mapUpdatev2(geometry_msgs::Pose p,std::vector<Voxel3D> &voxels,double dist){

         
        Transform tf_particle=Transform::byPose(p)*tf;

        for(unsigned int i=0;i<voxels.size();i++){

            auto point=tf_particle*voxels[i];
            if(sqrt((p.position.x-point.x)*(p.position.x-point.x)+(p.position.y-point.y)*(p.position.y-point.y))>dist){
                continue;
            }
            map->addXYZ(point.x,point.y,point.z,1);

        }
 

    }


 void mapUpdatev2(Grid3D<char> *m,geometry_msgs::Pose p,std::vector<Voxel3D> &voxels,double dist){

         
        Transform tf_particle=Transform::byPose(p)*tf;

        dist=dist*dist;

        for(unsigned int i=0;i<voxels.size();i++){

            auto point=tf_particle*voxels[i];
            if(((p.position.x-point.x)*(p.position.x-point.x)+(p.position.y-point.y)*(p.position.y-point.y))>dist){
                continue;
            }
            m->addXYZ(point.x,point.y,point.z,1);

        }
 

    }



 void mapKeypoints(geometry_msgs::Pose p,std::vector<geometry_msgs::Point32>  voxels){

         
    if(useKeypointsmap){  //quero ter o mapa de keypoints na mesma
        Transform tf_particle=Transform::byPose(p)*tf;

        for(unsigned int i=0;i<voxels.size();i++){

            auto point=tf_particle*voxels[i];
  
           
            keypoints.map(point,0.1); // se tiver um ponto a 0.1m de distancia n adiciona nada. (std 0.1 estava -0.1)

      }
 }

    }


    void mapUpdate(std::vector<Voxel3D> &voxels,double dist){

        //        int iterationfx=rand();


        auto p=getBestPose();
        //        int x=map->convertToXindex(p.position.x);
        //        int y=map->convertToYindex(p.position.y);

        Transform tf_particle=Transform::byPose(p)*tf;

        dist=dist*dist;

        for(unsigned int i=0;i<voxels.size();i++){




            auto point=tf_particle*voxels[i];
            if(((p.position.x-point.x)*(p.position.x-point.x)+(p.position.y-point.y)*(p.position.y-point.y))>dist){
                continue;
            }
            map->addXYZ(point.x,point.y,point.z,1);

        }

        //        for(unsigned int i=0;i<voxels.size();i++){


        //            if(sqrt((p.position.x-voxels[i].x)*(p.position.x-voxels[i].x)+(p.position.y-voxels[i].y)*(p.position.y-voxels[i].y))>dist){
        //                continue;
        //            }

        //            std::vector<std::pair<int,int> > vec;
        //            map->getLine(x,y,map->convertToXindex(voxels[i].x),map->convertToYindex(voxels[i].y),vec);
        //            LineSegment up(Point(p.position.x,p.position.y,p.position.z),Point(voxels[i].x,voxels[i].y,voxels[i].zmax));
        //            LineSegment down(Point(p.position.x,p.position.y,p.position.z),Point(voxels[i].x,voxels[i].y,voxels[i].zmin));

        //            for(int j=5;j<((int)vec.size());j++){

        //                auto cell= map->getIndexXY(vec[j].first,vec[j].second);
        //                double xl=map->convertToXFromIndex(vec[j].first); // x no mundo
        //                double yl=map->convertToYFromIndex(vec[j].second); // y no mundo
        //                double hmax=up.heightfromIntersectionWithVerticalLine(xl,yl,LineSegment::MaxValue); //zmin
        //                double hmin=down.heightfromIntersectionWithVerticalLine(xl,yl,LineSegment::MinValue); //zmax

        //                if(cell!=NULL){
        //                    bool ovl=cell->overlaps(hmin,hmax);

        //                    if(ovl==true && cell->iteration!=iterationfx){ // intersecao

        //                        cell->iteration=iterationfx;
        //                        cell->occ-=10;
        //                        cell->occ=std::max(cell->occ,(char)0);
        //                        if(cell->occ<50){
        //                            if(cell->isInside(hmin) && cell->isInside(hmax)){

        //                                double val=cell->getIOU(hmin,hmax);
        //                                if(val>0.75){
        //                                    HeightCell * value=NULL;
        //                                    map->addIndexXY(vec[j].first,vec[j].second,value);
        //                                }else{

        //                                }
        //                            }else if(cell->isInside(hmin) && !cell->isInside(hmax)){
        //                                cell->max=hmin;

        //                            }else if(!cell->isInside(hmin) && cell->isInside(hmax)){
        //                                cell->min=hmax;

        //                            }else if(!cell->isInside(hmin) && !cell->isInside(hmax)){
        //                                HeightCell * value=NULL;
        //                                map->addIndexXY(vec[j].first,vec[j].second,value);
        //                            }

        //                        }

        //                    }
        //                    else{ // nao ha intersecao


        //                    }

        //                }else{
        //                }
        //            }

        //        }



    }




    //LEGACY MODE;
    void run(std::vector<Voxel3D> &measure, NumericArray<double,3> temp){

        sensorhandle.accumulate("UNKNOWN::SENSOR",temp);

        if(PoseTools::distance(sensorhandle.sensorhandle["UNKNOWN::SENSOR"].second.state, empty.state)>sensorhandle.sensorhandle["UNKNOWN::SENSOR"].first.paramMinDistanceDiffForLocUpdate || PoseTools::diffYaw(sensorhandle.sensorhandle["UNKNOWN::SENSOR"].second.state, empty.state)> sensorhandle.sensorhandle["UNKNOWN::SENSOR"].first.paramMinAngleDiffForLocUpdate){

            temp=SensorHandle::motionmodel(empty.state,sensorhandle.sensorhandle["UNKNOWN::SENSOR"].second.state);
            sensorhandle.sensorhandle["UNKNOWN::SENSOR"].second.reset();

            pseudolock.store(true,std::memory_order_seq_cst);
            mtx.lock();


            if(type==0){
                smc->prediction(temp);
                init=true;
                smc->update(measure);
                smc->resampling();
            }else{
                smc->fullImplementation(temp,measure);
            }
            mtx.unlock();
            pseudolock.store(false,std::memory_order_seq_cst);
        }
    }

    void sampleStage(Particle< geometry_msgs::Pose>& p){

        p.state.position.x=-4.1; // -4.1(para piso 0)
        p.state.position.y=0;
        PoseTools::updateYaw(p.state,0); // 0
        //        p.state.position.x=Xdis(gen);
        //        p.state.position.y=Ydis(gen);
        //        PoseTools::updateYaw(p.state,Tdis(gen));
        p.w=1.0/1000.0;
        p.w_unnormalized=0;
    }


    static double normalize(double z)
    {
        return atan2(sin(z),cos(z));
    }

    template <typename Pt>
    void move( Pt &p,double yaw=0){

        for(unsigned int i=0;i<smc->numParticles;i++){

            smc->particles[i].state.position.x=p.x;
            smc->particles[i].state.position.y=p.y;


            PoseTools::updateYaw(smc->particles[i].state,yaw);
            //        p.state.position.x=Xdis(gen);
            //        p.state.position.y=Ydis(gen);
            //        PoseTools::updateYaw(p.state,Tdis(gen));
            smc->particles[i].w=1.0/smc->numParticles;
            smc->particles[i].w_unnormalized=0;
        }

    }

    void predictStage(Particle<geometry_msgs::Pose>&p,NumericArray<double,3>&d){
        //    lets say as in u=[x -x-1];
        // TAble 5.6 we receive sigr1 sigt sigr2

        double sig_rot1 = std::min(fabs(d[0]),fabs(normalize(d[0]-M_PI)));
        double sig_trans = d[1];
        double sig_rot2 = std::min(fabs(d[2]),fabs(normalize(d[2]-M_PI)));

        double sig_rot1t=d[0];
        double sig_transt=d[1];
        double sig_rot2t=d[2];
        //std::cout<<"  "<<sig_rot1<<"  "<<sig_trans<<"  "<<sig_rot2<<std::endl;
        sig_rot1t-=  normal_distribution<double,true>::sample( (alpha1*sig_rot1*sig_rot1+ alpha2*sig_trans*sig_trans));
        sig_transt-= normal_distribution<double,true>::sample( (alpha4*sig_rot2*sig_rot2+alpha4*sig_rot1*sig_rot1+ alpha3*sig_trans*sig_trans));
        sig_rot2t-=  normal_distribution<double,true>::sample( (alpha1*sig_rot2*sig_rot2+ alpha2*sig_trans*sig_trans));
        //std::cout<<"->  "<<sig_rot1t<<"  "<<sig_transt<<"  "<<sig_rot2t<<std::endl;


        double theta=PoseTools::getYaw(p.state);
        //std::cout<<"->  "<<theta<<"  "<<p.state.position.x<<"  "<<p.state.position.y<<std::endl;

        p.state.position.x+=(sig_transt*cos(theta+sig_rot1t));
        p.state.position.y+=(sig_transt*sin(theta+sig_rot1t));
        PoseTools::updateYaw(p.state,theta+sig_rot2t+sig_rot1t);//+Or*distribution_robotOr->operator ()(sample));
        //std::cout<<"->  "<<theta<<"  "<<p.state.position.x<<"  "<<p.state.position.y<<std::endl;


    }

    void predictStage2(Particle<geometry_msgs::Pose>&p,NumericArray<double,3>&d){
        //    lets say as in u=[x -x-1];
        // TAble 5.6 we receive sigr1 sigt sigr2

        double sig_rot1 = std::min(fabs(d[0]),fabs(normalize(d[0]-M_PI)));
        double sig_trans = d[1];
        double sig_rot2 = std::min(fabs(d[2]),fabs(normalize(d[2]-M_PI)));

        double sig_rot1t=d[0];
        double sig_transt=d[1];
        double sig_rot2t=d[2];
        //        std::cout<<"  "<<sig_rot1<<"  "<<sig_trans<<"  "<<sig_rot2<<std::endl;
        sig_rot1t-=  normal_distribution<double,true>::sample( (alpha1*sig_rot1*sig_rot1+ alpha2*sig_trans*sig_trans));
        sig_transt-= normal_distribution<double,true>::sample( (alpha4*sig_rot2*sig_rot2+alpha4*sig_rot1*sig_rot1+ alpha3*sig_trans*sig_trans));
        sig_rot2t-=  normal_distribution<double,true>::sample( (alpha1*sig_rot2*sig_rot2+ alpha2*sig_trans*sig_trans));
        //        std::cout<<"->  "<<sig_rot1t<<"  "<<sig_transt<<"  "<<sig_rot2t<<std::endl;


        double theta=PoseTools::getYaw(p.state);

        p.state.position.x+=(sig_transt*cos(theta+sig_rot1t));
        p.state.position.y+=(sig_transt*sin(theta+sig_rot1t));
        PoseTools::updateYaw(p.state,theta+sig_rot2t+sig_rot1t);//+Or*distribution_robotOr->operator ()(sample));

    }

    //nova funcao
    void updateStage2(Particle<geometry_msgs::Pose>&p, std::vector<Voxel3D> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        // Voxel3D
        // x=0;
        // y=0;
        // zmin=0;
        // zmax=0;

        //        p.w=0;
        //        double minz;
        //        double maxz;

        //        Transform tf_particle=Transform::byPose(p.state)*tf;
        //        Point pt;
        //        Point pt_base=tf_particle.point();

        //        int x0=map->convertToXindex(pt_base.x);
        //        int y0=map->convertToYindex(pt_base.y);

        //        //        int count=0;
        //        double a,b,c,fx,fy;
        //        for(unsigned int i=0;i<scan.size();i=i+10){

        //            if(fabs(scan[i].zmax-scan[i].zmin)<=0.05){
        //                continue;
        //            }

        //            pt=tf_particle.point(scan[i].x,scan[i].y,0);
        //            //            pt.z=0;

        //            double theta=PointTools::angleBetween(pt_base,pt);
        //            pt.x=pt.x+0.2*cos(theta);
        //            pt.y=pt.y+0.2*sin(theta);


        //            minz=pt.z+scan[i].zmin;
        //            maxz=pt.z+scan[i].zmax;

        //            auto landmark= map->nearest(x0,y0,pt.x,pt.y,minz,maxz); // ray tracing nearest obstacle


        //            if(landmark.first!=NULL){
        //                a=1.0;//landmark.first->getIOU(minz,maxz);
        //                fx=pt.x-map->convertToXFromIndex(landmark.second.first);
        //                fy=pt.y-map->convertToYFromIndex(landmark.second.second);
        //                b=distributionX->operator ()(fx);
        //                c=distributionY->operator ()(fy);
        //                p.w+= 10000.0*(a*b*c);
        //            }else{
        //                //                p.w=p.w*0.9;
        //            }
        //        }

        //        p.w=p.w/((double)scan.size()/10.0);
    }


    void updateStage3_v2(Particle<geometry_msgs::Pose>&p, std::vector<geometry_msgs::Point32> &scan){
        
        double minz;
        double maxz;

        Transform tf_particle=Transform::byPose(p.state)*tf;
        Point pt;
        //        Point pt_base=tf_particle.point();
        p.w=0;

        //        int count=0;
        double a,b,c,fx,fy,fz;


        for(unsigned int i=0;i<scan.size();i++){//10 alterar valor 12 repetir(15) 17 20 22 25 27 30 32

         
        pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);
        geometry_msgs::Point32 pt2;
        pt2.x=pt.x;
        pt2.y=pt.y;
        pt2.z=pt.z;



            bool valid=false;


        if(useKeypointsmap==true){
            auto landmark= keypoints.nearest(pt2,valid); // ray tracing nearest obstacle
            if(valid!=false){
                fx=pt.x-(landmark.x);
                fy=pt.y-(landmark.y);
                fz=pt.z-(landmark.z);

                //fx=(pt.x-(landmark.x))<=0.15?1.0:0.0;
                //fy=(pt.y-(landmark.y))<=0.15?1.0:0.0;
                //fz=(pt.z-(landmark.z))<=0.15?1.0:0.0;
                
                b=distributionX->operator ()(fx);
                a=distributionX->operator ()(fz);
                c=distributionY->operator ()(fy);
                
                p.w+= 10.0*(a*b*c); //tirar a
            }else{

            }
        }

            auto landmark2= map->nearest(pt.x,pt.y,pt.z,3,1,3); // ray tracing nearest obstacle


            if(landmark2.first!=false){             //aqui
                fx=pt.x-(landmark2.second.x);
                fy=pt.y-(landmark2.second.y);
                fz=pt.z-(landmark2.second.z);
                b=distributionX->operator ()(fx);
                c=distributionY->operator ()(fy);
                a=distributionY->operator ()(fz);
                //a=distributionY->operator ()(sqrt(fx*fx+fy*fy+fz*fz));
                p.w+= 10.0*(a*b*c); // a
  //            p.w+= 10.0*(a);
            }else{

            }
        }
    }




    void updateStage3(Particle<geometry_msgs::Pose>&p, std::vector<Voxel3D> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        // Voxel3D
        // x=0;
        // y=0;
        // zmin=0;
        // zmax=0;

        double minz;
        double maxz;

        Transform tf_particle=Transform::byPose(p.state)*tf;
        Point pt;
        //        Point pt_base=tf_particle.point();
        p.w=0;

        //        int count=0;
        double a,b,c,fx,fy,fz;
        for(unsigned int i=0;i<scan.size();i=i+30){//10 alterar valor 12 repetir(15) 17 20 22 25 27 30 32

            //            if(fabs(scan[i].zmax-scan[i].zmin)<=0.05){
            //                continue;
            //            }

            pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);



            auto landmark= map->nearest(pt.x,pt.y,pt.z,1,1); // ray tracing nearest obstacle


            if(landmark.first!=false){             //aqui
                fx=pt.x-(landmark.second.x);
                fy=pt.y-(landmark.second.y);
                fz=pt.z-(landmark.second.z);
                b=distributionX->operator ()(fx);
                c=distributionY->operator ()(fy);
                a=distributionY->operator ()(fz);
                //a=distributionY->operator ()(sqrt(fx*fx+fy*fy+fz*fz));
                p.w+= 10.0*(a*b*c); // a
  //            p.w+= 10.0*(a);
            }else{

            }
        }
    }


    void updateStage2(Particle<geometry_msgs::Pose>&p,std::vector<Point> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        //        p.w=0;

        //        Transform tf_particle=Transform::byPose(p.state)*tf;
        //        Point pt;

        //        int x0=map->convertToXindex(tf_particle.x());
        //        int y0=map->convertToYindex(tf_particle.y());

        //        for(unsigned int i=0;i<scan.size();i=i+10){

        //            pt=tf_particle.point(scan[i].x,scan[i].y,0);
        //            pt.z=0;

        //            auto landmark= map->nearest(x0,y0,pt.x,pt.y); // ray tracing nearest obstacle

        //            if(landmark.first!=NULL){
        //                p.w+=distributionX->operator ()(pt.x-map->convertToXFromIndex(landmark.second.first))*(distributionY->operator ()(pt.y-map->convertToYFromIndex(landmark.second.second)));
        //            }
        //        }
        std::cout<<__LINE__<<"    "<<__FUNCTION__<<"   "<<__FILE__<<std::endl;
    }

    void updateStage(Particle<geometry_msgs::Pose>&p,std::vector<Voxel3D>  &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrsun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        /*double range=scan.angle_min;

        p.w=0;

        Transform tf_particle=Transform::byPose(p.state)*tf;
        Point pt;

        int x0=map->convertToXindex(tf_particle.x());
        int y0=map->convertToYindex(tf_particle.y());

        for(unsigned int i=0;i<scan.ranges.size();i=i+10){

            if(scan.ranges[i] <scan.range_max && scan.ranges[i] >scan.range_min ) {

                pt=tf_particle.point(scan.ranges[i]*cos(range),scan.ranges[i]*sin(range));
                pt.z=0;

                auto landmark= map->nearest(x0,y0,pt.x,pt.y); // ray tracing nearest obstacle

                if(landmark.first!=NULL){
                    p.w+=distributionX->operator ()(pt.x-map->convertToXFromIndex(landmark.second.first))*(distributionY->operator ()(pt.y-map->convertToYFromIndex(landmark.second.second)));
                }

            }else{

            }
            range=range+10.0*scan.angle_increment;
        }*/
        std::cout<<__LINE__<<"    "<<__FUNCTION__<<"   "<<__FILE__<<std::endl;
    }

    void resampleStage(std::vector<Particle<geometry_msgs::Pose>> &ps){
        std::vector<Particle<geometry_msgs::Pose>> qg=ps;
        std::sort(qg.begin(),qg.end());

        ssch.prepare(&qg);
        //http://www.cs.utexas.edu/users/qr/software/kld-sampling.html
        w_slow=w_slow+ alpha_slow*(smc->w_mean-w_slow);
        w_fast=w_fast+ alpha_fast*(smc->w_mean-w_fast);

        //        pmin.first=100;
        //        pmin.second=100;

        //        pmax.first=-100;
        //        pmax.second=-100;

        //        for(int i=0;i<ps.size()/2;i++){
        //            pmin.first=std::min(pmin.first,ps[i].state.position.x);
        //            pmin.second=std::min(pmin.second,ps[i].state.position.y);
        //            pmax.first=std::max(pmax.first,ps[i].state.position.x);
        //            pmax.second=std::max(pmax.second,ps[i].state.position.y);
        //        }



        //        Xdis=std::uniform_real_distribution<double>(pmin.first,pmax.first);
        //        Ydis=std::uniform_real_distribution<double>(pmin.second,pmax.second);
        //https://github.com/Hincoin/augmented_mcl/blob/master/augmentedmcl.cpp
        //http://blog.cs4u.us/2014/08/surviving-kidnapping-particle-filter.html
        //augmented MCL
        //        std::uniform_int_distribution<int> sid(0,qg.size()/1.5);

        double th=std::max(0.0,1.0-w_fast/w_slow);

        //        std::cout<<w_slow<<"  "<<w_fast<<"  "<<smc->w_mean<<"  "<<th<<std::endl;

        for(unsigned int i=0;i<ps.size();i++){

            if(augmented(gen)<th){
                //ps[i]=qg[sid(gen)];
                sampleStage(ps[i]);
            }else{

                //                ps[i]=qg[sid(gen)];
                ps[i]=ssch.pick();

                //            sampleStage(ps[i]);
            }
        }
    }

    void fullImplementation(std::vector<Particle<geometry_msgs::Pose>> &ps,NumericArray<double,3>& data,std::vector<Voxel3D> & laser){

        std::vector<Particle<geometry_msgs::Pose>> qg=ps;

        std::sort(qg.begin(),qg.begin()+smc->numParticles);

        ssch.prepare(&qg);

        //        ps.clear();

        // KLD version
        double Mx= kldHandle.maxParticles;
        int M=0;
        double k=0;
        //std::uniform_int_distribution<int> sid(0,smc->numParticles/2-1);

        kldHandle.reset();
        do{

            ps[M]=ssch.pick();
            //            ps[M]=qg[sid(gen)];
            predictStage(ps[M],data);
            updateStage(ps[M],laser);


            if(kldHandle.inEmptyBin(ps[M].state)){
                k=k+1.0;
                kldHandle.set(ps[M].state);

                if(M>=kldHandle.minParticles){
                    //                   if(k>1){   book is wrong here see paper  http://www.robots.ox.ac.uk/~cvrg/hilary2005/adaptive.pdf

                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);

                }

            }

            M++;

            if(M>=kldHandle.maxParticles){
                break;
            }
        }while(M<Mx || M<kldHandle.minParticles);

        smc->numParticles=M;
        smc->normalize();

        //std::cout<<smc->numParticles<<std::endl;
    }

    // nova funcao
    void fullImplementation3(std::vector<Particle<geometry_msgs::Pose>> &ps,std::unordered_map<std::string,NumericArray<double,3> >& data, std::vector<Voxel3D>& laser){

        std::vector<Particle<geometry_msgs::Pose>> qg=ps;

        std::sort(qg.begin(),qg.begin()+smc->numParticles);

        ssch.prepare(&qg);

        // KLD version
        double Mx= kldHandle.maxParticles;
        int M=0;
        double k=0;
        int op = 0;   // variável que guarda a opção para o tipo de teste a realizar
        double ale = 0; // variável que guarda número aleatório para o método 1-alpha
        double d_pf_GPS = 0;
        double GPS_er = 0;

            std::uniform_real_distribution<double> uniform(0.0,1.0);
                std::default_random_engine gen;


        bool useLaser=data.find("laserodom")!=data.end();


 

        kldHandle.reset();

        do{
            

            ps[M]=ssch.pick();

            if(setlast){
                if(uniform(gen)<0.3){   //30% particles are created from the optimized pose, others are resampled
                   ps[M].state=lastPose;
                }
            }
            if(useLaser){
 
                if(uniform(gen)<0.25){ //25% particles  move acordingly to laserodom              
                                predictStage(ps[M],data["laserodom"]);

                }else{
                                predictStage(ps[M],data["odom_pf"]);

                }

            }else{
            predictStage(ps[M],data["odom_pf"]);
            }
 
            //            updateStage2(ps[M],laser);
            updateStage3(ps[M],laser);

            if(kldHandle.inEmptyBin(ps[M].state)){
                k=k+1.0;
                kldHandle.set(ps[M].state);

                if(M>=kldHandle.minParticles){
                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);

                }
            }

            M++;

            if(M>=kldHandle.maxParticles){
                break;
            }
        }while(M<Mx || M<kldHandle.minParticles);




        if(ale == 3 || ale == 4 || ale == 5){
            last_GPS[0] = data["GPS_pf"][0];
            last_GPS[1] = data["GPS_pf"][1];
            usingGPS=true;
        }else{
            usingGPS=false;
        }

        smc->numParticles=M;
        smc->normalize();

        //std::cout<<smc->numParticles<<std::endl;

        auto pose=getPose();
        //std::cout<<pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<" "<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w<<std::endl;

        setlast=false;
    }



  void fullImplementation3_v2(std::vector<Particle<geometry_msgs::Pose>> &ps,std::unordered_map<std::string,NumericArray<double,3> >& data, std::vector<geometry_msgs::Point32>& laser){
        //tic();

        std::vector<Particle<geometry_msgs::Pose>> qg=ps;
        //toc();

        std::sort(qg.begin(),qg.begin()+smc->numParticles);
        //toc();

        ssch.prepare(&qg);
        //toc();

        //std::mutex mtx;
        //std::mutex mtx2;


        omp_set_num_threads(8);


        // KLD version
        double Mx= kldHandle.maxParticles;
        int M=0;
        double k=0;
        int op = 0;   // variável que guarda a opção para o tipo de teste a realizar
        double ale = 0; // variável que guarda número aleatório para o método 1-alpha
        double d_pf_GPS = 0;
        double GPS_er = 0;

            std::uniform_real_distribution<double> uniform(0.0,1.0);
                std::default_random_engine gen;


        //toc();



        kldHandle.reset();

        //toc();


        //double ttime=0;
        bool breakHere=false;

        auto datapf=data["odom_pf"];

        for(unsigned int i=0;i<kldHandle.maxParticles;i=i+50){
                   if(breakHere){
                      break;
                   }

        int id=M;
        M=M+50;

        for(unsigned int kx=id;kx<(id+50);kx++){
            ps[kx]=ssch.pick();
            if(setlast){
                if(uniform(gen)<0.3){   //30% particles are generated from the optimized pose
               ps[kx].state=lastPose;
            }
            }
            predictStage(ps[kx],datapf);

            if(kldHandle.inEmptyBin(ps[kx].state)){
                k=k+1.0;
                kldHandle.set(ps[kx].state);

                if(kx>=kldHandle.minParticles){
                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);

                }
            }

            if(kx>Mx && kx>kldHandle.minParticles){
                breakHere=true;
            }


            if(kx>=kldHandle.maxParticles){
                breakHere=true; 
            }

        }
    }



        #pragma omp  parallel for
        for(unsigned int i=0;i<kldHandle.maxParticles;i=i+50){

    
            batchprocessStage(ps,laser,i,50);


        }

 
 


        smc->numParticles=M;
        smc->normalize();


        setlast=false;
    }



    void batchprocessStage(std::vector<Particle<geometry_msgs::Pose>> &ps,std::vector<geometry_msgs::Point32>& laser,int id, int count){

        for(unsigned int i=id;i<count;i++){
            updateStage3_v2(ps[i],laser);
        }

    }

































  void fullImplementation3_v2_old(std::vector<Particle<geometry_msgs::Pose>> &ps,std::unordered_map<std::string,NumericArray<double,3> >& data, std::vector<geometry_msgs::Point32>& laser){
        //tic();

        std::vector<Particle<geometry_msgs::Pose>> qg=ps;
        //toc();

        std::sort(qg.begin(),qg.begin()+smc->numParticles);
        //toc();

        ssch.prepare(&qg);
        //toc();

        //std::mutex mtx;
        //std::mutex mtx2;


omp_set_num_threads(6);
        omp_lock_t counterlock;
        omp_lock_t binlock;
omp_init_lock(&counterlock);
omp_init_lock(&binlock);


        // KLD version
        double Mx= kldHandle.maxParticles;
        int M=0;
        double k=0;
        int op = 0;   // variável que guarda a opção para o tipo de teste a realizar
        double ale = 0; // variável que guarda número aleatório para o método 1-alpha
        double d_pf_GPS = 0;
        double GPS_er = 0;

            std::uniform_real_distribution<double> uniform(0.0,1.0);
                std::default_random_engine gen;


        //toc();



        kldHandle.reset();

        //toc();


        //double ttime=0;
        bool breakHere=false;
        #pragma omp  parallel for
        for(unsigned int i=0;i<kldHandle.maxParticles;i=i+50){

       if(!breakHere){

        omp_set_lock(&counterlock);

        int id=M;
        M=M+50;

        omp_unset_lock(&counterlock);


        for(unsigned int kx=id;kx<(id+50);kx++){

            
            ps[kx]=ssch.pick();
            if(setlast){
                if(uniform(gen)<0.3){   //30% particles are generated from the optimized pose
               ps[kx].state=lastPose;
            }
            }
            predictStage(ps[kx],data["odom_pf"]);

            //            updateStage2(ps[M],laser);

            //             tic();

            updateStage3_v2(ps[kx],laser);

             //            ttime+=toc(2);


            //confirmar este codigo e  alterar locking do empty bin
            omp_set_lock(&binlock);

            if(kldHandle.inEmptyBin(ps[kx].state)){
                k=k+1.0;
                kldHandle.set(ps[kx].state);

                if(kx>=kldHandle.minParticles){
                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);

                }
            }
            //k++;
            omp_unset_lock(&binlock);


            if(kx>Mx && kx>kldHandle.minParticles){
                breakHere=true;
            //    break;
            }


            if(kx>=kldHandle.maxParticles){
                breakHere=true;
            //    break; 
            }
            }



         }

         }


         omp_destroy_lock(&counterlock);
         omp_destroy_lock(&binlock);




/*
        do{
            

            
            ps[M]=ssch.pick();
            //if(setlast){
            //    if(uniform(gen)<0.3){   //30% particles are generated from the optimized pose
            //       ps[M].state=lastPose;
            //    }
            //}
            predictStage(ps[M],data["odom_pf"]);

            //            updateStage2(ps[M],laser);

            //             tic();

            updateStage3_v2(ps[M],laser);

             //            ttime+=toc(2);

            if(kldHandle.inEmptyBin(ps[M].state)){
                k=k+1.0;
                kldHandle.set(ps[M].state);

                if(M>=kldHandle.minParticles){
                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);

                }
            }

            M++;

            if(M>=kldHandle.maxParticles){
                break;
            }
        }while(M<Mx || M<kldHandle.minParticles);
        */

        //std::cout<<"time "<<ttime<<std::endl;


        //if(ale == 3 || ale == 4 || ale == 5){
        //    last_GPS[0] = data["GPS_pf"][0];
        //    last_GPS[1] = data["GPS_pf"][1];
        //    usingGPS=true;
        //}else{
        //    usingGPS=false;
        //}

        smc->numParticles=M;
        smc->normalize();

        //std::cout<<smc->numParticles<<std::endl;

        //auto pose=getPose();
        //std::cout<<pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<" "<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w<<std::endl;

        setlast=false;
    }

















































    void fullImplementation4(std::vector<Particle<geometry_msgs::Pose>> &ps,std::unordered_map<std::string,NumericArray<double,3> >& data, std::vector<Voxel3D>& laser,geometry_msgs::Pose po,int count=100){

        std::vector<Particle<geometry_msgs::Pose>> qg=ps;

        std::sort(qg.begin(),qg.begin()+smc->numParticles);

        ssch.prepare(&qg);

        // KLD version
        double Mx= kldHandle.maxParticles;
        int M=0;
        double k=0;
        int op = 0;   // variável que guarda a opção para o tipo de teste a realizar
        double ale = 0; // variável que guarda número aleatório para o método 1-alpha
        double d_pf_GPS = 0;
        double GPS_er = 0;

        if(data.find("odom_pf")!=data.end()){
            op = 1;
        }
        if(data.find("odom_pf")!=data.end() && data.find("GPS_pf")!=data.end()){
            op = 3;
            //d_pf_GPS = sqrt((data["GPS_pf"][0] - last_pf[0])*(data["GPS_pf"][0] - last_pf[0]) + (data["GPS_pf"][1] - last_pf[1])*(data["GPS_pf"][1] - last_pf[1]));
            GPS_er = sqrt((data["GPS_pf"][0] - last_GPS[0])*(data["GPS_pf"][0] - last_GPS[0]) + (data["GPS_pf"][1] - last_GPS[1])*(data["GPS_pf"][1] - last_GPS[1]));
        }
        if(data.find("ekf_pf")!=data.end()){
            op = 2; // Esta opção fica para todos os testes só com ekf independentemente da informação que venha de lá
        }
        if(data.find("ekf_pf")!=data.end() && data.find("GPS_pf")!=data.end()){
            op = 4;
            //d_pf_GPS = sqrt((data["GPS_pf"][0] - last_pf[0])*(data["GPS_pf"][0] - last_pf[0]) + (data["GPS_pf"][1] - last_pf[1])*(data["GPS_pf"][1] - last_pf[1]));
            GPS_er = sqrt((data["GPS_pf"][0] - last_GPS[0])*(data["GPS_pf"][0] - last_GPS[0]) + (data["GPS_pf"][1] - last_GPS[1])*(data["GPS_pf"][1] - last_GPS[1]));

        }
        if (data.find("odom_pf")!=data.end() && data.find("ekf_pf")!=data.end() && data.find("GPS_pf")!=data.end()){
            op = 5;
            //d_pf_GPS = sqrt((data["GPS_pf"][0] - last_pf[0])*(data["GPS_pf"][0] - last_pf[0]) + (data["GPS_pf"][1] - last_pf[1])*(data["GPS_pf"][1] - last_pf[1]))/2;
            GPS_er = sqrt((data["GPS_pf"][0] - last_GPS[0])*(data["GPS_pf"][0] - last_GPS[0]) + (data["GPS_pf"][1] - last_GPS[1])*(data["GPS_pf"][1] - last_GPS[1]));
        }

        //std::cout << op <<"  "<<GPS_er<< std::endl;
        //std::cout<<"data "<<data["odom_pf"].toString()<<std::endl;

        kldHandle.reset();
        int ct=0;


        do{

            if(ct<=count){
                ps[M]=ssch.pick();
                ps[M].state=po;
                ct++;
            }
            switch (op) {
            case 1: {
                if(ct>count){
                    ps[M]=ssch.pick();
                }

                predictStage(ps[M],data["odom_pf"]);
                break;
            }

            case 2: {
                if(ct>count){
                    ps[M]=ssch.pick();
                }
                predictStage(ps[M],data["ekf_pf"]);
                break;
            }

            case 3: {
                if(ct>count){
                    ps[M]=ssch.pick();
                }
                ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                if(ale < alphaGPS){
                    predictStage(ps[M],data["odom_pf"]);}

                else{     if(op_GPS == 1) {

                        GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                        predictStage(ps[M],data["odom_pf"]);

                    } else{
                        if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er < 0.5)){
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }



                        else{  predictStage(ps[M],data["odom_pf"]);}
                    }
                }
                break;
            }

            case 4: {
                if(ct>count){
                    ps[M]=ssch.pick();
                }
                ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                if(ale < alphaGPS){

                    predictStage(ps[M],data["ekf_pf"]);}

                else{

                    if(op_GPS == 1) {
                        GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                        predictStage(ps[M],data["ekf_pf"]);
                    }
                    else{
                        if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er < 0.5)){
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }
                        else{  predictStage(ps[M],data["ekf_pf"]);}
                    }
                }
                break;
            }

            case 5: {
                ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                if(ct>count){
                    ps[M]=ssch.pick();
                }
                if(ale < 0.8){
                    predictStage(ps[M],data["odom_pf"]);}

                else if(ale < 0.95 && ale >= 0.8){
                    GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                    predictStage(ps[M],data["ekf_pf"]);}

                else{
                    predictStage(ps[M],data["ekf_pf"]);}

                break;
            }

            }


            //            updateStage2(ps[M],laser);
            updateStage3(ps[M],laser);

            if(kldHandle.inEmptyBin(ps[M].state)){
                k=k+1.0;
                kldHandle.set(ps[M].state);

                if(M>=kldHandle.minParticles){
                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);

                }
            }

            M++;

            if(M>=kldHandle.maxParticles){
                break;
            }
        }while(M<Mx || M<kldHandle.minParticles);




        if(ale == 3 || ale == 4 || ale == 5){
            last_GPS[0] = data["GPS_pf"][0];
            last_GPS[1] = data["GPS_pf"][1];
            usingGPS=true;
        }else{
            usingGPS=false;
        }

        smc->numParticles=M;
        smc->normalize();

        //std::cout<<smc->numParticles<<std::endl;

    }





   geometry_msgs::Pose lastPose;
   bool setlast=false;

   //void updatePreviousPrediction(geometry_msgs::Pose pose){
   // lastPose=pose;
    //setlast=true;
   //}





    void fullImplementation2(std::vector<Particle<geometry_msgs::Pose>> &ps,std::unordered_map<std::string,NumericArray<double,3> >& data,std::vector<Voxel3D> & laser){

        std::vector<Particle<geometry_msgs::Pose>> qg=ps;

        std::sort(qg.begin(),qg.begin()+smc->numParticles);

        ssch.prepare(&qg);

        // KLD version
        double Mx= kldHandle.maxParticles;
        int M=0;
        double k=0;
        int op = 0;   // variável que guarda a opção para o tipo de teste a realizar
        double ale = 0; // variável que guarda número aleatório para o método 1-alpha
        double d_pf_GPS = 0;
        double GPS_er = 0;

        if(data.find("odom_pf")!=data.end()){
            op = 1;
        }
        if(data.find("odom_pf")!=data.end() && data.find("GPS_pf")!=data.end()){
            op = 3;
            //d_pf_GPS = sqrt((data["GPS_pf"][0] - last_pf[0])*(data["GPS_pf"][0] - last_pf[0]) + (data["GPS_pf"][1] - last_pf[1])*(data["GPS_pf"][1] - last_pf[1]));
            GPS_er = sqrt((data["GPS_pf"][0] - last_GPS[0])*(data["GPS_pf"][0] - last_GPS[0]) + (data["GPS_pf"][1] - last_GPS[1])*(data["GPS_pf"][1] - last_GPS[1]));
        }
        if(data.find("ekf_pf")!=data.end()){
            op = 2; // Esta opção fica para todos os testes só com ekf independentemente da informação que venha de lá
        }
        if(data.find("ekf_pf")!=data.end() && data.find("GPS_pf")!=data.end()){
            op = 4;
            //d_pf_GPS = sqrt((data["GPS_pf"][0] - last_pf[0])*(data["GPS_pf"][0] - last_pf[0]) + (data["GPS_pf"][1] - last_pf[1])*(data["GPS_pf"][1] - last_pf[1]));
            GPS_er = sqrt((data["GPS_pf"][0] - last_GPS[0])*(data["GPS_pf"][0] - last_GPS[0]) + (data["GPS_pf"][1] - last_GPS[1])*(data["GPS_pf"][1] - last_GPS[1]));

        }
        if (data.find("odom_pf")!=data.end() && data.find("ekf_pf")!=data.end() && data.find("GPS_pf")!=data.end()){
            op = 5;
            //d_pf_GPS = sqrt((data["GPS_pf"][0] - last_pf[0])*(data["GPS_pf"][0] - last_pf[0]) + (data["GPS_pf"][1] - last_pf[1])*(data["GPS_pf"][1] - last_pf[1]))/2;
            GPS_er = sqrt((data["GPS_pf"][0] - last_GPS[0])*(data["GPS_pf"][0] - last_GPS[0]) + (data["GPS_pf"][1] - last_GPS[1])*(data["GPS_pf"][1] - last_GPS[1]));
        }

        //std::cout << op <<"  "<<GPS_er<< std::endl;
        /*lfe.extractFeatures(laser);

        std::cout<<lfe.Corners.size()<<" "<<lfe.Lines.size()<<std::endl;*/

        kldHandle.reset();

        do{
            if(useFeatures){

                switch (op) {
                case 1: {
                    ps[M]=ssch.pick();
                    predictStage2(ps[M],data["odom_pf"]);
                    break;
                }

                case 2: {
                    ps[M]=ssch.pick();
                    predictStage2(ps[M],data["ekf_pf"]);
                    break;
                }

                case 3: {

                    ps[M]=ssch.pick();
                    ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                    if(ale < alphaGPS){
                        predictStage2(ps[M],data["odom_pf"]);}

                    else{

                        if(op_GPS == 1) {
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage2(ps[M],data["odom_pf"]);
                        }else{
                            if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er <0.5)){
                                GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            }else{  predictStage2(ps[M],data["odom_pf"]);}
                        }
                    }
                    break;
                }

                case 4: {

                    ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                    ps[M]=ssch.pick();
                    if(ale < alphaGPS){
                        predictStage2(ps[M],data["ekf_pf"]);}
                    else{
                        if(op_GPS == 1) {
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage2(ps[M],data["ekf_pf"]);
                        }else{
                            if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er< 0.5)){
                                GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }

                            else{  predictStage2(ps[M],data["ekf_pf"]);}
                        }
                    }
                    break;
                }

                case 5: {
                    ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                    ps[M]=ssch.pick();
                    if(ale < 0.8){
                        predictStage2(ps[M],data["odom_pf"]);}

                    else if(ale < 0.95 && ale >= 0.8){
                        GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                        predictStage2(ps[M],data["ekf_pf"]);}

                    else{
                        predictStage2(ps[M],data["ekf_pf"]);}
                    break;
                }

                }

            }else{
                switch (op) {
                case 1: {
                    ps[M]=ssch.pick();
                    predictStage(ps[M],data["odom_pf"]);
                    break;
                }

                case 2: {
                    ps[M]=ssch.pick();
                    predictStage(ps[M],data["ekf_pf"]);
                    break;
                }

                case 3: {
                    ps[M]=ssch.pick();

                    ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                    if(ale < alphaGPS){
                        predictStage(ps[M],data["odom_pf"]);}

                    else{     if(op_GPS == 1) {

                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage(ps[M],data["odom_pf"]);

                        } else{
                            if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er < 0.5)){
                                GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }



                            else{  predictStage(ps[M],data["odom_pf"]);}
                        }
                    }
                    break;
                }

                case 4: {
                    ps[M]=ssch.pick();


                    ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                    if(ale < alphaGPS){

                        predictStage(ps[M],data["ekf_pf"]);}

                    else{

                        if(op_GPS == 1) {
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage(ps[M],data["ekf_pf"]);
                        }
                        else{
                            if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er < 0.5)){
                                GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }
                            else{  predictStage(ps[M],data["ekf_pf"]);}
                        }
                    }


                    break;
                }

                case 5: {
                    ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                    ps[M]=ssch.pick();
                    if(ale < 0.8){
                        predictStage(ps[M],data["odom_pf"]);}

                    else if(ale < 0.95 && ale >= 0.8){
                        GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                        predictStage(ps[M],data["ekf_pf"]);}

                    else{
                        predictStage(ps[M],data["ekf_pf"]);}



                    break;
                }

                }
            }

            /*


            if(useFeatures){

  switch (op) {
            case 1: {
                ps[M]=ssch.pick();
                predictStage2(ps[M],data["odom_pf"]);
                break;
                    }

            case 2: {
                ps[M]=ssch.pick();
                predictStage2(ps[M],data["ekf_pf"]);
                break;
                    }

            case 3: {

        if(op_GPS == 1) {
             ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                 if(ale < alphaGPS){  ps[M]=ssch.pick();
                        predictStage2(ps[M],data["odom_pf"]);}

                     else{
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage2(ps[M],data["odom_pf"]);

                         }
         }

        else{
                        ps[M]=ssch.pick();
                     if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er <0.5)){
                                    GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }

                     else{  predictStage2(ps[M],data["odom_pf"]);}
        }

                break;
                    }

            case 4: {

         if(op_GPS == 1) {
             ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                 if(ale < alphaGPS){  ps[M]=ssch.pick();

                        predictStage2(ps[M],data["ekf_pf"]);}



                     else{

                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);

                            predictStage2(ps[M],data["ekf_pf"]);



                         }
         }

        else{
                        ps[M]=ssch.pick();
                     if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er< 0.5)){
                                    GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }

                     else{  predictStage2(ps[M],data["ekf_pf"]);}
        }
                break;
                    }

            case 5: {
                ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                if(ale < 0.8){  ps[M]=ssch.pick();
                    predictStage2(ps[M],data["odom_pf"]);}

                else if(ale < 0.95 && ale >= 0.8){
                        GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                        predictStage2(ps[M],data["ekf_pf"]);}

                else{
                    ps[M]=ssch.pick();
                    predictStage2(ps[M],data["ekf_pf"]);}



                break;
                     }

            }

            }else{



            switch (op) {
            case 1: {
                ps[M]=ssch.pick();
                predictStage(ps[M],data["odom_pf"]);
                break;
                    }

            case 2: {
                ps[M]=ssch.pick();
                predictStage(ps[M],data["ekf_pf"]);
                break;
                    }

            case 3: {

        if(op_GPS == 1) {
             ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                 if(ale < alphaGPS){  ps[M]=ssch.pick();
                        predictStage(ps[M],data["odom_pf"]);}

                     else{
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage(ps[M],data["odom_pf"]);

                         }
         }

        else{
                        ps[M]=ssch.pick();
                     if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er < 0.5)){
                                    GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }



                     else{  predictStage(ps[M],data["odom_pf"]);}
        }

                break;
                    }

            case 4: {

         if(op_GPS == 1) {
             ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                 if(ale < alphaGPS){
                    ps[M]=ssch.pick();

                        predictStage(ps[M],data["ekf_pf"]);}

                     else{
                            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                            predictStage(ps[M],data["ekf_pf"]);

                         }
         }

        else{
                        ps[M]=ssch.pick();
                     if((sqrt((data["GPS_pf"][0] - ps[M].state.position.x)*(data["GPS_pf"][0] - ps[M].state.position.x) + (data["GPS_pf"][1] - ps[M].state.position.y)*(data["GPS_pf"][1] - ps[M].state.position.y)) > (thdGPS) && GPS_er < 0.5)){
                           GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]); }
                    else{  predictStage(ps[M],data["ekf_pf"]);}
        }
                break;
                    }

            case 5: {
                ale = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                if(ale < 0.8){  ps[M]=ssch.pick();
                    predictStage(ps[M],data["odom_pf"]);}

                else if(ale < 0.95 && ale >= 0.8){
            GPSPart(ps[M],data["GPS_pf"][0],data["GPS_pf"][1]);
                        predictStage(ps[M],data["ekf_pf"]);}

                else{
                    ps[M]=ssch.pick();
                    predictStage(ps[M],data["ekf_pf"]);}



                break;
                     }

            }
            }
*/

            //Garrote::Time::show("proc_time");

            // updateStage(ps[M],laser);
            //std::cout << op << std::endl;
            /*
            if(useFeatures){
                updateStage2(ps[M],lfe.Corners,lfe.Lines);
            }else{
                updateStage2(ps[M],lfe.Points);
            }*/


            if(kldHandle.inEmptyBin(ps[M].state)){
                k=k+1.0;
                kldHandle.set(ps[M].state);

                if(M>=kldHandle.minParticles){
                    //                   if(k>1){   book is wrong here see paper  http://www.robots.ox.ac.uk/~cvrg/hilary2005/adaptive.pdf

                    Mx=((k-1.0)/(2.0*kldHandle.epsilon))*pow((1.0-2.0/(9.0*(k-1.0))+sqrt(2.0/(9.0*(k-1.0)))*kldHandle.zeta),3);
                }
            }

            M++;

            if(M>=kldHandle.maxParticles){
                break;
            }
        }while(M<Mx || M<kldHandle.minParticles);


        if(ale == 3 || ale == 4 || ale == 5)
        {
            last_GPS[0] = data["GPS_pf"][0];
            last_GPS[1] = data["GPS_pf"][1];
            usingGPS=true;
        }else{
            usingGPS=false;
        }

        smc->numParticles=M;
        smc->normalize();

        //std::cout<<smc->numParticles<<std::endl;
        std::cout<<__LINE__<<"    "<<__FUNCTION__<<"   "<<__FILE__<<std::endl;
    }


    sensor_msgs::PointCloud getPoints(){
        sensor_msgs::PointCloud out;
        out.header.frame_id="map";

        for(unsigned int i=0; i < smc->numParticles; i++){
            geometry_msgs::Point32 p;
            p.x=smc->particles[i].state.position.x;
            p.y=smc->particles[i].state.position.y;
            p.z=smc->particles[i].state.position.z;
            out.points.push_back(p);
        }

        return out;
    }
    sensor_msgs::PointCloud getPoints2(double scale){
        sensor_msgs::PointCloud out;
        out.header.frame_id="map";

        double maxx=0;
        for(unsigned int i=0; i < smc->numParticles; i++){

            maxx=std::max(maxx,smc->particles[i].w);
        }

        for(unsigned int i=0; i < smc->numParticles; i++){
            geometry_msgs::Point32 p;
            p.x=smc->particles[i].state.position.x;
            p.y=smc->particles[i].state.position.y;
            p.z=(smc->particles[i].w/maxx)*scale;
            out.points.push_back(p);
        }

        return out;
    }


    std::vector<geometry_msgs::Pose> getParticles(double scale){
        std::vector<geometry_msgs::Pose> points;

        //double maxx=0;
        //for(unsigned int i=0; i < smc->numParticles; i++){

 //           maxx=std::max(maxx,smc->particles[i].w);
 //       }

        for(unsigned int i=0; i < smc->numParticles; i++){
            //geometry_msgs::Pose p=smc->particles[i].state;
            //p.position.z=(smc->particles[i].w/maxx)*scale;
            points.push_back(smc->particles[i].state);
        }

        return points;
    }







    /* sensor_msgs::PointCloud getCorners(){
        sensor_msgs::PointCloud out;
        out.header.frame_id="map";

        auto px=getPose();
        Transform tf_particle=Transform::byPose(px)*tf;

        for(unsigned int i=0;i<lfe.Corners.size();i++){

            Point pt=tf_particle.point(lfe.Corners[i].x,lfe.Corners[i].y);
            geometry_msgs::Point32 p;
            p.x=pt.x;
            p.y=pt.y;
            p.z=0.1;
            out.points.push_back(p);
        }
        return out;
    }*/

       //nova funçao
       visualization_msgs::MarkerArray getVoxelROS(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

           //std::cout<<"Size: "<<scan.size()<<std::endl;
           visualization_msgs::Marker temp;
           visualization_msgs::MarkerArray out;
           temp.header.frame_id=frame;
           //temp.header.stamp=ros::Time::now();
           temp.ns=np;
           temp.pose.orientation.w=1;
           temp.action = visualization_msgs::Marker::ADD;

           temp.type=visualization_msgs::Marker::CUBE;
           temp.scale.x=0.05;
           temp.scale.y=0.05;
           temp.scale.z= 0.05;

           temp.color.r=r;
           temp.color.g=g;
           temp.color.b=b;
           temp.color.a=1.0;

           for(unsigned int i=0;i<scan.size();i++){
               temp.id=i;
               temp.pose.position.x=scan[i].x;
               temp.pose.position.y=scan[i].y;
               temp.pose.position.z=scan[i].z;
               out.markers.push_back(temp);
           }
           //std::cout<<temp.pose.position.x<<" "<<temp.pose.position.y<<" "<<temp.pose.position.z<<" "<<temp.scale.z<<std::endl;
           return out;
       }


       //nova funçao
       visualization_msgs::MarkerArray getVoxel2ROS(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

           //std::cout<<"Size: "<<scan.size()<<std::endl;
           visualization_msgs::Marker temp;
           visualization_msgs::MarkerArray out;
           temp.header.frame_id=frame;
           //temp.header.stamp=ros::Time::now();
           temp.ns=np;
           temp.pose.orientation.w=1;
           temp.action = visualization_msgs::Marker::ADD;

           temp.type=visualization_msgs::Marker::POINTS; //CUBE
           temp.scale.x=0.05;
           temp.scale.y=0.05;
           temp.scale.z= 0.05;

           temp.color.r=r;
           temp.color.g=g;
           temp.color.b=b;
           temp.color.a=1.0;

           auto px=getPose();
           Transform tf_particle=Transform::byPose(px)*tf;

           for(unsigned int i=0;i<scan.size();i++){
               temp.id=i;
               Point pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);

               temp.pose.position.x=pt.x;
               temp.pose.position.y=pt.y;
               temp.pose.position.z=pt.z;
 
               out.markers.push_back(temp);
           }
           //std::cout<<temp.pose.position.x<<" "<<temp.pose.position.y<<" "<<temp.pose.position.z<<" "<<temp.scale.z<<std::endl;
           return out;
       }


              //nova funçao
       visualization_msgs::MarkerArray getVoxel2ROSGlobal(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=0,float g=1,float b=0){

           //std::cout<<"Size: "<<scan.size()<<std::endl;
           visualization_msgs::Marker temp;
           visualization_msgs::MarkerArray out;
           temp.header.frame_id=frame;
           //temp.header.stamp=ros::Time::now();
           temp.ns=np;
           temp.pose.orientation.w=1;
           temp.action = visualization_msgs::Marker::ADD;

           temp.type=visualization_msgs::Marker::CUBE;
           temp.scale.x=0.05;
           temp.scale.y=0.05;
           temp.scale.z= 0.05;

           temp.color.r=r;
           temp.color.g=g;
           temp.color.b=b;
           temp.color.a=1.0;

           auto px=getPose();
           Transform tf_particle=Transform::byPose(px)*tf;

           for(unsigned int i=0;i<scan.size();i++){
               temp.id=i;
 
               temp.pose.position.x=scan[i].x;
               temp.pose.position.y=scan[i].y;
               temp.pose.position.z=scan[i].z;
 
               out.markers.push_back(temp);
           }
           //std::cout<<temp.pose.position.x<<" "<<temp.pose.position.y<<" "<<temp.pose.position.z<<" "<<temp.scale.z<<std::endl;
           return out;
       }

    template <typename T>
    std::vector<T> getVoxel2 (std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

        std::vector<T> out;
        T temp;
        auto px=getPose();
        Transform tf_particle=Transform::byPose(px)*tf;

        for(unsigned int i=0;i<scan.size();i++){
            Point pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);

            temp.x=pt.x;
            temp.y=pt.y;
            temp.z=pt.z;

            out.push_back(temp);
        }
        return out;
    }
    template <typename T>
    std::vector<T> getVoxel2a(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

        std::vector<T> out;
        T temp;
        auto px=getPose();
        Transform tf_particle=Transform::byPose(px)*tf;

        for(unsigned int i=0;i<scan.size();i++){
            Point pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);


            temp.x=pt.x;
            temp.y=pt.y;
            temp.z=pt.z;


            out.push_back(temp);
        }
        return out;
    }

    template <typename T>
    std::vector<T> getVoxel4a(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

        std::vector<T> out;
        T temp;
        auto px=getBestPose();
        Transform tf_particle=Transform::byPose(px)*tf;

        for(unsigned int i=0;i<scan.size();i++){
            Point pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);


            temp.x=pt.x;
            temp.y=pt.y;
            temp.z=pt.z;


            out.push_back(temp);
        }
        return out;
    }
    template <typename T>
    std::vector<T> getVoxel4(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

        std::vector<T> out;
        T temp;
        auto px=getBestPose();
        Transform tf_particle=Transform::byPose(px)*tf;

        for(unsigned int i=0;i<scan.size();i++){
            Point pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);


            temp.x=pt.x;
            temp.y=pt.y;
            temp.z=pt.z;


            out.push_back(temp);
        }
        return out;
    }


    template <typename T>
    std::vector<T> getVoxel3(std::string frame,std::string np,const std::vector<Voxel3D> &scan, float r=1,float g=0,float b=0){

        std::vector<T> out;
        T temp;
        //        auto px=getPose();
        Transform tf_particle;//=Transform::byPose(px)*tf;

        for(unsigned int i=0;i<scan.size();i++){
            Point pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);

            //            temp.x=pt.x;
            //            temp.y=pt.y;
            //            temp.minz=pt.z+scan[i].zmin;
            //            temp.maxz=pt.z+scan[i].zmax;

            temp.x=pt.x;
            temp.y=pt.y;
            temp.z=pt.z;

            out.push_back(temp);
        }
        return out;
    }
    // Confidence Interval function
    void Update_map(){

        double x_m = 0;
        double y_m = 0;
        double yaw = 0;
        double yaw1_m = 0;
        double yaw2_m = 0;
        double theta_m = 0;

        double x_s = 0;
        double y_s = 0;
        double yaw1_s = 0;
        double yaw2_s = 0;
        double theta_s = 0;

        double x_ci_min = 0;
        double x_ci_max = 0;
        double y_ci_min = 0;
        double y_ci_max = 0;
        double theta_ci_min = 0;
        double theta_ci_max = 0;

        double AreaCI = 0;
        double Ang_coneCI = 0;

        update_flag = true;

        for(unsigned int i=0;i<smc->numParticles;i++){

            x_m +=smc->particles[i].w*smc->particles[i].state.position.x;
            y_m +=smc->particles[i].w*smc->particles[i].state.position.y;
            yaw = PoseTools::getYaw(smc->particles[i].state);
            yaw1_m+=smc->particles[i].w*std::sin(yaw);
            yaw2_m+=smc->particles[i].w*std::cos(yaw);
        }

        theta_m = std::atan2(yaw1_m,yaw2_m);

        for(unsigned int i=0;i<smc->numParticles;i++){


            x_s += smc->particles[i].w*std::pow((smc->particles[i].state.position.x - x_m),2);
            y_s += smc->particles[i].w*std::pow((smc->particles[i].state.position.y - y_m),2);
            yaw = PoseTools::getYaw(smc->particles[i].state);
            yaw1_s+= smc->particles[i].w*std::pow((std::sin(yaw) - yaw1_m),2);
            yaw2_s+= smc->particles[i].w*std::pow((std::cos(yaw) - yaw2_m),2);

        }



        theta_s = std::atan2(yaw1_s,yaw2_s);

        x_ci_min = x_m - 2.0*std::sqrt(x_s);
        x_ci_max = x_m + 2.0*std::sqrt(x_s);
        y_ci_min = y_m - 2.0*std::sqrt(y_s);
        y_ci_max = y_m + 2.0*std::sqrt(y_s);
        theta_ci_min = std::atan2(yaw1_m - 2.0*std::sqrt(yaw1_s),yaw2_m - 2.0*std::sqrt(yaw2_s));
        theta_ci_max = std::atan2(yaw1_m + 2.0*std::sqrt(yaw1_s),yaw2_m + 2.0*std::sqrt(yaw2_s));


        AreaCI = (x_ci_max-x_ci_min)*(y_ci_max-y_ci_min);
        Ang_coneCI = theta_ci_max - theta_ci_min;

        Ang_coneCI = atan2(sin(Ang_coneCI), cos(Ang_coneCI));


        std::cout << std::endl << "X_CI_MIN =" << x_ci_min << " X_CI_MAX =" << x_ci_max << " Y_CI_MIN =" << y_ci_min << " Y_CI_MAX =" << y_ci_max << " THETA_CI_MIN =" << (theta_ci_min*180/3.14) << " THETA_CI_MAX =" << (theta_ci_max*180/3.14) << std::endl;

        for (const auto& elem : Area)std::cout << elem << " ";
        std::cout << std::endl;
        for (const auto& elem : Ang_cone)std::cout << elem << " ";
        std::cout << std::endl << std::endl;

        double aux = Area.at(0);
        double aux2 = Ang_cone.at(0);

        for (size_t j = 0; j < Area.size() - 1; j++)
        {
            Area.at(j) = Area.at(j + 1);
            Ang_cone.at(j) = Ang_cone.at(j + 1);
        }
        Area.at(Area.size() - 1) = AreaCI;
        Ang_cone.at(Ang_cone.size() - 1) = Ang_coneCI;



        for (const auto& elem2 : Area)std::cout << elem2 << " ";
        std::cout << std::endl;
        for (const auto& elem2 : Ang_cone)std::cout << elem2 << " ";
        std::cout << std::endl << std::endl;

        for (size_t j = 0; j < Area.size() ; j++)
        {
            if (std::abs(Area.at(j)) > AreaCI_threshold || std::abs(Ang_cone.at(j)) > AngleCI_threshold){
                update_flag = false;
                break;
            }
        }


    }

    geometry_msgs::Pose getPose(){

        // mean value of valid w
        geometry_msgs::Pose out;
        out.orientation.w=1;

        double yaw=0;
        double yaw1=0;
        double yaw2=0;
        // std::sort(smc->particles.begin(),smc->particles.begin()+smc->numParticles);

        for(unsigned int i=0;i<smc->numParticles;i++){

 
            out.position.x+=smc->particles[i].w*smc->particles[i].state.position.x;
            out.position.y+=smc->particles[i].w*smc->particles[i].state.position.y;
            out.position.z+=smc->particles[i].w*smc->particles[i].state.position.z;
            yaw=PoseTools::getYaw(smc->particles[i].state);
            yaw1+=smc->particles[i].w*std::sin(yaw);
            yaw2+=smc->particles[i].w*std::cos(yaw);

        }


        yaw=std::atan2(yaw1,yaw2);

        PoseTools::updateYaw(out,yaw);
        last_pf[0] = (double)out.position.x;
        last_pf[1] = (double)out.position.y;
        last_pf[2] = (double)yaw;
        return out;
    }

    geometry_msgs::Pose getPose2(){

        // mean value of valid w
        geometry_msgs::Pose out;
                out.orientation.w=1;
        int ct=0;
        double yaw1=0;
        double yaw2=0;
        double yaw=0;


        std::vector<Particle<geometry_msgs::Pose>> pts = smc->particles;
        int  npt = smc -> numParticles;
        std::sort(pts.begin(),pts.begin()+npt);

        int ct2 = 0;

        double count=0;
        for(unsigned int i=0;i<(npt)/10;i++){

            //std::cout << smc->particles[i].w << std::endl;
            //if(ct2<3){std::cout << pts[i].w << std::endl;}

            if(pts[i].w>0.0){

                out.position.x+=pts[i].w*pts[i].state.position.x;
                out.position.y+=pts[i].w*pts[i].state.position.y;
                out.position.z+=pts[i].w*pts[i].state.position.z;
                yaw=pts[i].w*PoseTools::getYaw(pts[i].state);
                yaw1+=std::sin(yaw);
                yaw2+=std::cos(yaw);
                count+=pts[i].w;

            }

        }


        out.position.x/=(double)count;
        out.position.y/=(double)count;
        out.position.z/=(double)count;
        yaw=std::atan2(yaw1,yaw2);

        PoseTools::updateYaw(out,yaw);

        return out;
    }

    double computeWeight(geometry_msgs::Pose p,std::vector<Voxel3D> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        double w=0;

        Transform tf_particle=Transform::byPose(p)*tf;
        Point pt;

        //        int x0=map->convertToXindex(tf_particle.x());
        //        int y0=map->convertToYindex(tf_particle.y());

        double a,b,c,fx,fy,fz;

        for(unsigned int i=0;i<scan.size();i=i+1){
            pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);



            auto landmark= map->nearest(pt.x,pt.y,pt.z,1); // ray tracing nearest obstacle


            if(landmark.first!=false){
                fx=pt.x-map->convertToXFromIndex(landmark.second.x);
                fy=pt.y-map->convertToYFromIndex(landmark.second.y);
                fz=pt.z-map->convertToZFromIndex(landmark.second.z);
                b=distributionX->operator ()(fx);
                c=distributionY->operator ()(fy);
                a=distributionY->operator ()(fz);
                w+= 1.0*(a*b*c);
            }else{

            }
        }
        //std::cout << w << std::endl;
        return w;
    }





        double computeOverlapv1(geometry_msgs::Pose p,std::vector<Voxel3D> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        double w=0;

        Transform tf_particle=Transform::byPose(p)*tf;
        Point pt;

        //        int x0=map->convertToXindex(tf_particle.x());
        //        int y0=map->convertToYindex(tf_particle.y());

        double a,b,c,fx,fy,fz;

        for(unsigned int i=0;i<scan.size();i=i+1){
            pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);



            auto landmark= map->nearest(pt.x,pt.y,pt.z,1); // ray tracing nearest obstacle


            if(landmark.first!=false){
                fx=pt.x-(landmark.second.x);
                fy=pt.y-(landmark.second.y);
                fz=pt.z-(landmark.second.z);
                //std::cout<<"Distancia: "<<sqrt(fx*fx+fy*fy+fz*fz)<<std::endl;
                if(sqrt(fx*fx+fy*fy+fz*fz)<=0.075){
                  w+=1;
                }
            }else{

            }
        }
        //std::cout << w << std::endl;
        return w/scan.size();
    }

        double computeOverlapv2(geometry_msgs::Pose p,std::vector<Voxel3D> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        double w=0;

        Transform tf_particle=Transform::byPose(p)*tf;
        Point pt;

        //        int x0=map->convertToXindex(tf_particle.x());
        //        int y0=map->convertToYindex(tf_particle.y());

        double a,b,c,fx,fy,fz;

        for(unsigned int i=0;i<scan.size();i=i+1){
            pt=tf_particle.point(scan[i].x,scan[i].y,scan[i].z);



            auto landmark= map->nearest(pt.x,pt.y,pt.z,1); // ray tracing nearest obstacle


            if(landmark.first!=false){
                fx=pt.x-(landmark.second.x);
                fy=pt.y-(landmark.second.y);
                fz=pt.z-(landmark.second.z);
                b=distributionX->operator ()(fx);//b=distributionX->operator ()(sqrt(fx*fx+fy*fy+fz*fz));
                c=distributionY->operator ()(fy);
                a=distributionY->operator ()(fz);
                if(b>1)
                {
                    std::cout<<"Nx Ny Nz: "<<b<<std::endl;
                }
                w+= 1.0*(a*b*c);//a*b*c passou a b
            }else{

            }
        }
        //std::cout << w << std::endl;
        return w/scan.size();
    }



    double computeWeight2(geometry_msgs::Pose p,std::vector<Voxel3D> &scan){
        //https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ParticleFilters.pdf
        //http://www.lancaster.ac.uk/pg/turnerl/PartileFiltering.pdf
        //http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf
        //http://www.irisa.fr/aspi/legland/ref/arulampalam02a.pdf
        //http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf
        //http://ais.informatik.uni-freiburg.de/publications/papers/roewekaemper12iros.pdf
        //http://cdn.intechopen.com/pdfs/6023/InTech-Mobile_robot_localization_using_particle_filters_and_sonar_sensors.pdf

        double w=0;

        /*double range=scan.angle_min;

        Transform tf_particle=Transform::byPose(p)*tf;
        Point pt;

        int x0=map->convertToXindex(tf_particle.x());
        int y0=map->convertToYindex(tf_particle.y());

        for(unsigned int i=0;i<scan.ranges.size();i=i+1){

            if(scan.ranges[i] <scan.range_max && scan.ranges[i] >scan.range_min ) {

                pt=tf_particle.point(scan.ranges[i]*cos(range),scan.ranges[i]*sin(range));
                pt.z=0;

                auto landmark= map->getProbabilityXY(pt.x,pt.y); // ray tracing nearest obstacle

                if(landmark>0.65){
                    w+=1.0;
                }

            }else{

            }
            range=range+1.0*scan.angle_increment;
        }

        //std::cout << w << std::endl;
        */
        return w;

    }

    geometry_msgs::Pose getBestPose(){

        // mean value of valid w
        //geometry_msgs::Pose out;

        //double bestw=0;
        //for(unsigned int i=0; i < smc->numParticles; i++){

     //       if(smc->particles[i].w>bestw){
     //           bestw=smc->particles[i].w;
     //           out=smc->particles[i].state;
     //       }

     //   }

     //   return out;
        return getPose();
    }

};

}
}
} // namespace Garrote

#endif // LOCALIZATION3D_H
