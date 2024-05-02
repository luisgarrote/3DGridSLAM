#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "Localization3D/Localization3D.h"
#include "Localization3D/Map.h"
#include "Localization3D/KeyPointMap.h"
#include "Localization3D/Thread.h"
#include "Localization3D/Event.h"
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include "Localization3D/PointBasedLoopClosure.h"

Grid3D<char> *map;
Grid3D<char> *map2X;
Grid3D<char> *map4X;
Grid3D<char> *localmap;

Garrote::Localization3D::Map3D::LevenbergMarquardtGrid3D optimizer;
Garrote::Localization3D::Map3D::LevenbergMarquardtGrid3D optimizer2;

geometry_msgs::Pose odom_data;           //variável que recebe dados de odom
geometry_msgs::Pose lastOdom;
bool oninit_odom = false;
float delta_d = 0.05;
float delta_theta = 0.001;


Garrote::Localization3D::Map3D::ParticleFilterLocalization *pfl;
Transform tfx;
tf::TransformBroadcaster *odom_broadcaster;
ros::Publisher mapviz;
ros::Publisher mapvizlocal;
ros::Publisher odom_pub;
ros::Publisher arraypose;
ros::Publisher arraypath;
//ros::Publisher pc1_keypoints1;

std::vector<geometry_msgs::Pose> path;
std::vector<std::vector<Voxel3D>> processedPC;
std::vector<float> tdata;

std::vector<Voxel3D> data;
//int pontosmenos2=0; 
//double dist_x,dist_y,dist_z=0;

sensor_msgs::PointCloud scan;
sensor_msgs::LaserScan laserteste;


std::mutex mtx;

PointBasedLocalization obl; //TODO



// função para avaliar tempos de cada frame

void tic(int mode=0) {
    static std::chrono::_V2::system_clock::time_point t_start;
    
    if (mode==0)
        t_start = std::chrono::high_resolution_clock::now();
    else {
        auto t_end = std::chrono::high_resolution_clock::now();
        //std::cout << "Elapsed time is " << (t_end-t_start).count()*1E-9 << " seconds\n";
        
        //std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch());
        //std::cout << "2 " << (t_end-t_start).count()*1E-9 <<" "<<ms.count()<< "\n";
        //std::cout << "1 " << (t_end-t_start).count()*1E-9 << "\n";
        tdata.push_back(((t_end-t_start).count()*1E-9));
    }
}
void toc() { tic(1); }

float angleBetween(Point p1, Point p2)
{
    float result = atan2(p2.y-p1.y,p2.x-p1.x) * (180.0/3.141592653589793238463);
    if (result < 0)
    {
        result=result+360;
    }
    return result;
}

void vlpCallback(const sensor_msgs::PointCloud2ConstPtr& laser)
{
    
    
    tic();
    
    //Point p1,p2;
    //p1.x=0;
    //p1.y=0;

    if(pfl->run2KLD(laser)){ 
        
        sensor_msgs::convertPointCloud2ToPointCloud(*laser,scan);
        
        data.clear();
        Voxel3D point;
        //float angle2=0;

        for(unsigned int i=0; i<scan.points.size(); i++){
            //p2.x=scan.points[i].x;
            //p2.y=scan.points[i].y;
            //p2.z=scan.points[i].z;
            //angle2=angleBetween(p1,p2); (angle2>=195 || angle2<=165)
            if(scan.points[i].z<=1.75 && scan.points[i].z>-1.06){
                point.x=scan.points[i].x;
                point.y=scan.points[i].y;
                point.z=scan.points[i].z;
                data.push_back(point);
            }
        }
        
        
        geometry_msgs::Pose pose=pfl->getPose(); // aqui best 
        
        
        NumericArray<double,3> estimate;
        estimate[0]=pose.position.x;
        estimate[1]=pose.position.y;
        estimate[2]=PoseTools::getYaw(pose);
        //std::cout<<pose.position.x<<" "<<pose.position.y<<" "<<estimate[2]<<std::endl;

        optimizer2.alpha1=10;
        optimizer2.alpha2=10;
        optimizer2.alpha3=10;

        optimizer2.lamb=500; //aqui 500
        auto data2=pfl->tf*pfl->pointcloud_keypoints.points; //aqui


        //pfl->pointcloud_keypoints.header.frame_id="map";

        //pc1_keypoints1.publish(pfl->pointcloud_keypoints);






        //std::cout<<__LINE__<<std::endl;
        
        double bestERR=99999;
        NumericArray<double,3> lastestimate=estimate;
        NumericArray<double,3> beeststimate=estimate;
        
        for(unsigned int i=0;i<20;i++){ //aqui 20
            optimizer2.error=99999;
            lastestimate=estimate;
            bool res=optimizer2.iterate(estimate,data2,map4X,1,2);
            
            if(optimizer2.error<bestERR){
                bestERR=optimizer2.error;
                beeststimate=lastestimate;
            }
            
            if(res==false){
                break;
            }
            
        }
        estimate=beeststimate;
        lastestimate=estimate;
        bestERR=99999;
        optimizer2.lamb=500; //aqui 500
        //    std::cout<<__LINE__<<std::endl;
        
        for(unsigned int i=0;i<20;i++){ //aqui 20
            optimizer2.error=99999;
            lastestimate=estimate;
            bool res=optimizer2.iterate(estimate,data2,map2X,1,2);
            
            if(optimizer2.error<bestERR){
                bestERR=optimizer2.error;
                beeststimate=lastestimate;
            }
            
            if(res==false){
                break;
            }
            
        }
        
        //    std::cout<<__LINE__<<std::endl;
        
        estimate=beeststimate;
        lastestimate=estimate;
        bestERR=99999;
        optimizer2.lamb=500; //aqui 500
        
        for(unsigned int i=0;i<20;i++){ //aqui 20
            optimizer2.error=99999;
            lastestimate=estimate;
            bool res=optimizer2.iterate(estimate,data2,map,2,2);
            
            if(optimizer2.error<bestERR){
                bestERR=optimizer2.error;
                beeststimate=lastestimate;
            }
            
            if(res==false){
                break;
            }
            
        }
        //estimate[0]=pose.position.x;
        //estimate[1]=pose.position.y;
        //estimate[2]=PoseTools::getYaw(pose);
        //optimizer2.iterate(estimate,data2,map,2);

        //std::cout<<"Error "<<bestERR<<" vs "<<optimizer2.error<<std::endl;
        
        
        //    std::cout<<__LINE__<<std::endl;
        
        pose.position.x=beeststimate[0];
        pose.position.y=beeststimate[1];
        PoseTools::setRPY(pose,0,0,beeststimate[2]);
        //std::cout<<pose.position.x<<" "<<pose.position.y<<" "<<beeststimate[2]<<std::endl;
        
        pfl->setPoseData(pose);
        
        
        mtx.lock();
        
        pfl->setPoseDataFeatures(pose,data,pfl->pointcloud_keypoints);
        
        mtx.unlock();
        
        
        ros::Time current_time;
        current_time = ros::Time::now();
        //std::cout<<"Particles: "<<pfl->smc->numParticles<<std::endl;
        auto px=pfl->getPose();
        
        path.push_back(px);
        processedPC.push_back(data);
        
        // tf crap .... [TODO] confirmar com outros metodos
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        
        odom_trans.transform.translation.x = px.position.x;
        odom_trans.transform.translation.y = px.position.y;
        odom_trans.transform.translation.z = px.position.z;
        odom_trans.transform.rotation = px.orientation;
        
        //send the transform
        odom_broadcaster->sendTransform(odom_trans);
        
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        
        //set the position
        odom.pose.pose.position=px.position;
        odom.pose.pose.orientation = px.orientation;
        
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;
        
        //publish the message
        odom_pub.publish(odom);
        //aqui
        //posicao.push_back(pose);
        
        geometry_msgs::PoseArray particles;
        auto p=pfl->getParticles(1);
        particles.poses=p;
        particles.header.frame_id = "map";
        
        arraypose.publish(particles);
        
        
        geometry_msgs::PoseArray path2;
        
        path2.poses=path;
        path2.header.frame_id = "map";
        
        arraypath.publish(path2);
        
        toc();
        
    }
    
    
    //toc();
    //[TODO] publicar pose e tfs aqui....
    
    
}

void keypointCallback(const sensor_msgs::PointCloudConstPtr& keypoints)
{
    
    
    
    
    
    
    
    
    
}


//buscar odometria
void odomCallback(const nav_msgs::OdometryConstPtr& odometria)
{
    //  ROS_INFO("data_odom");
    odom_data.position = odometria->pose.pose.position;
    odom_data.orientation = odometria->pose.pose.orientation;
    if(oninit_odom){
        pfl -> sensorhandle.accumulateSensor("odom_pf", lastOdom, odom_data);
    }
    
    if (!oninit_odom){
        pfl -> sensorhandle.configure("odom_pf",true,delta_d,delta_theta);
    }
    
    oninit_odom=true;
    lastOdom = odom_data;
    
    
    
}

//buscar odometria
//void odomCallback(const geometry_msgs::PoseStampedConstPtr& odometria)
//{
//ROS_INFO("data_odom");
//    odom_data.position = odometria->pose.position;
//    odom_data.orientation = odometria->pose.orientation;
//   if(oninit_odom){
//     pfl -> sensorhandle.accumulateSensor("odom_pf", lastOdom, odom_data);
//   }

//     if (!oninit_odom){
//           pfl -> sensorhandle.configure("odom_pf",true,delta_d,delta_theta);
//         }

//          oninit_odom=true;
//            lastOdom = odom_data;

//}
//buscar laser_scan
void laserscanCallback(const sensor_msgs::LaserScanConstPtr& laser_lidar)
{
    //ROS_INFO("data_2d");
    //laserteste=*laser_lidar;
    //std::cout<<laserteste.ranges[3]<<std::endl;
}


static void slm(Garrote::Localization3D::Map3D::ParticleFilterLocalization* t){
    
    // std::cout<<"on Processed map"<<std::endl;
    
    NumericArray<double,3> estimate;
    mtx.lock();
    
    if(pfl->toProcess==false){
        //        std::cout<<"off Processed map"<<std::endl;
        
        mtx.unlock();
        
        return;
    }
    
    geometry_msgs::Pose pose=pfl->pose_; // aqui best 
    std::vector<Voxel3D> pointData=pfl->data_; // aqui best 
    sensor_msgs::PointCloud  keypointData=pfl->keypoints_; // aqui best 
    pfl->toProcess=false;
    mtx.unlock();
    
    
    estimate[0]=pose.position.x;
    estimate[1]=pose.position.y;
    estimate[2]=PoseTools::getYaw(pose);
    
    optimizer.lamb=500; //aqui 500
    auto data2=pfl->tf*pointData; //aqui
    //std::cout<<__LINE__<<std::endl;
    
    double bestERR=99999;
    NumericArray<double,3> lastestimate=estimate;
    NumericArray<double,3> beeststimate=estimate;
    
    for(unsigned int i=0;i<20;i++){ //aqui 20
        optimizer.error=99999;
        lastestimate=estimate;
        bool res=optimizer.iterate(estimate,data2,map4X,3);
        
        if(optimizer.error<bestERR){
            bestERR=optimizer.error;
            beeststimate=lastestimate;
        }
        
        if(res==false){
            break;
        }
        
    }
    estimate=beeststimate;
    lastestimate=estimate;
    bestERR=99999;
    optimizer.lamb=500; //aqui 500
    //    std::cout<<__LINE__<<std::endl;
    
    for(unsigned int i=0;i<20;i++){ //aqui 20
        optimizer.error=99999;
        lastestimate=estimate;
        bool res=optimizer.iterate(estimate,data2,map2X,3);
        
        if(optimizer.error<bestERR){
            bestERR=optimizer.error;
            beeststimate=lastestimate;
        }
        
        if(res==false){
            break;
        }
        
    }
    
    //    std::cout<<__LINE__<<std::endl;
    
    estimate=beeststimate;
    lastestimate=estimate;
    bestERR=99999;
    optimizer.lamb=500; //aqui 500
    
    for(unsigned int i=0;i<20;i++){ //aqui 20
        optimizer.error=99999;
        lastestimate=estimate;
        bool res=optimizer.iterate(estimate,data2,map,3);
        
        if(optimizer.error<bestERR){
            bestERR=optimizer.error;
            beeststimate=lastestimate;
        }
        
        if(res==false){
            break;
        }
        
    }
    
    
    //    std::cout<<__LINE__<<std::endl;
    
    pose.position.x=beeststimate[0];
    pose.position.y=beeststimate[1];
    PoseTools::setRPY(pose,0,0,beeststimate[2]);
    //pfl->updatePreviousPrediction(pose);
    
    //std::cout<<__LINE__<<std::endl;
    
    pfl->mapUpdatev2(pose,pointData,15);
    //std::cout<<__LINE__<<std::endl;
    
    pfl->mapUpdatev2(map2X,pose,pointData,15);
    //std::cout<<__LINE__<<std::endl;
    pfl->mapUpdatev2(map4X,pose,pointData,15);
    //std::cout<<__LINE__<<std::endl;
    pfl->mapKeypoints(pose, keypointData.points); //erro
    //std::cout<<__LINE__<<std::endl;
    map->setUpdated(true);
    //std::cout<<__LINE__<<std::endl;
    
    //std::cout<<"Processed map"<<std::endl;
    
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PF2");
    ros::NodeHandle n;  
    ros::Rate loop_rate(10);
    
    odom_broadcaster= new tf::TransformBroadcaster();
    mapviz=n.advertise<visualization_msgs::MarkerArray>("map3d", 1000);
    mapvizlocal=n.advertise<visualization_msgs::MarkerArray>("localmap3d", 1000);
    odom_pub = n.advertise<nav_msgs::Odometry>("pose", 50);
    arraypose= n.advertise<geometry_msgs::PoseArray>("particles", 50);
    arraypath= n.advertise<geometry_msgs::PoseArray>("path", 50);
    //pc1_keypoints1=n.advertise<sensor_msgs::PointCloud>("pc1_keypoints2",1000);

    map = new   Grid3D<char>( 2500,2500,160);
    map2X = new   Grid3D<char>( 1250,1250,80);
    map4X = new   Grid3D<char>( 625,625,40);
    localmap =new   Grid3D<char>( 1500,1500,160);
    localmap->setSizes(0.05,0.05,0.05); //size 0.05
    
    Point p;
    p.x=0;
    p.y=0;
    p.z=0;
    map->setSizes(0.05,0.05,0.05); //size 0.05
    map2X->setSizes(0.1,0.1,0.1); //size 0.05
    map4X->setSizes(0.2,0.2,0.2); //size 0.05
    
    map->setStart(p);
    map2X->setStart(p);
    map4X->setStart(p);
    localmap->setStart(p);
    
    int Np=1200; // 1200(pcl) (depth-based pf2_localization) testar com diferentes valores 1 a 1200
    float alpha1 = 0.4, alpha2 = 0.4, alpha3 = 0.4, alpha4 = 0.4; //0.4 diferentes valores 0.2 3 2 0.3 0.4 2.5 1 1.5 n 0.001 0.003 0.006 0.01 0.03 0.06 0.09 0.2 
    
    
    tfx.setTranslation(0,0,1.06); // para indicar a tf 92
    
    pfl = new Garrote::Localization3D::Map3D::ParticleFilterLocalization(map,tfx,Np);
    
    pfl->alpha1 = alpha1;
    pfl->alpha2 = alpha2;
    pfl->alpha3 = alpha3;
    pfl->alpha4 = alpha4;
    
    pfl->kldHandle.minParticles=600; //teste pcl 300 600 aqui 100 
    
    ros::Subscriber sub = n.subscribe("/velodyne_points",1000,vlpCallback);
    ros::Subscriber sub3 = n.subscribe("/keypoints",1000,keypointCallback);
    ros::Subscriber sub1 = n.subscribe("/odom",1000,odomCallback);
    //ros::Subscriber sub1 = n.subscribe("/PF_pose",1000,odomCallback);
    ros::Subscriber sub2 = n.subscribe("/scan",1000,laserscanCallback);
    std::cout<<"PF2_FINAL: a1 a2 a3 a4"<<" "<<alpha1<<" "<<alpha2<<" "<<alpha3<<" "<<alpha4<<std::endl;
    
    
    
    Event<Garrote::Localization3D::Map3D::ParticleFilterLocalization *> updater;
    updater.start(2,slm,pfl);  // 1(+-) passou a 2(+-) 4(n) 5(n) 1Hz
    
    
    
    while(ros::ok())
    {
        
        //ROS_INFO("while");
        ros::spinOnce();
        //ros::spin();
        
        
        //if(map->updated()){
        
        //auto voxels=map->getData2();
        //std::cout<<"MapData size"<<voxels.size()<<std::endl;
        //auto markers=pfl->getVoxel2ROSGlobal("map","global",voxels);
        //mapviz.publish(markers);
        //map->setUpdated(false);
        //}
        
        
        
        loop_rate.sleep();
        
    }
    
    
    auto voxels1=map->getData2();
    auto markers1=pfl->getVoxel2ROSGlobal("map","global",voxels1); 
    
    std::cout<<markers1.markers.size()<<std::endl;
    int tt=markers1.markers.size();
    
    //guardar mapa
    std::fstream fd;
    
    std::cout<<"tamanho tdata: "<<tdata.size()<<std::endl;
    float soma_t=0;
    
    fd.open("t_frame"+std::to_string(Np)+"_"+std::to_string(alpha1)+"_"+std::to_string(alpha2)+"_"+std::to_string(alpha3)+"_"+std::to_string(alpha4)+".txt", std::fstream::out );
    
    if(fd.is_open())
    {
        
        for(int i=0;i<tdata.size();i++)
        {
            fd<<std::to_string(i)<<" "<<std::to_string(tdata[i])<<std::endl;
            soma_t=soma_t+tdata[i];
        }
        fd<<"média: "<<std::to_string(soma_t/tdata.size())<<std::endl;
        fd.close();
        std::cout<<"T_frame Saved"<<std::endl;
    }


    map->save("mapa.map");
    map2X->save("mapa2X.map");
    map4X->save("mapa4X.map");

    
    fd.open("mapa"+std::to_string(Np)+"_"+std::to_string(alpha1)+"_"+std::to_string(alpha2)+"_"+std::to_string(alpha3)+"_"+std::to_string(alpha4)+".ply", std::fstream::out );
    
    if(fd.is_open())
    {
        
        fd<<"ply"<<std::endl;
        fd<<"format ascii 1.0"<<std::endl;
        fd<<"element vertex "<<std::to_string(tt)<<std::endl;
        fd<<"property float x"<<std::endl;
        fd<<"property float y"<<std::endl;
        fd<<"property float z"<<std::endl;
        fd<<"end_header"<<std::endl;
        
        for (int i=0;i<tt;i++)
        {
            
            fd<<std::to_string(markers1.markers[i].pose.position.x)<<" "<<std::to_string(markers1.markers[i].pose.position.y)<<" "<<std::to_string(markers1.markers[i].pose.position.z)<<std::endl;
        }
        
        fd.close();
        std::cout<<"Map Saved"<<std::endl;
    }
    
    pfl->keypoints.save("keypoints");
    
    
    
    //guardar pose
    
    fd.open("pontos"+std::to_string(Np)+"_"+std::to_string(alpha1)+"_"+std::to_string(alpha2)+"_"+std::to_string(alpha3)+"_"+std::to_string(alpha4)+".ply", std::fstream::out );
    
    if(fd.is_open())
    {
        
        for (int i=0;i<path.size();i++)
        {
            
            fd<<std::to_string(path[i].position.x)<<" "<<std::to_string(path[i].position.y)<<" "<<std::to_string(path[i].position.z)<<" "<<std::to_string(path[i].orientation.w)<<" "<<std::to_string(path[i].orientation.x)<<" "<<std::to_string(path[i].orientation.y)<<" "<<std::to_string(path[i].orientation.z)<<std::endl;
        }
        
        fd.close();
        std::cout<<"Poses Saved"<<std::endl;
    }
    
    
    //guardar overlap
    
    double overlap=0;
    double soma_overlap=0;
    std::cout<<"Confirmar: "<<path.size()<<" "<<processedPC.size()<<std::endl;
    
    fd.open("overlap_v1"+std::to_string(Np)+"_"+std::to_string(alpha1)+"_"+std::to_string(alpha2)+"_"+std::to_string(alpha3)+"_"+std::to_string(alpha4)+".ply", std::fstream::out );
    
    if(fd.is_open())
    {
        
        for (int i=0;i<path.size();i++)
        {
            
            overlap= pfl->computeOverlapv1(path[i],processedPC[i]);
            soma_overlap=soma_overlap+overlap;
            fd<<std::to_string(i)<<" "<<std::to_string(overlap)<<std::endl;
        }
        fd<<"média: "<<std::to_string(soma_overlap/path.size())<<std::endl;
        fd.close();
        std::cout<<"Overlap Saved"<<std::endl;
    }
    
    //guardar overlap
    
    double overlap2=0;
    double soma_overlap2=0;
    std::cout<<"Confirmar: "<<path.size()<<" "<<processedPC.size()<<std::endl;
    
    fd.open("overlap_v2"+std::to_string(Np)+"_"+std::to_string(alpha1)+"_"+std::to_string(alpha2)+"_"+std::to_string(alpha3)+"_"+std::to_string(alpha4)+".ply", std::fstream::out );
    
    if(fd.is_open())
    {
        
        for (int i=0;i<path.size();i++)
        {
            
            overlap2= pfl->computeOverlapv2(path[i],processedPC[i]);
            soma_overlap2=soma_overlap2+overlap2;
            fd<<std::to_string(i)<<" "<<std::to_string(overlap2)<<std::endl;
        }
        fd<<"média: "<<std::to_string(soma_overlap2/path.size())<<std::endl;
        fd.close();
        std::cout<<"Overlap Saved"<<std::endl;
    }
    
    //guardar tempo cada frame
    
    
    
    
    
    
    //std::cout<<"Pontos abaixo de 2.81 metros: "<<pontosmenos2<<std::endl;
    
    return 0;
}
