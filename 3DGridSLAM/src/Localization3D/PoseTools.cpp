#include "PoseTools.h"


#include "Transform.h"
#include "stringformat.h"
#include "Point.h"
#include "Euler.h"

std::vector<geometry_msgs::Pose> PoseTools::segment(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, int delta){

    std::vector<geometry_msgs::Pose> out;
    double angle = angleBetween(pose1,pose2);
    double d =  distance2D(pose1,pose2);
    double sigma=d/(delta+1.0);

    geometry_msgs::Pose p=pose1;

    out.push_back(p);

    for( int i=0;i<delta;i++){
        p.position.x+=sigma*cos(angle);
        p.position.y+=sigma*sin(angle);
        out.push_back(p);
    }

    out.push_back(pose2);

    return out;
}

geometry_msgs::Pose  PoseTools::operator+( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2){
    geometry_msgs::Pose value;

    value.position.x=value1.position.x+value2.position.x;
    value.position.y=value1.position.y+value2.position.y;
    value.position.z=value1.position.z+value2.position.z;

    //    value.orientation.x=value1.orientation.x+value2.orientation.x;
    //    value.orientation.y=value1.orientation.y+value2.orientation.y;
    //    value.orientation.z=value1.orientation.z+value2.orientation.z;
    //    value.orientation.w=value1.orientation.z+value2.orientation.w;


    return value;
}

bool PoseTools::operator==( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2){
    return value1.position.x==value2.position.x &&value1.position.y==value2.position.y&&value1.position.z==value2.position.z;
}

bool PoseTools::operator!=( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2){
    return !(value1.position.x==value2.position.x &&value1.position.y==value2.position.y&&value1.position.z==value2.position.z);
}

geometry_msgs::Pose PoseTools::operator*( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2){
    geometry_msgs::Pose value;

    value.position.x=value1.position.x*value2.position.x;
    value.position.y=value1.position.y*value2.position.y;
    value.position.z=value1.position.z*value2.position.z;

    //    value.orientation.x=value1.orientation.x+value2.orientation.x;
    //    value.orientation.y=value1.orientation.y+value2.orientation.y;
    //    value.orientation.z=value1.orientation.z+value2.orientation.z;
    //    value.orientation.w=value1.orientation.z+value2.orientation.w;


    return value;
}

geometry_msgs::Pose PoseTools::operator*( const geometry_msgs::Pose value1 ,double value2){
    geometry_msgs::Pose value;

    value.position.x=value1.position.x*value2;
    value.position.y=value1.position.y*value2;
    value.position.z=value1.position.z*value2;

    //    value.orientation.x=value1.orientation.x+value2.orientation.x;
    //    value.orientation.y=value1.orientation.y+value2.orientation.y;
    //    value.orientation.z=value1.orientation.z+value2.orientation.z;
    //    value.orientation.w=value1.orientation.z+value2.orientation.w;


    return value;
}
geometry_msgs::Pose PoseTools::operator*( double value2,const geometry_msgs::Pose value1){
    geometry_msgs::Pose value;

    value.position.x=value1.position.x*value2;
    value.position.y=value1.position.y*value2;
    value.position.z=value1.position.z*value2;

    //    value.orientation.x=value1.orientation.x+value2.orientation.x;
    //    value.orientation.y=value1.orientation.y+value2.orientation.y;
    //    value.orientation.z=value1.orientation.z+value2.orientation.z;
    //    value.orientation.w=value1.orientation.z+value2.orientation.w;


    return value;
}



geometry_msgs::Pose PoseTools::operator-( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2){
    geometry_msgs::Pose value;

    value.position.x=value1.position.x-value2.position.x;
    value.position.y=value1.position.y-value2.position.y;
    value.position.z=value1.position.z-value2.position.z;

    //    value.orientation.x=value1.orientation.x+value2.orientation.x;
    //    value.orientation.y=value1.orientation.y+value2.orientation.y;
    //    value.orientation.z=value1.orientation.z+value2.orientation.z;
    //    value.orientation.w=value1.orientation.z+value2.orientation.w;


    return value;
}
geometry_msgs::Pose PoseTools::operator/( const geometry_msgs::Pose value1 ,geometry_msgs::Pose value2){
    geometry_msgs::Pose value;

    value.position.x=value1.position.x/value2.position.x;
    value.position.y=value1.position.y/value2.position.y;
    value.position.z=value1.position.z/value2.position.z;

    //    value.orientation.x=value1.orientation.x+value2.orientation.x;
    //    value.orientation.y=value1.orientation.y+value2.orientation.y;
    //    value.orientation.z=value1.orientation.z+value2.orientation.z;
    //    value.orientation.w=value1.orientation.z+value2.orientation.w;


    return value;
}


double PoseTools::distanceToLine ( geometry_msgs::Pose base, geometry_msgs::Pose v, geometry_msgs::Pose w)
{
    // Return minimum distance between line segment vw and point p
    double l2 = distanceSquared(v,w);  // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return  distance(base,v);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    double t = dot((base - v),( w - v)) / l2;
    if (t < 0.0) return std::min(distance(base,v),distance(base,w));//0.0+( distance(base,v)+ distance(base,w))/2.0;       // Beyond the 'v' end of the segment
    else if (t > 1.0) return  0.0+std::min(distance(base,v),distance(base,w));//( distance(base,v)+ distance(base,w))/2.0;  // Beyond the 'w' end of the segment

    // if (t < 0.0) return  distance(v)*99;       // Beyond the 'v' end of the segment
    // else if (t > 1.0) return  distance(w)*99;  // Beyond the 'w' end of the segment
    geometry_msgs::Pose projection = v +  (w - v)*t;  // Projection falls on the segment
    return  distance(base,projection);
}


geometry_msgs::Pose PoseTools::PointInLine ( geometry_msgs::Pose base, geometry_msgs::Pose v, geometry_msgs::Pose w)
{
    // Return minimum distance between line segment vw and point p
    double l2 = distanceSquared(v,w);  // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return base;   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    double t = dot((base - v),( w - v)) / l2;
    if (t < 0.0) return distance(base,v)>distance(base,w)?w:v;//0.0+( distance(base,v)+ distance(base,w))/2.0;       // Beyond the 'v' end of the segment
    else if (t > 1.0) return  distance(base,v)>distance(base,w)?w:v;//( distance(base,v)+ distance(base,w))/2.0;  // Beyond the 'w' end of the segment

    // if (t < 0.0) return  distance(v)*99;       // Beyond the 'v' end of the segment
    // else if (t > 1.0) return  distance(w)*99;  // Beyond the 'w' end of the segment
    geometry_msgs::Pose projection = v +  (w - v)*t;  // Projection falls on the segment
    return  projection;
}

geometry_msgs::Twist TwistTools::fromString(std::string v){

    geometry_msgs::Twist pos;
    auto vec=$(v).numbers<double>();

    if(vec.size()==0){

        pos.linear.x=0;
        pos.linear.y=0;
        pos.linear.z=0;
        pos.angular.x=0;
        pos.angular.y=0;
        pos.angular.z=0;

    }else if (vec.size()==2){

        pos.linear.x=vec[0];
        pos.linear.y=0;
        pos.linear.z=0;
        pos.angular.x=0;
        pos.angular.y=0;
        pos.angular.z=vec[1];

    }else if (vec.size()==6){

        pos.linear.x=vec[0];
        pos.linear.y=vec[1];
        pos.linear.z=vec[2];
        pos.angular.x=vec[3];
        pos.angular.y=vec[4];
        pos.angular.z=vec[5];

    }
    return pos;

}

std::string TwistTools::toString(geometry_msgs::Twist pos){


    return "["+$("{}").format(pos.linear.x)+
            ","+$("{}").format(pos.linear.y)+
            ","+$("{}").format(pos.linear.z)+
            ","+$("{}").format(pos.angular.x)+
            ","+$("{}").format(pos.angular.y)+
            ","+$("{}").format(pos.angular.z)+"]";
}

std::string PoseTools::toString(geometry_msgs::Pose pose1){


    return "["+$("{}").format(pose1.position.x)+
            ","+$("{}").format(pose1.position.y)+
            ","+$("{}").format(pose1.position.z)+
            ","+$("{}").format(pose1.orientation.w)+
            ","+$("{}").format(pose1.orientation.x)+
            ","+$("{}").format(pose1.orientation.y)+
            ","+$("{}").format(pose1.orientation.z)+"]";
}


std::string PoseTools::toString2(geometry_msgs::Pose pose1){


    return "["+$("{}").format(pose1.position.x)+
            ","+$("{}").format(pose1.position.y)+
            ","+$("{}").format(pose1.position.z)+"]";
}






geometry_msgs::Pose PoseTools::fromString(std::string v){

    geometry_msgs::Pose pos;




    //    Garrote::String::StringTools::removeFromString(v,']');
    //    Garrote::String::StringTools::removeFromString(v,'[');

    //    Garrote::String::StringTools::removeFromString(v,' ');
    //    Garrote::String::StringTools::removeFromString(v,';');


    std::vector<double> vec=$(v).numbers<double>();

    if(vec.size()==7){


        pos.position.x= (vec[0]);
        pos.position.y= (vec[1]);
        pos.position.z= (vec[2]);
        pos.orientation.w= (vec[3]);
        pos.orientation.x= (vec[4]);
        pos.orientation.y= (vec[5]);
        pos.orientation.z= (vec[6]);

        /*
        pos.position.x=atof(vec[0].c_str());
        pos.position.y=atof(vec[1].c_str());
        pos.position.z=atof(vec[2].c_str());
        pos.orientation.w=atof(vec[3].c_str());
        pos.orientation.x=atof(vec[4].c_str());
        pos.orientation.y=atof(vec[5].c_str());
        pos.orientation.z=atof(vec[6].c_str());*/

    }else if(vec.size()==3){
        pos.position.x= (vec[0]);
        pos.position.y= (vec[1]);
        pos.position.z= (vec[2]);
    }else if(vec.size()==4){
        pos.position.x= (vec[0]);
        pos.position.y= (vec[1]);
        pos.position.z= (vec[2]);
        PoseTools::updateYaw(pos,vec[3]);
    }




    return pos;
}


std::vector<geometry_msgs::Pose > PoseTools::fromStringToVector(std::string v){
    std::vector<geometry_msgs::Pose > out;

    auto lines= $(v).split(";");

    for(unsigned int i=0;i<lines.size();i++){

        out.push_back(fromString(lines[i]));
    }


    return out;
}


double PoseTools::getYaw(geometry_msgs::Pose pose){



    //    q0=w_;
    //    q1=x_;
    //    q2=y_;
    //    q3=z_;

    double d=pose.orientation.x*pose.orientation.x+pose.orientation.y*pose.orientation.y+pose.orientation.z*pose.orientation.z+pose.orientation.w*pose.orientation.w;
    double s=double(2.0) / d;

    double ys = pose.orientation.y * s,   zs = pose.orientation.z * s;
    double wy = pose.orientation.w * ys,  wz = pose.orientation.w * zs;
    double xy = pose.orientation.x * ys,  xz = pose.orientation.x * zs;
    double yy = pose.orientation.y* ys,  zz = pose.orientation.z * zs;

    //get the pointer to the raw data
    // Check that pitch is not at a singularity
    // Check that pitch is not at a singularity

    if (fabs(xz - wy) >= 1.0)
    {
        return 0.0;
    }
    else
    {
        double pitch = - asin(xz - wy);
        return atan2((xy + wz)/cos(pitch),(1.0 - (yy + zz))/cos(pitch));
    }



    //    Transform *tf=Transform::createTransformFromPose(pose1);

    //   // double angle2=tf->getYaw();
    //    double angle2=tf->getYawFast();

    //    delete tf;
    //    return angle2;
}

double PoseTools::getYaw_old(geometry_msgs::Pose pose1){

    Transform *tf=Transform::createTransformFromPose(pose1);

    double angle2=tf->getYaw();
    //double angle2=tf->getYawFast();

    delete tf;
    return angle2;
}

Euler PoseTools::getEuler(geometry_msgs::Pose pose1){

    Transform *tf=Transform::createTransformFromPose(pose1);

    Euler out;
    tf->getRPY(out.roll,out.pitch,out.yaw);
    //double angle2=tf->getYawFast();
    delete tf;
    return out;
}

geometry_msgs::Pose PoseTools::move2D(geometry_msgs::Pose lp, double La){
    geometry_msgs::Pose pose=lp;

    auto yaw=getYaw(lp);

    pose.position.x+=La*cos(yaw);
    pose.position.y+=La*sin(yaw);

    return pose;
}

void PoseTools::updateYaw(geometry_msgs::Pose &pose, double yaw){

    Quaternion qt(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);

    qt.setRPY(0,0,yaw);

    pose.orientation.w=qt.w();
    pose.orientation.x=qt.x();
    pose.orientation.y=qt.y();
    pose.orientation.z=qt.z();


}

bool PoseTools::similarYaw(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double v){

//    Transform *tf=Transform::createTransformFromPose(pose1);
//    Transform *tf2=Transform::createTransformFromPose(pose2);

//    double angle1=tf2->getYaw();
//    double angle2=tf->getYaw();
    double angle1=getYaw(pose1);
    double angle2=getYaw(pose2);

    double t=angle1-angle2;

//    delete tf;
//    delete tf2;
    return fabs(atan2(sin(t), cos(t)))<=v;
}



double PoseTools::distance(geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2){
    double dx=pose1.position.x-pose2.position.x;
    double dy=pose1.position.y-pose2.position.y;
    double dz=pose1.position.z-pose2.position.z;
    return sqrt(dx*dx+dy*dy+dz*dz);
}

double PoseTools::distance2D(geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2){
    double dx=pose1.position.x-pose2.position.x;
    double dy=pose1.position.y-pose2.position.y;
    return sqrt(dx*dx+dy*dy);
}

double PoseTools::diffYaw(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){

//    Transform *tf=Transform::createTransformFromPose(pose1);
//    Transform *tf2=Transform::createTransformFromPose(pose2);

    double angle1=getYaw(pose1);
    double angle2=getYaw(pose2);

    double t=angle1-angle2;

    return atan2(sin(t), cos(t));
}

double PoseTools::squareDistance2D(geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2){
    double dx=pose1.position.x-pose2.position.x;
    double dy=pose1.position.y-pose2.position.y;
    return  (dx*dx+dy*dy);
}

double PoseTools::distance2D(double x0, double y0, double x1, double y1){
    double dx=x0-x1;
    double dy=y0-y1;
    return sqrt(dx*dx+dy*dy);
}

double PoseTools::distance(geometry_msgs::Pose &pose1, geometry_msgs::Point &pose2){
    double dx=pose1.position.x-pose2.x;
    double dy=pose1.position.y-pose2.y;
    double dz=pose1.position.z-pose2.z;
    return sqrt(dx*dx+dy*dy+dz*dz);
}

double PoseTools::distance2D(std::vector<geometry_msgs::Pose> &poses){

    double out=0;
    for(unsigned int i=0;i< poses.size()-1;i++){
        out=out+distance2D(poses[i],poses[i+1]);
    }

    return out;
}

double PoseTools::distance(std::vector<geometry_msgs::Pose> &poses){

    double out=0;
    for(unsigned int i=0;i< poses.size()-1;i++){

        out=out+distance(poses[i],poses[i+1]);

    }

    return out;
}

geometry_msgs::Pose PoseTools::cross(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){

    geometry_msgs::Pose pose;

    pose.position.x=pose1.position.y*pose2.position.z - pose1.position.z*pose2.position.y;
    pose.position.y=pose1.position.z*pose2.position.x - pose1.position.x*pose2.position.z;
    pose.position.z=pose1.position.x*pose2.position.y - pose1.position.y*pose2.position.x;

    return pose;
}

double PoseTools::distanceSquared(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
    double dx=pose1.position.x-pose2.position.x;
    double dy=pose1.position.y-pose2.position.y;
    double dz=pose1.position.z-pose2.position.z;
    return  (dx*dx+dy*dy+dz*dz);
}

double PoseTools::dot(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
    double dx=pose1.position.x*pose2.position.x;
    double dy=pose1.position.y*pose2.position.y;
    double dz=pose1.position.z*pose2.position.z;
    return  dx+dy+dz;
}

double PoseTools::angleBetween(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
    return atan2(pose2.position.y-pose1.position.y,pose2.position.x-pose1.position.x);
}

double PoseTools::angleBetween(geometry_msgs::Pose pose1, Point pose2){
    return atan2(pose2.getY()-pose1.position.y,pose2.getX()-pose1.position.x);
}

geometry_msgs::Pose PoseTools::projectionInLine(geometry_msgs::Pose Point, geometry_msgs::Pose LineStart, geometry_msgs::Pose LineEnd){


    float LineMag;
    float U;
    geometry_msgs::Pose Intersection;

    LineMag = distance( LineEnd, LineStart );

    U = ( ( ( Point.position.x - LineStart.position.x ) * ( LineEnd.position.x - LineStart.position.x ) ) +
          ( ( Point.position.y  - LineStart.position.y  ) * ( LineEnd.position.y  - LineStart.position.y ) ) +
          ( ( Point.position.z - LineStart.position.z ) * ( LineEnd.position.z - LineStart.position.z ) ) ) /
            ( LineMag * LineMag );

    if( U < 0.0f || U > 1.0f )
        return distance(LineStart,Point)>distance(Point,LineEnd)?LineEnd:LineStart;   // closest point does not fall within the line segment

    Intersection.position.x  = LineStart.position.x  + U * ( LineEnd.position.x  - LineStart.position.x  );
    Intersection.position.y  = LineStart.position.y  + U * ( LineEnd.position.y - LineStart.position.y );
    Intersection.position.z  = LineStart.position.z  + U * ( LineEnd.position.z - LineStart.position.z );

    Intersection.orientation=LineEnd.orientation;
    // *Distance = Magnitude( Point, &Intersection );

    return Intersection;

    //    }




}

geometry_msgs::Pose PoseTools::mean(geometry_msgs::Pose base, geometry_msgs::Pose h1){
    geometry_msgs::Pose out;

    out.position.x=(base.position.x+h1.position.x)/2.0;
    out.position.y=(base.position.y+h1.position.y)/2.0;
    out.position.z=(base.position.z+h1.position.z)/2.0;
    return out;
}

double PoseTools::mean(geometry_msgs::Pose base, std::vector<geometry_msgs::Pose> &h1){
    double out=0;



    for(unsigned int i=0;i<h1.size();i++){

        out=out+distance(base,h1[i]);
    }


    return out/(double)(h1.size());
}

double PoseTools::min(geometry_msgs::Pose base, std::vector<geometry_msgs::Pose> &h1){
    double out=9999;



    for(unsigned int i=0;i<h1.size();i++){

        out=std::min(out,distance(base,h1[i]));
    }


    return out;
}

double PoseTools::min2D(geometry_msgs::Pose base, std::vector<geometry_msgs::Pose> &h1){
    double out=9999;



    for(unsigned int i=0;i<h1.size();i++){

        out=std::min(out,distance2D(base,h1[i]));
    }


    return out;
}

double PoseTools::mean3p(geometry_msgs::Pose base, std::vector<geometry_msgs::Pose> &h1){
    double out=0;

    if(h1.size()>=3){
        out=out+distance(base,h1[0]);
        out=out+distance(base,h1[h1.size()/2]);
        out=out+distance(base,h1[h1.size()-1]);
        out=out/3.0;

    }else{
        out=mean(base,h1);
    }


    return out;
}

double PoseTools::min3p(geometry_msgs::Pose base, std::vector<geometry_msgs::Pose> &h1){
    double out=9999;

    if(h1.size()>=3){
        out=std::min(out,distance(base,h1[0]));
        out=std::min(out,distance(base,h1[h1.size()/2]));
        out=std::min(out,distance(base,h1[h1.size()-1]));

    }else{
        out=min(base,h1);
    }


    return out;
}

double PoseTools::min3p2D(geometry_msgs::Pose base, std::vector<geometry_msgs::Pose> &h1){
    double out=9999;

    if(h1.size()>=3){
        out=std::min(out,distance2D(base,h1[0]));
        out=std::min(out,distance2D(base,h1[h1.size()/2]));
        out=std::min(out,distance2D(base,h1[h1.size()-1]));

    }else{
        out=min2D(base,h1);
    }


    return out;
}

geometry_msgs::Pose PoseTools::nearPose(geometry_msgs::Pose base, geometry_msgs::Pose h1, geometry_msgs::Pose h2){

    double dist1=distance(base,h1);
    double dist2=distance(base,h2);

    if(dist1<dist2){
        return h1;
    }else{
        return h2;
    }
}

double PoseTools::distanceToSegment(geometry_msgs::Pose Point, geometry_msgs::Pose LineStart, geometry_msgs::Pose LineEnd){


    float LineMag;
    float U;
    geometry_msgs::Pose Intersection;

    LineMag = distance( LineEnd, LineStart );

    U = ( ( ( Point.position.x - LineStart.position.x ) * ( LineEnd.position.x - LineStart.position.x ) ) +
          ( ( Point.position.y  - LineStart.position.y  ) * ( LineEnd.position.y  - LineStart.position.y ) ) +
          ( ( Point.position.z - LineStart.position.z ) * ( LineEnd.position.z - LineStart.position.z ) ) ) /
            ( LineMag * LineMag );

    if( U < 0.0f || U > 1.0f )
        return std::max(distance(LineStart,Point),distance(Point,LineEnd));   // closest point does not fall within the line segment

    Intersection.position.x  = LineStart.position.x  + U * ( LineEnd.position.x  - LineStart.position.x  );
    Intersection.position.y  = LineStart.position.y  + U * ( LineEnd.position.y - LineStart.position.y );
    Intersection.position.z  = LineStart.position.z  + U * ( LineEnd.position.z - LineStart.position.z );

    // *Distance = Magnitude( Point, &Intersection );

    return distance(Point, Intersection);

    //    geometry_msgs::Pose v = P1 - P0;
    //    geometry_msgs::Pose w =base- P0;

    //    double c1 = dot(w,v);
    //    if ( c1 <= 0 )
    //        return (c1/fabs(c1))* distance(base,P0);

    //    double c2 = dot(v,v);
    //    if ( c2 <= c1 )
    //        return (c2/fabs(c2))* distance(base,P1);

    //    double b = c1 / c2;
    //    geometry_msgs::Pose Pb = P0 +  v*b;
    //    return (b/fabs(b))* distance(base, Pb);




}

double PoseTools::angleBetween(double x0, double y0, double x1, double y1){
    return atan2(y1-y0,x1-x0);
}

double PoseTools::angleBetween(geometry_msgs::Pose a, geometry_msgs::Pose b, geometry_msgs::Pose c){

    geometry_msgs::Pose ab;
    ab.position.x=( b.position.x - a.position.x);
    ab.position.y=( b.position.y - a.position.y);
    geometry_msgs::Pose cb;

    cb.position.x=b.position.x - c.position.x;
    cb.position.y=b.position.y - c.position.y;


    double dot = ((ab.position.x * cb.position.x) + (ab.position.y * cb.position.y)); // dot product
    double cross = ((ab.position.x * cb.position.y) - (ab.position.y * cb.position.x)); // cross product

    return  atan2(cross, dot);

}

double PoseTools::perpendicular(geometry_msgs::Pose u, geometry_msgs::Pose v){

    return (u.position.x * v.position.y - u.position.y * v.position.x) ;

}

int PoseTools::insideSegment(geometry_msgs::Pose P, geometry_msgs::Pose SP0, geometry_msgs::Pose SP1)
{
    if (SP0.position.x != SP1.position.x) {    // S is not  vertical
        if (SP0.position.x <= P.position.x && P.position.x <= SP1.position.x)
            return 1;
        if (SP0.position.x >= P.position.x && P.position.x >= SP1.position.x)
            return 1;
    }
    else {    // S is vertical, so test y  coordinate
        if (SP0.position.y <= P.position.y && P.position.y <= SP1.position.y)
            return 1;
        if (SP0.position.y >= P.position.y && P.position.y >= SP1.position.y)
            return 1;
    }
    return 0;
}

int PoseTools::intersect2DLines(geometry_msgs::Pose S1P0, geometry_msgs::Pose S1P1, geometry_msgs::Pose S2P0, geometry_msgs::Pose &S2P1, geometry_msgs::Pose *I0, geometry_msgs::Pose *I1)
{
    geometry_msgs::Pose   u = S1P1 - S1P0;
    geometry_msgs::Pose   v = S2P1 - S2P0;
    geometry_msgs::Pose   w = S1P0 - S2P0;
    float     D = perpendicular(u,v);

    // test if  they are parallel (includes either being a point)
    if (fabs(D) <  0.00000001) {           // S1 and S2 are parallel
        if (perpendicular(u,w) != 0 || perpendicular(v,w) != 0)  {
            return 0;                    // they are NOT collinear
        }
        // they are collinear or degenerate
        // check if they are degenerate  points
        float du = dot(u,u);
        float dv = dot(v,v);
        if (du==0 && dv==0) {            // both segments are points
            if (S1P0 !=  S2P0)         // they are distinct  points
                return 0;
            *I0 = S1P0;                 // they are the same point
            return 1;
        }
        if (du==0) {                     // S1 is a single point
            if  (insideSegment(S1P0, S2P0,S2P1) == 0)  // but is not in S2
                return 0;
            *I0 = S1P0;
            return 1;
        }
        if (dv==0) {                     // S2 a single point
            if  (insideSegment(S2P0, S1P0,S1P1) == 0)  // but is not in S1
                return 0;
            *I0 = S2P0;
            return 1;
        }
        // they are collinear segments - get  overlap (or not)
        float t0, t1;                    // endpoints of S1 in eqn for S2
        geometry_msgs::Pose w2 = S1P1 - S2P0;
        if (v.position.x != 0) {
            t0 = w.position.x / v.position.x;
            t1 = w2.position.x / v.position.x;
        }
        else {
            t0 = w.position.y / v.position.y;
            t1 = w2.position.y / v.position.y;
        }
        if (t0 > t1) {                   // must have t0 smaller than t1
            float t=t0; t0=t1; t1=t;    // swap if not
        }
        if (t0 > 1 || t1 < 0) {
            return 0;      // NO overlap
        }
        t0 = t0<0? 0 : t0;               // clip to min 0
        t1 = t1>1? 1 : t1;               // clip to max 1
        if (t0 == t1) {                  // intersect is a point
            *I0 = S2P0 +  t0 * v;
            return 1;
        }

        // they overlap in a valid subsegment
        *I0 = S2P0 + t0 * v;
        *I1 = S2P0 + t1 * v;
        return 2;
    }

    // the segments are skew and may intersect in a point
    // get the intersect parameter for S1
    float     sI = perpendicular(v,w) / D;
    if (sI < 0 || sI > 1)                // no intersect with S1
        return 0;

    // get the intersect parameter for S2
    float     tI = perpendicular(u,w) / D;
    if (tI < 0 || tI > 1)                // no intersect with S2
        return 0;

    *I0 = S1P0 + sI * u;                // compute S1 intersect point
    return 1;
}

std::string PoseTools::toStringCompressed(geometry_msgs::Pose pose1){

    return "["+$("{}").format(pose1.position.x)+
            ","+$("{}").format(pose1.position.y)+
            ","+$("{}").format(pose1.position.z)+
            ","+$("{}").format(getYaw(pose1))+"]";


}

std::string PoseTools::toStringCompressed(std::vector<geometry_msgs::Pose> pose1){

    std::stringstream str;

    for(unsigned int i=0;i<pose1.size();i++){
        str<< "["+$("{}").format(pose1[i].position.x)+
              ","+$("{}").format(pose1[i].position.y)+
              ","+$("{}").format(pose1[i].position.z)+
              ","+$("{}").format(pose1[i].orientation.w)+
              ","+$("{}").format(pose1[i].orientation.x)+
              ","+$("{}").format(pose1[i].orientation.y)+
              ","+$("{}").format(pose1[i].orientation.z)+"];";
    }
    return str.str();

}

std::string PoseTools::toStringPointCompressedMatlab(std::vector<geometry_msgs::Pose> &pose1){

    std::stringstream str;

    for(unsigned int i=0;i<pose1.size();i++){
        str<< $("{}").format(pose1[i].position.x)+
              ","+$("{}").format(pose1[i].position.y)+
              ","+$("{}").format(pose1[i].position.z)+";";
    }
    return str.str();

}


std::string PoseTools::toStringCompressed(geometry_msgs::Twist pose1){

    return "["+$("{}").format(pose1.linear.x)+
            ","+$("{}").format(pose1.angular.z) +"]";


}

geometry_msgs::Pose PoseTools::add(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    geometry_msgs::Pose out;
    out.position.x=p1.position.x+p2.position.x;
    out.position.y=p1.position.y+p2.position.y;
    out.position.z=p1.position.z+p2.position.z;


    return out;
}

geometry_msgs::Pose PoseTools::diff(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    geometry_msgs::Pose out;
    out.position.x=p1.position.x-p2.position.x;
    out.position.y=p1.position.y-p2.position.y;
    out.position.z=p1.position.z-p2.position.z;
    return out;
}

geometry_msgs::Pose PoseTools::fulladd(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    geometry_msgs::Pose out;
    out.position.x=p1.position.x+p2.position.x;
    out.position.y=p1.position.y+p2.position.y;
    out.position.z=p1.position.z+p2.position.z;
    //    out.orientation.w=p1.orientation.w+p2.orientation.w;
    //    out.orientation.x=p1.orientation.x+p2.orientation.x;
    //    out.orientation.y=p1.orientation.y+p2.orientation.y;
    //    out.orientation.z=p1.orientation.z+p2.orientation.z;

    Euler p1_=PoseTools::getEuler(p1);
    Euler p2_=PoseTools::getEuler(p2);

    Euler p3_=p1_+p2_;
    out.orientation=p3_.toQuaternionROS();

    return out;
}

geometry_msgs::Pose PoseTools::fulladd(geometry_msgs::Pose p1, double x, double y, double theta){
    geometry_msgs::Pose out;
    out.position.x=p1.position.x+x;
    out.position.y=p1.position.y+y;


    Euler p1_=PoseTools::getEuler(p1);
    p1_.yaw+=theta;

    out.orientation=p1_.toQuaternionROS();

    return out;
}

void PoseTools::fullUpdate(geometry_msgs::Pose &p1, double x, double y, double theta){

    p1.position.x+=x;
    p1.position.y+=y;
    Euler p1_=PoseTools::getEuler(p1);
    p1_.yaw+=theta;

    p1.orientation=p1_.toQuaternionROS();

    //    return out;
}

geometry_msgs::Pose PoseTools::fulldiff(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    geometry_msgs::Pose out;
    out.position.x=p1.position.x-p2.position.x;
    out.position.y=p1.position.y-p2.position.y;
    out.position.z=p1.position.z-p2.position.z;

    Euler p1_=PoseTools::getEuler(p1);
    Euler p2_=PoseTools::getEuler(p2);

    Euler p3_=p1_-p2_;
    out.orientation=p3_.toQuaternionROS();



    //    out.orientation.w=p1.orientation.w-p2.orientation.w;
    //    out.orientation.x=p1.orientation.x-p2.orientation.x;
    //    out.orientation.y=p1.orientation.y-p2.orientation.y;
    //    out.orientation.z=p1.orientation.z-p2.orientation.z;

    return out;
}

std::string PointTools::toStringMatlab(geometry_msgs::Pose pose1){
    return ""+$("{}").format(pose1.position.x)+
            ","+$("{}").format(pose1.position.y)+
            ","+$("{}").format(pose1.position.z);
}

std::string PointTools::toStringMatlab(std::vector<geometry_msgs::Pose> &poses){

    std::stringstream str;
    for(unsigned int i=0;i<poses.size();i++){
        str<<toStringMatlab(poses[i])<<";";
    }

    return str.str();
}


double PoseTools::distance(double x0, double y0, double z0, double x1, double y1, double z1){
    double dx=x0-x1;
    double dy=y0-y1;
    double dz=z0-z1;
    return sqrt(dx*dx+dy*dy+dz*dz);

}

void PoseTools::setRPY(geometry_msgs::Pose &pose, double roll, double pitch, double yaw){

    Quaternion qt;
    qt.setRPY(roll,pitch,yaw);

    pose.orientation.w=qt.w();
    pose.orientation.x=qt.x();
    pose.orientation.y=qt.y();
    pose.orientation.z=qt.z();

}

void PoseTools::updateRPY(geometry_msgs::Pose &pose, double roll, double pitch, double yaw){
    auto eu=PoseTools::getEuler(pose);
    //(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    Quaternion qt;

    qt.setRPY(eu.roll+roll,eu.pitch+pitch,eu.yaw+yaw);

    pose.orientation.w=qt.w();
    pose.orientation.x=qt.x();
    pose.orientation.y=qt.y();
    pose.orientation.z=qt.z();


}
