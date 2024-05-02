
#include "Transform.h"

void Transform::setDisposeable(bool dispose_){
    dispose=dispose_;
}

Transform *Transform::getParentTransform(){
    return Parent;
}

std::string Transform::getTransformFrame(){
    return frame;
}

void Transform::setParentTransform(Transform *Pr){
    Parent=Pr;
}

void Transform::setTransformFrame(std::string f){
    frame=f;
}

double Transform::get(unsigned int i, unsigned int j){
    return matrix->get(i,j);
}

void Transform::set(unsigned int i, unsigned int j, double d){
    matrix->set(i,j,d);
}

double *Transform::getMatrix(){
    return matrix->matrix;
}

bool Transform::isTranslation(){
    return std::fabs(matrix->matrix[12])>0.00001 || std::fabs(matrix->matrix[13])>0.00001 || std::fabs(matrix->matrix[14])>0.00001;
}

bool Transform::isIdentity(){

    if(std::fabs(matrix->matrix[0]-1.0)>0.00001){
        return false;
    }
    if(std::fabs(matrix->matrix[5]-1.0)>0.00001){
        return false;
    }
    if(std::fabs(matrix->matrix[10]-1.0)>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[15]-1.0)>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[1])>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[2])>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[3])>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[4])>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[6])>0.00001){
        return false;
    }


    if(std::fabs(matrix->matrix[7])>0.00001){
        return false;
    }


    if(std::fabs(matrix->matrix[8])>0.00001){
        return false;
    }


    if(std::fabs(matrix->matrix[9])>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[11])>0.00001){
        return false;
    }


    if(std::fabs(matrix->matrix[12])>0.00001){
        return false;
    }
    if(std::fabs(matrix->matrix[13])>0.00001){
        return false;
    }

    if(std::fabs(matrix->matrix[14])>0.00001){
        return false;
    }
    return true;

}

Garrote::Math::Matrix::Matrix<float> *Transform::getMatrixFloatingPoint(){

    Garrote::Math::Matrix:: Matrix<float> *var=matrix->cloneCast<float>();



    /*new Garrote::Math::Matrix::Matrix<float>(4,4);

        for(int i=0;i<4;i++){for(int j=0;j<4;j++){ (*var)(i,j)=(float)(*matrix)(i,j); }}*/

    return var;
}



Transform::Transform(){
    matrix=new Garrote::Math::Matrix::Matrix<double>(4,4);

    matrix->matrix[0]=1;
    matrix->matrix[1]=0;
    matrix->matrix[2]=0;
    matrix->matrix[3]=0;

    matrix->matrix[4]=0;
    matrix->matrix[5]=1;
    matrix->matrix[6]=0;
    matrix->matrix[7]=0;

    matrix->matrix[8]=0;
    matrix->matrix[9]=0;
    matrix->matrix[10]=1;
    matrix->matrix[11]=0;

    matrix->matrix[12]=0;
    matrix->matrix[13]=0;
    matrix->matrix[14]=0;
    matrix->matrix[15]=1;

    Parent=NULL;
    frame="";
}

Transform::Transform(Garrote::Math::Matrix::Matrix<float> *matrix_){
    matrix=matrix_->cloneCast<double>();
    Parent=NULL;
    frame="";
}

Transform::Transform(Garrote::Math::Matrix::Matrix<double> *matrix_){
    //std::cout<<"USING CLONE Transform(Garrote::Math::Matrix::Matrix<double>* matrix_)"<<std::endl;
    matrix=matrix_->clone();
    Parent=NULL;
    frame="";
}

Transform::Transform(Garrote::Math::Matrix::Matrix<double> *matrix_, bool reuse){
    //std::cout<<"USING CLONE Transform(Garrote::Math::Matrix::Matrix<double>* matrix_)"<<std::endl;

    if(!reuse){
        matrix=matrix_->clone();
    }else{
        matrix=matrix_;
    }
    Parent=NULL;
    frame="";
}

Transform::Transform(const Transform &other){
    matrix=other.matrix->clone();
    Parent=other.Parent;
    frame=other.frame;
}

Transform::Transform(std::string mat){

    matrix=new Garrote::Math::Matrix::Matrix<double>(4,4);

    setIdentity();

    Parent=NULL;
    frame="";
    fromString(mat);

}

Transform::Transform(Quaternion q){

    matrix=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //matrix->matrix[0]=1;
    //matrix->matrix[1]=0;
    //matrix->matrix[2]=0;
    matrix->matrix[3]=0;

    //matrix->matrix[4]=0;
    //matrix->matrix[5]=1;
    //matrix->matrix[6]=0;
    matrix->matrix[7]=0;

    //matrix->matrix[8]=0;
    //matrix->matrix[9]=0;
    //matrix->matrix[10]=1;
    matrix->matrix[11]=0;

    matrix->matrix[12]=0;
    matrix->matrix[13]=0;
    matrix->matrix[14]=0;
    matrix->matrix[15]=1;



    Parent=NULL;
    frame="";



    double d = sqrt(q.norm());
    double s = double(2.0) / d;

    double xs = q.x() * s;
    double ys = q.y() * s;
    double zs = q.z() * s;
    double wx = q.w() * xs;
    double wy = q.w() * ys;
    double wz = q.w() * zs;
    double xx = q.x() * xs;
    double xy = q.x() * ys;
    double xz = q.x() * zs;
    double yy = q.y() * ys;
    double yz = q.y() * zs;
    double zz = q.z() * zs;


    //matrix!!
    //0 4 8 12
    //1 5 9 13
    //2 6 10 14
    //3 7 11 15


    matrix->matrix[0]=1.0 - (yy + zz);
    matrix->matrix[4]=xy - wz;
    matrix->matrix[8]=xz + wy;

    matrix->matrix[1]= xy + wz;
    matrix->matrix[5]= (1.0) - (xx + zz);
    matrix->matrix[9]=yz - wx;

    matrix->matrix[2]=xz - wy;
    matrix->matrix[6]=yz + wx;
    matrix->matrix[10]=(1.0) - (xx + yy);


//    matrix->matrix[0]=1.0 - (yy + zz);
//    matrix->matrix[4]=xy - wz;
//    matrix->matrix[8]=xz + wy;

//    matrix->matrix[1]= xy + wz;
//    matrix->matrix[5]= double(1.0) - (xx + zz);
//    matrix->matrix[9]=yz - wx;

//    matrix->matrix[2]=xz - wy;
//    matrix->matrix[6]=yz + wx;
//    matrix->matrix[10]=double(1.0) - (xx + yy);


}

void Transform::setRotation(double w, double x, double y, double z){

    //matrix=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //matrix->matrix[0]=1;
    //matrix->matrix[1]=0;
    //matrix->matrix[2]=0;
    //        matrix->matrix[3]=0;

    //        //matrix->matrix[4]=0;
    //        //matrix->matrix[5]=1;
    //        //matrix->matrix[6]=0;
    //        matrix->matrix[7]=0;

    //        //matrix->matrix[8]=0;
    //        //matrix->matrix[9]=0;
    //        //matrix->matrix[10]=1;
    //        matrix->matrix[11]=0;

    //        matrix->matrix[12]=0;
    //        matrix->matrix[13]=0;
    //        matrix->matrix[14]=0;
    //        matrix->matrix[15]=1;

    Quaternion q(w,x,y,z,true);

    //        Parent=NULL;
    //        frame="";



    double d = q.norm();
    double s = double(2.0) / d;

    double xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
    double wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
    double xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
    double yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;


    //matrix!!
    //0 4 8 12
    //1 5 9 13
    //2 6 10 14
    //3 7 11 15


    matrix->matrix[0]=1.0 - (yy + zz);
    matrix->matrix[4]=xy - wz;
    matrix->matrix[8]=xz + wy;

    matrix->matrix[1]= xy + wz;
    matrix->matrix[5]= double(1.0) - (xx + zz);
    matrix->matrix[9]=yz - wx;

    matrix->matrix[2]=xz - wy;
    matrix->matrix[6]=yz + wx;
    matrix->matrix[10]=double(1.0) - (xx + yy);


}

//void Transform::getEulerYPR(double &yaw, double &pitch, double &roll, int solution_number){


//    Euler euler_out;
//    Euler euler_out2; //second solution
//    //get the pointer to the raw data

//    // Check that pitch is not at a singularity
//    // Check that pitch is not at a singularity





//    if (fabs(matrix->matrix[2]) >= 1.0)
//    {
//        euler_out.yaw = 0;
//        euler_out2.yaw = 0;

//        // From difference of angles formula
//        if (matrix->matrix[2] < 0.0)  //gimbal locked down
//        {
//            double delta = atan2(matrix->matrix[4],matrix->matrix[8]);
//            euler_out.pitch = M_PI / double(2.0);
//            euler_out2.pitch = M_PI / double(2.0);
//            euler_out.roll = delta;
//            euler_out2.roll = delta;
//        }
//        else // gimbal locked up
//        {
//            double delta = atan2(-matrix->matrix[4],-matrix->matrix[8]);
//            euler_out.pitch = -M_PI / double(2.0);
//            euler_out2.pitch = -M_PI / double(2.0);
//            euler_out.roll = delta;
//            euler_out2.roll = delta;
//        }
//    }
//    else
//    {

//        //matrix!!
//        //0 4 8 12
//        //1 5 9 13
//        //2 6 10 14
//        //3 7 11 15
//        euler_out.pitch = - asin(matrix->matrix[2]);
//        euler_out2.pitch = M_PI - euler_out.pitch;

//        euler_out.roll = atan2(matrix->matrix[6]/cos(euler_out.pitch), matrix->matrix[7]/cos(euler_out.pitch));
//        euler_out2.roll = atan2(matrix->matrix[6]/cos(euler_out2.pitch),matrix->matrix[7]/cos(euler_out2.pitch));

//        euler_out.yaw = atan2(matrix->matrix[1]/cos(euler_out.pitch),matrix->matrix[0]/cos(euler_out.pitch));
//        euler_out2.yaw = atan2(matrix->matrix[1]/cos(euler_out2.pitch), matrix->matrix[0]/cos(euler_out2.pitch));
//    }

//    if (solution_number == 1)
//    {
//        yaw = euler_out.yaw;
//        pitch = euler_out.pitch;
//        roll = euler_out.roll;
//    }
//    else
//    {
//        yaw = euler_out2.yaw;
//        pitch = euler_out2.pitch;
//        roll = euler_out2.roll;
//    }


//}




void Transform::getEulerYPR(double &yaw, double &pitch, double &roll, int solution_number){


    Euler euler_out;
    Euler euler_out2; //second solution
    //get the pointer to the raw data

    // Check that pitch is not at a singularity
    // Check that pitch is not at a singularity


    /*
 * http://docs.ros.org/melodic/api/tf/html/c++/Matrix3x3_8h_source.html
x-0	x-1	x-2	0-3
y-4	y-5	y-6	0-7
z-8	z-9	z-10	0=11
12	13	14	15
*/


    if (fabs(matrix->matrix[2]) >= 1.0)
    {
        euler_out.yaw = 0;
        euler_out2.yaw = 0;

        // From difference of angles formula
        if (matrix->matrix[2] < 0.0)  //gimbal locked down
        {
            double delta = atan2(matrix->matrix[4],matrix->matrix[8]);
            euler_out.pitch = M_PI / double(2.0);
            euler_out2.pitch = M_PI / double(2.0);
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
        else // gimbal locked up
        {
            double delta = atan2(-matrix->matrix[4],-matrix->matrix[8]);
            euler_out.pitch = -M_PI / double(2.0);
            euler_out2.pitch = -M_PI / double(2.0);
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
    }
    else
    {

        //matrix!!
        //0 4 8 12
        //1 5 9 13
        //2 6 10 14
        //3 7 11 15
        euler_out.pitch = - asin(matrix->matrix[2]);
        euler_out2.pitch = M_PI - euler_out.pitch;

        euler_out.roll = atan2(matrix->matrix[6]/cos(euler_out.pitch), matrix->matrix[10]/cos(euler_out.pitch));
        euler_out2.roll = atan2(matrix->matrix[6]/cos(euler_out2.pitch),matrix->matrix[10]/cos(euler_out2.pitch));

        euler_out.yaw = atan2(matrix->matrix[1]/cos(euler_out.pitch),matrix->matrix[0]/cos(euler_out.pitch));
        euler_out2.yaw = atan2(matrix->matrix[1]/cos(euler_out2.pitch), matrix->matrix[0]/cos(euler_out2.pitch));
    }

    if (solution_number == 1)
    {
        yaw = euler_out.yaw;
        pitch = euler_out.pitch;
        roll = euler_out.roll;
    }
    else
    {
        yaw = euler_out2.yaw;
        pitch = euler_out2.pitch;
        roll = euler_out2.roll;
    }


}







double Transform::getYaw(){

    double roll;
    double pitch;
    double yaw;
    getRPY(roll,pitch,yaw);

    return yaw;
}

double Transform::getYawFast(){


    Euler euler_out;

    //get the pointer to the raw data
    // Check that pitch is not at a singularity
    // Check that pitch is not at a singularity

    if (fabs(matrix->matrix[2]) >= 1.0)
    {
        euler_out.yaw = 0;
    }
    else
    {
        euler_out.pitch = - asin(matrix->matrix[2]);
        euler_out.yaw = atan2(matrix->matrix[1]/cos(euler_out.pitch),matrix->matrix[0]/cos(euler_out.pitch));
    }

    return euler_out.yaw;
}

void Transform::getRPY(double &roll, double &pitch, double &yaw, int solution_number){

    getEulerYPR(yaw, pitch, roll, solution_number);

    //        struct Euler
    //        {
    //            double yaw;
    //            double pitch;
    //            double roll;
    //        };

    //        Euler euler_out;
    //        Euler euler_out2; //second solution
    //        //get the pointer to the raw data

    //        // Check that pitch is not at a singularity
    //        if (fabs(matrix->operator ()(2,0)) >= 1)
    //        {
    //            euler_out.yaw = 0;
    //            euler_out2.yaw = 0;

    //            // From difference of angles formula
    //            double delta = atan2(matrix->operator ()(2,1),matrix->operator ()(2,2));
    //            if (matrix->operator ()(2,0) < 0.0)  //gimbal locked down
    //            {
    //                euler_out.pitch = M_PI / double(2.0);
    //                euler_out2.pitch = M_PI / double(2.0);
    //                euler_out.roll = delta;
    //                euler_out2.roll = delta;
    //            }
    //            else // gimbal locked up
    //            {
    //                euler_out.pitch = -M_PI / double(2.0);
    //                euler_out2.pitch = -M_PI / double(2.0);
    //                euler_out.roll = delta;
    //                euler_out2.roll = delta;
    //            }
    //        }
    //        else
    //        {
    //            euler_out.pitch = - asin(matrix->operator ()(2,0));
    //            euler_out2.pitch = M_PI - euler_out.pitch;

    //            euler_out.roll = atan2(matrix->operator ()(2,1)/cos(euler_out.pitch),
    //                                   matrix->operator ()(2,2)/cos(euler_out.pitch));
    //            euler_out2.roll = atan2(matrix->operator ()(2,1)/cos(euler_out2.pitch),
    //                                    matrix->operator ()(2,2)/cos(euler_out2.pitch));

    //            euler_out.yaw = atan2(matrix->operator ()(1,0)/cos(euler_out.pitch),
    //                                  matrix->operator ()(0,0)/cos(euler_out.pitch));
    //            euler_out2.yaw = atan2(matrix->operator ()(1,0)/cos(euler_out2.pitch),
    //                                   matrix->operator ()(0,0)/cos(euler_out2.pitch));
    //        }

    //        if (solution_number == 1)
    //        {
    //            yaw = euler_out.yaw;
    //            pitch = euler_out.pitch;
    //            roll = euler_out.roll;
    //        }
    //        else
    //        {
    //            yaw = euler_out2.yaw;
    //            pitch = euler_out2.pitch;
    //            roll = euler_out2.roll;
    //        }

}

Transform Transform::byPose(geometry_msgs::Pose pose){


    //EullerAngles ea;
    //ea.fromQuaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);

    Transform tf(Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    tf.setDisposeable(true); // dont delete matrix;
    //        double useless_pitch, useless_roll, yaw;

    //        tf.getRPY(useless_roll, useless_pitch,yaw);
    //        out->applyRotationZ(yaw);
    //        out->applyTranslation(pose.position.x,pose.position.y,pose.position.z);
    tf.matrix->matrix[12]=pose.position.x;
    tf.matrix->matrix[13]=pose.position.y;
    tf.matrix->matrix[14]=pose.position.z;





    return tf;

}

Transform *Transform::createTransformFromPose(geometry_msgs::Pose pose){


    //EullerAngles ea;
    //ea.fromQuaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);

    Transform tf(Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    tf.setDisposeable(false); // dont delete matrix;
    //        double useless_pitch, useless_roll, yaw;

    //        tf.getRPY(useless_roll, useless_pitch,yaw);
    //        out->applyRotationZ(yaw);
    //        out->applyTranslation(pose.position.x,pose.position.y,pose.position.z);
    tf.matrix->matrix[12]=pose.position.x;
    tf.matrix->matrix[13]=pose.position.y;
    tf.matrix->matrix[14]=pose.position.z;





    return new Transform(tf.matrix,true);

}

std::vector<double> Transform::scale(){
std::vector<double> out;

out.push_back(std::sqrt(matrix->matrix[0]*matrix->matrix[0]+matrix->matrix[1]*matrix->matrix[1]+matrix->matrix[2]*matrix->matrix[2]));
out.push_back(std::sqrt(matrix->matrix[4]*matrix->matrix[4]+matrix->matrix[5]*matrix->matrix[5]+matrix->matrix[6]*matrix->matrix[6]));
out.push_back(std::sqrt(matrix->matrix[8]*matrix->matrix[8]+matrix->matrix[9]*matrix->matrix[9]+matrix->matrix[10]*matrix->matrix[10]));





return out;
}

Transform *Transform::fromPose(geometry_msgs::Pose pose){
    return createTransformFromPose(pose);
}

void Transform::updatefromPose(geometry_msgs::Pose pose){


    //        EullerAngles ea;
    //        ea.fromQuaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    //        identity();
    //        applyRotationZ(ea.yaw);
    //        applyTranslation(pose.position.x,pose.position.y,pose.position.z);

    // Parent=NULL;
    // frame="";

    Transform tf(Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));

    tf.matrix->matrix[12]=pose.position.x;
    tf.matrix->matrix[13]=pose.position.y;
    tf.matrix->matrix[14]=pose.position.z;

    delete matrix;

    matrix=tf.matrix;
    tf.setDisposeable(false);

    //        double useless_pitch, useless_roll, yaw;

    //        tf.getRPY( useless_roll,useless_pitch,yaw);

    //        setIdentity();
    //        applyRotationZ(yaw);
    //        applyTranslation(pose.position.x,pose.position.y,pose.position.z);



}

geometry_msgs::Pose Transform::toPose(Transform *r){

    geometry_msgs::Pose pose;

    if(r!=NULL){
        //            Point pt=(*r)*Point(0,0,0);
        Point pt=r->point();

        Quaternion cm;

        double pitch, roll, yaw;

        r->getRPY(roll,pitch,yaw);
        //        std::cout<<"internals "<<roll<<"  "<<pitch<<"  "<<yaw<<"  "<<std::endl;
        cm.setRPY(roll,pitch,yaw);


        //http://docs.ros.org/indigo/api/tf/html/c++/Quaternion_8h_source.html

        //        cm.q0 = 0.5*sqrt(1.0+r->get(0,0)+r->get(1,1)+r->get(2,2)); //( r->get(0,0) + r->get(1,1) + r->get(2,2) + 1.0f) / 4.0f;
        //        cm.q1 = (0.25/cm.q0)*(r->get(2,1)-r->get(1,2));//( r->get(0,0) - r->get(1,1) - r->get(2,2) + 1.0f) / 4.0f;
        //        cm.q2 = (0.25/cm.q0)*(r->get(0,2)-r->get(2,0)); //(-r->get(0,0) + r->get(1,1) - r->get(2,2) + 1.0f) / 4.0f;
        //        cm.q3 = (0.25/cm.q0)*(r->get(1,0)-r->get(0,1));//(-r->get(0,0) - r->get(1,1) + r->get(2,2) + 1.0f) / 4.0f;


        //http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
        //http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html
        //http://wiki.ros.org/tf/Overview/Transformations
        pose.position.x=pt.X();
        pose.position.y=pt.Y();
        pose.position.z=pt.Z();

        pose.orientation.x=cm.q1;
        pose.orientation.y=cm.q2;
        pose.orientation.z=cm.q3;
        pose.orientation.w=cm.q0;

    }
    return pose;


}

geometry_msgs::Pose Transform::toPose(){

    geometry_msgs::Pose pose;
     Quaternion cm;

    double pitch, roll, yaw;

    //    std::cout<<toString()<<std::endl;
    getRPY(roll,pitch,yaw);
    cm.setRPY(roll,pitch,yaw);


    pose.position.x=x();
    pose.position.y=y();
    pose.position.z=z();

    pose.orientation.x=cm.q1;
    pose.orientation.y=cm.q2;
    pose.orientation.z=cm.q3;
    pose.orientation.w=cm.q0;


    return pose;

}

geometry_msgs::Quaternion Transform::toQuaternion(){

    geometry_msgs::Quaternion pose;
    Quaternion cm;

    double pitch, roll, yaw;

    getRPY(roll,pitch,yaw);
    cm.setRPY(roll,pitch,yaw);

    pose.x=cm.q1;
    pose.y=cm.q2;
    pose.z=cm.q3;
    pose.w=cm.q0;

    return pose;
}

void Transform::copy(Transform *tf){

    matrix->matrix[0]=tf->matrix->matrix[0];
    matrix->matrix[1]=tf->matrix->matrix[1];
    matrix->matrix[2]=tf->matrix->matrix[2];
    matrix->matrix[3]=tf->matrix->matrix[3];

    matrix->matrix[4]=tf->matrix->matrix[4];
    matrix->matrix[5]=tf->matrix->matrix[5];
    matrix->matrix[6]=tf->matrix->matrix[6];
    matrix->matrix[7]=tf->matrix->matrix[7];

    matrix->matrix[8]=tf->matrix->matrix[8];
    matrix->matrix[9]=tf->matrix->matrix[9];
    matrix->matrix[10]=tf->matrix->matrix[10];
    matrix->matrix[11]=tf->matrix->matrix[11];

    matrix->matrix[12]=tf->matrix->matrix[12];
    matrix->matrix[13]=tf->matrix->matrix[13];
    matrix->matrix[14]=tf->matrix->matrix[14];
    matrix->matrix[15]=tf->matrix->matrix[15];


}

void Transform::fromString(std::string mat){

    //matrix!!
    //0 4 8 12
    //1 5 9 13
    //2 6 10 14
    //3 7 11 15


    double a0,a1,a2,a3;
    double b0,b1,b2,b3;
    double c0,c1,c2,c3;
    double d0,d1,d2,d3;


    sscanf(mat.c_str(),"%lf %lf %lf %lf ; %lf %lf %lf %lf ; %lf %lf %lf %lf ; %lf %lf %lf %lf ",&a0,&a1,&a2,&a3,&b0,&b1,&b2,&b3,&c0,&c1,&c2,&c3,&d0,&d1,&d2,&d3);

    matrix->matrix[0]=a0;
    matrix->matrix[4]=a1;
    matrix->matrix[8]=a2;
    matrix->matrix[12]=a3;

    matrix->matrix[1]=b0;
    matrix->matrix[5]=b1;
    matrix->matrix[9]=b2;
    matrix->matrix[13]=b3;

    matrix->matrix[2]=c0;
    matrix->matrix[6]=c1;
    matrix->matrix[10]=c2;
    matrix->matrix[14]=c3;

    matrix->matrix[3]=d0;
    matrix->matrix[7]=d1;
    matrix->matrix[11]=d2;
    matrix->matrix[15]=d3;

    //        matrix->operator ()(1,0)=b0;
    //        matrix->operator ()(1,1)=b1;
    //        matrix->operator ()(1,2)=b2;
    //        matrix->operator ()(1,3)=b3;

    //        matrix->operator ()(2,0)=c0;
    //        matrix->operator ()(2,1)=c1;
    //        matrix->operator ()(2,2)=c2;
    //        matrix->operator ()(2,3)=c3;


    //        matrix->operator ()(3,0)=d0;
    //        matrix->operator ()(3,1)=d1;
    //        matrix->operator ()(3,2)=d2;
    //        matrix->operator ()(3,3)=d3;



}

void Transform::cloneRotation(Transform &tf){

    delete matrix;
    matrix=tf.matrix->clone();

    //        matrix->operator ()(0,3)=0;
    //        matrix->operator ()(1,3)=0;
    //        matrix->operator ()(2,3)=0;
    //        matrix->operator ()(3,3)=1;

    matrix->matrix[12]=0;
    matrix->matrix[13]=0;
    matrix->matrix[14]=0;
    matrix->matrix[15]=1;

    matrix->matrix[3]=0;
    matrix->matrix[7]=0;
    matrix->matrix[11]=0;


}

Matrix<double> Transform::getRotation(){


    Matrix<double> cs(3,3);


    cs.matrix[0]=matrix->matrix[0];
    cs.matrix[1]=matrix->matrix[1];
    cs.matrix[2]=matrix->matrix[2];

    cs.matrix[3]=matrix->matrix[4];
    cs.matrix[4]=matrix->matrix[5];
    cs.matrix[5]=matrix->matrix[6];

    cs.matrix[6]=matrix->matrix[8];
    cs.matrix[7]=matrix->matrix[9];
    cs.matrix[8]=matrix->matrix[10];
    //matrix!!
    //0 4 8 12
    //1 5 9 13
    //2 6 10 14
    //3 7 11 15

    //        for(unsigned int i=0;i<cs.getRows();i++){
    //            for(unsigned int j=0;j<cs.getColumns();j++){
    //                cs.set(i,j,matrix->get(i,j));
    //            }
    //        }

    return cs;

}

void Transform::cloneRotation3x3(Matrix<double> *tf){



    if((tf->getColumns()>=3) && (tf->getRows()>=3)){


        matrix->matrix[0]=tf->matrix[0];
        matrix->matrix[1]=tf->matrix[1];
        matrix->matrix[2]=tf->matrix[2];

        matrix->matrix[4]=tf->matrix[3];
        matrix->matrix[5]=tf->matrix[4];
        matrix->matrix[6]=tf->matrix[5];

        matrix->matrix[8]=tf->matrix[6];
        matrix->matrix[9]=tf->matrix[7];
        matrix->matrix[10]=tf->matrix[8];

    }else{
        throw "invalid dimensions to clone Rotation";
    }

    //        for(unsigned int i=0;i<tf->getRows();i++){
    //            for(unsigned int j=0;j<tf->getColumns();j++){
    //                matrix->set(i,j,tf->get(i,j));
    //            }
    //        }

}

void Transform::cloneRotation(Matrix<double> *tf){



    if((tf->getColumns()>=3) && (tf->getRows()>=3)){


        matrix->matrix[0]=tf->matrix[0];
        matrix->matrix[1]=tf->matrix[1];
        matrix->matrix[2]=tf->matrix[2];

        matrix->matrix[4]=tf->matrix[4];
        matrix->matrix[5]=tf->matrix[5];
        matrix->matrix[6]=tf->matrix[6];

        matrix->matrix[8]=tf->matrix[8];
        matrix->matrix[9]=tf->matrix[9];
        matrix->matrix[10]=tf->matrix[10];

    }else{
        throw "invalid dimensions to clone Rotation";
    }

    //        for(unsigned int i=0;i<tf->getRows();i++){
    //            for(unsigned int j=0;j<tf->getColumns();j++){
    //                matrix->set(i,j,tf->get(i,j));
    //            }
    //        }

}

void Transform::cloneMatrix(Garrote::Math::Matrix::Matrix<float> &tfmatrix){


    if((tfmatrix.getColumns()==matrix->getColumns()) && (tfmatrix.getRows()==matrix->getRows())){

        delete matrix;
        matrix=tfmatrix.cloneCast<double>();

    }else{
        throw "invalid dimensions to clone";
    }



    //        for(int i=0;i<4;i++){for(int j=0;j<4;j++){
    //                matrix->set(i,j,tfmatrix.get(i,j));
    //            }}

}

void Transform::cloneMatrix(Garrote::Math::Matrix::Matrix<double> &tfmatrix){
    if((tfmatrix.getColumns()==matrix->getColumns()) && (tfmatrix.getRows()==matrix->getRows())){
        delete matrix;
        matrix=tfmatrix.clone();
    }else{
        throw "invalid dimensions to clone";
    }
}

Transform Transform::identity(){
    static const Transform staticeye;
    return staticeye;
}

void Transform::setIdentity(){


    matrix->matrix[0]=1;
    matrix->matrix[1]=0;
    matrix->matrix[2]=0;
    matrix->matrix[3]=0;

    matrix->matrix[4]=0;
    matrix->matrix[5]=1;
    matrix->matrix[6]=0;
    matrix->matrix[7]=0;

    matrix->matrix[8]=0;
    matrix->matrix[9]=0;
    matrix->matrix[10]=1;
    matrix->matrix[11]=0;

    matrix->matrix[12]=0;
    matrix->matrix[13]=0;
    matrix->matrix[14]=0;
    matrix->matrix[15]=1;


}

Point Transform::point(){

    return Point(matrix->matrix[12],matrix->matrix[13],matrix->matrix[14]);
}

double Transform::x(){

    return matrix->matrix[12];

}

double Transform::y(){

    return matrix->matrix[13];
}

double Transform::z(){

    return matrix->matrix[14];
}

Point Transform::point(double x, double y, double z){

    Point output;

    output.x=matrix->matrix[0]*x + matrix->matrix[4]*y + matrix->matrix[8]*z + matrix->matrix[12];
    output.y=matrix->matrix[1]*x + matrix->matrix[5]*y + matrix->matrix[9]*z + matrix->matrix[13];
    output.z=matrix->matrix[2]*x + matrix->matrix[6]*y + matrix->matrix[10]*z + matrix->matrix[14];
    return output;
}
Point Transform::point(double x, double y){

    Point output;

    output.x=matrix->matrix[0]*x + matrix->matrix[4]*y + matrix->matrix[12];
    output.y=matrix->matrix[1]*x + matrix->matrix[5]*y + matrix->matrix[13];
    return output;
}
void Transform::removeTransform(){
    matrix->matrix[12]=0;
    matrix->matrix[13]=0;
    matrix->matrix[14]=0;
}

void Transform::removeRotation(){
    matrix->matrix[0]=1;
    matrix->matrix[1]=0;
    matrix->matrix[2]=0;

    matrix->matrix[4]=0;
    matrix->matrix[5]=1;
    matrix->matrix[6]=0;

    matrix->matrix[8]=0;
    matrix->matrix[9]=0;
    matrix->matrix[10]=1;
}

int Transform::inverter(double *m, double *out)
{
    double wtmp[4][8];
    double m0, m1, m2, m3, s;
    double *r0, *r1, *r2, *r3;
    r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];
    r0[0] = MAT(m, 0, 0), r0[1] = MAT(m, 0, 1),
            r0[2] = MAT(m, 0, 2), r0[3] = MAT(m, 0, 3),
            r0[4] = 1.0, r0[5] = r0[6] = r0[7] = 0.0,
            r1[0] = MAT(m, 1, 0), r1[1] = MAT(m, 1, 1),
            r1[2] = MAT(m, 1, 2), r1[3] = MAT(m, 1, 3),
            r1[5] = 1.0, r1[4] = r1[6] = r1[7] = 0.0,
            r2[0] = MAT(m, 2, 0), r2[1] = MAT(m, 2, 1),
            r2[2] = MAT(m, 2, 2), r2[3] = MAT(m, 2, 3),
            r2[6] = 1.0, r2[4] = r2[5] = r2[7] = 0.0,
            r3[0] = MAT(m, 3, 0), r3[1] = MAT(m, 3, 1),
            r3[2] = MAT(m, 3, 2), r3[3] = MAT(m, 3, 3),
            r3[7] = 1.0, r3[4] = r3[5] = r3[6] = 0.0;
    /* choose pivot - or die */
    if (fabs(r3[0]) > fabs(r2[0]))
        SWAP_ROWS(r3, r2);
    if (fabs(r2[0]) > fabs(r1[0]))
        SWAP_ROWS(r2, r1);
    if (fabs(r1[0]) > fabs(r0[0]))
        SWAP_ROWS(r1, r0);
    if (0.0 == r0[0])
        return 0;
    /* eliminate first variable     */
    m1 = r1[0] / r0[0];
    m2 = r2[0] / r0[0];
    m3 = r3[0] / r0[0];
    s = r0[1];
    r1[1] -= m1 * s;
    r2[1] -= m2 * s;
    r3[1] -= m3 * s;
    s = r0[2];
    r1[2] -= m1 * s;
    r2[2] -= m2 * s;
    r3[2] -= m3 * s;
    s = r0[3];
    r1[3] -= m1 * s;
    r2[3] -= m2 * s;
    r3[3] -= m3 * s;
    s = r0[4];
    if (s != 0.0) {
        r1[4] -= m1 * s;
        r2[4] -= m2 * s;
        r3[4] -= m3 * s;
    }
    s = r0[5];
    if (s != 0.0) {
        r1[5] -= m1 * s;
        r2[5] -= m2 * s;
        r3[5] -= m3 * s;
    }
    s = r0[6];
    if (s != 0.0) {
        r1[6] -= m1 * s;
        r2[6] -= m2 * s;
        r3[6] -= m3 * s;
    }
    s = r0[7];
    if (s != 0.0) {
        r1[7] -= m1 * s;
        r2[7] -= m2 * s;
        r3[7] -= m3 * s;
    }
    /* choose pivot - or die */
    if (fabs(r3[1]) > fabs(r2[1]))
        SWAP_ROWS(r3, r2);
    if (fabs(r2[1]) > fabs(r1[1]))
        SWAP_ROWS(r2, r1);
    if (0.0 == r1[1])
        return 0;
    /* eliminate second variable */
    m2 = r2[1] / r1[1];
    m3 = r3[1] / r1[1];
    r2[2] -= m2 * r1[2];
    r3[2] -= m3 * r1[2];
    r2[3] -= m2 * r1[3];
    r3[3] -= m3 * r1[3];
    s = r1[4];
    if (0.0 != s) {
        r2[4] -= m2 * s;
        r3[4] -= m3 * s;
    }
    s = r1[5];
    if (0.0 != s) {
        r2[5] -= m2 * s;
        r3[5] -= m3 * s;
    }
    s = r1[6];
    if (0.0 != s) {
        r2[6] -= m2 * s;
        r3[6] -= m3 * s;
    }
    s = r1[7];
    if (0.0 != s) {
        r2[7] -= m2 * s;
        r3[7] -= m3 * s;
    }
    /* choose pivot - or die */
    if (fabs(r3[2]) > fabs(r2[2]))
        SWAP_ROWS(r3, r2);
    if (0.0 == r2[2])
        return 0;
    /* eliminate third variable */
    m3 = r3[2] / r2[2];
    r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4],
            r3[5] -= m3 * r2[5], r3[6] -= m3 * r2[6], r3[7] -= m3 * r2[7];
    /* last check */
    if (0.0 == r3[3])
        return 0;
    s = 1.0 / r3[3];		/* now back substitute row 3 */
    r3[4] *= s;
    r3[5] *= s;
    r3[6] *= s;
    r3[7] *= s;
    m2 = r2[3];			/* now back substitute row 2 */
    s = 1.0 / r2[2];
    r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
            r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
    m1 = r1[3];
    r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1,
            r1[6] -= r3[6] * m1, r1[7] -= r3[7] * m1;
    m0 = r0[3];
    r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0,
            r0[6] -= r3[6] * m0, r0[7] -= r3[7] * m0;
    m1 = r1[2];			/* now back substitute row 1 */
    s = 1.0 / r1[1];
    r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
            r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
    m0 = r0[2];
    r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0,
            r0[6] -= r2[6] * m0, r0[7] -= r2[7] * m0;
    m0 = r0[1];			/* now back substitute row 0 */
    s = 1.0 / r0[0];
    r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
            r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);
    MAT(out, 0, 0) = r0[4];
    MAT(out, 0, 1) = r0[5], MAT(out, 0, 2) = r0[6];
    MAT(out, 0, 3) = r0[7], MAT(out, 1, 0) = r1[4];
    MAT(out, 1, 1) = r1[5], MAT(out, 1, 2) = r1[6];
    MAT(out, 1, 3) = r1[7], MAT(out, 2, 0) = r2[4];
    MAT(out, 2, 1) = r2[5], MAT(out, 2, 2) = r2[6];
    MAT(out, 2, 3) = r2[7], MAT(out, 3, 0) = r3[4];
    MAT(out, 3, 1) = r3[5], MAT(out, 3, 2) = r3[6];
    MAT(out, 3, 3) = r3[7];
    return 1;
}

Transform Transform::inverse(){
    Transform result;

    int test=inverter(this->matrix->matrix, result.matrix->matrix);


    if(test!=1){
        Garrote::Math::Matrix:: Matrix<double> cd(3,3);
        Garrote::Math::Matrix:: Matrix<double> tr(3,1);


        cd.matrix[0]=matrix->matrix[0];
        cd.matrix[1]=matrix->matrix[1];
        cd.matrix[2]=matrix->matrix[2];

        cd.matrix[3]=matrix->matrix[4];
        cd.matrix[4]=matrix->matrix[5];
        cd.matrix[5]=matrix->matrix[6];

        cd.matrix[6]=matrix->matrix[8];
        cd.matrix[7]=matrix->matrix[9];
        cd.matrix[8]=matrix->matrix[10];


        //            for(int i=0;i<3;i++){
        //                for(int f=0;f<3;f++){
        //                    cd.set(i,f,matrix->operator ()(i,f));
        //                }
        //            }
        cd= cd.transpose();

        //            tr.matrix[0]=matrix->operator ()(0,3);
        //            tr.matrix[1]=matrix->operator ()(1,3);
        //            tr.matrix[2]=matrix->operator ()(2,3);
        tr.matrix[0]=-1.0*matrix->matrix[12];// ()(0,3);
        tr.matrix[1]=-1.0*matrix->matrix[13];// ()(1,3);
        tr.matrix[2]=-1.0*matrix->matrix[14];// ()(2,3);


        //            tr=cd*(-1.0)*tr;
        tr=cd*tr;

        result.matrix[0]=cd.matrix[0];
        result.matrix[1]=cd.matrix[1];
        result.matrix[2]=cd.matrix[2];
        result.matrix[3]=0;

        result.matrix[4]=cd.matrix[3];
        result.matrix[5]=cd.matrix[4];
        result.matrix[6]=cd.matrix[5];
        result.matrix[7]=0;

        result.matrix[8]=cd.matrix[6];
        result.matrix[9]=cd.matrix[7];
        result.matrix[10]=cd.matrix[8];
        result.matrix[11]=0;

        result.matrix[12]=tr.matrix[0];
        result.matrix[13]=tr.matrix[1];
        result.matrix[14]=tr.matrix[2];
        result.matrix[15]=1.0;

        //            result.set(0,0,cd(0,0));
        //            result.set(1,0,cd(1,0));
        //            result.set(2,0,cd(2,0));

        //            result.set(0,1,cd(0,1));
        //            result.set(1,1,cd(1,1));
        //            result.set(2,1,cd(2,1));

        //            result.set(0,2,cd(0,2));
        //            result.set(1,2,cd(1,2));
        //            result.set(2,2,cd(2,2));

        //            result.set(0,3,tr(0,0));
        //            result.set(1,3,tr(1,0));
        //            result.set(2,3,tr(2,0));

        //            result.set(3,0,matrix->operator ()(3,0));
        //            result.set(3,1,matrix->operator ()(3,1));
        //            result.set(3,2,matrix->operator ()(3,2));
        //            result.set(3,3,1.0);

    }


    return result;
}

void Transform::onlyRotation(){

    matrix->matrix[12]=0;
    matrix->matrix[13]=0;
    matrix->matrix[14]=0;
}

Transform Transform::inverse4x4(){

    //TODO http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche23.html
    Transform result;


    Garrote::Math::Matrix:: Matrix<double> cd(3,3);
    Garrote::Math::Matrix:: Matrix<double> tr(3,1);


    cd.matrix[0]=matrix->matrix[0];
    cd.matrix[1]=matrix->matrix[1];
    cd.matrix[2]=matrix->matrix[2];

    cd.matrix[3]=matrix->matrix[4];
    cd.matrix[4]=matrix->matrix[5];
    cd.matrix[5]=matrix->matrix[6];

    cd.matrix[6]=matrix->matrix[8];
    cd.matrix[7]=matrix->matrix[9];
    cd.matrix[8]=matrix->matrix[10];



    cd= cd.transpose();



    tr.matrix[0]=-1.0*matrix->matrix[12];// ()(0,3);
    tr.matrix[1]=-1.0*matrix->matrix[13];// ()(1,3);
    tr.matrix[2]=-1.0*matrix->matrix[14];// ()(2,3);


    //            tr=cd*(-1.0)*tr;
    tr=cd*tr;

    result.matrix[0]=cd.matrix[0];
    result.matrix[1]=cd.matrix[1];
    result.matrix[2]=cd.matrix[2];
    result.matrix[3]=0;

    result.matrix[4]=cd.matrix[3];
    result.matrix[5]=cd.matrix[4];
    result.matrix[6]=cd.matrix[5];
    result.matrix[7]=0;

    result.matrix[8]=cd.matrix[6];
    result.matrix[9]=cd.matrix[7];
    result.matrix[10]=cd.matrix[8];
    result.matrix[11]=0;

    result.matrix[12]=tr.matrix[0];
    result.matrix[13]=tr.matrix[1];
    result.matrix[14]=tr.matrix[2];
    result.matrix[15]=1.0;




    return result;
}

void Transform::applyRotationXinOrigin(double xs){

    //        cs.matrix[0]=matrix->matrix[0];
    //        cs.matrix[1]=matrix->matrix[1];
    //        cs.matrix[2]=matrix->matrix[2];

    //        cs.matrix[3]=matrix->matrix[4];
    //        cs.matrix[4]=matrix->matrix[5];
    //        cs.matrix[5]=matrix->matrix[6];

    //        cs.matrix[6]=matrix->matrix[8];
    //        cs.matrix[7]=matrix->matrix[9];
    //        cs.matrix[8]=matrix->matrix[10];


    //        Matrix<double> rot(3,3);
    //        rot<<1,0,0,
    //             0,cos(xs),sin(xs),
    //             0,-sin(xs),cos(xs);
    //        Matrix<double> result=rot*getRotation();

    Matrix<double> result(3,3);

    double cs=cos(xs);
    double ss=sin(xs);

    result.matrix[0]=matrix->matrix[0];
    result.matrix[3]=matrix->matrix[4];
    result.matrix[6]=matrix->matrix[8];
    result.matrix[1]=cs*matrix->matrix[1]-ss*matrix->matrix[2];
    result.matrix[4]=cs*matrix->matrix[5]-ss*matrix->matrix[6];
    result.matrix[7]=cs*matrix->matrix[9]-ss*matrix->matrix[10];
    result.matrix[2]=ss*matrix->matrix[1]+cs*matrix->matrix[2];
    result.matrix[5]=ss*matrix->matrix[5]+cs*matrix->matrix[6];
    result.matrix[8]=ss*matrix->matrix[9]+cs*matrix->matrix[10];

    cloneRotation3x3(&result);

}

void Transform::applyRotationYinOrigin(double xs){

    //        Matrix<double> rot(3,3);
    //        rot<<cos(xs),  0,-sin(xs),
    //               0,      1,0,
    //               sin(xs),0,cos(xs);
    //        Matrix<double> result=rot*getRotation();

    Matrix<double> result(3,3);

    double cs=cos(xs);
    double ss=sin(xs);

    result.matrix[0]=cs*matrix->matrix[0]+ss*matrix->matrix[2];
    result.matrix[3]=cs*matrix->matrix[4]+ss*matrix->matrix[6];
    result.matrix[6]=cs*matrix->matrix[8]+ss*matrix->matrix[10];
    result.matrix[1]=matrix->matrix[1];
    result.matrix[4]=matrix->matrix[5];
    result.matrix[7]=matrix->matrix[9];
    result.matrix[2]=-ss*matrix->matrix[0]+cs*matrix->matrix[2];
    result.matrix[5]=-ss*matrix->matrix[4]+cs*matrix->matrix[6];
    result.matrix[8]=-ss*matrix->matrix[8]+cs*matrix->matrix[10];


    cloneRotation3x3(&result);
}

void Transform::applyRotationZinOrigin(double xs){


    //        Matrix<double> rot(3,3);
    //        rot<<cos(xs),sin(xs),0
    //        -sin(xs),cos(xs),0
    //         0, 0, 1;
    //        Matrix<double> result=rot*getRotation();
    Matrix<double> result(3,3);

    double cs=cos(xs);
    double ss=sin(xs);


    result.matrix[0]=cs*matrix->matrix[0]-ss*matrix->matrix[1];
    result.matrix[3]=cs*matrix->matrix[4]-ss*matrix->matrix[5];
    result.matrix[6]=cs*matrix->matrix[8]-ss*matrix->matrix[9];
    result.matrix[1]=+ss*matrix->matrix[0]+cs*matrix->matrix[1];
    result.matrix[4]=+ss*matrix->matrix[4]+cs*matrix->matrix[5];
    result.matrix[7]=+ss*matrix->matrix[8]+cs*matrix->matrix[9];
    result.matrix[2]=matrix->matrix[2];
    result.matrix[5]=matrix->matrix[6];
    result.matrix[8]=matrix->matrix[10];

    cloneRotation3x3(&result);

}

void Transform::take(Transform &value){

    //        Matrix<double> xt;
    //        //xt.reshape(4,4);
    //        xt=*matrix -(*value.matrix);
    //        (*matrix)=xt;

    //        (*matrix)(0,3)=(*matrix)(0,3)-(*value.matrix)(0,3);
    //        (*matrix)(1,3)=(*matrix)(1,3)-(*value.matrix)(1,3);

    //        (*matrix)(2,3)=(*matrix)(2,3)-(*value.matrix)(2,3);

    matrix->matrix[12]-=value.matrix->matrix[12];
    matrix->matrix[13]-=value.matrix->matrix[13];
    matrix->matrix[14]-=value.matrix->matrix[14];


}

void Transform::add(Transform &value){
    //        Matrix<double> xt;
    //        //xt.reshape(4,4);
    //        xt=*matrix +(*value.matrix);
    //        (*matrix)=xt;
    //        (*matrix)(0,3)=(*matrix)(0,3)+(*value.matrix)(0,3);
    //        (*matrix)(1,3)=(*matrix)(1,3)+(*value.matrix)(1,3);
    //        (*matrix)(2,3)=(*matrix)(2,3)+(*value.matrix)(2,3);

    matrix->matrix[12]+=value.matrix->matrix[12];
    matrix->matrix[13]+=value.matrix->matrix[13];
    matrix->matrix[14]+=value.matrix->matrix[14];
}

Transform &Transform::operator=(const Transform &value){
    delete matrix;
    matrix=value.matrix->clone();
    Parent=value.Parent;
    frame=value.frame;
    dispose=true;
    return *this;
}

bool Transform::operator==(const Transform &other){

    return (*matrix)==(*other.matrix);
}

bool Transform::operator!=(const Transform &other){
    return (*matrix)!=(*other.matrix);
}

Transform &Transform::operator*=(const Transform &Tf){

    (*matrix)=(*matrix)*(*Tf.matrix);
    return *this;
}

void Transform::applyRotationX(double a){


    //        Garrote::Math::Matrix::Matrix<double> xt(4,4);
    //        (*xt)<<1,0,0,0,0,cos(a),-sin(a),0,0,sin(a),cos(a),0,0,0,0,1;
    //        (*matrix)=(*xt)*(*matrix);


    //        delete xt;
    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);



    xt->matrix[0]= matrix->matrix[0];
    xt->matrix[4]=matrix->matrix[4];
    xt->matrix[8]=matrix->matrix[8];
    xt->matrix[12]=matrix->matrix[12];
    xt->matrix[1]=cs*matrix->matrix[1]-ss*matrix->matrix[2];
    xt->matrix[5]=cs*matrix->matrix[5]-ss*matrix->matrix[6];
    xt->matrix[9]=cs*matrix->matrix[9]-ss*matrix->matrix[10];
    xt->matrix[13]=cs*matrix->matrix[13]-ss*matrix->matrix[14];
    xt->matrix[2]=ss*matrix->matrix[1]+cs*matrix->matrix[2];
    xt->matrix[6]=ss*matrix->matrix[5]+cs*matrix->matrix[6];
    xt->matrix[10]=ss*matrix->matrix[9]+cs*matrix->matrix[10];
    xt->matrix[14]=ss*matrix->matrix[13]+cs*matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];

    delete matrix;

    matrix=xt;


}

void Transform::applyRotationY(double a){

    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<cos(a),0,sin(a),0,0,1,0,0,-sin(a),0,cos(a),0, 0,0,0,1;
    //        (*matrix)=(*xt)*(*matrix);
    //        delete xt;



    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);

    xt->matrix[0]= cs*matrix->matrix[0] + ss*matrix->matrix[2];
    xt->matrix[4]=cs*matrix->matrix[4] + ss*matrix->matrix[6];
    xt->matrix[8]=cs*matrix->matrix[8] + ss*matrix->matrix[10];
    xt->matrix[12]=cs*matrix->matrix[12] + ss*matrix->matrix[14];
    xt->matrix[1]=  matrix->matrix[1];
    xt->matrix[5]=  matrix->matrix[5];
    xt->matrix[9]= matrix->matrix[9];
    xt->matrix[13]= matrix->matrix[13];
    xt->matrix[2]=-ss*matrix->matrix[0] + cs*matrix->matrix[2];
    xt->matrix[6]=-ss*matrix->matrix[4] + cs*matrix->matrix[6];
    xt->matrix[10]=-ss*matrix->matrix[8] + cs*matrix->matrix[10];
    xt->matrix[14]=-ss*matrix->matrix[12] + cs*matrix->matrix[14];
    xt->matrix[3]= matrix->matrix[3];
    xt->matrix[7]= matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];



    delete matrix;

    matrix=xt;


}

void Transform::applyRotationZ(double a){

    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<cos(a),-sin(a),0,0,
    //        sin(a),cos(a),0,0
    //         0,0,1,0
    //         0,0,0,1;
    //        (*matrix)=(*xt)*(*matrix);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);

    xt->matrix[0]= cs*matrix->matrix[0]-ss*matrix->matrix[1];
    xt->matrix[4]=cs*matrix->matrix[4]-ss*matrix->matrix[5];
    xt->matrix[8]=cs*matrix->matrix[8] -ss*matrix->matrix[9];
    xt->matrix[12]=cs*matrix->matrix[12]-ss*matrix->matrix[13];
    xt->matrix[1]=ss*matrix->matrix[0]+cs*matrix->matrix[1];
    xt->matrix[5]=ss*matrix->matrix[4]+cs*matrix->matrix[5];
    xt->matrix[9]=ss*matrix->matrix[8]+cs*matrix->matrix[9];
    xt->matrix[13]=ss*matrix->matrix[12]+cs*matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2];
    xt->matrix[6]=matrix->matrix[6];
    xt->matrix[10]=matrix->matrix[10];
    xt->matrix[14]=matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];

    delete matrix;

    matrix=xt;
}

void Transform::unit(){

    //        Garrote::Math::Matrix::Matrix<double> xt(4,4);
    //        Garrote::Math::Matrix::Matrix<double> mt(4,4);
    //        double a=0.33;
    //        //      xt<<1,0,0,0,0,cos(a),-sin(a),0,0,sin(a),cos(a),0,0,0,0,1;
    //        //      mt.transcribeOperationX2(xt);
    //        //      xt<<cos(a),0,sin(a),0,0,1,0,0,-sin(a),0,cos(a),0,0,0,0,1;
    //        //      mt.transcribeOperationX2(xt);
    //        //      xt<<cos(a),-sin(a),0,0,sin(a),cos(a),0,0,0,0,1,0,0,0,0,1;
    //        //      mt.transcribeOperationX2(xt);
    //        xt<<1,0,0,20,0,1,0,20,0,0,1,20,0,0,0,1;
    //        mt.transcribeOperationX2(xt);

    Transform tf;

    //        tf.applyRotationX(M_PI);
    //        std::cout<<tf.toString()<<std::endl<<std::endl;
    //        tf.setIdentity();
    //        tf.applyRotationY(M_PI);
    //        std::cout<<tf.toString()<<std::endl<<std::endl;
    //        tf.setIdentity();
    //        tf.applyRotationZ(M_PI);
    //        std::cout<<tf.toString()<<std::endl<<std::endl;
    //        tf.setIdentity();


    tf.applyRotationXr(0.25);
    std::cout<<tf.toString()<<std::endl<<std::endl;
    tf.setIdentity();


    tf.applyRotationYr(0.25);
    std::cout<<tf.toString()<<std::endl<<std::endl;
    tf.setIdentity();


    tf.applyRotationZr(0.25);
    std::cout<<tf.toString()<<std::endl<<std::endl;



}

void Transform::applyRotationXr(double a){


    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<1,0,0,0,
    //               0,cos(a),-sin(a),0,
    //               0,sin(a),cos(a),0,
    //               0,0,0,1;
    //        (*matrix)=(*matrix)*(*xt);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);


    xt->matrix[0]=matrix->matrix[0];
    xt->matrix[4]=matrix->matrix[4]*cs+matrix->matrix[8]*ss;
    xt->matrix[8]=matrix->matrix[4]*(-ss)+matrix->matrix[8]*cs;
    xt->matrix[12]=matrix->matrix[12];
    xt->matrix[1]=matrix->matrix[1];
    xt->matrix[5]=matrix->matrix[5]*cs+matrix->matrix[9]*ss;
    xt->matrix[9]=matrix->matrix[5]*(-ss)+matrix->matrix[9]*cs;
    xt->matrix[13]=matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2];
    xt->matrix[6]=matrix->matrix[6]*cs+matrix->matrix[10]*ss;
    xt->matrix[10]=matrix->matrix[6]*(-ss)+matrix->matrix[10]*cs;
    xt->matrix[14]=matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7]*cs+matrix->matrix[11]*ss;
    xt->matrix[11]=matrix->matrix[7]*(-ss)+matrix->matrix[11]*cs;
    xt->matrix[15]=matrix->matrix[15];

    delete matrix;

    matrix=xt;

}

void Transform::applyRotationYr(double a){

    //        Garrote::Math::Matrix:: Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<cos(a),0,sin(a),0,
    //        0,1,0,0,
    //        -sin(a),0,cos(a),0,
    //         0,0,0,1;
    //        (*matrix)=(*matrix)*(*xt);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);

    xt->matrix[0]=matrix->matrix[0]*cs+matrix->matrix[8]*(-ss);
    xt->matrix[4]=matrix->matrix[4];
    xt->matrix[8]=matrix->matrix[0]*ss+matrix->matrix[8]*cs;
    xt->matrix[12]=matrix->matrix[12];
    xt->matrix[1]=matrix->matrix[1]*cs+matrix->matrix[9]*(-ss);
    xt->matrix[5]=matrix->matrix[5];
    xt->matrix[9]=matrix->matrix[1]*ss+matrix->matrix[9]*cs;
    xt->matrix[13]=matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2]*cs+matrix->matrix[10]*(-ss);
    xt->matrix[6]=matrix->matrix[6];
    xt->matrix[10]=matrix->matrix[2]*ss+matrix->matrix[10]*cs;
    xt->matrix[14]=matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3]*cs+matrix->matrix[11]*(-ss);
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[3]*ss+matrix->matrix[11]*cs;
    xt->matrix[15]=matrix->matrix[15];

    delete matrix;

    matrix=xt;

}

void Transform::applyRotationZr(double a){

    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<cos(a),-sin(a),0,0,
    //                sin(a),cos(a),0,0
    //                ,0,0,1,0
    //                ,0,0,0,1;
    //        (*matrix)=(*matrix)*(*xt);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);

    xt->matrix[0]=matrix->matrix[0]*cs+matrix->matrix[4]*ss;
    xt->matrix[4]=matrix->matrix[0]*(-ss)+matrix->matrix[4]*cs;
    xt->matrix[8]=matrix->matrix[8];
    xt->matrix[12]=matrix->matrix[12];
    xt->matrix[1]=matrix->matrix[1]*cs+matrix->matrix[5]*ss;
    xt->matrix[5]=matrix->matrix[1]*(-ss)+matrix->matrix[5]*cs;
    xt->matrix[9]=matrix->matrix[9];
    xt->matrix[13]=matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2]*cs+matrix->matrix[6]*ss;
    xt->matrix[6]=matrix->matrix[2]*(-ss)+matrix->matrix[6]*cs;
    xt->matrix[10]=matrix->matrix[10];
    xt->matrix[14]=matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3]*cs+matrix->matrix[7]*ss;
    xt->matrix[7]=matrix->matrix[3]*(-ss)+matrix->matrix[7]*cs;
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];

    delete matrix;

    matrix=xt;
}

void Transform::applyTranslation(Point data){

    //        matrix->operator ()(0,3)=matrix->operator ()(0,3)+data.X();
    //        matrix->operator ()(1,3)=matrix->operator ()(1,3)+data.Y();
    //        matrix->operator ()(2,3)=matrix->operator ()(2,3)+data.Z();
    matrix->matrix[12]+=data.X();
    matrix->matrix[13]+=data.Y();
    matrix->matrix[14]+=data.Z();
    //        Matrix<double>* xt=new Matrix<double>(4,4);
    //        (*xt)<<1,0,0,data.X(),0,1,0,data.Y(),0,0,1,data.Z(),0,0,0,1;

    //        (*matrix)=(*xt)*(*matrix);
    //        delete xt;
}

void Transform::applyTranslationr(Point data){

    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<1,0,0,data.X(),0,1,0,data.Y(),0,0,1,data.Z(),0,0,0,1;
    //        (*matrix)=(*matrix)*(*xt);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);


    xt->matrix[0]=matrix->matrix[0];
    xt->matrix[4]=matrix->matrix[4];
    xt->matrix[8]=matrix->matrix[8];
    xt->matrix[12]=matrix->matrix[0]*data.X()+matrix->matrix[4]*data.Y()+matrix->matrix[8]*data.Z()+matrix->matrix[12];
    xt->matrix[1]=matrix->matrix[1];
    xt->matrix[5]=matrix->matrix[5];
    xt->matrix[9]=matrix->matrix[9];
    xt->matrix[13]=matrix->matrix[1]*data.X()+matrix->matrix[5]*data.Y()+matrix->matrix[9]*data.Z()+matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2];
    xt->matrix[6]=matrix->matrix[6];
    xt->matrix[10]=matrix->matrix[10];
    xt->matrix[14]=matrix->matrix[2]*data.X()+matrix->matrix[6]*data.Y()+matrix->matrix[10]*data.Z()+matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[3]*data.X()+matrix->matrix[7]*data.Y()+matrix->matrix[11]*data.Z()+matrix->matrix[15];


    delete matrix;

    matrix=xt;


}

void Transform::applyTranslation(double x, double y, double z){

    //        matrix->operator ()(0,3)=matrix->operator ()(0,3)+x;
    //        matrix->operator ()(1,3)=matrix->operator ()(1,3)+y;
    //        matrix->operator ()(2,3)=matrix->operator ()(2,3)+z;

    matrix->matrix[12]+=x;
    matrix->matrix[13]+=y;
    matrix->matrix[14]+=z;


    //        Matrix<double>* xt=new Matrix<double>(4,4);
    //        (*xt)<<1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1;

    //        (*matrix)=(*xt)*(*matrix);
    //        delete xt;

}

void Transform::applyTranslationr(double x, double y, double z){

    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1;
    //        (*matrix)=(*matrix)*(*xt);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    xt->matrix[0]=matrix->matrix[0];
    xt->matrix[4]=matrix->matrix[4];
    xt->matrix[8]=matrix->matrix[8];
    xt->matrix[12]=matrix->matrix[0]*x+matrix->matrix[4]*y+matrix->matrix[8]*z+matrix->matrix[12];
    xt->matrix[1]=matrix->matrix[1];
    xt->matrix[5]=matrix->matrix[5];
    xt->matrix[9]=matrix->matrix[9];
    xt->matrix[13]=matrix->matrix[1]*x+matrix->matrix[5]*y+matrix->matrix[9]*z+matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2];
    xt->matrix[6]=matrix->matrix[6];
    xt->matrix[10]=matrix->matrix[10];
    xt->matrix[14]=matrix->matrix[2]*x+matrix->matrix[6]*y+matrix->matrix[10]*z+matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[3]*x+matrix->matrix[7]*y+matrix->matrix[11]*z+matrix->matrix[15];


    delete matrix;

    matrix=xt;

}

Transform Transform::rotateX(double a){


    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<1,0,0,0,0,cos(a),-sin(a),0,0,sin(a),cos(a),0,0,0,0,1;
    //        Garrote::Math::Matrix::Matrix<double>* result;
    //        result=matrix->clone();
    //        (*result)=(*xt)*(*result);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);


    xt->matrix[0]= matrix->matrix[0];
    xt->matrix[4]=matrix->matrix[4];
    xt->matrix[8]=matrix->matrix[8];
    xt->matrix[12]=matrix->matrix[12];
    xt->matrix[1]=cs*matrix->matrix[1]-ss*matrix->matrix[2];
    xt->matrix[5]=cs*matrix->matrix[5]-ss*matrix->matrix[6];
    xt->matrix[9]=cs*matrix->matrix[9]-ss*matrix->matrix[10];
    xt->matrix[13]=cs*matrix->matrix[13]-ss*matrix->matrix[14];
    xt->matrix[2]=ss*matrix->matrix[1]+cs*matrix->matrix[2];
    xt->matrix[6]=ss*matrix->matrix[5]+cs*matrix->matrix[6];
    xt->matrix[10]=ss*matrix->matrix[9]+cs*matrix->matrix[10];
    xt->matrix[14]=ss*matrix->matrix[13]+cs*matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];

    return Transform(xt,true);
}

Transform Transform::rotateY(double a){

    //        Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //        (*xt)<<cos(a),0,sin(a),0,0,1,0,0,-sin(a),0,cos(a),0,0,0,0,1;
    //        Garrote::Math::Matrix::Matrix<double>* result;
    //        result=matrix->clone();
    //        (*result)=(*xt)*(*result);
    //        delete xt;

    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);

    xt->matrix[0]= cs*matrix->matrix[0]+ ss*matrix->matrix[2];
    xt->matrix[4]=cs*matrix->matrix[4]+ ss*matrix->matrix[6];
    xt->matrix[8]=cs*matrix->matrix[8] + ss*matrix->matrix[10];
    xt->matrix[12]=cs*matrix->matrix[12] +ss*matrix->matrix[14];
    xt->matrix[1]=  matrix->matrix[1];
    xt->matrix[5]=  matrix->matrix[5];
    xt->matrix[9]= matrix->matrix[9];
    xt->matrix[13]= matrix->matrix[13];
    xt->matrix[2]=-ss*matrix->matrix[0] +cs*matrix->matrix[2];
    xt->matrix[6]=-ss*matrix->matrix[4] +cs*matrix->matrix[6];
    xt->matrix[10]=-ss*matrix->matrix[8] +cs*matrix->matrix[10];
    xt->matrix[14]=-ss*matrix->matrix[12] +cs*matrix->matrix[14];
    xt->matrix[3]= matrix->matrix[3];
    xt->matrix[7]= matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];

    return Transform(xt,true);

}

Transform Transform::rotateZ(double a){

    //        Garrote::Math::Matrix:: Matrix<double>* xt=new Garrote::Math::Matrix:: Matrix<double>(4,4);
    //        (*xt)<<cos(a),-sin(a),0,0,sin(a),cos(a),0,0,0,0,1,0,0,0,0,1;
    //        Garrote::Math::Matrix::Matrix<double>* result;
    //        result=matrix->clone();
    //        (*result)=(*xt)*(*result);
    //        delete xt;


    Garrote::Math::Matrix::Matrix<double> *xt=new Garrote::Math::Matrix::Matrix<double>(4,4);

    double cs=cos(a);
    double ss=sin(a);

    xt->matrix[0]= cs*matrix->matrix[0]-ss*matrix->matrix[1];
    xt->matrix[4]=cs*matrix->matrix[4]-ss*matrix->matrix[5];
    xt->matrix[8]=cs*matrix->matrix[8] -ss*matrix->matrix[9];
    xt->matrix[12]=cs*matrix->matrix[12]-ss*matrix->matrix[13];
    xt->matrix[1]=ss*matrix->matrix[0]+cs*matrix->matrix[1];
    xt->matrix[5]=ss*matrix->matrix[4]+cs*matrix->matrix[5];
    xt->matrix[9]=ss*matrix->matrix[8]+cs*matrix->matrix[9];
    xt->matrix[13]=ss*matrix->matrix[12]+cs*matrix->matrix[13];
    xt->matrix[2]=matrix->matrix[2];
    xt->matrix[6]=matrix->matrix[6];
    xt->matrix[10]=matrix->matrix[10];
    xt->matrix[14]=matrix->matrix[14];
    xt->matrix[3]=matrix->matrix[3];
    xt->matrix[7]=matrix->matrix[7];
    xt->matrix[11]=matrix->matrix[11];
    xt->matrix[15]=matrix->matrix[15];

    return Transform(xt,true);

}

Transform Transform::translate(Point data){


    Garrote::Math::Matrix:: Matrix<double>* result;
    result=matrix->clone();
    result->matrix[12]+=data.X();
    result->matrix[13]+=data.Y();
    result->matrix[14]+=data.Z();
    //        result->operator ()(0,3)=result->operator ()(0,3)+data.X();
    //        result->operator ()(1,3)=result->operator ()(1,3)+data.Y();
    //        result->operator ()(2,3)=result->operator ()(2,3)+data.Z();
    return Transform(result,true);
}

Transform Transform::translate(double x, double y, double z){
    Transform tf;
    tf.setTranslation(x,y,z);

    return this->operator *(tf);//Transform(result,true);
}

void Transform::setTranslation(double x, double y, double z){


    matrix->matrix[12]=x;
    matrix->matrix[13]=y;
    matrix->matrix[14]=z;

    //        result->operator ()(1,3)=result->operator ()(1,3)+y;
    //        result->operator ()(2,3)=result->operator ()(2,3)+z;
}

void Transform::expand(double val){


    matrix->multiply(val);
    //                Garrote::Math::Matrix::Matrix<double>* xt=new Garrote::Math::Matrix::Matrix<double>(4,4);
    //                (*xt)<<val,0,0,0,0,val,0,0,0,0,val,0,0,0,0,1;

    //                (*matrix)=(*matrix)*(*xt);
    //                delete xt;
}

std::string Transform::toString(){

    return matrix->toStringLine();

}

std::string Transform::toStringTranslation(){
    return $("{} {} {}").format(matrix->matrix[12],matrix->matrix[13],matrix->matrix[14]);
}

std::string Transform::toStringQuaternion(){

    auto p=Transform::toPose(this);

    return $("{} {} {} {}").format(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
}

Transform Transform::transformationFromBase(){

    Transform tf=*this;
    Transform *pr=this->Parent;

    while(pr!=NULL){

        tf=pr->operator*(tf);
        pr=pr->Parent;
    }
    return tf;
}

Transform::~Transform(){

    if(dispose){
        delete matrix;
    }
}

Transform Transform::operator*(const Transform &value){


    //        transcribeOperationX(value);
    Transform result;


    //result.matrix=matrix*value.matrix;
    // (*result.matrix)=matrix->operator *(*value.matrix);

    //        for(unsigned int i=0;i<4;i++){
    //            for(unsigned int f=0;f<4;f++){

    //                double val=0.0;

    //                for( unsigned int k=0;k<4;k++){
    //                    val=val+matrix->unprotected_get(i,k)*value.matrix->unprotected_get(k,f);
    //                }
    //                result.matrix->unprotected_set(i,f,val);
    //            }
    //        }


    result.matrix->matrix[0]= matrix->matrix[0]*value.matrix->matrix[0]+ matrix->matrix[4]*value.matrix->matrix[1]+ matrix->matrix[8]*value.matrix->matrix[2]+ matrix->matrix[12]*value.matrix->matrix[3];
    result.matrix->matrix[4]=matrix->matrix[0]*value.matrix->matrix[4]+matrix->matrix[4]*value.matrix->matrix[5]+matrix->matrix[8]*value.matrix->matrix[6]+matrix->matrix[12]*value.matrix->matrix[7];
    result.matrix->matrix[8]=matrix->matrix[0]*value.matrix->matrix[8] +matrix->matrix[4]*value.matrix->matrix[9]+matrix->matrix[8]*value.matrix->matrix[10]+matrix->matrix[12]*value.matrix->matrix[11];
    result.matrix->matrix[12]=matrix->matrix[0]*value.matrix->matrix[12]+matrix->matrix[4]*value.matrix->matrix[13]+matrix->matrix[8]*value.matrix->matrix[14] +matrix->matrix[12]*value.matrix->matrix[15];
    result.matrix->matrix[1]=matrix->matrix[1]*value.matrix->matrix[0]+ matrix->matrix[5]*value.matrix->matrix[1]+ matrix->matrix[9]*value.matrix->matrix[2]+ matrix->matrix[13]*value.matrix->matrix[3];
    result.matrix->matrix[5]= matrix->matrix[1]*value.matrix->matrix[4]+matrix->matrix[5]*value.matrix->matrix[5]+matrix->matrix[9]*value.matrix->matrix[6]+matrix->matrix[13]*value.matrix->matrix[7];
    result.matrix->matrix[9]=matrix->matrix[1]*value.matrix->matrix[8]+matrix->matrix[5]*value.matrix->matrix[9]+matrix->matrix[9]*value.matrix->matrix[10]+matrix->matrix[13]*value.matrix->matrix[11];
    result.matrix->matrix[13]=matrix->matrix[1]*value.matrix->matrix[12]+matrix->matrix[5]*value.matrix->matrix[13]+matrix->matrix[9]*value.matrix->matrix[14]+matrix->matrix[13]*value.matrix->matrix[15];
    result.matrix->matrix[2]=matrix->matrix[2]*value.matrix->matrix[0]+matrix->matrix[6]*value.matrix->matrix[1]+matrix->matrix[10]*value.matrix->matrix[2]+matrix->matrix[14]*value.matrix->matrix[3];
    result.matrix->matrix[6]=matrix->matrix[2]*value.matrix->matrix[4]+matrix->matrix[6]*value.matrix->matrix[5]+matrix->matrix[10]*value.matrix->matrix[6]+matrix->matrix[14]*value.matrix->matrix[7];
    result.matrix->matrix[10]=matrix->matrix[2]*value.matrix->matrix[8]+matrix->matrix[6]*value.matrix->matrix[9]+matrix->matrix[10]*value.matrix->matrix[10]+matrix->matrix[14]*value.matrix->matrix[11];
    result.matrix->matrix[14]=matrix->matrix[2]*value.matrix->matrix[12]+matrix->matrix[6]*value.matrix->matrix[13]+matrix->matrix[10]*value.matrix->matrix[14]+matrix->matrix[14]*value.matrix->matrix[15];
    result.matrix->matrix[3]=matrix->matrix[3]*value.matrix->matrix[0]+matrix->matrix[7]*value.matrix->matrix[1]+matrix->matrix[11]*value.matrix->matrix[2]+matrix->matrix[15]*value.matrix->matrix[3];
    result.matrix->matrix[7]=matrix->matrix[3]*value.matrix->matrix[4]+matrix->matrix[7]*value.matrix->matrix[5]+matrix->matrix[11]*value.matrix->matrix[6]+matrix->matrix[15]*value.matrix->matrix[7];
    result.matrix->matrix[11]=matrix->matrix[3]*value.matrix->matrix[8]+matrix->matrix[7]*value.matrix->matrix[9]+matrix->matrix[11]*value.matrix->matrix[10]+matrix->matrix[15]*value.matrix->matrix[11];
    result.matrix->matrix[15]=matrix->matrix[3]*value.matrix->matrix[12]+matrix->matrix[7]*value.matrix->matrix[13]+matrix->matrix[11]*value.matrix->matrix[14]+matrix->matrix[15]*value.matrix->matrix[15];



    return result;


}

Transform Transform::operator*(const double &value){
    Transform result;

    // (*result.matrix)=matrix->operator *(value);
    result.matrix->matrix[0]= matrix->matrix[0]*value;
    result.matrix->matrix[1]= matrix->matrix[1]*value;
    result.matrix->matrix[2]= matrix->matrix[2]*value;
    result.matrix->matrix[3]= matrix->matrix[3]*value;

    result.matrix->matrix[4]= matrix->matrix[4]*value;
    result.matrix->matrix[5]= matrix->matrix[5]*value;
    result.matrix->matrix[6]= matrix->matrix[6]*value;
    result.matrix->matrix[7]= matrix->matrix[7]*value;

    result.matrix->matrix[8]= matrix->matrix[8]*value;
    result.matrix->matrix[9]= matrix->matrix[9]*value;
    result.matrix->matrix[10]= matrix->matrix[10]*value;
    result.matrix->matrix[11]= matrix->matrix[11]*value;

    result.matrix->matrix[12]= matrix->matrix[12]*value;
    result.matrix->matrix[13]= matrix->matrix[13]*value;
    result.matrix->matrix[14]= matrix->matrix[14]*value;
    result.matrix->matrix[15]= matrix->matrix[15]*value;


    return result;
}

std::vector<Point> Transform::operator*(const std::vector<Point> &values){

    std::vector<Point> out;
    if(values.size()==0){
        return  out;
    }
    out.reserve(values.size());

    Point output;
    Point value;


    for(unsigned int i=0;i<values.size();i++){

        value=values[i];
        output.x=matrix->matrix[0]*value.x + matrix->matrix[4]*value.y + matrix->matrix[8]*value.z + matrix->matrix[12];
        output.y=matrix->matrix[1]*value.x + matrix->matrix[5]*value.y + matrix->matrix[9]*value.z + matrix->matrix[13];
        output.z=matrix->matrix[2]*value.x + matrix->matrix[6]*value.y + matrix->matrix[10]*value.z + matrix->matrix[14];

        out.push_back(output);
    }
    return out;
}

void Transform::applyScale(double sx, double sy, double sz){


    //https://en.wikipedia.org/wiki/Scaling_(geometry)
    Transform tf;
    tf.set(0,0,sx);
    tf.set(1,1,sy);
    tf.set(2,2,sz);
    tf.set(3,3,1);

    this->operator*=(tf);

}
