#ifndef POINTBASEDLOOPCLOSURE_H
#define POINTBASEDLOOPCLOSURE_H

#include <vector>
#include <map>
#include <iostream>

//Location temp( obl.objectClasses,detections, lastpose);
//temp.frames.push_back( objectdetector.id-1);

//obl.compute(temp, lastpose);


template <typename T>
class Histogram{


    std::vector<std::vector<T> > buffer;

    double minValue;

public:
    double step;
    std::vector<int> count;
    int datapoints;

    Histogram(double min_,double step_){

        minValue=min_;
        step=step_;
        datapoints=0;
    }


    void reset(){
        count.clear();
        buffer.clear();
        datapoints=0;
    }



    void remove(double value){

        int i=std::ceil((value-minValue)/step);

        if(i<0){
            i=0;
        }

        if(i<count.size()){
            if(count[i]>0){
                count[i]=count[i]-1;
                datapoints--;

            }
        }
    }


    void add(double value){

        datapoints++;
        int i=std::ceil((value-minValue)/step);

        if(i<0){
            i=0;
        }

        if(i<count.size()){
            count[i]=count[i]+1;
        }else{

            while(count.size()<=i){
                count.push_back(0);
                buffer.push_back(std::vector<T>());
            }

            count[i]=count[i]+1;

        }

    }

    double prob(double dist){

        int i=std::ceil((dist-minValue)/step);

        if(i<0){
            i=0;
        }

        if(datapoints==0){

            return 0;

        }
        if(i<count.size()){
            return (double)count[i]/(double)datapoints;
        }else{
            return 0;
        }
    }





    void add(double value,T &data){

        datapoints++;
        int i=std::ceil((value-minValue)/step);

        if(i<0){
            i=0;
        }

        if(i<count.size()){
            count[i]=count[i]+1;
            buffer[i].push_back(data);
        }else{


            while(count.size()<=i){
                count.push_back(0);
                buffer.push_back(std::vector<T>());
            }
            count[i]=count[i]+1;
            buffer[i].push_back(data);
        }

    }




    int getDatapoints() const
    {
        return datapoints;
    }


    std::pair<double,unsigned int> MAX(){

        unsigned  int maxid=0;
        unsigned  int countv=0;
        for(unsigned int i=0;i<count.size();i++){

            if(count[i]>=countv){
                maxid=i;
                countv=count[i];
            }
        }

        return std::pair<double,unsigned int>(maxid*step+ minValue,countv);
        //+step/2.0
    }
    std::pair<double,double> MAX2(int cut){

        unsigned  int maxid=0;
        unsigned  int countv=0;
        unsigned  int maxid2=0;
        unsigned  int countv2=0;

        for(unsigned int i=0;i<count.size();i++){

            if(count[i]>=countv){
                maxid2=maxid;
                countv2=countv;
                maxid=i;
                countv=count[i];
            }else if(count[i]>=countv2){

                maxid2=i;
                countv2=count[i];
            }
        }

        if(countv2<cut){
            return std::pair<double,unsigned int>(maxid*step+ minValue,maxid*step+ minValue);

        }else {

            return std::pair<double,unsigned int>(maxid*step+ minValue,maxid2*step+ minValue);

        }
        //+step/2.0
    }

    std::pair<unsigned int,unsigned int> max(){

        unsigned  int maxid=0;
        unsigned  int countv=0;
        for(unsigned int i=0;i<count.size();i++){

            if(count[i]>=countv){
                maxid=i;
                countv=count[i];
            }
        }

        return std::pair<unsigned int,unsigned int>(maxid,countv);
    }



    std::vector<T> getBinValues(unsigned int i){
        return buffer[i];
    }



    std::vector<double> normalized(){
        std::vector<double> out;


        for(unsigned int i=0;i<count.size();i++){
            out.push_back((double)count[i]/(double)datapoints);
        }

        return out;
    }


    std::vector<double> middlelabels(){

        std::vector<double> out;

        for(unsigned int i=0;i<count.size();i++){
            out.push_back(minValue+((double)i)*step+step/2.0);
        }


        return out;

    }


    std::vector<double> labels(){
        std::vector<double> out;

        for(unsigned int i=0;i<count.size();i++){
            out.push_back(minValue+((double)i)*step);
        }

        return out;
    }








};

class PointDistanceHistogram{
public:
    Histogram<double> distance;

    PointDistanceHistogram():distance(0,1){

        distance.count.resize(100,0);
    }


    void reset(){
        distance.reset();
    }

    void take(double d,bool t){
        if(t){
            distance.add(0.0);


        }else{
            distance.add(d); //non-Gaussian model

        }
    }

    void update(double dist ){
        distance.add(dist); //non-Gaussian model
    }

    void update(PointDistanceHistogram &dist){

        int size=std::max(dist.distance.count.size(),distance.count.size());


        for(unsigned int i=0;i<size;i++){

            if(i>=dist.distance.count.size()){
                distance.count[i]+=(round(-distance.count[i])/4.0); //decay;
            }else if(i>=distance.count.size()){
                int d=dist.distance.count[i] ;
                if(d==0){
                    continue;
                }else{
                    distance.add(((double)i+0.5)*distance.step);
                    distance.count[i]=0;
                    distance.count[i]+=(round(dist.distance.count[i] )/4.0);
                }

            }else{
                distance.count[i]+= (round(dist.distance.count[i]-distance.count[i])/4.0);
            }
        }

        recompute();
    }

    void recompute(){
        int ct=0;
        for(unsigned int i=0;i<distance.count.size();i++){
            if(distance.count[i]<0){
                distance.count[i]=0;
            }
            ct+=distance.count[i];
        }
        distance.datapoints=ct;
    }

    double get(double dist ){
        return distance.prob(dist); //smoothing *0.99+0.01
    }

    double get(double dist,int idx ){

        double p=0;
        for( int i=-idx;i<=idx;i++){
            p+=distance.prob(dist+i*distance.step);
        }

        return p; //smoothing *0.99+0.01
    }

    double get(double d, double sig,int idx=5){

        if(idx==0){
            return get(d);
        }
        normal_distribution<double,false> g(d,sig,1.0);
        double p=0;

        for( int i=-idx;i<=idx;i++){
            p+=g(d+(double)i*distance.step)*distance.prob(d+(double)i*distance.step);
        }


        return p; //smoothing *0.99+0.01
    }

    double getmax(double d, double sig,int idx=5){

        if(idx==0){
            return get(d);
        }
        normal_distribution<double,false> g(d,sig,1.0);
        double p=0;

        double mx=0;
        for( int i=-idx;i<=idx;i++){
            double l=distance.prob(d+(double)i*distance.step);
            if(l>mx){
                mx=l;
                p=std::max(g(d+(double)i*distance.step),p);
            }
        }


        return p; //smoothing *0.99+0.01
    }
    void update(double d, double sig,int idx,double factor=5){

        if(idx==0){
            update(d);
            return;
        }
        normal_distribution<double,false> g(d,sig,1.0);

        for( int i=-idx;i<=idx;i++){
            int fx=std::round(factor*g(d+(double)i*distance.step));
            for( int gl=0;gl<fx;gl++){
                distance.add(d+(double)i*distance.step); //non-Gaussian model
            }
        }
    }


    double DIFF2(PointDistanceHistogram &h){

        double minx=0;

        double ctr=0.0;
        double x=0.0;

        for(unsigned int i=0;i<h.distance.count.size();i++){
            int g1=(h.distance.count[i]);
            int g2=0;
            if(i>=distance.count.size()){
            }else{
                g2=(distance.count[i]);
            }

            x=fabs(g1-g2);
            minx+=x;
            if(g1!=0||g2!=0){
                ctr+=(double)(std::max(g1,g2));
            }
        }
        if(ctr<0.001){
            return 0;
        }
        return minx/ctr;//(double)std::max(h.distance.datapoints,distance.datapoints);

    }

    double DIFF(PointDistanceHistogram &h){

        double minx=0;

        double ctr=0.0;

        for(unsigned int i=0;i<h.distance.count.size();i++){
            int g1=(h.distance.count[i]);
            int g2=0;
            if(i>=distance.count.size()){
            }else{
                g2=(distance.count[i]);
            }

            minx+=(fabs(g1-g2));

            if(g1!=0||g2!=0){
                ctr+=(double)(std::max(g1,g2));
            }
        }

        if(ctr<0.001){
            return 1;
        }

        return 1.0-minx/ctr;//(double)std::max(h.distance.datapoints,distance.datapoints);

    }

    double IOU(PointDistanceHistogram &h){

        double minx=0;
        double maxx=0;

        if(distance.count.size()==0 && h.distance.count.size()==0){
            return (1.0);
        }else if(distance.count.size()>h.distance.count.size()){

            for(unsigned int i=0;i<distance.count.size();i++){
                double g1=(((double)distance.count[i]/(double)distance.datapoints));
                double g2=0.0;
                if(i>=h.distance.count.size()){
                }else{
                    g2=(((double)h.distance.count[i]/(double)h.distance.datapoints));
                }

                minx+=std::min(g1,g2);
                maxx+=std::max(g1,g2);
            }
            //                    logP+=log(local*0.8+0.01);
            if(maxx<0.000001){
                return 1.0;
            }else{
                return (((minx/maxx))*0.99+0.01);

            }
        }else{
            for(unsigned int i=0;i<h.distance.count.size();i++){
                double g1=(((double)h.distance.count[i]/(double)h.distance.datapoints));
                double g2=0.0;

                if(i>=distance.count.size()){
                    //                            logP+= log(0.1);
                }else{
                    g2=(((double)distance.count[i]/(double)distance.datapoints));
                }

                minx+=std::min(g1,g2);
                maxx+=std::max(g1,g2);
            }



            if(maxx<0.000001){
                return (1.0);


            }else{
                return(((minx/maxx))*0.99+0.01);

            }

        }
        return 0.01;

    }

    double L0(PointDistanceHistogram &h){

        int minx=0;

        if(distance.count.size()==0 && h.distance.count.size()==0){
            return (1.0);
        }else if(distance.count.size()>h.distance.count.size()){

            for(unsigned int i=0;i<distance.count.size();i++){
                int g1=(distance.count[i]);
                int g2=0;
                if(i>=h.distance.count.size()){
                }else{
                    g2=h.distance.count[i];
                }

                minx+=(g1==g2?1:0);
            }
            return (((minx/(double)distance.count.size()))*0.99+0.01);

        }else{
            for(unsigned int i=0;i<h.distance.count.size();i++){
                int g1=(h.distance.count[i]);
                int g2=0;

                if(i>=distance.count.size()){
                    //                            logP+= log(0.1);
                }else{
                    g2=(distance.count[i]);
                }

                minx+=(g1==g2?1:0);

            }



            return(((minx/(double)h.distance.count.size()))*0.99+0.01);


        }
        return 0.01;

    }


    double L1(PointDistanceHistogram &h){

        double vl=0;

        if(distance.count.size()==0 && h.distance.count.size()==0){
            return (0.99);
        }else if(distance.count.size()>h.distance.count.size()){

            for(unsigned int i=0;i<distance.count.size();i++){
                double g1=(((double)distance.count[i]/(double)distance.datapoints));
                double g2=0.0;
                if(i>=h.distance.count.size()){
                }else{
                    g2=(((double)h.distance.count[i]/(double)h.distance.datapoints));
                }

                vl+=std::fabs(g1-g2);
            }

            return ((1.0-vl))*0.99+0.01;

        }else{
            for(unsigned int i=0;i<h.distance.count.size();i++){
                double g1=(((double)h.distance.count[i]/(double)h.distance.datapoints));
                double g2=0.0;

                if(i>=distance.count.size()){

                }else{
                    g2=(((double)distance.count[i]/(double)distance.datapoints));
                }

                vl+=std::fabs(g1-g2);

            }
            return ((1.0-vl))*0.99+0.01;


        }
        return 0.01;


    }

    double L2(PointDistanceHistogram &h){

        double vl=0;

        if(distance.count.size()==0 && h.distance.count.size()==0){
            return (0.99);
        }else if(distance.count.size()>h.distance.count.size()){

            for(unsigned int i=0;i<distance.count.size();i++){
                double g1=(((double)distance.count[i]/(double)distance.datapoints));
                double g2=0.0;
                if(i>=h.distance.count.size()){
                }else{
                    g2=(((double)h.distance.count[i]/(double)h.distance.datapoints));
                }

                vl+=(g1-g2)*(g1-g2);
            }

            return ((1.0-sqrt(vl))*0.99+0.01);

        }else{
            for(unsigned int i=0;i<h.distance.count.size();i++){
                double g1=(((double)h.distance.count[i]/(double)h.distance.datapoints));
                double g2=0.0;

                if(i>=distance.count.size()){

                }else{
                    g2=(((double)distance.count[i]/(double)distance.datapoints));
                }

                vl+=(g1-g2)*(g1-g2);

            }
            return ((1.0-sqrt(vl))*0.99+0.01);


        }
        return 0.01;


    }



    double BFF(PointDistanceHistogram &h){

        double vl=0;
        double ct=0;

        if(distance.count.size()==0 && h.distance.count.size()==0){
            return (0.99);
        }else if(distance.count.size()>h.distance.count.size()){

            for(unsigned int i=0;i<distance.count.size();i++){
                //                double g1=(((double)distance.count[i]/(double)distance.datapoints));
                double g2=0.0;
                if(i>=h.distance.count.size()){
                }else{
                    g2=(((double)h.distance.count[i]));///(double)h.distance.datapoints;
                }

                if(g2>0.0001){
                    vl+=getmax(((double)i+0.5)*distance.step, 0.5,2)*g2;
                }

            }

            return (( (vl))*0.99+0.01);

        }else{
            for(unsigned int i=0;i<h.distance.count.size();i++){
                double g1=(double)h.distance.count[i];///(double)h.distance.datapoints;
                double g2=0.0;


                if(i>=distance.count.size()){

                }else{
                    //                    g2=(((double)distance.count[i]/(double)distance.datapoints));
                }

                if(g1>0.0001){
                    vl+=getmax(((double)i+0.5)*distance.step, 0.5,2)*g1;
                }
            }
            return ( (vl))*0.99+0.01;


        }
        return 0.01;
    }


};

class Location{

public:
    bool  active=false;
    geometry_msgs::Pose pose; //debug only
    PointDistanceHistogram histogram;

    std::vector<std::pair<int,std::vector<double>>> neighbours;
    std::vector< int > parentOf ;

    std::vector<int> frames;

    std::vector<Location> submodels;


    double sublikelihood=0;
    double subModellikelihood=0;

    double likelihood=0;
    double likelihood2=0;
    double selflikelihood=0;
    double prior=0.75;
    double prob=1.0;
    double local=0;

    double base=0.5;



    double getLikelihood(Location &actual){
        double mx=computeLikelihood(actual);
        for(unsigned int i=0;i<submodels.size();i++){
            mx=std::max(submodels[i].computeLikelihood(actual),mx);
        }
        sublikelihood=mx;
        return mx;
    }








    Location(){
    }


    void reset(){
        histogram.reset();

    }

    template <typename PointDATA>
    Location(std::vector<PointDATA> &data,geometry_msgs::Pose &pose_){

        histogram=PointDistanceHistogram();

        pose=pose_;

        for(int i=0;i<data.size();i++){
            for(int j=0;j<data.size();j++){

                double d=PoseTools::distance(data[i],data[j]);
                if(d>=0.0001){  //avoid own distance
                    histogram.update(d,0.01,0);
                }
            }
        }




    }





    double computeLikelihood(Location &loc){
        //https://stats.stackexchange.com/questions/7400/how-to-assess-the-similarity-of-two-histograms
        likelihood=0;
        double d=histogram.DIFF(loc.histogram);
        likelihood+=( d);
        return likelihood;
    }





    double clampUP(double v,double l){
        if(v>l){
            v=l;
        }
        return v;
    }

    double clampDOWN(double v,double l){
        if(v<l){
            v=l;
        }
        return v;
    }

    double updateProb(double val){
        return clampUP(val+0.123,0.99);
    }

    double updateProbDecrease(double val){
        return clampDOWN(val-0.123,0.01);
    }


    void update2(Location &actual){

        for(unsigned int i=0;i<actual.frames.size();i++){
            frames.push_back(actual.frames[i]);
        }

    }

    void update(Location &actual){
            histogram.update(actual.histogram);
    }





};

class PointBasedLocalization{
    std::vector<Location> locations;
public:
    double detectionThreshold=0.98;
    double detectionThreshold2=0.75;
    double detectionThreshold3=0.9;
    bool showLike=true;
    bool showCut=false;
    int lastLocation=0;
    int selectedID=-1;
    Location *fake=NULL;


    double distNearest=99999;

    void clear(){

        lastLocation=0;
        selectedID=-1;
        delete fake;
        fake=NULL;
        locations.clear();
    }

    bool save(std::string filename){

        std::fstream fd;
        fd.open (filename.c_str(), std::fstream::in | std::fstream::out  | std::fstream::trunc);

        if(fd.is_open()){

            for(unsigned int i=0;i<locations.size();i++){
                for(unsigned int j=0;j<locations[i].frames.size();j++){
                    fd<<(int)locations[i].frames[j]<<" ";
                }
                fd<<std::endl;
            }
            fd.close();

            return true;

        }else{
            std::cout<<"Failed to save to "<<filename<<std::endl;
            return  false;

        }

    }

    unsigned int compute(Location &actual,geometry_msgs::Pose &pose){

        if(locations.size()==0){
            //            locations.emplace_back(objectClasses,data,pose);
            locations.push_back(actual);
            fake=new Location(actual);
            locations[0].prob=0.8;
            locations[0].prior=0.8;
            lastLocation=0;
            return 0;
        }else{
            //            Location actual(objectClasses,data,pose);

            fake->pose=actual.pose; //debug only
            fake->histogram=actual.histogram;
            fake->neighbours=actual.neighbours;

            double fprior=locations[lastLocation].prior;
            locations[lastLocation].computeLikelihood(actual);

            long double likelihoodprior=  (locations[lastLocation].computeLikelihood(actual)*(fprior));
            long double normalizationTermR = likelihoodprior+(1.0L- (locations[lastLocation].computeLikelihood(actual)))*(1.0L-fprior);
            double plz_last= ( (likelihoodprior)/ (normalizationTermR));

            //https://stats.stackexchange.com/questions/66616/converting-normalizing-very-small-likelihood-values-to-probability

            //checking if is previous
            //            std::cout<<"p_last "<<plz_last <<"  "<<fprior<<"  "<<likelihoodprior<<"  "<<normalizationTermR<<std::endl;
            //            std::cout<<"C_like "<<locations[lastLocation].classModelLikelyhood(actual)<<" L_like "<<locations[lastLocation].likelihood<<std::endl;

            if(plz_last>=detectionThreshold){
                locations[lastLocation].update2(actual);
                locations[lastLocation].prior=std::min(plz_last,0.8);
                return lastLocation;
            }

            std::vector<int> explorableLocations;

            for(unsigned int i=0;i<locations.size();i++){

                double f=locations[i].computeLikelihood(actual);
                if(f>detectionThreshold3){
                    double d=std::sqrt((locations[i].pose.position.x-actual.pose.position.x)*(locations[i].pose.position.x-actual.pose.position.x)+(locations[i].pose.position.y-actual.pose.position.y)*(locations[i].pose.position.y-actual.pose.position.y));
                    if(d<=distNearest){
                        explorableLocations.push_back(i);
                    }
                }
            }
            explorableLocations.push_back(lastLocation);

            double bestP=0;
            int id=-1;
            fprior=0;
            for(unsigned int i=0;i<explorableLocations.size();i++){
                locations[explorableLocations[i]].prob=locations[explorableLocations[i]].likelihood;
                if(locations[explorableLocations[i]].likelihood>bestP){
                    id=explorableLocations[i];
                    bestP=locations[explorableLocations[i]].likelihood;
                }
            }

            if(id!=-1){

                fprior=locations[id].computeLikelihood(actual);
                likelihoodprior=  (locations[id].computeLikelihood(actual)*(fprior));
                normalizationTermR = likelihoodprior+(1.0L- (locations[id].computeLikelihood(actual)))*(1.0L-fprior);
                plz_last= ( (likelihoodprior)/ (normalizationTermR));

            }else{
                plz_last=-1;
            }

            bestP=plz_last;
            if( plz_last>= detectionThreshold2){

                locations[id].prob=plz_last;
                locations[id].prior=std::min(plz_last,0.8);

                if(lastLocation!=id){

                    std::pair<int,std::vector<double> > out;
                    out.first=lastLocation;
                    out.second={bestP,actual.prob};
                    locations[id].neighbours.push_back(out);
                    locations[lastLocation].parentOf.push_back(id);
                    locations[id].update2(actual);

                }else{
                    locations[id].update2(actual);
                }
                lastLocation=id;
                return id;
            }else{

                locations.push_back(actual);

                std::pair<int,std::vector<double> > out;
                out.first=lastLocation;
                out.second={bestP,actual.prob};
                locations[lastLocation].parentOf.push_back(locations.size()-1);

                lastLocation=locations.size()-1;
                locations[lastLocation].neighbours.push_back(out);
                locations[lastLocation].prob=0.8;
                locations[lastLocation].prior=0.8;

                return lastLocation;

            }
        }
    }

};


#endif // POINTBASEDLOOPCLOSURE_H
