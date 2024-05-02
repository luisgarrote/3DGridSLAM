#ifndef SEQUENTIALMONTECARLO_H
#define SEQUENTIALMONTECARLO_H

#include <vector>
#include <functional>
#include <unordered_map>
// or particle filter


/*


*/
class RLvars{
public:
    int state=0;
    int action=0;
    double score=0;

};
template <typename T>
class Particle{
public:
    T state;
    RLvars RL;
    //    std::vector<Point> temp;
    double w=0;
    double w_unnormalized=0;
    Particle(T st){
        state=st;
    }
    Particle(){

        state.orientation.w=1.0;
    }

    bool operator() (const Particle &i, const Particle &j)const { return (i.w>j.w);}
    bool operator < (const Particle & other) const //(1)
    {
        return w > other.w;
    }


    void reset(){
        state=T();
        state.orientation.w=1.0;
        w=0;
        w_unnormalized=0;
    }
};

template <typename T,typename F,typename G>
class SequentialMonteCarlo{


    std::function<void (Particle<T>&)> sampleStage;
    std::function<void (Particle<T>&,F&)> predictStage;
    std::function<void (Particle<T>&,G&)> updateStage;
    std::function<void (std::vector<Particle<T>> &)> resampleStage;
    std::function<void (std::vector<Particle<T>> &,F&,G&)> completeStage;
    std::function<void (std::vector<Particle<T>> &,std::unordered_map<std::string,F >&,G&)> completeStage2;






public:

    double w_mean=0;
    double w_total=0;
    std::vector<Particle<T>> particles;
    int numParticles;


    SequentialMonteCarlo(int size,std::function<void (Particle<T>&)> sample_,std::function<void (Particle<T>&,F&)> predict_,std::function<void (Particle<T>&,G&)> update_,std::function<void (std::vector<Particle<T>> &)> resample_,std::function<void (std::vector<Particle<T>> &,F&,G&)> completeStage_=std::function<void (std::vector<Particle<T>> &,F&,G&)>(),std::function<void (std::vector<Particle<T>> &,std::unordered_map<std::string,F >&,G&)> completeStage2_=std::function<void (std::vector<Particle<T>> &,std::unordered_map<std::string,F >&,G&)>()){
        numParticles=size;
        sampleStage=sample_;
        predictStage=predict_;
        updateStage=update_;
        resampleStage=resample_;
        completeStage=completeStage_;
        completeStage2=completeStage2_;


        particles.resize(numParticles);


    }



    void init(){
        for(  int i=0;i<numParticles;i++){
            sampleStage(particles[i]);
        }
    }
    void sample(){
        for(  int i=0;i<numParticles;i++){
            sampleStage(particles[i]);
        }
    }

    void prediction(F &data){
        //model
        for(  int i=0;i<numParticles;i++){
            predictStage(particles[i],data);
        }
    }

    void update(G &data){
        //measurements

        double total=0;
        for(  int i=0;i<numParticles;i++){
            updateStage(particles[i],data);
            total+=particles[i].w;
            particles[i].w_unnormalized=particles[i].w;
        }


        w_total=total;
        w_mean=total/((double)numParticles);
        if(total>0.0000000001){
            for(  int i=0;i<numParticles;i++){
                particles[i].w=particles[i].w/total;
            }
        }
    }
    void normalize(){

        //        double xmax=0;
        //        double xmin=999999;

        //        for(  int i=0;i<numParticles;i++){
        //            xmax=std::max(particles[i].w,xmax);
        //            xmin=std::min(particles[i].w,xmin);
        //         }

        //        for(  int i=0;i<numParticles;i++){
        //            particles[i].w=(particles[i].w-xmin)/(xmax-xmin);
        //         }

        double total=0;


        for(  int i=0;i<numParticles;i++){
            total+=particles[i].w;
        }


        w_mean=total/((double)numParticles);
        total=0;
        for(  int i=0;i<numParticles;i++){
            if(particles[i].w<w_mean){
                particles[i].w=0;
            }
            total+=particles[i].w;
            particles[i].w_unnormalized=particles[i].w;
        }



        w_total=total;


        //        double tt=0;
        if(total>0.0000000001){
            for(  int i=0;i<numParticles;i++){
                particles[i].w=particles[i].w/total;

                //                tt+=particles[i].w;
            }
        }

        //        std::cout<<"New total "<<tt<<std::endl;

    }
    void resampling(){
        resampleStage(particles);
    }

    void fullImplementation(F &data,G & dt){
        completeStage(particles,data,dt);
    }

    void fullImplementation2(std::unordered_map<std::string,F > &data,G& dt){
        completeStage2(particles,data,dt);
    }

public:
};

#endif // SEQUENTIALMONTECARLO_H
