#ifndef NUMERICARRAY_H
#define NUMERICARRAY_H

#include <array>
#include <functional>
#include <sstream>
#include <algorithm>
#include <cmath>

template<typename T, unsigned int Count>
class NumericArray{

public:
    std::array<T, Count> data;

    NumericArray(){
        for(unsigned int i=0;i<Count;i++){
            data[i]=0;
        }
    }

    NumericArray<T, Count>& operator=(const NumericArray<T, Count> &rhs) {

        if (this != &rhs) {
            data=rhs.data;
        }

        return *this;
    }
    double distance(NumericArray<T, Count> &rhs){

        double out=0;
        for(unsigned int i=0;i<Count;i++){
            out+=(data[i]-rhs.data[i])*(data[i]-rhs.data[i]);
        }

        return std::sqrt(out);
    }

    NumericArray<T, Count> & operator+=(const NumericArray<T, Count> &rhs) {
        for(unsigned int i=0;i<Count;i++){
            data[i]+=rhs.data[i];
        }
        return *this;
    }


    std::string toString(){
        std::stringstream str;
        for(unsigned int i=0;i<Count;i++){
            str<<data[i];
            str<<" ";
        }

        return str.str();

    }

    NumericArray<T, Count> & operator-=(const NumericArray<T, Count> &rhs) {
        for(unsigned int i=0;i<Count;i++){
            data[i]-=rhs.data[i];
        }
        return *this;
    }

    const NumericArray<T, Count> operator+(const NumericArray<T, Count> &other) const {
        NumericArray<T, Count> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result += other;            // Use += to add other to the copy.
        return result;              // All done!
    }

    const NumericArray<T, Count> operator-(const NumericArray<T, Count> &other) const {
        NumericArray<T, Count> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result -= other;            // Use += to add other to the copy.
        return result;              // All done!
    }

    template<typename G>
    NumericArray<T, Count> & operator+=(const G &rhs) {
        for(unsigned int i=0;i<Count;i++){
            data[i]+=(T)rhs;
        }
        return *this;
    }
    template<typename G>
    const NumericArray<T, Count> operator+(const G &other) const {
        NumericArray<T, Count> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result += other;            // Use += to add other to the copy.
        return result;              // All done!
    }

    template<typename G>
    NumericArray<T, Count> & operator-=(const G &rhs) {
        for(unsigned int i=0;i<Count;i++){
            data[i]-=(T)rhs;
        }
        return *this;
    }
    template<typename G>
    const NumericArray<T, Count> operator-(const G &other) const {
        NumericArray<T, Count> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result -= other;            // Use += to add other to the copy.
        return result;              // All done!
    }

    template<typename G>
    NumericArray<T, Count> & operator*=(const G &rhs) {
        for(unsigned int i=0;i<Count;i++){
            data[i]*=(T)rhs;
        }
        return *this;
    }
    template<typename G>
    const NumericArray<T, Count> operator*(const G &other) const {
        NumericArray<T, Count> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result *= other;            // Use += to add other to the copy.
        return result;              // All done!
    }

    template<typename G>
    NumericArray<T, Count> & operator/=(const G &rhs) {
        for(unsigned int i=0;i<Count;i++){
            data[i]/=(T)rhs;
        }
        return *this;
    }






    template<typename G>
    const NumericArray<T, Count> operator/(const G &other) const {
        NumericArray<T, Count> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
        result /= other;            // Use += to add other to the copy.
        return result;              // All done!
    }

    bool operator!=(const NumericArray<T, Count> &other) const {
        return !(*this == other);
    }

    bool operator==(const NumericArray<T, Count> &other) const {
        for(unsigned int i=0;i<Count;i++){
            if(data[i]!=other.data[i]){
                return false;
            }
        }
        return true;
    }


    T& operator [](unsigned int b){
        return data[b];
    }

    std::pair<T,unsigned int> min(){
        auto v=std::min_element(&data[0],&data[0]+Count);
        return std::pair<T,unsigned int>(v,data[std::distance(&data[0], v)]);
    }
    std::pair<T,unsigned int> max(){
        auto v=std::max_element(&data[0],&data[0]+Count);
        return std::pair<T,unsigned int>(v,data[std::distance(&data[0], v)]);
    }



    void filter(std::function<T(T)>  fu){
        for(unsigned int i=0;i<data.size();i++){
            data[i]=fu(data[i]);
        }
    }

    int count(std::function<int(T)> fu){
        int out=0;
        for(unsigned int i=0;i<data.size();i++){
            out=out+fu(data[i]);
        }
        return out;
    }


    T mean(){
        T out=0;
        unsigned int counter=0;
        for(unsigned int i=0;i<data.size();i++){
            out+=data[i];
        }
        return out/((double)counter);


    }

};

#endif // NUMERICARRAY_H
