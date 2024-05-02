#ifndef STRINGFORMAT_H
#define STRINGFORMAT_H

#include <iostream>
#include <vector>
#include <sstream>
#include <map>
#include <algorithm>
#include <iomanip>
#include <typeinfo>
#include <cmath>
#include <string>

#define $(x) StringFormat(x)

namespace Garrote {





/*

#include <type_traits>

// Primary template with a static assertion
// for a meaningful error message
// if it ever gets instantiated.
// We could leave it undefined if we didn't care.

template<typename, typename T>
struct has_serialize {
    static_assert(
        std::integral_constant<T, false>::value,
        "Second template parameter needs to be of function type.");
};

// specialization that does the checking

template<typename C, typename Ret, typename... Args>
struct has_serialize<C, Ret(Args...)> {
private:
    template<typename T>
    static constexpr auto check(T*)
    -> typename
        std::is_same<
            decltype( std::declval<T>().serialize( std::declval<Args>()... ) ),
            Ret    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        >::type;  // attempt to call it and see if the return type is correct

    template<typename>
    static constexpr std::false_type check(...);

    typedef decltype(check<C>(0)) type;

public:
    static constexpr bool value = type::value;
};











*/
//template<typename T, template <typename,typename = std::allocator<T> > class Container>
//static std::string to_string(Container<T> data){

//    std::stringstream ss;
//    ss<<" [ ";
//    int ct=0;
//    for(auto const& value: data) {
//        if(ct!=0){
//            ss<<" , ";
//        }
//        ss << value;
//        ct++;
//    }
//    ss<<" ] ";

//    return ss.str();

//}

template<typename T>
struct has_const_iterator
{
private:
    typedef char                      yes;
    typedef struct { char array[2]; } no;

    template<typename C> static yes test(typename C::const_iterator*);
    template<typename C> static no  test(...);
public:
    static const bool value = sizeof(test<T>(0)) == sizeof(yes);
    typedef T type;
};

template <typename T>
struct has_begin_end
{
    template<typename C> static char (&f(typename std::enable_if<
                                         std::is_same<decltype(static_cast<typename C::const_iterator (C::*)() const>(&C::begin)),
                                         typename C::const_iterator(C::*)() const>::value, void>::type*))[1];

    template<typename C> static char (&f(...))[2];

    template<typename C> static char (&g(typename std::enable_if<
                                         std::is_same<decltype(static_cast<typename C::const_iterator (C::*)() const>(&C::end)),
                                         typename C::const_iterator(C::*)() const>::value, void>::type*))[1];

    template<typename C> static char (&g(...))[2];

    static bool const beg_value = sizeof(f<T>(0)) == 1;
    static bool const end_value = sizeof(g<T>(0)) == 1;
};


template<typename T>
struct is_container : std::integral_constant<bool, has_const_iterator<T>::value && has_begin_end<T>::beg_value && has_begin_end<T>::end_value>
{ };


template<typename T,typename D>
inline std::ostream& operator<< (std::ostream &out,  const typename std::pair<T,D> &b)
{
    out<<"{ "<<b.first<<": "<<b.second<<" }";
    return out;
}




template<bool> struct toString
{
    template<typename t> static inline std::string converter(t& data);
};

template<> template<typename t> std::string toString<false>::converter(t& data)
{
    // std::cout << "not 32bit pod" << std::endl;

    std::stringstream ss;
    ss << data;
    return ss.str();

}
template<> template<typename t> std::string toString<true>::converter(t& data)
{
    // std::cout << "32bit pod" << std::endl;

    std::stringstream ss;
    ss<<" [ ";
    int ct=0;
    for(auto value: data) {
        if(ct!=0){
            ss<<" , ";
        }
        ss<<value;
        ct++;
    }
    ss<<" ] ";

    return ss.str();


}

template<typename t> inline std::string to_string(t& data)
{
    return toString<is_container<t>::value && !std::is_same<std::string, t>::value>::template converter<t>(data);
}






template<bool> struct toStringOrStream
{
    template<typename t> static inline std::string converter2(t& data,std::stringstream &ss);
};

template<> template<typename t> std::string toStringOrStream<false>::converter2(t& data,std::stringstream &ss)
{
    // std::cout << "not 32bit pod" << std::endl;

    //    std::stringstream ss;
    ss << data;
    //    return ss.str();
    return "";


}
template<> template<typename t> std::string toStringOrStream<true>::converter2(t& data,std::stringstream &ss)
{
    // std::cout << "32bit pod" << std::endl;

    //    std::stringstream ss;
    ss<<" [ ";
    int ct=0;
    for(auto value: data) {
        if(ct!=0){
            ss<<" , ";
        }
        ss<<value;
        ct++;
    }
    ss<<" ] ";

    //    return ss.str();
    return "";


}

template<typename t> inline std::string to_stringOrStream(t& data,std::stringstream &str)
{
    return toStringOrStream<is_container<t>::value && !std::is_same<std::string, t>::value>::template converter2<t>(data,str);
}





//template <typename T>
//static std::string to_string( std::vector<T> data){

//    std::stringstream ss;
//    ss<<" [ ";
//    int ct=0;
//    for(auto const& value: data) {
//        if(ct!=0){
//            ss<<" , ";
//        }
//        ss << value;
//        ct++;
//    }
//    ss<<" ] ";

//    return ss.str();

//}




//template <typename T>
//static std::string to_string(T data){
//        std::stringstream ss;
//        ss << data;
//        return ss.str();
//}

}



class StringFormat{


    std::string data;



public:




    enum TokenType : signed int { Freeform=-1,FreeToken=-2};
    StringFormat(std::string str){
        data=str;
    }


    std::string format(){
        return data;
    }



private:






    class formatchain{
        StringFormat &element;
    public:
        formatchain(StringFormat &element_) :element(element_){

        }

        std::string str(){
            return element.data;
        }
        StringFormat &end(){
            return element;
        }

        formatchain & reverse(){
            element.reverse();

            return *this;
        }
        formatchain &  remove(char c){
            element.remove(c);

            return *this;
        }
        formatchain & replace(std::string from, std::string to){
            element.replace(from,to);
            return *this;
        }
        formatchain & replace(std::map<std::string,std::string> mape){
            element.replace(mape);
            return *this;
        }
        formatchain & replaceAnyCharWith(std::string st,char c){
            element.replaceAnyCharWith(st,c);
            return *this;
        }
        formatchain & upper(){
            element.upper();
            return *this;

        }
        formatchain & lower(){
            element.lower();
            return *this;
        }


        formatchain & format(){
            return *this;
        }

        template<typename... Targs>
        formatchain & format(Targs... Fargs){

            auto str1=element.format(Fargs...);

            element.data=str1;
            return *this;
        }


    };

public:

    formatchain chain(){
        return formatchain(*this);
    }

    unsigned int count(){
        return data.size();
    }

    unsigned int count(char ch){

        int ct=0;
        for(unsigned int i=0;i<data.size();i++){

            if(data[i]==ch){
                ct++;
            }
        }
        return ct;
    }


    std::string reverse(){
        return std::string(data.rbegin(), data.rend());
    }


    std::string replace(std::string from, std::string to) {
        if(from.empty())
            return data;
        size_t start_pos = 0;
        while((start_pos = data.find(from, start_pos)) != std::string::npos) {
            data.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }

        return data;
    }


    std::string replace(std::map<std::string,std::string> mape) {

        for(auto e:mape){
            replace(e.first,e.second);
        }

        return data;
    }



    std::string remove(char c)
    {
        data.erase( std::remove( data.begin(), data.end(), c ),data.end() ) ;
        return data;
    }



    std::vector<std::string> blocks(){
        //       (dsdsdfsdf)
        std::vector<std::string> out;

        int id=-1;

        bool inblock=false;
        std::vector<char> start={'{','(','['};
        std::vector<char> end={'}',')',']'};

        std::string outblock;
        for (unsigned int i=0;i<data.size();i++){
            if(inblock){
                if(data[i]==end[id]){
                    outblock+=data[i];
                    id=-1;
                    out.push_back(outblock);
                    outblock.clear();
                    inblock=false;
                    continue;
                }else{
                    outblock+=data[i];
                }
            }else{

                if(data[i]==start[0]){
                    outblock.clear();
                    outblock+=data[i];

                    inblock=true;
                    id=0;
                }else  if(data[i]==start[1]){
                    outblock.clear();
                    outblock+=data[i];

                    inblock=true;
                    id=1;
                }else  if(data[i]==start[2]){
                    outblock.clear();
                    outblock+=data[i];

                    inblock=true;
                    id=2;
                }else {

                }
            }
        }

        return out;
    }



    inline std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v")
    {
        s.erase(0, s.find_first_not_of(t));
        return s;
    }

    // trim from right
    inline std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v")
    {
        s.erase(s.find_last_not_of(t) + 1);
        return s;
    }

    // trim from left & right
    inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v")
    {
        return ltrim(rtrim(s, t), t);
    }

    inline std::string& trim(const char* t = " \t\n\r\f\v")
    {
        return ltrim(rtrim(data, t), t);
    }




    std::map<std::string,std::string> map(char vaseparator=':', char lineseparator='\n', bool trimspaces=true){
        //uhuuh: yghygyg


        //        std::stringstream str;
        std::map<std::string,std::string> out;


        std::string variable;
        std::string datavar;
        int ct=0;
        bool onvar=true;
        for (unsigned int i=0;i<data.size();i++){


            if(onvar){
                if(data[i]==vaseparator){
                    for(unsigned int f=i+1;f<data.size();f++){
                        if(data[f]==' '){
                            onvar=false;
                            i=i+1;
                            break;
                        }else{
                            variable+=data[i];  // its correct!!

                            break;
                        }
                    }

                }else if(data[i]==lineseparator){


                    out["raw"+std::to_string(ct)]=variable;
                    ct++;

                    variable.clear();
                    datavar.clear();

                    onvar=true;
                }else{
                    variable+=data[i];
                }

            }else{

                if(data[i]==lineseparator){

                    if(trimspaces){
                        ltrim(variable,"{");
                        if($(datavar).count('}')!=$(datavar).count('{')){
                            rtrim(datavar,"}");
                        }
                        trim(variable);
                        trim(datavar);
                    }

                    out[variable]=datavar;
                    variable.clear();
                    datavar.clear();
                    onvar=true;

                }else{
                    datavar+=data[i];
                }
            }

        }

        if(!variable.empty()){
            if(!datavar.empty()){
                if(trimspaces){
                    ltrim(variable,"{");
                    if($(datavar).count('}')!=$(datavar).count('{')){
                        rtrim(datavar,"}");
                    }

                    trim(variable);
                    trim(datavar);
                }
                out[variable]=datavar;
            }else{

                out["raw"+std::to_string(ct)]=variable;
            }
        }

        //str<<data;

        //        std::string buffer;
        //        while(!str.eof()){

        //            std::getline(str,buffer);

        //            unsigned long int index;

        //            if(( index=buffer.find(": "))!=std::string::npos){

        //                std::string variable,variable2;
        //                variable=buffer.substr(0,index);
        //                if(buffer[index+1]==' '){
        //                    variable2=buffer.substr(index+2);
        //                }else{
        //                    variable2=buffer.substr(index+1);
        //                }
        //                //  std::cout<<"parseText v1 ->#"<<variable<<"#v2 -> "<<variable2<<std::endl;
        //                out[variable]=variable2;
        //            }else{

        //                if(!buffer.empty()){
        //                    out["raw"]=buffer;
        //                    //      std::cout<<"parseText v1 ->#"<<"raw"<<"#v2 ->#"<<buffer<<"#"<<std::endl;
        //                }
        //            }

        //        }


        return out;


    }

    std::vector<std::string> split(char c=' '){

        std::vector<std::string> Lines;

        std::string d;
        d.reserve(std::max(data.size()/10.0,10.0));
        for(unsigned int i=0;i<data.size();i++){

            if(data[i]==c){

                if(d.length()>0){
                    Lines.push_back(d);
                    d.clear();
                }
            }else{
                d+=( data.at(i));
            }

        }

        if(d.size()>0){
            Lines.push_back(d);
        }

        return Lines;


    }


    std::vector<std::string> split(std::string c){

        std::vector<std::string> Lines;


        //        vector<size_t> positions; // holds all the positions that sub occurs within str

        size_t pos = data.find(c, 0);
        size_t temp=0;
        while(pos != std::string::npos)
        {

            auto str=data.substr(temp,pos-temp);

            if(str.size()==0){

            }else if(str==c){

            }else{
                Lines.push_back(str);
            }
            temp=pos+c.size() ;
            pos = data.find(c,temp);

        }

        if(temp<data.size()){
            std::string str=data.substr(temp,pos-temp);


            if(str.size()!=0){
                if(str!=c){
                    Lines.push_back(str);
                }
            }
        }



        return Lines;
    }



    std::pair<std::string,std::string> token(char separator=' '){
        std::pair<std::string,std::string> out;
        bool detect=false;
        for(unsigned int i=0;i<data.size();i++){

            if(detect){
                out.second+=data[i];
            }else{
                if(data[i]==separator){
                    detect=true;
                }else{
                    out.first+=data[i];
                }
            }
        }

        bool cleanning =true;
        for(unsigned int i=0;i<out.second.size();i++){


            if(cleanning){

                if(out.second[i]!=separator){

                    break;
                }else{
                    out.second.erase(out.second.begin()+i);
                    i--;
                }
            }
        }

        return out;
    }






    std::vector<std::pair<int,std::string> > tokenlist(std::string data__){

        std::vector<std::pair<int,std::string> > out;

        std::stringstream outbuffer;

        for(unsigned int i=0;i<data__.size();i++){

            if(data__[i]!='{'){
                outbuffer<<data__[i];
            }else{
                int ct=-1;
                for(unsigned int g=i+1;g<data__.size();g++){

                    if(data__[g]=='{'){
                        break;
                    }
                    if(data__[g]=='}'){
                        ct=g;
                        break;
                    }
                }
                if(ct!=-1){
                    std::string str2 = data__.substr (i,ct-i +1);

                    //formating based on str2, skipping for now

                    if(str2.size()<=16){

                        if(!outbuffer.str().empty()){
                            out.push_back(std::pair<int,std::string>(-1,outbuffer.str()));
                        }
                        out.push_back(std::pair<int,std::string>(-2,str2));
                        outbuffer.str(std::string());

                        i=ct;
                    }else{
                        outbuffer<<data__[i];
                    }

                }else{
                    outbuffer<<data__[i];

                }
            }



        }


        if(!outbuffer.str().empty()){
            out.push_back(std::pair<int,std::string>(-1,outbuffer.str()));
        }

        return out;

    }



    std::vector<std::string> tokenlist2(std::string data__){

        std::vector<std::string > out;

        int start=-1;

        for(unsigned int i=0;i<data__.size();i++){

            if(data__[i]=='{'){
                start=i;
            }else if(data__[i]=='}'){
                if(start>=0){
                    std::string str2 = data__.substr (start,i-start+1);
                    out.push_back(str2);
                    start=-1;
                }
            }

        }



        return out;

    }




    std::vector<std::string> rawnumbers(){
        std::vector<std::string> out;
        std::stringstream str;
        bool number=false;
        for(unsigned int i=0;i<data.size();i++){

            if(number){
                if(isdigit(data[i])|| data[i]=='e' || data[i]=='.' || data[i]=='-'){
                    str<<data[i];
                }else{
                    //str<<data[i];
                    //                     std::cout<<str.str()<<std::endl;
                    if(str.str()!="." && str.str()!="-"){
                        number=false;
                        out.push_back(str.str());
                        //                        std::cout<<str.str()<<std::endl;
                        str.str("");
                        str.clear();
                    }else{
                        number=false;

                        str.str("");
                        str.clear();
                    }

                }
            }else{

                if(isdigit(data[i]) || data[i]=='.' || data[i]=='-'){
                    str<<data[i];
                    number=true;
                }


            }

        }

        if (number==true){

            if(str.str()!="." && str.str()!="-"){
                out.push_back(str.str());

                str.str("");
                str.clear();
            }
        }


        return out;
    }



    std::string remove(std::string str){


        std::string datacopy=data;
        while (true){

            std::string::size_type i = datacopy.find(str);

            if (i != std::string::npos){
                datacopy.erase(i, str.length());
            }else{
                break;
            }
        }
        return datacopy;

    }

    template<typename T>
    std::vector<T> numbers(){
        std::vector<T> out;
        std::stringstream str;
        T temp;
        bool number=false;
        for(unsigned int i=0;i<data.size();i++){

            if(number){
                if(isdigit(data[i])|| data[i]=='e' || data[i]=='.' || data[i]=='-'){
                    str<<data[i];
                }else{
                    //str<<data[i];
                    //                     std::cout<<str.str()<<std::endl;
                    if(str.str()!="." && str.str()!="-"){
                        number=false;
                        str>>temp;
                        out.push_back(temp);
                        //                        std::cout<<str.str()<<std::endl;
                        str.str("");
                        str.clear();
                    }else{
                        number=false;

                        str.str("");
                        str.clear();
                    }

                }
            }else{

                if(isdigit(data[i]) || data[i]=='.' || data[i]=='-'){
                    str<<data[i];
                    number=true;
                }


            }

        }

        if (number==true){

            if(str.str()!="." && str.str()!="-"){
                str>>temp;
                out.push_back(temp);
                //                std::cout<<str.str()<<std::endl;

                str.str("");
                str.clear();
            }
        }


        return out;
    }

    template<typename T,typename... Targs>
    std::string format( T value,Targs... Fargs){

        //std::string data_=data;
        //std::stringstream out;

        std::vector<std::pair<int,std::string> > breaker=tokenlist(data);

        //        for(int i=0;i<breaker.size();i++){
        //            std::cout<<"|"<<breaker[i].second<<"|"<<std::endl;

        //        }

        _format(breaker,0,value,Fargs...);

        std::stringstream out;


        for(unsigned int i=0;i<breaker.size();i++){
            out<<breaker[i].second;
            //            std::cout<<breaker[i].second<<std::endl;

        }


        return out.str();



    }


    //    std::string format2( std::vector<std::pair<int,std::string> > &breaker){

    //                std::stringstream out;

    //                for(unsigned int i=0;i<breaker.size();i++){
    //                    out<<breaker[i].second;
    //                }
    //                return out.str();
    //     }


    //    template<typename T,typename... Targs>
    //    std::string format2(std::vector<std::pair<int,std::string> > &breaker, T value,Targs... Fargs){


    //        _format(breaker,0,value,Fargs...);



    //        return format2(breaker);

    //    }

    void replacer(std::string oldS,std::string newS){
        data=replace(oldS, newS);
    }

    std::string replacer(std::map<std::string,std::string>  values){

        std::vector<std::string > breaker=tokenlist2(data);

        //        if(breaker.size()>0){
        //            for(unsigned int i=0;i<breaker.size();i++){
        //                std::cout<<"## "<<breaker[i]<<std::endl;
        //            }

        //        }

        for(unsigned int i=0;i<breaker.size();i++){
            if(values.find(breaker[i])!=values.end()){
                replacer(breaker[i],values[breaker[i]]);
            }else{
//                replacer(breaker[i],"???");
            }
        }

        return data;
    }


    //    template<typename... Targs>
    //    std::string format(std::map<std::string,std::string> &values){

    //        std::vector<std::pair<int,std::string> > breaker=tokenlist(data);

    //        _format2(breaker,0,values);

    //        return format2(breaker);
    ////        _format(breaker,1,Fargs...);


    ////        std::stringstream out;


    ////        for(unsigned int i=0;i<breaker.size();i++){
    ////            out<<breaker[i].second;

    ////        }


    ////        return out.str();



    //    }



    template <typename T>
    std::string formatter(T data_,std::string fmt){





        if(fmt.find(":")==fmt.npos){
            return Garrote::to_string(data_);
        }

        std::string temp;
        bool swt=false;
        for(unsigned int i=0;i<fmt.size();i++){
            if(swt){
                if(fmt[i]!='}'){
                    temp+=fmt[i];
                }
            }
            if(fmt[i]==':'){
                swt=true;
            }
        }

        fmt=temp;


        char step=' ';
        unsigned char align=0; //0 left, 1 right, 2 center
        char signal='o';
        bool card=false;
        // bool zero=false;
        unsigned int width=0;
        //bool comma=false;
        int precision=-1;
        int moddify=0;
        std::string type="";

        int it=0;

        if(fmt.find_first_of("<>=")!=fmt.npos){

            if(fmt[0]!='<' && fmt[0]!='>' && fmt[0]!='='){
                step=fmt[0];
                if(fmt[1]=='<'){
                    align=0;
                }else if(fmt[1]=='>'){
                    align=1;
                }else if(fmt[1]=='='){
                    align=2;
                }

            }else{
                if(fmt[0]=='<'){
                    align=0;
                }else if(fmt[0]=='>'){
                    align=1;
                }else if(fmt[0]=='='){
                    align=2;
                }
            }
            it=1;
        }


        //format_spec ::=  [[fill]align][sign][#][0][width][,][.precision][type]
        //fill        ::=  <any character>
        //align       ::=  "<" | ">" | "=" | "^"
        //sign        ::=  "+" | "-" | " "
        //width       ::=  integer
        //precision   ::=  integer
        //type        ::=  "b" | "c" | "d" | "e" | "E" | "f" | "F" | "g" | "G" | "n" | "o" | "s" | "x" | "X" | "%"

        for(unsigned int i=it;i<fmt.size();i++){

            if(fmt[i]=='<' || fmt[i]=='>' || fmt[i]=='='){
                continue;
            }

            if(fmt[i]=='_' || fmt[i]=='!'){
                if(fmt[i]=='_'){

                    moddify=1;
                }
                if(fmt[i]=='!'){
                    moddify=2;
                }

            }

            if(fmt[i]=='+' || fmt[i]=='-' || fmt[i]==' '){
                if(fmt[i]=='+'){
                    signal='+';
                }else if(fmt[i]=='-'){
                    signal='-';
                }else if(fmt[i]==' '){
                    signal=' ';
                }
                continue;
            }

            if(fmt[i]=='#'){
                card=true;
                continue;
            }

            if(isdigit(fmt[i])){

                std::string tmp;
                tmp+=fmt[i];
                unsigned int g;
                for(  g=i+1;g<fmt.size();g++){

                    if(isdigit(fmt[g])){
                        tmp+=fmt[g];
                    }else{

                        break;
                    }
                }

                i=g-1;
                width=atoi(tmp.c_str());



            }
            //            if(fmt[i]==','){

            //                comma=true;
            //                continue;
            //            }

            if(fmt[i]=='.'){

                std::string tmp;
                unsigned int g;
                for( g=i+1;g<fmt.size();g++){

                    if(isdigit(fmt[g])){
                        tmp+=fmt[g];
                    }else{

                        break;
                    }
                }


                precision=atoi(tmp.c_str());


                if(g<temp.size()){
                    type=temp.substr(g);
                    break;
                }
                continue;
            }



        }



        std::string finalf;
        //        if(type=="" || !std::is_trivial<T>::value){

        std::stringstream str;

        str.setf(std::ios_base::boolalpha);

        if(precision!=-1){
            str.setf(std::ios_base::fixed);
            str.setf(std::ios_base::showpoint);
            //str<< std::ios_base::fixed << std::ios_base::showpoint;

            str.precision(precision);
        }

        if(signal!='o'){
            str.setf(std::ios_base::showpos);
        }

        if(card){
            str.setf(std::ios_base::showbase);
        }

        if((type.find("a")!=type.npos) || (type.find("A")!=type.npos) || (type.find("x")!=type.npos) || (type.find("X")!=type.npos)){

            str.setf(std::ios_base::hex);
        }else if((type.find("o")!=type.npos)){

            str.setf(std::ios_base::oct);

        }
        if((type.find("e")!=type.npos) || (type.find("E")!=type.npos)){

            str.setf(std::ios_base::scientific);
        }


        str<<Garrote::to_stringOrStream(data_,str);
        finalf=str.str();

        //        }else{

        //            char buffer[255];

        //            std::string form="%";
        //            if(signal!='o'){
        //                form+=signal;
        //            }
        //            if(precision!=-1){
        //                form+=".";
        //                form+=Garrote::to_string(precision);
        //            }

        //            if(card){
        //                form+="#";
        //            }
        //            form+=type;



        //            sprintf(buffer,form.c_str(),data_);
        //            finalf=std::string(buffer);

        //        }



        if(moddify==1){
            lower_(finalf);

        }else if(moddify==2){
            upper_(finalf);
        }






        if(width>finalf.size()){


            std::string gf;
            if(align==0){

                for(unsigned int i=0;i<(width-finalf.size());i++){
                    gf+=step;
                }
                finalf=finalf+gf;

            }else if(align==1){

                for(unsigned int i=0;i<(width-finalf.size());i++){
                    gf+=step;
                }
                finalf=gf+finalf;


            }else if(align==2){


                for(unsigned int i=0;i<std::floor((width-finalf.size())/2.0);i++){
                    gf+=step;
                }

                finalf=gf+finalf+gf;

                if(finalf.size()!=width){
                    finalf+=" ";
                }
            }

        }




        return finalf;


    }






    //empty format?
    template<typename T>
    void _format(std::vector<std::pair<int,std::string> > &out,int ctr,T value){

        bool changed=false;
        bool used=false;
        for(unsigned int i=0;i<out.size();i++){

            if(out[i].first!=-1){
                auto str= out[i].second;
                std::string base=str;
                removeFromString(str,' ');

                if(changed==false){
                    if(str.size()==2){ //tokens should remove spaces
                        //use sequence
                        out[i].second=Garrote::to_string(value);
                        out[i].first=-1;
                        changed=true;
                        used=true;

                    }else if(str.find("{:")!=std::string::npos){


                        //we still need format!!!!!!!!!!!!!!!!!!!!!!!!!!
                        out[i].second=formatter(value,base);
                        out[i].first=-1;
                        changed=true;
                        used=true;

                        //this has just format
                    }else if(str.find(":")!=std::string::npos){

                        std::string id;

                        bool swt=false;
                        for(unsigned int g=0;g<str.size();g++){

                            if(str[g]=='{'||str[g]=='}'){
                                continue;
                            }
                            if(str[g]==':'){
                                swt=true;
                                continue;
                            }
                            if(!swt){
                                id+=str[g];

                            }else{

                                break;
                            }
                        }


                        int v=atoi(id.c_str());

                        if(v!=-1){

                            if(v==ctr){

                                out[i].second=formatter(value,base);
                                out[i].first=-1;
                                //changed=true;
                                used=true;

                            }else{

                                out[i].first=v;

                            }

                        }else{

                            out[i].second=formatter(value,base);
                            out[i].first=-1;
                            changed=true;
                            used=true;


                        }



                        //this has format and id or just id
                    }else{


                        if(out[i].first!=-2){

                            if(out[i].first==ctr){

                                out[i].second=Garrote::to_string(value);
                                out[i].first=-1;
                                //changed=true;
                                used=true;

                            }


                        }else{
                            removeChars(str);

                            int v=atoi(str.c_str());

                            if(v!=-1){

                                if(v==ctr){

                                    out[i].second=Garrote::to_string(value);
                                    out[i].first=-1;
                                    //changed=true;
                                    used=true;

                                }else{

                                    out[i].first=v;

                                }

                            }else{

                                //can be empty so...
                                out[i].second=Garrote::to_string(value);
                                out[i].first=-1;
                                changed=true;
                                used=true;

                            }
                        }

                    }
                }else{

                    if(str.size()==2){ //tokens should remove spaces
                        continue;
                    }
                    else if(str.find("{:")!=std::string::npos){

                        continue;

                    }else if(str.find(":")!=std::string::npos){


                        std::string id;

                        bool swt=false;
                        for(unsigned int g=0;g<str.size();g++){

                            if(str[g]=='{'||str[g]=='}'){
                                continue;
                            }
                            if(str[g]==':'){
                                swt=true;
                                continue;
                            }
                            if(!swt){
                                id+=str[g];

                            }else{


                                break;

                                //                                format+=str[g];
                            }
                        }


                        int v=atoi(id.c_str());

                        if(v!=-1){

                            if(v==ctr){

                                out[i].second=formatter(value,base);
                                out[i].first=-1;
                                //changed=true;
                                used=true;

                            }else{

                                out[i].first=v;

                            }

                        }
                        //this has format and id or just format
                    }else{


                        if(out[i].first!=-2){

                            if(out[i].first==ctr){

                                out[i].second=Garrote::to_string(value);
                                out[i].first=-1;
                                // changed=true;
                                used=true;

                            }


                        }else{
                            removeChars(str);

                            int v=atoi(str.c_str());

                            if(v!=-1){

                                if(v==ctr){

                                    out[i].second=Garrote::to_string(value);
                                    out[i].first=-1;
                                    //  changed=true;
                                    used=true;

                                }else{

                                    out[i].first=v;

                                }

                            }

                        }
                    }

                }
            }
        }


        if (used==false){
            std::stringstream out_;
            out_<<" ["<<ctr<<"?"<<"("<<Garrote::to_string(value)<<")]";
            out.push_back(std::pair<int,std::string>(0,out_.str()));
        }

    }

    //    template< typename... Targs>
    //    void _format2(std::vector<std::pair<int,std::string> > &out,int ctr,std::map<std::string,std::string> &value){

    //        for(unsigned int i=0;i<out.size();i++){
    //            replacer(out[i].second,value);
    //        }
    //    }

    //    template<typename T, typename... Targs>
    //    void _format(std::vector<std::pair<int,std::string> > &out,int ctr,std::map<std::string,std::string> &value){

    //        for(unsigned int i=0;i<out.size();i++){
    //            replacer(out[i].second,value);
    //        }



    //    }

    //https://docs.python.org/2/library/string.html
    template<typename T, typename... Targs>
    void _format(std::vector<std::pair<int,std::string> > &out,int ctr,T value, Targs... Fargs){


        bool changed=false;
        bool used=false;
        for(unsigned int i=0;i<out.size();i++){

            if(out[i].first!=-1){
                auto str= out[i].second;
                std::string base=str;
                removeFromString(str,' ');

                if(changed==false){
                    if(str.size()==2){ //tokens should remove spaces
                        //use sequence
                        out[i].second=Garrote::to_string(value);
                        out[i].first=-1;
                        changed=true;
                        used=true;

                    }else if(str.find("{:")!=std::string::npos){


                        //we still need format!!!!!!!!!!!!!!!!!!!!!!!!!!
                        out[i].second=formatter(value,base);
                        out[i].first=-1;
                        changed=true;
                        used=true;

                        //this has just format
                    }else if(str.find(":")!=std::string::npos){

                        std::string id;

                        bool swt=false;
                        for(unsigned int g=0;g<str.size();g++){

                            if(str[g]=='{'||str[g]=='}'){
                                continue;
                            }
                            if(str[g]==':'){
                                swt=true;
                                continue;
                            }
                            if(!swt){
                                id+=str[g];

                            }else{

                                break;
                            }
                        }


                        int v=atoi(id.c_str());

                        if(v!=-1){

                            if(v==ctr){

                                out[i].second=formatter(value,base);
                                out[i].first=-1;
                                //changed=true;
                                used=true;

                            }else{

                                out[i].first=v;

                            }

                        }else{

                            out[i].second=formatter(value,base);
                            out[i].first=-1;
                            changed=true;
                            used=true;


                        }



                        //this has format and id or just id
                    }else{


                        if(out[i].first!=-2){

                            if(out[i].first==ctr){

                                out[i].second=Garrote::to_string(value);
                                out[i].first=-1;
                                //changed=true;
                                used=true;

                            }


                        }else{
                            removeChars(str);

                            int v=atoi(str.c_str());

                            if(v!=-1){

                                if(v==ctr){

                                    out[i].second=Garrote::to_string(value);
                                    out[i].first=-1;
                                    //changed=true;
                                    used=true;

                                }else{

                                    out[i].first=v;

                                }

                            }else{

                                //can be empty so...
                                out[i].second=Garrote::to_string(value);
                                out[i].first=-1;
                                changed=true;
                                used=true;

                            }
                        }

                    }
                }else{

                    if(str.size()==2){ //tokens should remove spaces
                        continue;
                    }
                    else if(str.find("{:")!=std::string::npos){

                        continue;

                    }else if(str.find(":")!=std::string::npos){


                        std::string id;

                        bool swt=false;
                        for(unsigned int g=0;g<str.size();g++){

                            if(str[g]=='{'||str[g]=='}'){
                                continue;
                            }
                            if(str[g]==':'){
                                swt=true;
                                continue;
                            }
                            if(!swt){
                                id+=str[g];

                            }else{


                                break;

                                //                                format+=str[g];
                            }
                        }


                        int v=atoi(id.c_str());

                        if(v!=-1){

                            if(v==ctr){

                                out[i].second=formatter(value,base);
                                out[i].first=-1;
                                //changed=true;
                                used=true;

                            }else{

                                out[i].first=v;

                            }

                        }
                        //this has format and id or just format
                    }else{


                        if(out[i].first!=-2){

                            if(out[i].first==ctr){

                                out[i].second=Garrote::to_string(value);
                                out[i].first=-1;
                                // changed=true;
                                used=true;

                            }


                        }else{
                            removeChars(str);

                            int v=atoi(str.c_str());

                            if(v!=-1){

                                if(v==ctr){

                                    out[i].second=Garrote::to_string(value);
                                    out[i].first=-1;
                                    //  changed=true;
                                    used=true;

                                }else{

                                    out[i].first=v;

                                }

                            }

                        }
                    }

                }
            }
        }


        if (used==false){
            std::stringstream out_;
            out_<<" ["<<ctr<<"?"<<"("<<Garrote::to_string(value)<<")]";
            out.push_back(std::pair<int,std::string>(0,out_.str()));
        }
        //        bool changed=false;

        //        for(int i=0;i<out.size();i++){

        //            if(out[i].first!=-1){
        //                auto str= out[i].second;
        //                removeFromString(str,' ');

        //                if(changed==false){
        //                    if(str.size()==2){ //tokens should remove spaces
        //                        //use sequence
        //                        out[i].second=Garrote::to_string(value);
        //                        out[i].first=-1;
        //                        changed=true;
        //                    }else if(str.find("{:")!=std::string::npos){


        //                        //we still need format!!!!!!!!!!!!!!!!!!!!!!!!!!
        //                        out[i].second=Garrote::to_string(value);
        //                        out[i].first=-1;
        //                        changed=true;

        //                        //this has just format
        //                    }else if(str.find(":")!=std::string::npos){

        //                        std::string id;
        //                        std::string format;

        //                        bool swt=false;
        //                        for(unsigned int g=0;g<str.size();g++){

        //                            if(str[g]=='{'||str[g]=='}'){
        //                                continue;
        //                            }
        //                            if(str[g]==':'){
        //                                swt=true;
        //                                continue;
        //                            }
        //                            if(!swt){
        //                                id+=str[g];

        //                            }else{

        //                                format+=str[g];
        //                            }
        //                        }


        //                        int v=atoi(id.c_str());

        //                        if(v!=-1){

        //                            if(v==ctr){

        //                                out[i].second=Garrote::to_string(value);
        //                                out[i].first=-1;
        //                                changed=true;
        //                            }else{

        //                                out[i].first=v;

        //                            }

        //                        }else{

        //                            out[i].second=Garrote::to_string(value);
        //                            out[i].first=-1;
        //                            changed=true;

        //                        }



        //                        //this has format and id or just id
        //                    }else{


        //                        if(out[i].first!=-2){

        //                            if(out[i].first==ctr){

        //                                out[i].second=Garrote::to_string(value);
        //                                out[i].first=-1;
        //                                changed=true;
        //                            }


        //                        }else{
        //                            removeChars(str);

        //                            int v=atoi(str.c_str());

        //                            if(v!=-1){

        //                                if(v==ctr){

        //                                    out[i].second=Garrote::to_string(value);
        //                                    out[i].first=-1;
        //                                    changed=true;
        //                                }else{

        //                                    out[i].first=v;

        //                                }

        //                            }else{

        //                                //can be empty so...
        //                                out[i].second=Garrote::to_string(value);
        //                                out[i].first=-1;
        //                                changed=true;

        //                            }
        //                        }

        //                    }
        //                }else{

        //                    if(str.size()==2){ //tokens should remove spaces
        //                        continue;
        //                    }
        //                    else if(str.find("{:")!=std::string::npos){

        //                        continue;

        //                    }else if(str.find(":")!=std::string::npos){


        //                        std::string id;
        //                        std::string format;

        //                        bool swt=false;
        //                        for(int g=0;g<str.size();g++){

        //                            if(str[g]=='{'||str[g]=='}'){
        //                                continue;
        //                            }
        //                            if(str[g]==':'){
        //                                swt=true;
        //                                continue;
        //                            }
        //                            if(!swt){
        //                                id+=str[g];

        //                            }else{

        //                                format+=str[g];
        //                            }
        //                        }


        //                        int v=atoi(id.c_str());

        //                        if(v!=-1){

        //                            if(v==ctr){

        //                                out[i].second=Garrote::to_string(value);
        //                                out[i].first=-1;
        //                                changed=true;
        //                            }else{

        //                                out[i].first=v;

        //                            }

        //                        }
        //                        //this has format and id or just format
        //                    }else{


        //                        if(out[i].first!=-2){

        //                            if(out[i].first==ctr){

        //                                out[i].second=Garrote::to_string(value);
        //                                out[i].first=-1;
        //                                changed=true;
        //                            }


        //                        }else{
        //                            removeChars(str);

        //                            int v=atoi(str.c_str());

        //                            if(v!=-1){

        //                                if(v==ctr){

        //                                    out[i].second=Garrote::to_string(value);
        //                                    out[i].first=-1;
        //                                    changed=true;
        //                                }

        //                            }

        //                        }
        //                    }

        //                }
        //            }
        //        }


        //        if (changed==false){
        //            std::stringstream out_;
        //            out_<<" ["<<ctr<<"?"<<"("<<Garrote::to_string(value)<<")]";
        //            out.push_back(std::pair<int,std::string>(0,out_.str()));
        //        }
        ctr++;
        _format(out,ctr,Fargs...);

    }





    //    void tprintf(const char* format) // base function
    //    {
    //        std::cout << format;
    //    }

    //    template<typename T, typename... Targs>
    //    void tprintf(const char* format, T value, Targs... Fargs) // recursive variadic function
    //    {
    //        for ( ; *format != '\0'; format++ ) {
    //            if ( *format == '%' ) {
    //                std::cout << value;
    //                tprintf(format+1, Fargs...); // recursive call
    //                return;
    //            }
    //            std::cout << *format;
    //        }
    //    }

    //    int main()
    //    {
    //        tprintf("% world% %\n","Hello",'!',123);
    //        return 0;
    //    }



    void lower_(std::string &data_)
    {
        std::transform(data_.begin(), data_.end(), data_.begin(), ::tolower);
    }
    void upper_(std::string &data_)
    {
        std::transform(data_.begin(), data_.end(), data_.begin(), ::toupper);
    }



    std::string upper(){

        upper_(data);
        return data;
    }

    std::string lower(){
        lower_(data);
        return data;
    }
    void removeFromString(std::string &x,char c)
    {
        x.erase( std::remove( x.begin(), x.end(), c ), x.end() ) ;
    }



    void replaceAnyCharWith(std::string str,char c){
        //http://www.cplusplus.com/reference/string/string/find_first_of/
        std::size_t found = data.find_first_of(str);

        if(c!='\0'){

            while (found!=std::string::npos)
            {
                data[found]=c;
                found=data.find_first_of(str,found+1);
            }
        }else{

            while (found!=std::string::npos)
            {
                data.erase(found);
                found=data.find_first_of(str,found);
            }

        }


    }


    void removeChars(std::string &str){

        size_t i = 0;
        size_t len = str.length();
        while(i < len){
            if (isdigit (str[i]) || str[i] == 'e' || str[i] == '.' ||str[i] == ','){
                i++;
            }else
            {
                str.erase(i,1);
                len--;
            }
        }
    }




    void unit(){


        std::cout<<$("1 testing stuff").format()<<std::endl;
        std::cout<<$("2 testing stuff {0} {1} {2}").format(25,32,55)<<std::endl;

        std::cout<<$("3 testing stuff {} {0} {1} {2}").format(25)<<std::endl;
        std::cout<<$("4 testing stuff {0}").format(32,44,66)<<std::endl;
        std::cout<<$("5 testing stuff {1}").format(32,44,66)<<std::endl;
        std::cout<<$("6 testing stuff {2}").format(32,44,66)<<std::endl;
        std::cout<<$("7 testing stuff {1}").format(44)<<std::endl;
        std::cout<<$("8 testing stuff {2}").format(54)<<std::endl;
        std::cout<<$("9 testing stuff {3}").format(33)<<std::endl;
        std::cout<<$("10 testing stuff {:<30}").format(33)<<std::endl;
        std::cout<<$("11 testing stuff {:>30}").format(33)<<std::endl;
        std::cout<<$("12 testing stuff {:=30}").format(33)<<std::endl;
        std::cout<<$("13 testing stuff {:*<30}").format(33)<<std::endl;
        std::cout<<$("14 testing stuff {:*>30}").format(33)<<std::endl;
        std::cout<<$("15 testing stuff {:*=30}").format(33)<<std::endl;
        std::cout<<$("16 testing stuff {: =30!}").format("data")<<std::endl;
        std::cout<<$("17 testing stuff {: =30_}").format("DATA")<<std::endl;

        std::cout<<$("18 testing stuff {:.2}").format(300.141516)<<std::endl;
        std::cout<<$("19 testing stuff {:.3}").format(3.141516)<<std::endl;
        std::cout<<$("20 testing stuff {:.4}").format(3.141516)<<std::endl;
        std::cout<<$("21 testing stuff {:.5}").format(3.141516)<<std::endl;
        std::cout<<$("22 testing stuff {:.2f}").format(3.141516)<<std::endl;
        std::cout<<$("23 testing stuff {:.3f}").format(3.141516)<<std::endl;
        std::cout<<$("24 testing stuff {:.4f}").format(3.141516)<<std::endl;
        std::cout<<$("25 testing stuff {:.5f}").format(3.141516)<<std::endl;


        std::cout<<$("26 testing stuff {1} {0} {1} {0} {1} {} {}").format(3.141516,223.2)<<std::endl;

        std::cout<<$("27 testing stuff {1:#.32A} {0:.32X} {1:.8x} {0} {1} {} {}").format(3.141516,223)<<std::endl;





    }



};
#endif // STRINGFORMAT_H
