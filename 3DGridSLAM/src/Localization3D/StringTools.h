/** 
*@file //TODO 
*  @date 9 Oct 2013 
*  @version 1.0 
*  @brief //TODO. 

This file is part of Luis Garrote Phd work. 
Copyright (C) 2012-2014 Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt) 
All rights reserved.

*
*  @author Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt)
*  @bug No know bugs.
*/ 
#ifndef STRINGTOOLS_H
#define STRINGTOOLS_H


#include <algorithm>
#include <sstream>
#include <map>
#include <iostream>
#include <iomanip>
#include <vector>

namespace Garrote {
namespace String {


class StringTools{
public:
    template <typename T>
    static std::string toString(T data){

        std::ostringstream ss;
        ss <<std::setprecision(15)<< data;

        return ss.str();

    }
    //    static inline std::string &ltrim(std::string &s) {
    //            s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    //            return s;
    //    }

    //    // trim from end
    //    static inline std::string &rtrim(std::string &s) {
    //            s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    //            return s;
    //    }

    //    // trim from both ends
    //    static inline std::string &trim(std::string &s) {
    //            return ltrim(rtrim(s));
    //    }




    // trim from start
    static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int c) {return !std::isspace(c);}));
        return s;
    }

    // trim from end
    static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int c) {return !std::isspace(c);}).base(), s.end());
        return s;
    }

    // trim from both ends
    static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
    }

    static void toLower(std::string &data)
    {
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);
    }
    static void toUpper(std::string &data)
    {
        std::transform(data.begin(), data.end(), data.begin(), ::toupper);
    }


    static std::string returntoLower(std::string &x)
    {
        std::string data=x;
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);
        return data;
    }
    static std::string returntoUpper(std::string &x)
    {
        std::string data=x;
        std::transform(data.begin(), data.end(), data.begin(), ::toupper);
        return data;
    }


    static void removeFromString(std::string &x,char c)
    {
        x.erase( std::remove( x.begin(), x.end(), c ), x.end() ) ;
    }
    static std::string stringPadding(std::string original, int charCount, char ch=' ' )
    {

        std::string copy=original;
        copy.resize( charCount, ch );
        return copy;
    }


    static std::vector<std::string> breakIntoLines(std::string &data,char c){

        std::vector<std::string> Lines;

        std::string d;
        for(unsigned int i=0;i<data.size();i++){

            if(data[i]==c){

                if(d.length()>0){
                    Lines.push_back(d);
                    d.clear();
                }
            }else{
                d=d+data[i];
            }

        }

        if(d.size()>0){
            Lines.push_back(d);
        }

        return Lines;



    }


    static std::vector<std::string> split(std::string str, std::string delim, bool ignoreEmptyTokens = true) {
        std::vector<std::string> strings;
        size_t pos = 0;
        std::string token;

        while ((pos = str.find(delim)) != std::string::npos) {
            token = str.substr(0, pos);
            if (token == "") {
                if (!ignoreEmptyTokens) {
                    strings.push_back(token);
                }
            }
            else {
                strings.push_back(token);
            }
            str.erase(0, pos + delim.length());
        }

        if (str.length() > 0) {
            strings.push_back(str);
        }

        return strings;
    }


    static std::map<std::string,std::string> breakToMapIgnoringInVariable(std::string &data,char inSeparator,char separator,std::vector<char> tout){

        std::map<std::string,std::string> varmap;

        std::string s1;
        std::string s2;
        bool var=false;

        for(unsigned int i=0;i<data.size();i++){

            if(data[i]==inSeparator){

                var=true;


            }else if(data[i]==separator){


                if(var==true && s2.length()>0 && s1.length()>0){
                    varmap[s1]=s2;
                    var=false;

                    std::cout<<" var :#"<<s1<<"#   #"<<s2<<"#"<<std::endl;
                    s1.clear();
                    s2.clear();
                }else{

                    var=false;
                    s1.clear();
                    s2.clear();
                }

            }else{

                if(var==true){
                    s2=s2+(data[i]);
                }else{

                    bool ignore=false;
                    for(unsigned int g=0;g<tout.size();g++){
                        if(tout[g]==data[i]){

                            ignore=true;
                            break;
                        }

                    }

                    if(ignore==false){
                        s1=s1+(data[i]);
                    }
                }

            }

        }

        if(s2.length()>0 && s1.length()>0){

            if(var==true && s2.length()>0 && s1.length()>0){
                varmap[s1]=s2;
                std::cout<<" var :#"<<s1<<"#   #"<<s2<<"#"<<std::endl;

            }
        }

        return varmap;



    }





    static std::map<std::string,std::string> breakToMap(std::string &data,char inSeparator,char separator){

        std::map<std::string,std::string> varmap;

        std::string s1;
        std::string s2;
        bool var=false;

        for(unsigned int i=0;i<data.size();i++){

            if(data[i]==inSeparator){

                var=true;


            }else if(data[i]==separator){


                if(var==true && s2.length()>0 && s1.length()>0){
                    varmap[s1]=s2;
                    var=false;
                    s1.clear();
                    s2.clear();
                }else{

                    var=false;
                    s1.clear();
                    s2.clear();
                }

            }else{

                if(var==true){
                    s2=s2+(data[i]);
                }else{
                    s1=s1+(data[i]);
                }

            }

        }

        if(s2.length()>0 && s1.length()>0){

            if(var==true){
                varmap[s1]=s2;
            }
        }

        return varmap;



    }


    static void replace(std::string& str, const std::string& from, const std::string& to) {
        if(from.empty())
            return;
        size_t start_pos = 0;
        while((start_pos = str.find(from, start_pos)) != std::string::npos) {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
    }


    static void removeChars(std::string &str){

        size_t i = 0;
        size_t len = str.length();
        while(i < len){
            if (isdigit (str[i]) || str[i] == 'e' || str[i] == '.' ||str[i] == ',' ||str[i] == '-' ||str[i] == '+'){
                i++;
            }else
            {
                str.erase(i,1);
                len--;
            }
        }
    }


    static void to_bin(unsigned char c, char *out) {
        *(unsigned long long*)out = 3472328296227680304ULL +
                (((c * 9241421688590303745ULL) / 128) & 72340172838076673ULL);
    }


    //    /**
    //    * @param terms values to analyze
    //    * @return a map containing unique
    //    * terms and their frequency
    //    */
    //   public static Map<String, Integer> getTermFrequencyMap(String[] terms) {
    //       Map<String, Integer> termFrequencyMap = new HashMap<>();
    //       for (String term : terms) {
    //           Integer n = termFrequencyMap.get(term);
    //           n = (n == null) ? 1 : ++n;
    //           termFrequencyMap.put(term, n);
    //       }
    //       return termFrequencyMap;
    //   }

    //   /**
    //    * @param text1
    //    * @param text2
    //    * @return cosine similarity of text1 and text2
    //    */
    //   public static double cosineSimilarity(String text1, String text2) {
    //       //Get vectors
    //       Map<String, Integer> a = getTermFrequencyMap(text1.split("\\W+"));
    //       Map<String, Integer> b = getTermFrequencyMap(text2.split("\\W+"));

    //       //Get unique words from both sequences
    //       HashSet<String> intersection = new HashSet<>(a.keySet());
    //       intersection.retainAll(b.keySet());

    //       double dotProduct = 0, magnitudeA = 0, magnitudeB = 0;

    //       //Calculate dot product
    //       for (String item : intersection) {
    //           dotProduct += a.get(item) * b.get(item);
    //       }

    //       //Calculate magnitude a
    //       for (String k : a.keySet()) {
    //           magnitudeA += Math.pow(a.get(k), 2);
    //       }

    //       //Calculate magnitude b
    //       for (String k : b.keySet()) {
    //           magnitudeB += Math.pow(b.get(k), 2);
    //       }

    //       //return cosine similarity
    //       return dotProduct / Math.sqrt(magnitudeA * magnitudeB);
    //   }

};

}
}


#endif // STRINGTOOLS_H
