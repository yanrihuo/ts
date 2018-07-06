#ifndef PARKINGLOT_PARAMETERREADER_H
#define PARKINGLOT_PARAMETERREADER_H

#include <string>
#include <iostream>
#include <map>
#include <fstream>

const double pi = 3.1415926535898;

using namespace std;

class parameterReader
{
private:
    parameterReader( string filename = "../data/parameters.txt" );

public:
    static parameterReader *GetInstance();
    string getData( string key );
    ~parameterReader();

private:
    map<string, string> data;
    static parameterReader *instance;

};


#endif //PARKINGLOT_PARAMETERREADER_H
