#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>
using namespace std;

void readFile();

int main()
{

}

void readFile()
{
    ifstream reader("roadNet-TX.txt");
	string newLine = "";


    //skip the first 4 lines
    for(int i = 0; i < 4; i++)
    {
        getline(reader, newLine);
    }

    int from, to, space;
    while(reader.is_open())
    {
        getline(reader, newLine);
        space = newLine.find(' ');
        from = stoi(newLine.substr(0, space));
        to = stoi(newLine.substr(space+1));



    }

}