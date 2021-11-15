#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>


void readFile();

int main()
{

}

void readFile()
{
    std::ifstream reader("roadNet-TX.txt");
	std::string newLine = "";


    //skip the first 4 lines
    for(int i = 0; i < 4; i++)
    {
        std::getline(reader, newLine);
    }

    int from, to, space;
    while(reader.is_open())
    {
        std::getline(reader, newLine);
        space = newLine.find(' ');
        from = std::stoi(newLine.substr(0, space));
        to = std::stoi(newLine.substr(space+1));



    }

}