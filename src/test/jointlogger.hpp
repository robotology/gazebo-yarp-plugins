#ifndef JOINT_LOGGER_HPP
#define JOINT_LOGGER_HPP
#include <string>




class jointLogger{
    
public:
jointLogger(){
    joint_name="";
    logger=0;
}    

void log(double d)
{
    if (logger!=0)
    {
        fprintf(logger,"%f \n",d);
    }
}

bool initialize(std::string joint_name,const char* filename)
{
    this->joint_name=joint_name;
     char temp[100];
     sprintf(temp,"%s%s.txt",filename,joint_name.c_str());
     logger=fopen(temp,"w");
}

private:
std::string joint_name;    
FILE* logger;    
};


#endif //JOINT_LOGGER_HPP