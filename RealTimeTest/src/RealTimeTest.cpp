// [Leo Wang 2021.02.07]
// [E-mail: liangyu@student.chalmers.se]
#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>
#include <dirent.h>
#include <Eigen/Dense>
#include "open3d/Open3D.h"

auto fileCount(const std::string &filePath){
    DIR *path;
    int count = 0;
    struct dirent *ep;
    char path_array[(int) filePath.length() + 1];
    strcpy(path_array, filePath.c_str());
    path = opendir(path_array);
    if(path != NULL){
        while(ep = readdir(path))
            count ++;
        (void) closedir(path);
    }
    else
        perror("Couldn't open the directory...");
    count -= 2;
    std::cout << "File number in " << filePath << 
                " :" << count << std::endl;
    return count;
}

int main(){
    std::string filePath = "../data/data_2";
    auto fileNum = fileCount(filePath);
    return 0;
}