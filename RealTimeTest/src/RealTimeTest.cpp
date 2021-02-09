/* \author Leo Wang */
// Custom function for pointcloud processing 
// using Open3D and Eigen

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
#include "processPCD.h"
#include "supportFunction.cpp"
#include "processPCD.cpp"

int main(){
    std::string folderPath = "../data/data_2/";
    int16_t fileNum = 0;
    std::vector<std::string> filePaths;
    std::tie(filePaths, fileNum) = fileSYS(folderPath);
    // for(int i = 0; i < filePaths.size(); i ++){
    //     std::cout << filePaths[i] << std::endl;
    // } // For test
    int16_t NUM = 0;
    // Loop through all files
    while(NUM != fileNum){
        //loadPCD(filePaths, NUM);
        NUM ++;
    }
    return 0;
}