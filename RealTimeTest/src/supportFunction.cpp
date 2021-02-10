#include "processPCD.h"
/*Supporting functions*/

auto fileSystem(const std::string &folderPath = "../data/data_2"){
    // Count the total number of files in the path and return the path of all files.
    std::vector<std::string> filePaths; 
    DIR *path;
    struct dirent *ep;
    char path_array[(int) folderPath.length() + 1];
    strcpy(path_array, folderPath.c_str());
    path = opendir(path_array);
    int16_t count = 0;
    if(path != NULL){
        while(ep = readdir(path)){
            if(!ep -> d_name || ep -> d_name[0] == '.')
                continue;
            filePaths.push_back(folderPath + ep -> d_name);
            count ++;
        }
        (void) closedir(path);
        std::sort(filePaths.begin(), filePaths.end());
    }
    else
        perror("Couldn't open the directory...");
    std::cout << "Found " << count << " files in folder [" << folderPath << "]."<< std::endl;
    return std::make_tuple(filePaths, count);
}

double timer_cal(const std::chrono::_V2::system_clock::time_point &start_time){
    // Should use "auto start_fast = std::chrono::system_clock::now()" to start timer.
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    return time_passed;
}