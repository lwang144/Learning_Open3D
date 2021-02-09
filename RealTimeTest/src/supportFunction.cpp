#include "processPCD.h"

int16_t fileCount(const std::string &filePath){
    // Count the total number of files in the path
    DIR *path;
    int16_t count = 0;
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
    std::cout << "File number in [" << filePath << "] : " << count << std::endl;
    return count;
}

double timer_cal(const std::chrono::_V2::system_clock::time_point &start_time){
    // Should use "auto start_fast = std::chrono::system_clock::now()" to start timer.
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    return time_passed;
}