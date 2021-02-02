#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

int main(){
    auto volume = open3d::pipelines::integration::ScalableTSDFVolume(4.0/512.0, 0.04, 
                            open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
    return 0;
}