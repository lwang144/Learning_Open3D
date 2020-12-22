# KDTree
The examples included above are based on the tutorial from Open3D using both Python and C++. Hope this document can help you! 

Open3D uses FLANN to build KDTrees for fast retrieval of nearest neighbors. [FLANN Introduction](https://www.cs.ubc.ca/research/flann/uploads/FLANN/flann_manual-1.8.4.pdf)

[C++ Examples]

[Python Examples]

**RUN C++ Example:** 
```
  mkdir build
  cd build
  cmake ..
  make
  cd ../bin
  ./KDTree ../../test_data/Feature/cloud_bin_0.pcd
```
