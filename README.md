# a project of VTD
## env
1. ubuntu20.04
2. ros
3. matplotlibcpp
4. yaml-cpp
5. osqp (in the ./thirdparty)
6. osqp-eigen(in the ./thirdparty)
7. eigen
8. VTD2022
## lib
1. VtdRosConnector

upzip the ./other/VtdRosConnector.zip， as the ros package

the workspace will be like 
```
ros_ws
├── build
├── devel
└── src
    ├── CMakeLists.txt
    ├── test_ctrl
    └── VtdRosConnector
```
then make it
