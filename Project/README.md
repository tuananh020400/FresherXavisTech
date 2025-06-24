# Build configuration
## Setup environment
- This project will be built with vc15 (visual 2019).
- Create the Environment Variables as follow:
    + OPENCV_BIN_430 'Path to your opencv4.3.0 bin directory'
    + OPENCV_INCLUDE_430 'Path to your opencv4.3.0 include directory'
    + OPENCV_LIB_430 'Path to your opencv4.3.0 lib directory'
- git add new module example:
    ```git submodule add --force --name xvtCV http://192.168.1.254:8081/xvtlib/xvtCV.git lib/xvtCV```

## Libs
- OpenCV 4.3.0.

#Test Configuration
- Go to test -> Properties -> Debugging -> Working Directory. Set it to $(SolutionDir).
- Go to Test -> Options -> Test Adapter for Google Test -> General -> Working Directory. Set it to $(SolutionDir).

#Docs
- Detail xvtBattery Manual at: "\\BUILD-SERVER-WI\Disk 2\3. AXI (Automated X-ray Inspection)\2. Battery\docs\html\index.html"