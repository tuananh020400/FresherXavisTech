# Building the project {#Building}
Xavis Tech Battery library for Battery inspection using [OpenCV](https://opencv.org/), [xvtCV]

## Dependency Libs
This lib using bellow libraries.
|Lib   |Version|Description|
|------|-------|-----------|
|OpenCV|4.8.0  |[Computer vision library](https://opencv.org/)|
|mxml1 |3.1.0  |[Mini-XML](https://www.msweet.org/mxml/) is a tiny XML library that you can use to read and write XML and XML-like data files|
|zlib  |1.2.11 |[Compression and Decompression library](https://www.zlib.net/)|
|xvtCV |1.0.0  |Xavis Tech common library |

## Setup environment
- This project will be built with vc15 (visual 2019).
- Create the Environment Variables as follow:
    + **OPENCV_480** 'Path to your opencv4.8.0 directory'
    + **MXML** 'Path to your mxml1 directory'
    + **ZLIB** 'Path to your zlib directory'
    + **xvtCV** 'Path to your xvtCV directory'

## Test Configuration
- Go to **Test** -> Properties -> Debugging -> Working Directory. Set it to $(SolutionDir).
- Go to **Test** -> Options -> Test Adapter for Google Test -> General -> Working Directory. Set it to $(SolutionDir).
