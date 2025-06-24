# How to Drawing a Signal {#Drawing}

**xvtCV** supports some classes to help draw some basic information
in xvt::Drawing, xvt::CVPen and some predefined color in "ColorDefine.h".

## Introduction
This tutorial demonstrates two methods for extracting edge information from a signal in an image using OpenCV. 
The first method uses the Sobel operator combined with a Gaussian kernel, while the second method involves manually smoothing the signal and then differentiating it.
Then drawing those signals onto the image.

## Prerequisites
Before you begin, you should have a basic understanding of C++ and OpenCV. 
Ensure that OpenCV is installed and configured in your development environment.

## Step-by-Step Guide
1. **Reading the Signal Image**  
    First, the image is loaded in grayscale mode from the specified file path (Ver2W2B.jpg in the line_image_test directory). 
    The image is then reduced to a single row by averaging across all rows. This reduction simplifies the signal to a one-dimensional array, which is easier to process.
    ```cplusplus
    cv::Mat im = cv::imread("line_image_test/Ver2W2B.jpg", 0);
    if (im.empty()) return;
    
    // Reduce image to single row
    std::vector<float> reduce;
    cv::reduce(im, reduce, 0, cv::ReduceTypes::REDUCE_AVG);
    ```
2. **Method 1: Edge Detection Using the Sobel Operator**  
    **Creating the Gaussian Kernel**  
    A Gaussian kernel is first created. This kernel will be used to smooth the signal before applying the Sobel operator.
    ```cplusplus
    cv::Mat gauKernel = cv::getGaussianKernel(9, -1, CV_32F);
    ```
    **Deriving the Sobel Kernel**  
    The Sobel kernel is derived by applying the Sobel operator to the Gaussian kernel. 
    This operator is used to detect edges by calculating the gradient of the image intensity.
    ```cplusplus
    cv::Mat sobKernelX;
    cv::Sobel(-gauKernel, sobKernelX, CV_32F, 0, 1, 3);
    ```
    **Applying the Sobel Operator**  
    The Sobel kernel is then applied to the reduced signal to obtain the edge information.
    ```cplusplus
    cv::Mat sobelImg;
    cv::filter2D(reduce, sobelImg, CV_32F, sobKernelX.t());
    ```
3. **Method 2: Manual Smoothing and Differentiation**  
    In this method, the signal is first smoothed using a Gaussian blur. 
    The smoothed signal is then differentiated using a custom kernel to obtain the edge information.
    **Smoothing the Signal**  
    The Gaussian blur is applied to the reduced signal to reduce noise and smooth out any irregularities.
    ```cplusplus
    cv::Mat im2 = cv::Mat_<float>(reduce);
    cv::GaussianBlur(reduce, reduce, cv::Size(5, 1), -1);
    ```
    **Differentiating the Signal**  
    A custom kernel is used to differentiate the smoothed signal, which helps in detecting edges.  
    ```cplusplus
    std::vector<float> kernel = { -2, 0 , 2 };
    std::vector<float> diff;
    cv::filter2D(reduce, diff, -1, kernel);
    ```
4. **Visualizing the Results**  
    The final step is to visualize the original signal, and the edge information obtained using both methods. 
    The xvt::Drawing class is used for this purpose.
    Drawing the Original Signal  
    The original smoothed signal is plotted in yellow on the image.
    ```cplusplus
    xvt::Drawing drawtool;
    drawtool.yTopOrigin = false;
    drawtool.Plot(im, xList, reduce, true;
    ```
    **Drawing the Edge Information from Method 1 (Sobel Operator)**  
    The edge information obtained using the Sobel operator is plotted in red.  
    ```cplusplus
    drawtool.color = cv::Scalar(0, 0, 255);
    drawtool.Plot(im, xList, diff, true);
    ```
    **Drawing the Edge Information from Method 2 (Manual Differentiation)**  
    The edge information obtained from the manual differentiation method is plotted in green.  
    ```cplusplus
    drawtool.color = cv::Scalar(0, 255, 0);
    drawtool.Plot(im, xList, std::vector<float>(sobelImg), true);
    ```
    
    Original image<img src="Ver2W2B.png" alt="Ver2W2B Image" width="400px"/> result image <img src="Ver2W2B_edge.png" alt="Ver2W2B_edge Image" width="400px"/>  
    \image latex Ver2W2B.png "Ver2W2B Image" \image latex Ver2W2B_edge.png "Ver2W2B_edge Image"  

## Code Example
This example code is implemented in the `DrawingTest.cpp` in **test project**.

@snippet test/DrawingTest.cpp xvtCV Drawing Example

## More information
See xvt::Drawing, xvt::CVPen, ColorDefine.h, DrawingTest.cpp, FindPeakTest.cpp