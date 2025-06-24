//#include "stdafx.h"
#include <iostream>
#include "xvtCV/KuwaharaFilter.h"

namespace xvt {
namespace filter {
Regions::Regions(int _kernel) : kernel(_kernel)
{
    for (int i = 0; i < 4; i++)
    {
        Area[i] = new int[kernel * static_cast<int64>(kernel)];
        Size[i] = 0;
        Sum[i] = 0;
        Var[i] = 0.0;
    }
}

Regions::Regions() : kernel{ 0 }
{
    for (int i = 0; i < 4; i++)
    {
        Area[i] = NULL;
        Size[i] = 0;
        Sum[i] = 0;
        Var[i] = 0.0;
    }
}

//Update data, increase the size of the area, update the sum
void Regions::SendData(int area, int data)
{
    Area[area][Size[area]] = data;
    Sum[area] += data;
    Size[area]++;
}

//Calculate the variance of each area
double Regions::GetAreaVariance(int area)
{
    double __mean = static_cast<float>(Sum[area]) / Size[area];
    double temp = 0;
    for (int i = 0; i < Size[area]; i++)
    {
        double a = Area[area][i] - __mean;
        temp += (a * a);
    }
    if (Size[area] == 1) return 1.7e38; //If there is only one pixel inside the region then return the maximum of double
                                        //So that with this big number, the region will never be considered in the below GetMinVariance()
    return sqrt(temp / (Size[area] - 1.0));
}

//Call the above function to calc the variances of all 4 areas
void Regions::CalculateVariance()
{
    for (int i = 0; i < 4; i++)
    {
        Var[i] = GetAreaVariance(i);
    }
}

//Find out which regions has the least variance
int Regions::GetMinVariance()
{
    CalculateVariance();
    int i = 0;
    double __var = Var[0];
    if (__var > Var[1]) { __var = Var[1]; i = 1; }
    if (__var > Var[2]) { __var = Var[2]; i = 2; }
    if (__var > Var[3]) { __var = Var[3]; i = 3; }
    return i;
}

//Return the mean of that regions
uchar Regions::GetResult()
{
    int i = GetMinVariance();
    return cv::saturate_cast<uchar> ((double)(Sum[i] * 1.0 / Size[i]));
}

Regions::Regions(const Regions& src) :Regions()
{
    *this = src;
}

Regions& Regions::operator=(Regions const& src)
{
    if (&src != this)
    {
        this->kernel = src.kernel;
        //std::copy(std::begin(src.Area), std::end(src.Area), std::begin(this->Area));
        for (int i = 0; i < 4; i++)
        {
            if (this->Area[i] != NULL)
                delete[] this->Area[i];
            this->Area[i] = new int[kernel * static_cast<int64>(kernel)];
            std::memcpy(this->Area[i], src.Area[i], kernel * static_cast<int64>(kernel) * sizeof(int));
        }
        std::copy(std::begin(src.Size), std::end(src.Size), std::begin(this->Size));
        std::copy(std::begin(src.Sum), std::end(src.Sum), std::begin(this->Sum));
        std::copy(std::begin(src.Var), std::end(src.Var), std::begin(this->Var));
    }

    return *this;
}

Regions::~Regions()
{
    for (int iArea = 0; iArea < 4; iArea++)
    {
        if (this->Area[iArea] != NULL)
        {
            delete[] this->Area[iArea];
            this->Area[iArea] = NULL;
        }
    }
}

Regions Kuwahara::GetRegions(int x, int y)
{
    Regions regions(kernel);

    uchar* data = image.data;

    //Update data for each region, pixels that are outside the image's boundary will be ignored.

    //Area 1 (upper left)
    for (int j = (y - pad >= 0) ? y - pad : 0; j >= 0 && j <= y && j < hei; j++)
        for (int i = ((x - pad >= 0) ? x - pad : 0); i >= 0 && i <= x && i < wid; i++)
        {
            regions.SendData(1, data[(j * wid) + i]);
        }
    //Area 2 (upper right)
    for (int j = (y - pad >= 0) ? y - pad : 0; j <= y && j < hei; j++)
        for (int i = x; i <= x + pad && i < wid; i++)
        {
            regions.SendData(2, data[(j * wid) + i]);
        }
    //Area 3 (bottom left)
    for (int j = y; j <= y + pad && j < hei; j++)
        for (int i = ((x - pad >= 0) ? x - pad : 0); i <= x && i < wid; i++)
        {
            regions.SendData(3, data[(j * wid) + i]);
        }
    //Area 0 (bottom right)
    for (int j = y; j <= y + pad && j < hei; j++)
        for (int i = x; i <= x + pad && i < wid; i++)
        {
            regions.SendData(0, data[(j * wid) + i]);
        }
    return regions;
}

//Constructor
Kuwahara::Kuwahara(const cv::Mat& _image, int _kernel) : kernel(_kernel)
{
    image = _image.clone();
    wid = image.cols; hei = image.rows;
    pad = kernel - 1;
}

//Create new image and replace its pixels by the results of Kuwahara filter on the original pixels
cv::Mat Kuwahara::Apply()
{
    cv::Mat temp;
    temp.create(image.size(), CV_8U);
    uchar* data = temp.data;

    for (int j = 0; j < hei; j++)
    {
        for (int i = 0; i < wid; i++)
            data[j * wid + i] = GetRegions(i, j).GetResult();
    }
    return temp;
}
}
}