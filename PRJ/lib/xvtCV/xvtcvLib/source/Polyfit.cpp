#include "xvtCV/Polyfit.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/Utils.h"

#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <iostream>

//#define _DEBUGRANSACFIT_INFO
//#define _DEBUGRANSACFIT
//#define _DEBUGRANSACFIT_DELAY 10
namespace xvt {

Polyfit::Polyfit(std::vector<double> const& xdata, std::vector<double> const& ydata, int order)
{
    auto& eff = GetCoeff();
    LSQFit(xdata, ydata, order, eff, mRMS);
}

Polyfit::Polyfit(const std::vector<cv::Point>& points, int order)
{
    auto& eff = GetCoeff();
    LSQFit(points, order, eff, mRMS);
}

auto Polyfit::GetRMS(cv::Mat const& A, cv::Mat const& coeff, cv::Mat const& ydata)->double
{
    CHECK_INPUT(A.rows == ydata.rows && A.cols == coeff.rows && coeff.cols == 1 && ydata.cols == 1);

    cv::Mat E = A * coeff - ydata;

    cv::Mat ET;
    cv::transpose(E, ET);

    cv::Mat rms = ET * E;
    return std::sqrt(rms.at<double>(0, 0) / ydata.rows);
}

auto Polyfit::LSQFit(std::vector<double>const& xdata, std::vector<double>const& ydata, int order, std::vector<double>& coeff)->double
{
    CHECK_INPUT(order > 0  && xdata.size() >= order && xdata.size() == ydata.size());

    //coeff = new double[(int64)order + 1];
    int data_size = xdata.size();
    cv::Mat b = cv::Mat(ydata);
    cv::Mat coeffMatrix;
    cv::Mat matrixData = MakeDataMatrix(xdata, order);
    bool rtn = cv::solve(matrixData, b, coeffMatrix, cv::DecompTypes::DECOMP_QR);
    assert(rtn);

    auto tmpCoff = std::vector<double>(matrixData.cols);
    double* coeffMatrixPtr = coeffMatrix.ptr<double>();
    int rows = coeffMatrix.rows;
    int64 cols = coeffMatrix.cols;
    for (int i = 0; i < rows; ++i)
    {
        tmpCoff[i] = *(coeffMatrixPtr + i * cols);
    }
    double rms = GetRMS(matrixData, coeffMatrix, b);

    std::swap(tmpCoff, coeff);
    b.release();
    coeffMatrix.release();
    return rms;
}

auto Polyfit::LSQFit(std::vector<double>const& xdata, std::vector<double>const& ydata, int order, std::vector<double>& coeff, double& error)->bool {
    bool bResult = (xdata.size() > 0
        && order > 0
        && (xdata.size() > order) && xdata.size()==ydata.size());

    error = bResult ? LSQFit(xdata, ydata, order, coeff) : std::numeric_limits<double>::infinity();

    return bResult;
}

auto Polyfit::LSQFit(const std::vector<cv::Point>& points, int order, std::vector<double>& coeff)->double
{
    int n =static_cast<int>(points.size());
    std::vector<double> x = std::vector<double>(n);
    std::vector<double> y = std::vector<double>(n);
    for (int i = 0; i < n; i++)
    {
        x[i] = points[i].x;
        y[i] = points[i].y;
    }
    double error = LSQFit(x, y, order, coeff);

    return error;
}

bool Polyfit::LSQFit(const std::vector<cv::Point>& points, int order, std::vector<double>& coeff, double& error)
{
    bool bResult = (points.size() > 0
        && order > 0
        && (points.size() > order));

    error = bResult ? LSQFit( points, order, coeff) : std::numeric_limits<double>::infinity();
    
    return bResult;
}

#pragma region not verify functions
auto Polyfit::GetMinMax(VecDouble const& coeff
    , int min_max
    , float xStart, float xEnd
    , double& xMax, double& maxValue
    , double e
    , int reso
)->double
{
    int const order = coeff.size() - 1;

    VecDouble x = VecDouble(reso, 0.0);
    cv::Mat yMatrix;

    if (min_max <= 0)
    {
        //Find the minimum
        min_max = -1;
    }
    else
    {
        //Find the maximum
        min_max = 1;
    }

    double epsilon = 1000;
    int t = 3;
    do
    {
        double stepSize = (xEnd - xStart) / reso;

        for (int i = 0; i < reso; i++)
        {
            x[i] = xStart + i * stepSize;
        }

        auto matrixData = MakeDataMatrix(x, order);
        cv::Mat const coeffMatrix = cv::Mat(coeff, false);
        yMatrix = matrixData * coeffMatrix;
        double* yMatrixPtr = yMatrix.ptr<double>();

        xMax = 0;
        maxValue = 0;
        int index = 0;
        for (int i = 0; i < reso; i++)
        {
            if (min_max * (*(yMatrixPtr + i) - maxValue) > 0)
            {
                xMax = x[i];
                maxValue = *(yMatrixPtr + i);
                index = i;
            }
        }
        xStart = static_cast<float>(xMax - stepSize);
        xEnd = static_cast<float>(xMax + stepSize);
        epsilon = abs(*(yMatrixPtr + index - 1) / 2 + *(yMatrixPtr + index + 1) / 2 - maxValue);

        yMatrixPtr = nullptr;
        delete yMatrixPtr;
        t--;
    } while (epsilon > e && t > 0);

    yMatrix.release();
    return epsilon;
}

//using the AMSGrad (2018) algorithm
auto Polyfit::GetMaxMinDescent(VecDouble const& coeff
    , int min_max
    , float xStart
    , double& xMax, double& maxValue
    , float alpha, int maxIteritor
    , double e
)->double
{
    int const order = coeff.size() - 1;
    double beta1 = 0.9;
    double beta2 = 0.999;
    double epsilon = 1.e-7;
    double w = xStart;
    double V = 0;
    double S = 0;
    double SHat = 0;

    double y = f(coeff, w);
    double yold = 0;
    double xold = 0;
    double ef = 10000;

    if (min_max <= 0)
    {
        //Find the minimum
        min_max = -1;
    }
    else
    {
        //Find the maximum
        min_max = 1;
    }

    VecDouble diffCoeff;

    for (int i = 0; i < maxIteritor && ef > e; i++)
    {
        //Caculation V
        VecDouble diffCoeff = GetFirstDerivativeCoeff(coeff);
        double fw = f(diffCoeff, w);
        V = beta1 * V + (1 - beta1) * fw;

        //Caculation S
        S = beta2 * S + (1 - beta2) * fw * fw;

        //update SHat
        SHat = (SHat > S) ? SHat : S;

        xold = w;
        w = w + min_max * (alpha / (sqrt(SHat) + epsilon)) * V;
        yold = y;
        y = f(coeff, w);
        ef = sqrt((w - xold) * (w - xold) + (y - yold) * (y - yold));
        //std::cout << "i: "<<i << ", E:"<<ef<<",w:" << w << ", SHat:"<< SHat <<", V:"<< V<<", S: "<< S<< std::endl;
    }

    xMax = w;
    maxValue = y;
    return ef;
}

VecDouble Polyfit::GetFirstDerivativeCoeff(VecDouble const& coeff)
{
    VecDouble newCoeff;
    if (coeff.size() > 1)
    {
        for (int i = 1, size = coeff.size(); i < size; i++)
        {
            newCoeff.push_back((i)*coeff[i]);
        }
    }

    return newCoeff;
}
#pragma endregion

//A is a matrix [data_size x order]. Each collumn is [1, x, x^2, ..., x^n]
auto Polynomial::MakeDataMatrix(std::vector<double> const& xdata, int order)->cv::Mat
{
    cv::Mat A = cv::Mat();
    if (xdata.size() > 1 && order > 0)
    {
        const int cols = (order + 1);
        const int rows = xdata.size();
        A = cv::Mat(rows, cols, CV_64FC1);

        A.col(0) = 1.0;
        for (int i = 0; i < rows; i++)
        {
            double* rptr = A.ptr<double>(i);
            for (int c = 1; c < cols; c++)
            {
                auto ptr = (rptr + c);
                *ptr = (*(ptr - 1)) * xdata[i];
            }
        }
    }

    return A;
}

auto Polynomial::GetRMSE(double x, double y) const -> double
{
    return abs(y - f(x));
}

auto Polynomial::GetRMSE(cv::Point2d p) const -> double
{
    return abs(p.y - f(p.x));
}

auto Polynomial::GetRMSE(VecPoint p) const -> double
{
    if (p.empty())return 0.0;

    int n = static_cast<int>(p.size());
    std::vector<double> x = std::vector<double>(n);
    std::vector<double> y = std::vector<double>(n);
    for (int i = 0; i < n; i++)
    {
        x[i] = p[i].x;
        y[i] = p[i].y;
    }
    return GetRMSE(x, y);
}

auto Polynomial::GetRMSE(VecDouble const& x, VecDouble const& y) const -> double
{
    if (x.empty() || y.empty())return 0.0;

    auto e = cv::Mat(mCoeff);
    auto y2 = cv::Mat(y);
    auto A = MakeDataMatrix(x, GetOrder());

    return GetRMSE(A, e, y2);
}

auto Polynomial::f(VecDouble const& coeff, double x)->double
{
    long double rtn = 0;
    if (!coeff.empty())
    {
        auto size = coeff.size();
        rtn = coeff[0];
        long double xx = 1;

        for (int i = 1; i < size; i++)
        {
            xx *= x;
            rtn += (xx * coeff[i]);
        }
    }
    return rtn;
}

void Polynomial::f(VecDouble const& coeff, VecDouble const& x, VecDouble& y)
{
    if(!coeff.empty())
    {
        auto size = x.size();

        cv::Mat yMatrix = cv::Mat(size, 1, CV_64F);
        cv::Mat coeffMatrix = cv::Mat(coeff);

        auto a = MakeDataMatrix(x, coeff.size() - 1);
        yMatrix = a * coeffMatrix;
        y = yMatrix;
    }
    return;
}

auto Polynomial::GetRMSE(cv::Mat const& A, cv::Mat const& coeff, cv::Mat const& ydata)->double
{
    if (!(A.rows == ydata.rows && A.cols == coeff.rows && coeff.cols == 1 && ydata.cols == 1))
        return std::numeric_limits<double>::infinity();

    cv::Mat E = A * coeff - ydata;

    cv::Mat ET;
    cv::transpose(E, ET);

    cv::Mat rms = ET * E;
    return std::sqrt(rms.at<double>(0, 0) / ydata.rows);
}

void PolynomialResult::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty()|| mPoints.empty()) return;
    if (xvt::ConvertRGB(img, img, false)) return;

    int n = static_cast<int>(mPoints.size());
    std::vector<double> x = std::vector<double>(n);
    std::vector<double> y = std::vector<double>(n);
    for (int i = 0; i < n; i++)
    {
        x[i] = mPoints[i].x;
        y[i] = mPoints[i].y;
    }

    std::vector<double> y2 = f(x);
    xvt::Drawing drawTool;
   
    drawTool.color = COLOR_CV_BLUE;
    drawTool.Plot(img, x, y);
    drawTool.color = COLOR_CV_YELLOW;
    drawTool.Plot(img, x, y2, false);

    //InspectionResult::DrawResult(img, offSetPoint, pen);
}

auto PolynomialDetector::FitLSQE(xvt::VecPoint const& points) const -> ModelResultType
{
    auto res = ModelResultType(mOrder);
    if (points.empty())return res;

    int n = static_cast<int>(points.size());
    std::vector<double> x = std::vector<double>(n);
    std::vector<double> y = std::vector<double>(n);
    for (int i = 0; i < n; i++)
    {
        x[i] = points[i].x;
        y[i] = points[i].y;
    }

    res = FitLSQE(x, y, mOrder);

    return res;
}

auto PolynomialDetector::FitLSQE(std::vector<double> const& xdata, std::vector<double> const& ydata, int order) -> ModelResultType
{
    auto res = ModelResultType(order);
    if (!(order > 0 && xdata.size() >= order && xdata.size() == ydata.size())) return res;

    //coeff = new double[(int64)order + 1];
    int data_size = xdata.size();
    cv::Mat b = cv::Mat(ydata);
    cv::Mat coeffMatrix;
    cv::Mat matrixData = res.MakeDataMatrix(xdata, order);
    bool rtn = cv::solve(matrixData, b, coeffMatrix, cv::DecompTypes::DECOMP_QR);

    if(rtn)
    {
        auto tmpCoff = std::vector<double>(matrixData.cols);
        double* coeffMatrixPtr = coeffMatrix.ptr<double>();
        int rows = coeffMatrix.rows;
        int64 cols = coeffMatrix.cols;
        for (int i = 0; i < rows; ++i)
        {
            tmpCoff[i] = *(coeffMatrixPtr + i * cols);
        }

        //double rms = res.GetRMSE(matrixData, coeffMatrix, b);

        res = ModelResultType(std::move(tmpCoff));
        res(EResult::OK, "");
        b.release();
        coeffMatrix.release();
    }
    else
    {
        res(EResult::ER, "Polyfit Can not find the model");
    }

    return res;
}

auto PolynomialDetector::IsValidModel(ModelResultType const& res) const -> bool
{
    return true;
}

}