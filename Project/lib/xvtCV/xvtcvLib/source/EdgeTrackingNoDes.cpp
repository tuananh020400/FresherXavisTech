#include "xvtCV/EdgeTrackingNoDes.h"
#include "opencv2/imgproc.hpp"
#include <map>

namespace xvt {

constexpr double DEG2RAD = CV_PI / 180.0;

cv::Mat get_bin_img_adaptive_threshold(const cv::Mat& src)
{
    cv::Mat dst;
    cv::adaptiveThreshold(~src, dst, 255, 0, 0, 2 * 5 + 1, 37 - 40.0);
    return dst;
}

std::vector<cv::Point2d> get_arr_direction_dppoint(cv::Point2d start_point, int angle, int num_of_next_point)
{
    cv::Point2d a;
    double angle_rad = angle * DEG2RAD;
    double x_max = num_of_next_point * cos(angle_rad);
    double y_max = num_of_next_point * sin(angle_rad);
    double tanAlpha = tan(angle_rad);
    double cotanAlpha = tan(CV_PI / 2 - angle_rad);

    std::vector< cv::Point2d> rs;

    if (abs(x_max) > abs(y_max))
    {
        if (x_max > 0)
        {
            for (int i = 0; i <= x_max; i++)
            {
                double j = ((double)i) * tanAlpha;
                a.x = start_point.x + i;
                a.y = start_point.y + j;
                rs.push_back(a);
            }
        }
        else
        {
            for (int i = 0; i >= x_max; i--)
            {
                double j = ((double)i) * tanAlpha;
                a.x = start_point.x + i;
                a.y = start_point.y + j;
                rs.push_back(a);
            }
        }

    }
    else
    {
        if (y_max > 0)
        {
            for (int j = 0; j <= y_max; j++)
            {
                double i = ((double)j) * cotanAlpha;
                a.x = start_point.x + i;
                a.y = start_point.y + j;
                rs.push_back(a);
            }
        }
        else
        {
            for (int j = 0; j >= y_max; j--)
            {
                double i = ((double)j) * cotanAlpha;
                a.x = start_point.x + i;
                a.y = start_point.y + j;
                rs.push_back(a);
            }
        }

    }

    return rs;
}

uchar get_intensity_double_point_image(const cv::Mat& src, double x, double y)
{

    if (x < 0 || x >= (int64)src.cols - 1)
    {
        return 0;
    }

    if (y < 0 || y >= (int64)src.rows - 1)
    {
        return 0;
    }

    int x0 = (int)floor(x);
    int x1 = (int)ceil(x);

    int y0 = (int)floor(y);
    int y1 = (int)ceil(y);

    double dx = abs(x - x0);
    double dy = abs(y - y0);
    const uchar* rowY0 = src.ptr<uchar>(y0);
    const uchar* rowY1 = src.ptr<uchar>(y1);
    uchar A = *(rowY0 + x0);
    uchar B = *(rowY0 + x1);
    uchar C = *(rowY1 + x1);
    uchar D = *(rowY1 + x0);

    /*uchar A = src.at<uchar>(y0, x0);
    uchar B = src.at<uchar>(y0, x1);
    uchar C = src.at<uchar>(y1, x1);
    uchar D = src.at<uchar>(y1, x0);*/

    double E = A * (1 - dy) + D * dy;
    double F = B * (1 - dy) + C * dy;
    double G = E * (1 - dx) + F * dx;
    return (uchar)G;
}

uchar get_intensity_cv_point_image(const cv::Mat& src, cv::Point dp)
{
    int x = dp.x;
    int y = dp.y;

    if (x < 0 || x >= src.cols)
    {
        return 0;
    }

    if (y < 0 || y >= src.rows)
    {
        return 0;
    }

    uchar G = src.at<uchar>(dp);

    return (uchar)G;

}

double calc_value_direction_edge(const std::vector<cv::Point2d>& direction, const cv::Mat& src, int angle_degree)
{

    double sum = 0;
    for (size_t i = 0; i < direction.size(); i++)
    {
        sum += get_intensity_double_point_image(src, direction[i].x, direction[i].y);
    }
    return sum;
}

double GetDirectionValue(const cv::Point2d& start_point, const cv::Mat& img, int angle, int num_of_next_point)
{
    double angle_rad = angle * CV_PI / 180;
    double x_i = cos(angle_rad);
    double y_i = sin(angle_rad);
    double x;
    double y;
    double sum = 0;

    for (int j = 0; j <= num_of_next_point; j++)
    {
        x = start_point.x + j * x_i;
        y = start_point.y + j * y_i;

        sum += get_intensity_double_point_image(img, x, y);
    }
    return sum;
}

double GetDirectionValue(const cv::Mat& img, const cv::Point2f& start_point, const cv::Point2f& refVector, int angle_degree, int num_of_next_point, cv::Point2f& newVector)
{
    //cv::Point2f newVector;
    double angle_rad = angle_degree * DEG2RAD;

    newVector.x = static_cast<float>(refVector.x * cos(angle_rad) - refVector.y * sin(angle_rad));
    newVector.y = static_cast<float>(refVector.x * sin(angle_rad) + refVector.y * cos(angle_rad));
    double sum = 0;
    cv::Point2f a;
    for (int j = 0; j <= num_of_next_point; j++)
    {
        a = start_point + j * newVector;

        sum += get_intensity_double_point_image(img, a.x, a.y);
    }
    return sum;
}

cv::Point2d get_next_dpoint_from_angle(const cv::Point2d& pt, int angle_degree)
{

    double angle_rad = angle_degree * DEG2RAD;
    double dx = cos(angle_rad);
    double dy = sin(angle_rad);
    /*
    cv::Point2d v(cos(angle_rad), sin(angle_rad));
    double factor = max(abs(v.x), abs(v.y));
    v.x /= factor;
    v.y /= factor;
    cv::Point2d dp(pt.x + v.x, pt.y + v.y);
    */
    cv::Point2d dp(pt.x + dx, pt.y + dy);
    return dp;

}

double calc_distance(const cv::Point2d& start, const cv::Point2d& end)
{
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double ret = sqrt(dx * dx + dy * dy);
    return ret;
}

double calc_distance(const cv::Point2f& start, const cv::Point2f& end)
{
    double dx = (double)end.x - start.x;
    double dy = (double)end.y - start.y;
    double ret = sqrt(dx * dx + dy * dy);
    return ret;
}

cv::Point double_point_to_cv_point(const cv::Point2d& pt)
{
    cv::Point pts = cv::Point(int(round(pt.x)), int(round(pt.y)));
    return pts;
}

double push_to_queue_get_num_of_white_pixel(const cv::Mat& src, std::list<cv::Point2d>& list_dp, cv::Point2d dp, int max_size)
{

    if (list_dp.size() >= max_size)
    {
        list_dp.pop_front();
    }
    else
    {

    }
    list_dp.push_back(dp);
    std::list<cv::Point2d>::iterator it_list = list_dp.begin();
    double sum = 0;
    while (it_list != list_dp.end())
    {

        double tmp = get_intensity_cv_point_image(src, double_point_to_cv_point(*it_list)) / 10;
        sum += tmp;
        it_list++;


    }
    double val = 1;
    if (max_size == list_dp.size())
    {
        val = sum / list_dp.size();
        return val;
    }
    return 0;
}

bool GetSartPoint(const cv::Mat& img, cv::Point2f pt, int size, cv::Point2f& outPoint)
{
    double x = 0;
    double y = 0;
    int count = 0;
    int firstPointX = (int)(pt.x - (size - 1) / 2);
    int firstPointy = (int)(pt.y - (size - 1) / 2);
    int rs = img.rows;
    int cs = img.cols;

    if (firstPointX > rs || firstPointX < 0 || firstPointy > cs || firstPointy < 0)
    {
        outPoint = pt;
        return false;
    }
    for (int row = firstPointy; row < size + firstPointy; row++)
    {
        const uchar* src_pixel = img.ptr<uchar>(row);
        for (int col = firstPointX; col < size + firstPointX; col++)
        {
            if (*(src_pixel + col) > 250)
            {
                x += col;
                y += row;
                count++;
            }
        }
    }

    if (0 == count)
    {
        outPoint = pt;
        return false;
    }
    else
    {
        x /= count;
        y /= count;
        outPoint = cv::Point2f((float)x, (float)y);
        return true;
    }
}

void process_edge_tracking(const  cv::Mat& src,
						   const cv::Point2d& src_pt,
						   const int& angle_degree, double intentThreshold,
						   std::vector<point_angle>& vt_result,
						   std::vector<point_angle>& vt_result_filter,
						   std::vector<point_angle>& vt_result_final
)
{
	vt_result.clear();

	point_angle pa;
	pa.angle = angle_degree;
	pa.dp = src_pt;

	vt_result.push_back(pa);

	int num_of_next_point = 6;
	int num_of_missing_point = 0;
	int angleRange = 5; //in degree
	for (int p = 0; p < 100; p++)
	{
		cv::Point2d start_pt;

		int start_angle;
		int end_angle;

		start_pt = vt_result.back().dp;
		start_angle = vt_result.back().angle - angleRange;
		end_angle = vt_result.back().angle + angleRange;

		double mid_angle = ((double)start_angle + end_angle) / 2.0;
		double min_weight = 1;
		double max_weight = 1;
		std::map<int, double> map_w;

		double ofset_w = (max_weight - min_weight) / (mid_angle - start_angle) + min_weight;
		double ofset_angle = mid_angle - start_angle;

		for (int i = start_angle; i <= mid_angle; i++)
		{
			double val = (i - (double)start_angle) * ofset_w;
			map_w[i] = val;
		}

		for (int i = (int)std::ceil(mid_angle); i < end_angle; i++)
		{
			double val = ((double)end_angle - i) * ofset_w;
			map_w[i] = val;
		}
		std::multimap<double, int> dic_value;

		for (int i = start_angle; i < end_angle; i++)
		{
			std::vector<cv::Point2d> a = get_arr_direction_dppoint(start_pt, i, num_of_next_point);
			double value_of_direct = calc_value_direction_edge(a, src, i);
			/*std::vector<cv::Point2d> a0 = get_arr_direction_dppoint(cv::Point(start_pt.x-1,start_pt.y), i, num_of_next_point);
			std::vector<cv::Point2d> a1 = get_arr_direction_dppoint(cv::Point(start_pt.x +1, start_pt.y), i, num_of_next_point);
			double value_of_direct0 = calc_value_direction_edge(a0, src, i);
			double value_of_direct1 = calc_value_direction_edge(a1, src, i);
			value_of_direct = (value_of_direct + value_of_direct0 + value_of_direct1) / 3;*/
			value_of_direct *= map_w[i];

			dic_value.insert(std::pair<double, int>(value_of_direct, i));
		}
		std::multimap<double, int>::iterator it = dic_value.end(); it--;
		int good_angle_degree = it->second;
		cv::Point2d good_pt = get_next_dpoint_from_angle(start_pt, good_angle_degree);

		point_angle result;
		result.dp = good_pt;
		result.angle = good_angle_degree;

		vt_result.push_back(result);
		uchar i_dp = get_intensity_double_point_image(src, good_pt.x, good_pt.y);
		if (i_dp == 0)
		{
			num_of_missing_point++;
		}
		else
		{
			num_of_missing_point = 0;
		}
		if (num_of_missing_point > 5)
		{
			break;
		}
	}

	std::vector<point_angle> vt_bin;
	for (size_t i = 0; i < vt_result.size(); i++)
	{
		cv::Point2d dp = vt_result[i].dp;
		point_angle bin_val;
		bin_val.dp = dp;
		bin_val.angle = (int)std::round(get_intensity_double_point_image(src, (int)std::round(bin_val.dp.x), (int)std::round(bin_val.dp.y)) / 255.0);
		vt_bin.push_back(bin_val);
	}

	vt_result_filter.clear();
	std::list<cv::Point2d> vt_cumulative;
	std::vector<double> vt_sumIntent;
	for (size_t i = 0; i < vt_result.size(); i++)
	{
		point_angle dpa = vt_result[i];
		cv::Point2d dp = vt_result[i].dp;

		double sum = push_to_queue_get_num_of_white_pixel(src, vt_cumulative, dp, num_of_next_point);
		vt_sumIntent.push_back(sum);
		if (sum < intentThreshold)
		{
			size_t new_size = vt_result_filter.size() - num_of_next_point;
			if (new_size >= 0 && new_size <= vt_result_filter.size())
			{
				vt_result_filter.resize(new_size);

			}
			/*namedWindow("showSource", cv::WINDOW_NORMAL);
			cv::imshow("showSource", src);
				cv::waitKey();*/
			break;
		}
		else
		{
			vt_result_filter.push_back(dpa);
		}
	}
}

}//xvt