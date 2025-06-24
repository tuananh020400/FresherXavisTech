#include "xvtCV/CurveCSS.h"
#include "xvtCV/CircleDetector.h"
#include "xvtCV/xvtPen.h"

#define _USE_MATH_DEFINES
#ifdef HAVE_MATHGL
#include <mgl2/mgl.h>
#include <mgl2/window.h>
#endif

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <map>
#include <math.h>

//#pragma mark Gaussian Smoothing and Curvature 

// This one should be fixed later
#pragma warning( disable : 26451)

using namespace std;
using namespace cv;

#ifdef _DEBUG
//#define _SHOW_CURVECSS_INFO_
#endif
namespace xvt {
namespace shape {
/* 1st and 2nd derivative of 1D gaussian
 */
void getGaussianDerivs(double sigma, int M, vector<double>& gaussian, vector<double>& dg, vector<double>& d2g) {
	int L = (M - 1) / 2;
	double sigma_sq = sigma * sigma;
	double sigma_quad = sigma_sq * sigma_sq;
	dg.resize(M); d2g.resize(M); gaussian.resize(M);

	cv::Mat_<double> g = cv::getGaussianKernel(M, sigma, CV_64F);
	for (double i = -L; i < L + 1.0; i += 1.0) {
		int idx = (int)(i + L);
		gaussian[idx] = g(idx);
		// from http://www.cedar.buffalo.edu/~srihari/CSE555/Normal2.pdf
		dg[idx] = (-i / sigma_sq) * g(idx);
		d2g[idx] = (-sigma_sq + i * i) / sigma_quad * g(idx);
	}
}

/* 1st and 2nd derivative of smoothed curve point */
void getdX(vector<double> x,
	int n,
	double sigma,
	double& gx,
	double& dgx,
	double& d2gx,
	vector<double> g,
	vector<double> dg,
	vector<double> d2g,
	bool isOpen = false)
{
	int L = (int)((g.size() - (size_t)1) / (size_t)2);

	gx = dgx = d2gx = 0.0;
	//	cout << "Point " << n << ": ";
	for (int k = -L; k < L + 1; k++) {
		double x_n_k;
		if (n - k < 0) {
			if (isOpen) {
				//open curve - mirror values on border
				x_n_k = x[-(n - k)];
			}
			else {
				//closed curve - take values from end of curve
				x_n_k = x[x.size() + (n - k)];
			}
		}
		else if (n - k > x.size() - 1) {
			if (isOpen) {
				//mirror value on border
				x_n_k = x[n + k];
			}
			else {
				x_n_k = x[(n - k) - (x.size())];
			}
		}
		else {
			//			cout << n-k;
			x_n_k = x[n - k];
		}
		//		cout << "* g[" << g[k + L] << "], ";

		gx += x_n_k * g[k + L]; //gaussians go [0 -> M-1]
		dgx += x_n_k * dg[k + L];
		d2gx += x_n_k * d2g[k + L];
	}
	//	cout << endl;
}


/* 0th, 1st and 2nd derivatives of whole smoothed curve */
void getdXcurve(vector<double> x,
	double sigma,
	vector<double>& gx,
	vector<double>& dx,
	vector<double>& d2x,
	vector<double> g,
	vector<double> dg,
	vector<double> d2g,
	bool isOpen = false)
{
	gx.resize(x.size());
	dx.resize(x.size());
	d2x.resize(x.size());
	for (int i = 0; i < x.size(); i++) {
		double gausx, dgx, d2gx; getdX(x, i, sigma, gausx, dgx, d2gx, g, dg, d2g, isOpen);
		gx[i] = gausx;
		dx[i] = dgx;
		d2x[i] = d2gx;
	}
}

void ResampleCurve(const vector<double>& curvex, const vector<double>& curvey,
	vector<double>& resampleX, vector<double>& resampleY,
	int N,
	bool isOpen
) {
	assert(curvex.size() > 0 && curvey.size() > 0 && curvex.size() == curvey.size());

	vector<cv::Point2d> resamplepl(N); resamplepl[0].x = curvex[0]; resamplepl[0].y = curvey[0];
	vector<cv::Point2i> pl; PolyLineMerge(pl, curvex, curvey);

	double pl_length = cv::arcLength(pl, false);
	double resample_size = pl_length / (double)N;
	int curr = 0;
	double dist = 0.0;
	for (int i = 1; i < N; ) {
		assert(curr < pl.size() - 1);
		double last_dist = cv::norm(pl[curr] - pl[curr + 1]);
		dist += last_dist;
		//		cout << curr << " and " << curr+1 << "\t\t" << last_dist << " ("<<dist<<")"<<endl;
		if (dist >= resample_size) {
			//put a point on line
			double _d = last_dist - (dist - resample_size);
			cv::Point2d cp(pl[curr].x, pl[curr].y), cp1(pl[curr + 1].x, pl[curr + 1].y);
			cv::Point2d dirv = cp1 - cp; dirv = dirv * (1.0 / norm(dirv));
			//			cout << "point " << i << " between " << curr << " and " << curr+1 << " remaining " << dist << endl;
			assert(i < resamplepl.size());
			resamplepl[i] = cp + dirv * _d;
			i++;

			dist = last_dist - _d; //remaining dist			

			//if remaining dist to next point needs more sampling... (within some epsilon)
			while (dist - resample_size > 1e-3) {
				//				cout << "point " << i << " between " << curr << " and " << curr+1 << " remaining " << dist << endl;
				assert(i < resamplepl.size());
				resamplepl[i] = resamplepl[i - 1] + dirv * resample_size;
				dist -= resample_size;
				i++;
			}
		}

		curr++;
	}

	PolyLineSplit(resamplepl, resampleX, resampleY);
}

//#pragma mark CSS image


/* compute curvature of curve after gaussian smoothing
 from "Shape similarity retrieval under affine transforms", Mokhtarian & Abbasi 2002
 curvex - x position of points
 curvey - y position of points
 kappa - curvature coeff for each point
 sigma - gaussian sigma
 */
void ComputeCurveCSS(const vector<double>& curvex,
	const vector<double>& curvey,
	vector<double>& kappa,
	vector<double>& smoothX, vector<double>& smoothY,
	double sigma,
	bool isOpen
)
{
	int M = (int)(round((10.0 * sigma + 1.0) / 2.0) * 2 - 1);
	assert(M % 2 == 1); //M is an odd number

	vector<double> g, dg, d2g; getGaussianDerivs(sigma, M, g, dg, d2g);

	vector<double> X, XX, Y, YY;
	getdXcurve(curvex, sigma, smoothX, X, XX, g, dg, d2g, isOpen);
	getdXcurve(curvey, sigma, smoothY, Y, YY, g, dg, d2g, isOpen);

	kappa.resize(curvex.size());
	for (int i = 0; i < curvex.size(); i++) {
		// Mokhtarian 02' eqn (4)
		kappa[i] = round(10 * (X[i] * YY[i] - XX[i] * Y[i]) / pow(X[i] * X[i] + Y[i] * Y[i], 1.5)) / 10;
	}
}

/* find zero crossings on curvature */
vector<int> FindCSSInterestPoints(const vector<double>& kappa) {
	vector<int> crossings;
	for (int i = 0; i < kappa.size() - 1; i++) {
		/*if ((kappa[i] <= 0 && kappa[i+1] > 0) || (kappa[i] > 0 && kappa[i+1] <= 0)) {*/
		if ((kappa[i] <= 0 && kappa[i + 1] > 0) || (kappa[i] > 0 && kappa[i + 1] <= 0) || (kappa[i] * kappa[i + 1] == 0 && kappa[i + 1] + kappa[i] != 0)) {
			/*if ((kappa[i] * kappa[i + 1] < 0) || (abs(abs(kappa[i]) - abs(kappa[i + 1])) >= 0.5)) {*/
			crossings.push_back(i);
		}
	}
	return crossings;
}


bool isBorderPoint(cv::Point p, int width, int height) {
	bool isBorder = (p.x == 0 || p.y == 0 || p.x == width || p.y == height);
	return isBorder;
}

vector<int> FindCSSInterestPoints0(const vector<double>& kappa) {
	vector<int> crossings;
	for (int i = 0; i < kappa.size() - 1; i++) {
		//if ((kappa[i] <= 0 && kappa[i+1] > 0) || (kappa[i] > 0 && kappa[i+1] <= 0) || abs(kappa[i]) >= 0.8) {
		//if ((kappa[i] <= 0 && kappa[i + 1] > 0) || (kappa[i] > 0 && kappa[i + 1] <= 0) || (kappa[i] * kappa[i+1] == 0 && kappa[i + 1]+ kappa[i] != 0)) {
		//if ((kappa[i] * kappa[i + 1] < 0 && abs(kappa[i + 1] - kappa[i])>0.1) || (kappa[i] * kappa[i + 1] == 0 && kappa[i + 1] + kappa[i] < 0))
		if ((kappa[i] * kappa[i + 1] < 0 && abs(kappa[i + 1] - kappa[i])>0.12))
		{
			crossings.push_back(i);
		}
	}
	return crossings;
}


vector<int> EliminateCloseMaximas(const vector<int>& maximasv, map<int, double>& maximas) {
	//eliminate degenerate segments (of very small length)
	vector<int> maximasvv;
	for (int i = 0; i < maximasv.size(); i++) {
		if (i < maximasv.size() - 1 &&
			maximasv[i + 1] - maximasv[i] <= 4)
		{
			//segment of small length (1 - 4) - eliminate one point, take largest sigma 
			if (maximas[maximasv[i]] > maximas[maximasv[i + 1]]) {
				maximasvv.push_back(maximasv[i]);
			}
			else {
				maximasvv.push_back(maximasv[i + 1]);
			}
			i++; //skip next element as well
		}
		else {
			maximasvv.push_back(maximasv[i]);
		}
	}
	return maximasvv;
}

/* compute the CSS image */
vector<int> ComputeCSSImageMaximas(const vector<double>& contourx_, const vector<double>& contoury_,
	vector<double>& contourx, vector<double>& contoury,
	bool isClosedCurve
)
{
	ResampleCurve(contourx_, contoury_, contourx, contoury, 200, !isClosedCurve);
	vector<cv::Point2d> pl; PolyLineMerge(pl, contourx, contoury);

	map<int, double> maximas;

	cv::Mat_<cv::Vec3b> img(500, 200, cv::Vec3b(0, 0, 0)), contourimg(350, 350, cv::Vec3b(0, 0, 0));
	bool done = false;
	//#pragma omp parallel for
	for (int i = 0; i < 490; i++) {
		if (!done) {
			double sigma = 1.0 + ((double)i) * 0.1;
			vector<double> kappa, smoothx, smoothy;
			ComputeCurveCSS(contourx, contoury, kappa, smoothx, smoothy, sigma);

			//			vector<vector<Point> > contours(1);
			//			PolyLineMerge(contours[0], smoothx, smoothy);
			//			contourimg = cv::Vec3b(0,0,0);
			//			drawContours(contourimg, contours, 0, cv::Scalar(255,255,255), CV_FILLED);

			vector<int> crossings = FindCSSInterestPoints(kappa);
			if (crossings.size() > 0) {
				for (int c = 0; c < crossings.size(); c++) {
					img(i, crossings[c]) = cv::Vec3b(255, 0, 0);
					//					circle(contourimg, contours[0][crossings[c]], 5, cv::Scalar(0,0,255), CV_FILLED);

					if (c < crossings.size() - 1) {
						if (fabs((double)(crossings[c] - crossings[c + 1])) < 5.0) {
							//this is a maxima
							int idx = (crossings[c] + crossings[c + 1]) / 2;
							//#pragma omp critical
							maximas[idx] = (maximas[idx] < sigma) ? sigma : maximas[idx];

							circle(img, cv::Point(idx, i), 1, cv::Scalar(0, 0, 255), cv::FILLED);
						}
					}
				}
				//				char buf[128]; sprintf(buf, "evolution_%05d.png", i);
				//				imwrite(buf, contourimg);
				//				imshow("evolution", contourimg);
				//				waitKey(30);
			}
			else {
				done = true;
			}

		}
	}

	//find largest sigma
	double max_sigma = 0.0;
	for (map<int, double>::iterator itr = maximas.begin(); itr != maximas.end(); ++itr) {
		if (max_sigma < (*itr).second) {
			max_sigma = (*itr).second;
		}
	}
	//get segments with largest sigma
	vector<int> maximasv;
	for (map<int, double>::iterator itr = maximas.begin(); itr != maximas.end(); ++itr) {
		if ((*itr).second > max_sigma / 8.0) {
			maximasv.push_back((*itr).first);
		}
	}
	//eliminate degenerate segments (of very small length)
	vector<int> maximasvv = EliminateCloseMaximas(maximasv, maximas);	//1st pass
	maximasvv = EliminateCloseMaximas(maximasvv, maximas);				//2nd pass
	maximasv = maximasvv;
	for (vector<int>::iterator itr = maximasv.begin(); itr != maximasv.end(); ++itr) {
		cout << *itr << " - " << maximas[*itr] << endl;
	}
	//	cv::Mat zoom; resize(img,zoom,Size(img.rows*2,img.cols*2));
	//cv::imshow("css image", img);
	//cv::waitKey();
	return maximasv;
}

//#pragma mark Curve Matching

/* calculate the "centroid distance" for the curve */
void GetCurveSignature(const vector<cv::Point2d>& a, vector<double>& signature) {
	signature.resize(a.size());
	cv::Scalar a_mean = mean(a); cv::Point2d a_mpt(a_mean[0], a_mean[1]);

	//centroid distance
	for (int i = 0; i < a.size(); i++) {
		signature[i] = norm(a[i] - a_mpt);
	}
}

/* from http://paulbourke.net/miscellaneous/correlate/ */
double CalcCrossCorrelation(const vector<double>& x, const vector<double>& y) {
	assert(x.size() == y.size());
	int i, j, n = (int)x.size();
	double mx, my, sx, sy, sxy, denom, r;

	/* Calculate the mean of the two series x[], y[] */
	mx = 0;
	my = 0;
	for (i = 0; i < n; i++) {
		mx += x[i];
		my += y[i];
	}
	mx /= n;
	my /= n;

	/* Calculate the denominator */
	sx = 0;
	sy = 0;
	for (i = 0; i < n; i++) {
		sx += (x[i] - mx) * (x[i] - mx);
		sy += (y[i] - my) * (y[i] - my);
	}
	denom = sqrt(sx * sy);

	/* Calculate the correlation series */
//	for (delay=-maxdelay;delay<maxdelay;delay++) 
	int delay = 0;
	{
		sxy = 0;
		for (i = 0; i < n; i++) {
			j = i + delay;
			if (j < 0 || j >= n)
				continue;
			else
				sxy += (x[i] - mx) * (y[j] - my);
			/* Or should it be (?)
			 if (j < 0 || j >= n)
			 sxy += (x[i] - mx) * (-my);
			 else
			 sxy += (x[i] - mx) * (y[j] - my);
			 */
		}
		r = sxy / denom;

		/* r is the correlation coefficient at "delay" */
	}
	return r;
}

/* calculate the similarity score between two curve segments
 Mai 2010, "Affine-invariant shape matching and recognition under partial occlusion", section 4.1
 */
double MatchTwoSegments(const vector<cv::Point2d>& a_, const vector<cv::Point2d>& b_) {
	assert(a_.size() == b_.size()); //cross correlation will work only for similar length curves
	if (a_.size() <= 1 || b_.size() <= 1) {
		cerr << "degenerate: a_.size() " << a_.size() << " b_.size() " << b_.size() << endl;
		return -1.0; //check degenrate case
	}

	vector<double> a_x(a_.size()), a_y(a_.size()), b_x(b_.size()), b_y(b_.size());
	vector<double> a_x_(a_.size()), a_y_(a_.size()), b_x_(b_.size()), b_y_(b_.size());
	vector<cv::Point2d> a = a_, b = b_;
	//	PolyLineSplit(a_, a_x_, a_y_); ResampleCurve(a_x_, a_y_, a_x, a_y, 50); PolyLineMerge(a, a_x, a_y);
	//	PolyLineSplit(b_, b_x_, b_y_); ResampleCurve(b_x_, b_y_, b_x, b_y, 50); PolyLineMerge(b, b_x, b_y);

	cv::Scalar a_mean = mean(a), b_mean = mean(b);
	cv::Point2d a_mpt(a_mean[0], a_mean[1]), b_mpt(b_mean[0], b_mean[1]);
	vector<cv::Point2d> a_m(a.size()), b_m(b.size());
	for (int i = 0; i < a.size(); i++) { a_m[i] = a[i] - a_mpt; }
	for (int i = 0; i < b.size(); i++) { b_m[i] = b[i] - b_mpt; }

	cv::Mat_<double> a_mM = cv::Mat(a_m).reshape(1).t();
	cv::Mat_<double> b_mM = cv::Mat(b_m).reshape(1).t();
	cv::SVD asvd(a_mM), bsvd(b_mM);
	vector<cv::Point2d> a_canon(a.size()), b_canon(b.size());
	cv::Mat(asvd.vt.t()).copyTo(a_mM);
	a_mM.reshape(2).copyTo(cv::Mat(a_canon));
	cv::Mat(bsvd.vt.t()).copyTo(b_mM);
	b_mM.reshape(2).copyTo(cv::Mat(b_canon));


	vector<double> a_sig; GetCurveSignature(a_canon, a_sig);
	vector<double> b_sig; GetCurveSignature(b_canon, b_sig);

	double cc = CalcCrossCorrelation(a_sig, b_sig);

#if 0
#ifdef HAVE_MATHGL
	{
		mglGraph gr;
		gr.SubPlot(2, 1, 0, "");

		vector<double> a_canon_x, a_canon_y;
		PolyLineSplit(a_canon, a_canon_x, a_canon_y);
		vector<double> b_canon_x, b_canon_y;
		PolyLineSplit(b_canon, b_canon_x, b_canon_y);

		mglData mgl_a_x(&(a_canon_x[0]), a_canon_x.size()), mgl_a_y(&(a_canon_y[0]), a_canon_y.size());
		mglData mgl_b_x(&(b_canon_x[0]), b_canon_x.size()), mgl_b_y(&(b_canon_y[0]), b_canon_y.size());

		gr.Title("Canonical");
		gr.Aspect(1, 1);
		gr.SetRanges(-.5, .5, -.5, .5);
		gr.Axis();
		gr.Grid();
		gr.Plot(mgl_a_x, mgl_a_y);
		gr.Plot(mgl_b_x, mgl_b_y);


		gr.SubPlot(2, 1, 1, "");
		mglData x(&(a_sig[0]), a_sig.size()), x1(&(b_sig[0]), b_sig.size());

		gr.Title("Signature");
		gr.SetRanges(0, max(a_sig.size(), b_sig.size()), 0, 0.55);
		gr.Axis();
		gr.Grid();
		gr.Plot(x);
		gr.Plot(x1);

		cv::Mat img(gr.GetHeight(), gr.GetWidth(), CV_8UC3, (void*)gr.GetRGB());
		stringstream ss; ss << "cross correlation " << cc;
		putText(img, ss.str(), Point(10, 20), CV_FONT_NORMAL, 1.0, cv::Scalar(255), 2);
		imshow("tmp", img);
		waitKey();
	}
#endif
#endif

	return cc; // > 0.8 ? cc : 0.0;
}

/* match the two curves using adapted Smith-Waterman aligning algorithm
 Mai 2010, "Affine-invariant shape matching and recognition under partial occlusion", section 4.2 */

cv::Mat_<double> GetSmithWatermanHMatrix(const vector<vector<cv::Point2d> >& a, const vector<vector<cv::Point2d> >& b) {
	int M = (int)a.size();
	int N = (int)b.size();

	//Smith-Waterman
	cv::Mat_<double> H(M, N - 1, 0.0);
	for (int i = 1; i < M; i++) {
		for (int j = 1; j < N - 1; j++) {
			vector<double> v(4, 0.0);
			v[1] = H(i - 1, j - 1) + MatchTwoSegments(a[i], b[j]);
			v[2] = H(i - 1, j) - 1.0;
			v[3] = H(i, j - 1) - 1.0;
			H(i, j) = *(max_element(v.begin(), v.end()));
		}
	}
	cout << H << endl;
	return H;
}

/* original Smith Waterman algorithm */
double MatchCurvesSmithWaterman(const vector<vector<cv::Point2d> >& a, const vector<vector<cv::Point2d> >& b, vector<cv::Point>& traceback)
{
	cv::Mat_<double> H = GetSmithWatermanHMatrix(a, b);
	cv::Point maxp; double maxval;
	minMaxLoc(H, NULL, &maxval, NULL, &maxp);
	while (H(maxp.y, maxp.x) != 0) {
		//				cout << "H(maxp.y-1,maxp.x-1) > H(maxp.y,maxp.x-1)" << H(maxp.y-1,maxp.x-1) << " > " << H(maxp.y,maxp.x-1) << endl;
		if (H(maxp.y - 1, maxp.x - 1) > H(maxp.y, maxp.x - 1) &&
			H(maxp.y - 1, maxp.x - 1) > H(maxp.y - 1, maxp.x))
		{
			maxp = maxp - cv::Point(1, 1);
			traceback.push_back(maxp);
		}
		else
			if (H(maxp.y - 1, maxp.x) > H(maxp.y - 1, maxp.x - 1) &&
				H(maxp.y - 1, maxp.x) > H(maxp.y, maxp.x - 1))
			{
				maxp.y--;
				traceback.push_back(maxp);
			}
			else
				if (H(maxp.y, maxp.x - 1) > H(maxp.y - 1, maxp.x - 1) &&
					H(maxp.y, maxp.x - 1) > H(maxp.y - 1, maxp.x))
				{
					maxp.x--;
					traceback.push_back(maxp);
				}
				else {
					break;
				}
	}
	for (int k = 0; k < traceback.size(); k++) {
		cout << traceback[k] << " -> ";
	}
	cout << endl;
	return maxval;
}

/* adapted Smith Waterman */
double AdaptedMatchCurvesSmithWaterman(const vector<vector<cv::Point2d> >& a, const vector<vector<cv::Point2d> >& b, vector<cv::Point>& traceback)
{
	int M = (int)a.size();
	int N = (int)b.size();

	cv::Mat_<double> H = GetSmithWatermanHMatrix(a, b);

	vector<vector<cv::Point> > tracebacks;
	vector<cv::Point> max_traceback;
	int max_traceback_len = 0;
	for (int i = M - 1; i >= 2; i--) {
		for (int j = N - 2; j >= 2; j--) {
			if (i < max_traceback_len || j < max_traceback_len) {
				continue; //skip it, it already can't be longer..
			}

			//Traceback
			vector<cv::Point> tmp_traceback;
			cv::Point maxp = cv::Point(i, j);
			tmp_traceback.push_back(maxp);
			//			maxp = maxp - Point(1,1);
			//			tmp_traceback.push_back(maxp);
			bool movedup = false, movedleft = false;
			while (H(maxp.y, maxp.x) != 0 && maxp.y > 1 && maxp.x > 1) {
				if (H(maxp.y - 1, maxp.x - 1) > H(maxp.y, maxp.x - 1) &&
					H(maxp.y - 1, maxp.x - 1) > H(maxp.y - 1, maxp.x))
				{
					//					cout << "move left-up" << endl;
					maxp = maxp - cv::Point(1, 1);
					traceback.push_back(maxp);
				}
				else if (H(maxp.y - 1, maxp.x) > H(maxp.y - 1, maxp.x - 1) &&
					H(maxp.y - 1, maxp.x) > H(maxp.y, maxp.x - 1))
				{
					//					cout << "move up" << endl;
					maxp.y--;
					movedup = true;
				}
				else if (H(maxp.y, maxp.x - 1) > H(maxp.y - 1, maxp.x - 1) &&
					H(maxp.y, maxp.x - 1) > H(maxp.y - 1, maxp.x))
				{
					//					cout << "move left" << endl;
					maxp.x--;
					movedleft = true;
				}
				if (movedup && movedleft) {
					traceback.push_back(maxp);
					movedup = movedleft = false;
				}
			}
			for (int k = 0; k < tmp_traceback.size(); k++) {
				cout << tmp_traceback[k] << " -> ";
			}
			cout << endl;
			if (tmp_traceback.size() > max_traceback_len ||
				(
					tmp_traceback.size() == max_traceback_len && //if equal - look for highest match
					H(tmp_traceback.front().y, tmp_traceback.front().x) > H(max_traceback.front().y, max_traceback.front().x)
					)
				)
			{
				max_traceback_len = (int)tmp_traceback.size();
				max_traceback = tmp_traceback;
				cout << "taking traceback of length " << max_traceback_len << endl;
			}
		}
	}

	traceback = max_traceback;

	return H(traceback[0].y, traceback[0].x);
}


void checkCircleFitting(std::vector<int> crossings, std::vector<cv::Point2f> vecContourPoints, double minRadius, double maxRadius, cv::Mat ImgRes, std::vector<cv::Point2f>& vtCenter, std::vector<double>& vtRadius) {
	int crossSize = (int)crossings.size();
	int nPoint4Fit = 5;
	if (crossSize <= 1) {
		cv::Point2d center;
		double radius;
		fit_circle(vecContourPoints, center, radius);
		if (radius > minRadius && radius < maxRadius) {
#ifdef _SHOW_CURVECSS_INFO_
			std::cout << "fitted Radius: " << radius << std::endl;
#endif
			cv::circle(ImgRes, cv::Point((int)round(center.x), (int)round(center.y)), (int)round(radius), cv::Scalar(0, 0, 255), 1);
			vtCenter.push_back(center);
			vtRadius.push_back(radius);
		}
	}
	//Check curve in each 2 turning points
	for (int idx = 0; idx < crossSize - 1; idx++) {
		if (crossings[idx + 1] - crossings[idx] >= nPoint4Fit) {
			cv::Point2d center;
			double radius;
			std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[idx], vecContourPoints.begin() + crossings[idx + 1]);
			if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1)) {
				fit_circle(corner, center, radius);
				if (radius > minRadius && radius < maxRadius) {
					cv::circle(ImgRes, cv::Point((int)round(center.x), (int)round(center.y)), (int)round(radius), cv::Scalar(0, 0, 255), 1);
					vtCenter.push_back(center);
					vtRadius.push_back(radius);
				}
			}
		}
	}

	for (int idx = 0; idx < crossSize - 2; idx++) {
		if (crossings[idx + 2] - crossings[idx] >= nPoint4Fit) {
			cv::Point2d center;
			double radius;
			std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[idx], vecContourPoints.begin() + crossings[idx + 2]);
			if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1)) {
				fit_circle(corner, center, radius);
				if (radius > minRadius && radius < maxRadius) {
					cv::circle(ImgRes, cv::Point((int)round(center.x), (int)round(center.y)), (int)round(radius), cv::Scalar(0, 0, 255), 1);
					vtCenter.push_back(center);
					vtRadius.push_back(radius);
				}
			}
		}
	}

	// check curve from last turning point to first turning point
	if ((crossSize > 1) && (crossings[0] + vecContourPoints.size() - crossings[crossSize - 1] >= nPoint4Fit)) {
		std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[crossings.size() - 1], vecContourPoints.end());
		std::vector<cv::Point2f> corner2(vecContourPoints.begin(), vecContourPoints.begin() + crossings[0]);
		corner.insert(corner.end(), corner2.begin(), corner2.end());
		cv::Point2d center;
		double radius;
		if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1)) {
			fit_circle(corner, center, radius);
			if (radius > minRadius && radius < maxRadius) {
#ifdef _SHOW_CURVECSS_INFO_
				std::cout << "fitted Radius: " << radius << std::endl;
#endif
				cv::circle(ImgRes, cv::Point((int)round(center.x),(int)round(center.y)), (int)round(radius), cv::Scalar(255, 0, 255), 1);
				vtCenter.push_back(center);
				vtRadius.push_back(radius);
			}
		}
	}

	if ((crossSize > 2) && (crossings[0] + vecContourPoints.size() - crossings[crossSize - 2] >= nPoint4Fit)) {
		std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[crossings.size() - 2], vecContourPoints.end());
		std::vector<cv::Point2f> corner2(vecContourPoints.begin(), vecContourPoints.begin() + crossings[0]);
		corner.insert(corner.end(), corner2.begin(), corner2.end());
		cv::Point2d center;
		double radius;
		if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1)) {
			fit_circle(corner, center, radius);
			if (radius > minRadius && radius < maxRadius) {
#ifdef _SHOW_CURVECSS_INFO_
				std::cout << "fitted Radius: " << radius << std::endl;
#endif
				cv::circle(ImgRes, cv::Point((int)round(center.x), (int)round(center.y)), (int)round(radius), cv::Scalar(0, 255, 255), 1);
				vtCenter.push_back(center);
				vtRadius.push_back(radius);
			}
		}
	}

	if ((crossSize > 2) && (crossings[1] + vecContourPoints.size() - crossings[crossSize - 1] >= nPoint4Fit)) {
		std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[crossings.size() - 1], vecContourPoints.end());
		std::vector<cv::Point2f> corner2(vecContourPoints.begin(), vecContourPoints.begin() + crossings[1]);
		corner.insert(corner.end(), corner2.begin(), corner2.end());
		cv::Point2d center;
		double radius;
		if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1)) {
			fit_circle(corner, center, radius);
			//BasicLibs::CircleDetector::FitLeastSquare(contours[maxIdx], center, radius, pe);
			if (radius > minRadius && radius < maxRadius) {
#ifdef _SHOW_CURVECSS_INFO_
				std::cout << "fitted Radius: " << radius << std::endl;
#endif
				cv::circle(ImgRes, cv::Point((int)round(center.x), (int)round(center.y)), (int)round(radius), cv::Scalar(0, 255, 0), 1);
				vtCenter.push_back(center);
				vtRadius.push_back(radius);
			}
		}
	}
	for (int idx = 0; idx < crossSize - 1; idx++) {
		if (crossings[idx] < vecContourPoints.size() - nPoint4Fit) {
			cv::Point2d center;
			double radius;
			std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[idx], vecContourPoints.begin() + crossings[idx] + nPoint4Fit);
			if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1)) {
				fit_circle(corner, center, radius);
#ifdef _SHOW_CURVECSS_INFO_
				//cv::circle(ImgRes, cv::Point(round(center.x), round(center.y)), round(radius), cv::Scalar(255, 255, 255), 1);
				/*if (radius > mMinRadius && radius < mMaxRadius) {
					cv::circle(ImgRes, cv::Point(round(center.x), round(center.y)), round(radius), cv::Scalar(255, 255, 255), 1);
				}*/
#endif
			}
		}
		else {
			cv::Point2d center;
			double radius;
			std::vector<cv::Point2f> corner(vecContourPoints.begin() + crossings[crossings.size() - 1], vecContourPoints.end());
			std::vector<cv::Point2f> corner2(vecContourPoints.begin(), vecContourPoints.begin() + crossings[0]);
			corner.insert(corner.end(), corner2.begin(), corner2.end());
			if (!isBoundaryContour(corner, ImgRes.cols - 1, ImgRes.rows - 1) && corner.size() > nPoint4Fit) {
				fit_circle(corner, center, radius);
				if (radius > minRadius && radius < maxRadius) {
					cv::circle(ImgRes, cv::Point((int)round(center.x), (int)round(center.y)), (int)round(radius), cv::Scalar(110, 110, 255), 1);
				}
			}
		}
	}

#ifdef _SHOW_CURVECSS_INFO_
//	cv::namedWindow("Check Circle", WINDOW_NORMAL);
//	cv::imshow("Check Circle", ImgRes);
#endif //_DEBUG
}

inline double gauss(double sigma, double x) {
	double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
	double divider = sqrt(2 * M_PI * pow(sigma, 2));
	return (1 / divider) * exp(expVal);
}

std::vector<double> gaussKernel(int samples, double sigma) {
	std::vector<double> v;

	bool doubleCenter = false;
	if (samples % 2 == 0) {
		doubleCenter = true;
		samples--;
	}
	int steps = (samples - 1) / 2;
	double stepSize = (3 * sigma) / steps;

	for (int i = steps; i >= 1; i--) {
		v.push_back(gauss(sigma, i * stepSize * -1));
	}

	v.push_back(gauss(sigma, 0));
	if (doubleCenter) {
		v.push_back(gauss(sigma, 0));
	}

	for (int i = 1; i <= steps; i++) {
		v.push_back(gauss(sigma, i * stepSize));
	}
	assert(v.size() == samples);

	return v;
}

std::vector<double> gaussSmoothen(std::vector<double> values, double sigma, int samples) {
	std::vector<double> out;
	auto kernel = gaussKernel(samples, sigma);
	long sampleSide = (long)(samples / 2);
	int valueIdx = (int)(samples / 2 + 1);
	long ubound = (long)values.size();
	std::vector<double> tmpValues(values);
	for (int i = 0; i < sampleSide; i++) {
		tmpValues.insert(tmpValues.begin(), values[ubound - i - 1]);
	}
	for (long i = sampleSide; i < ubound + sampleSide; i++) {
		double sample = 0;
		int sampleCtr = 0;
		for (long j = i - sampleSide; j <= i + sampleSide; j++) {
			if (j > 0 && j < ubound) {
				int sampleWeightIndex = sampleSide + (j - i);
				sample += kernel[sampleWeightIndex] * tmpValues[j];
				sampleCtr++;
			}
		}
		double smoothed = sample / (double)sampleCtr;
		//std::cout << " S: " << sample << " C: " << sampleCtr << " V: " << values[i] << " SM: " << smoothed << std::endl;
		out.push_back(sample);
	}
	return out;
}

std::vector<double> computeCurvature(std::vector<cv::Point2f> vecContourPoints) {
	std::vector<double> vecCurvature(vecContourPoints.size());
	auto frontToBack = vecContourPoints.front() - vecContourPoints.back();
	bool isClosed = ((int)(std::max)(std::abs(frontToBack.x), std::abs(frontToBack.y))) <= 1;

	cv::Point2f pplus, pminus;
	cv::Point2f f1stDerivative, f2ndDerivative;

	int step = 2;
	int gaussSigma = 2;
	int sampleWindow = 7;
	vector<double> vecContourX;
	vector<double> vecContourY;
	PolyLineSplit(vecContourPoints, vecContourX, vecContourY);
	vector<double> vecSmoothedX, vecSmoothedY;
	std::vector<cv::Point2f> vecSmoothedContour;
	vecSmoothedX = gaussSmoothen(vecContourX, gaussSigma, sampleWindow);
	vecSmoothedY = gaussSmoothen(vecContourY, gaussSigma, sampleWindow);
	//vecSmoothedX = vecContourX;
	//vecSmoothedY = vecContourY;

	PolyLineMerge(vecSmoothedContour, vecSmoothedX, vecSmoothedY);

	//vecSmoothedContour = vecContourPoints;
	for (int i = 0; i < vecContourPoints.size(); i++)
	{
		const cv::Point2f& pos = vecSmoothedContour[i];

		int maxStep = step;
		if (!isClosed)
		{
			maxStep = (std::min)((std::min)(step, i), (int)vecContourPoints.size() - 1 - i);
			if (maxStep == 0)
			{
				vecCurvature[i] = std::numeric_limits<double>::infinity();
				continue;
			}
		}
		int iminus = i - maxStep;
		int iplus = i + maxStep;
		pminus = vecSmoothedContour[iminus < 0 ? iminus + vecContourPoints.size() : iminus];
		pplus = vecSmoothedContour[iplus >= vecContourPoints.size() ? iplus - vecContourPoints.size() : iplus];

		f1stDerivative.x = (pplus.x - pos.x) / (iplus - iminus);
		f1stDerivative.y = (pplus.y - pos.y) / (iplus - iminus);
		f2ndDerivative.x = (pplus.x - 2 * pos.x + pminus.x) / ((iplus - iminus) / 2 * (iplus - iminus) / 2);
		f2ndDerivative.y = (pplus.y - 2 * pos.y + pminus.y) / ((iplus - iminus) / 2 * (iplus - iminus) / 2);

		double curvature2D;
		double divisor = f1stDerivative.x * f1stDerivative.x + f1stDerivative.y * f1stDerivative.y;
		if (std::abs(divisor) > 10e-8)
		{
			curvature2D = (f2ndDerivative.y * f1stDerivative.x - f2ndDerivative.x * f1stDerivative.y) /
				pow(divisor, 3.0 / 2.0);

		}
		else
		{
			curvature2D = std::numeric_limits<double>::infinity();
		}
		vecCurvature[i] = curvature2D;
	}
	return vecCurvature;
}


void fit_circle(std::vector<cv::Point2f>& pnts, cv::Point2d& centre, double& radius)
{
	int cols = 3;
	cv::Mat X(static_cast<int>(pnts.size()), cols, CV_64F);
	cv::Mat Y(static_cast<int>(pnts.size()), 1, CV_64F);
	cv::Mat C;

	if (int(pnts.size()) >= 3)
	{
		for (size_t i = 0; i < pnts.size(); i++)
		{
			X.at<double>(static_cast<int>(i), 0) = 2 * pnts[i].x;
			X.at<double>(static_cast<int>(i), 1) = 2 * pnts[i].y;
			X.at<double>(static_cast<int>(i), 2) = -1.0;
			Y.at<double>(static_cast<int>(i), 0) = (pnts[i].x * pnts[i].x + pnts[i].y * pnts[i].y);
		}
		cv::solve(X, Y, C, cv::DECOMP_SVD);

		std::vector<double> coefs;
		C.col(0).copyTo(coefs);
		centre.x = coefs[0];
		centre.y = coefs[1];
		radius = sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1] - coefs[2]);
	}
}
void plotHist(cv::Mat img, std::string tilte) {
	cv::Mat hist;
	int histSize = 64;
	float range[] = { 0, 255 }; //the upper boundary is exclusive
	const float* histRange = { range };
	calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, true);
	hist = hist / (img.rows * img.cols);
	// Compute the CDF of histogram
	cv::Mat cdfHist = hist.clone();
	int cdfIdx = 1;
	cdfHist.at<float>(0) = hist.at<float>(histSize - 1);
	for (int i = histSize; i > 1; i--) {
		cdfHist.at<float>(cdfIdx) += cdfHist.at<float>(cdfIdx - 1);
		cdfIdx++;
	}

	int xAx = 30, yAx = 30;
	int hist_w = 768 + yAx, hist_h = 450;
	cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
	hist_h -= xAx;
	hist_w -= yAx;
	int bin_w = cvRound((double)hist_w / histSize);
	// nomalized the graph the fit the size of windows

#pragma warning (suppress:26812)
	normalize(hist, hist, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
	for (int i = 1; i < histSize; i++)
	{
		rectangle(histImage, cv::Point(yAx + bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			cv::Point(yAx + bin_w * (i), hist_h),
			cv::Scalar(255, 0, 0), 2, 8, 0);
		if (i % 2 == 0)
		{
			putText(histImage, std::to_string(i * 256 / histSize), cv::Point((int)(yAx + bin_w * (i - 0.5)), (int)(hist_h + xAx - 10)), 1, 0.4, cv::Scalar(0, 0, 255));
			putText(histImage, std::to_string((int)hist.at<float>(i - 1)), cv::Point((int)(yAx + bin_w * (i - 0.8)), (int)(hist_h - cvRound(hist.at<float>(i - 1)) - 10)), 1, 0.5, cv::Scalar(0, 255, 255));
		}
	}
	cv::namedWindow(tilte, cv::WINDOW_NORMAL);
	cv::imshow(tilte, histImage);
	//cv::imwrite(tilte + ".bmp", histImage);
}
float getAccHistAtThreshold(cv::Mat image, int threshold) {
	cv::Mat hist;
	int histSize = 256;
	float range[] = { 0, 255 }; //the upper boundary is exclusive
	const float* histRange = { range };
	calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, true);
	hist = hist / (image.rows * image.cols);
	// Compute the CDF of histogram
	float cdfHist = 0.0;
	int cdfIdx = 1;
	cdfHist += hist.at<float>(0);
	for (int i = 1; i < threshold; i++) {
		cdfHist += hist.at<float>(i);
	}
	return cdfHist;
}
bool isBoundaryContour(std::vector<cv::Point2f> vec, int width, int height) {
	std::vector<cv::Point2f>::iterator it0 = std::find(vec.begin(), vec.end(), cv::Point2f(0, 0));
	std::vector<cv::Point2f>::iterator it1 = std::find(vec.begin(), vec.end(), cv::Point2f((float)width, 0));
	std::vector<cv::Point2f>::iterator it2 = std::find(vec.begin(), vec.end(), cv::Point2f(0, (float)height));
	std::vector<cv::Point2f>::iterator it3 = std::find(vec.begin(), vec.end(), cv::Point2f((float)width, (float)height));
	std::vector<cv::Point2f>::iterator itB = std::find_if(vec.begin(), vec.end(), [&](const cv::Point2f& o) {
		return o.x == 0 || o.y == 0 || o.x == width || o.y == height;
	});
	bool isContainBounder = itB != vec.end();
	bool res = (it0 != vec.end()) || (it1 != vec.end()) || (it2 != vec.end()) || (it3 != vec.end());
	return isContainBounder;
}
cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows) {
	int N = (int)vecMat.size();
	nRows = nRows > N ? N : nRows;
	int edgeThickness = 5;
	int imagesPerRow = (int)ceil(double(N) / nRows);
	int resizeHeight = (int)floor(2.0 * ((floor(double(windowHeight - edgeThickness) / nRows)) / 2.0)) - edgeThickness;
	int maxRowLength = 0;

	std::vector<int> resizeWidth;
	for (int i = 0; i < N;) {
		int thisRowLen = 0;
		for (int k = 0; k < imagesPerRow; k++) {
			double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
			int temp = int(ceil(resizeHeight * aspectRatio));
			resizeWidth.push_back(temp);
			thisRowLen += temp;
			if (++i == N) break;
		}
		if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength) {
			maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1);
		}
	}
	int windowWidth = maxRowLength;
	cv::Mat canvasImage(windowHeight, windowWidth, CV_8UC3, cv::Scalar(0, 0, 0));

	for (int k = 0, i = 0; i < nRows; i++) {
		int y = i * resizeHeight + (i + 1) * edgeThickness;
		int x_end = edgeThickness;
		for (int j = 0; j < imagesPerRow && k < N; k++, j++) {
			int x = x_end;
			cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
			cv::Size s = canvasImage(roi).size();
			// change the number of channels to three
			cv::Mat target_ROI(s, CV_8UC3);
			if (vecMat[k].channels() != canvasImage.channels()) {
				if (vecMat[k].channels() == 1) {
					cv::cvtColor(vecMat[k], target_ROI, cv::COLOR_GRAY2BGR);
				}
			}
			else {
				vecMat[k].copyTo(target_ROI);
			}
			cv::resize(target_ROI, target_ROI, s);
			if (target_ROI.type() != canvasImage.type()) {
				target_ROI.convertTo(target_ROI, canvasImage.type());
			}
			target_ROI.copyTo(canvasImage(roi));
			x_end += resizeWidth[k] + edgeThickness;
		}
	}
	return canvasImage;
}
cv::Mat makeCirKernel(int radius, int& nPoints, int value) {
	nPoints = 0;
	int r2 = static_cast<int>(radius * radius) + 1;
	int kRadius = static_cast<int>(std::sqrt(r2 + 1e-10));
	int kHeight = 2 * kRadius + 1;
	cv::Mat kernel = cv::Mat::zeros(kHeight, kHeight, CV_8UC1);

	nPoints = kHeight;
	for (int col = 0; col < kHeight; col++)
	{
		kernel.at<uchar>(kRadius, col) = value;
	}

	for (int y = 1; y <= kRadius; y++)
	{ //lines above and below center together
		int dx = static_cast<int>(std::sqrt(r2 - y * y + 1e-10));
		for (int col = kRadius - dx; col <= kRadius + dx; col++)
		{
			kernel.at<uchar>(kRadius - y, col) = value;
			kernel.at<uchar>(kRadius + y, col) = value;
		}
		nPoints += 4 * dx + 2;	//2*dx+1 for each line, above&below
	}
	return kernel;
}
void diff(std::vector<float> in, std::vector<float>& out)
{
	out = std::vector<float>(in.size() - 1);

	for (int i = 1; i < in.size(); ++i)
		out[i - 1] = in[i] - in[i - 1];
}
void vectorProduct(std::vector<float> a, std::vector<float> b, std::vector<float>& out)
{
	out = std::vector<float>(a.size());

	for (int i = 0; i < a.size(); ++i)
		out[i] = a[i] * b[i];
}
void findIndicesLessThan(std::vector<float> in, float threshold, std::vector<int>& indices)
{
	for (int i = 0; i < in.size(); ++i)
		if (in[i] < threshold)
			indices.push_back(i + 1);
}
//template <class T>
//void selectElements(std::vector<T> in, std::vector<T> indices, std::vector<T>& out)
//{
//	for (int i = 0; i < indices.size(); ++i)
//		out.push_back(in[indices[i]]);
//}
void signVector(std::vector<float> in, std::vector<int>& out)
{
	out = std::vector<int>(in.size());

	for (int i = 0; i < in.size(); ++i)
	{
		if (in[i] > 0)
			out[i] = 1;
		else if (in[i] < 0)
			out[i] = -1;
		else
			out[i] = 0;
	}
}
void findPeaks(std::vector<float> x0, std::vector<int>& peakInds, float selection, float thresh, int extrema, bool includeEndpoints, bool interpolate)

{
	int x0Len = (int)x0.size();
	// % Make it so we are finding maxima regardless
	for (size_t i = 0; i < x0Len; i++)
		x0.at(i) = x0.at(i) * extrema;
	auto minIdx = std::min_element(x0.begin(), x0.end());
	auto maxIdx = std::max_element(x0.begin(), x0.end());

	float sel = (*maxIdx - *minIdx) / selection;

	// Adjust threshold according to extrema.
	thresh = thresh * extrema;
	std::vector<float> dx;
	//  % Find derivative
	diff(x0, dx);
	// derivative changes sign at the first of repeated values (e.g 4,5,5,5)
	std::replace(dx.begin(), dx.end(), 0.0f, -EPS);

	// Find where the derivative changes sign
	std::vector<float> dx0(dx.begin(), dx.end() - 1);
	std::vector<float> dx1(dx.begin() + 1, dx.end());
	std::vector<float> dx2;
	vectorProduct(dx0, dx1, dx2);
	std::vector<int> ind;
	findIndicesLessThan(dx2, 0, ind);

	// Include firstpoint and endpoint into potential peaks and valleys
	ind.insert(ind.begin(), 0);
	ind.insert(ind.end(), x0Len - 1);
	// x only has the peaks, valleys, and possibly endpoints
	std::vector<float> x;
	//selectElements(x0, ind, x);
	for (int i : ind)
	{
		x.push_back(x0[i]);
	}

	auto minMagIdx = std::min_element(x.begin(), x.end());
	float minMag = *minMagIdx;
	float leftMin = minMag;
	int xLen = (int)x.size();

	// % Function with peaks and valleys
	if (xLen > 2)
	{
		float tempMag = minMag;
		bool foundPeak = false;
		int ii;

		// Deal with first point a little differently since tacked it on
		// Calculate the sign of the derivative since we tacked the first
		//  point on it does not neccessarily alternate like the rest.
		std::vector<float> xSub0(x.begin(), x.begin() + 3);//tener cuidado subvector
		std::vector<float> xDiff;//tener cuidado subvector
		diff(xSub0, xDiff);

		std::vector<int> signDx;
		signVector(xDiff, signDx);

		if (signDx[0] <= 0) // The first point is larger or equal to the second
		{
			if (signDx[0] == signDx[1]) // Want alternating signs
			{
				x.erase(x.begin() + 1);
				ind.erase(ind.begin() + 1);
				xLen = xLen - 1;
			}
		}
		else // First point is smaller than the second
		{
			if (signDx[0] == signDx[1]) // Want alternating signs
			{
				x.erase(x.begin());
				ind.erase(ind.begin());
				xLen = xLen - 1;
			}
		}
		// % Skip the first point if it is smaller so we always start on a
		// maxima
		if (x[0] >= x[1])
			ii = 0;
		else
			ii = 1;

		// % Preallocate max number of maxima
		int maxPeaks = (int)std::ceil((float)xLen / 2.0);
		std::vector<int> peakLoc(maxPeaks, 0);
		std::vector<float> peakMag(maxPeaks, 0.0);
		int cInd = 1;
		int tempLoc = 0;

		// % Loop through extrema which should be peaks and then valleys
		while (ii < xLen)
		{
			ii = ii + 1;//This is a peak
			//Reset peak finding if we had a peak and the next peak is bigger
			//than the last or the left min was small enough to reset.
			if (foundPeak)
			{
				tempMag = minMag;
				foundPeak = false;
			}

			//Found new peak that was lager than temp mag and selectivity larger
			//than the minimum to its left.

			if (x[ii - 1] > tempMag && x[ii - 1] > leftMin + sel)
			{
				tempLoc = ii - 1;
				tempMag = x[ii - 1];
			}

			//Make sure we don't iterate past the length of our std::vector
			if (ii == xLen)
				break; //We assign the last point differently out of the loop

			ii = ii + 1; // Move onto the valley

			//Come down at least sel from peak
			if (!foundPeak && tempMag > sel + x[ii - 1])
			{
				foundPeak = true; //We have found a peak
				leftMin = x[ii - 1];
				peakLoc[cInd - 1] = tempLoc; // Add peak to index
				peakMag[cInd - 1] = tempMag;
				cInd = cInd + 1;
			}
			else if (x[ii - 1] < leftMin) // New left minima
				leftMin = x[ii - 1];

		}

		// Check end point
		if (x[x.size() - 1] > tempMag && x[x.size() - 1] > leftMin + sel)
		{
			peakLoc[cInd - 1] = xLen - 1;
			peakMag[cInd - 1] = x[x.size() - 1];
			cInd = cInd + 1;
		}
		else if (!foundPeak && tempMag > minMag)// Check if we still need to add the last point
		{
			peakLoc[cInd - 1] = tempLoc;
			peakMag[cInd - 1] = tempMag;
			cInd = cInd + 1;
		}

		//Create output
		if (cInd > 0)
		{
			std::vector<int> peakLocTmp(peakLoc.begin(), peakLoc.begin() + cInd - 1);
			//selectElements<int>(ind, peakLocTmp, peakInds);
			for (int i = 0 ;i < cInd;i++)
			{
				peakInds.push_back(ind[peakLoc[i]]);
			}
			//peakMags = std::vector<float>(peakLoc.begin(), peakLoc.begin()+cInd-1);
		}
	}
}

void findinglargestcontour(cv::Mat image, std::vector<std::vector<cv::Point>>& vtr_contours,std::vector<cv::Point>& largest_contour,double& area_largest_contour,int& index_of_largest_area) {
	// Finding the largest contours
	std::vector <cv::Vec4i> vtr_hierrachy;
	cv::findContours(image, vtr_contours, vtr_hierrachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	index_of_largest_area = 0;
	area_largest_contour = 0;
	for (int ct = 0; ct < vtr_contours.size(); ct++)
	{
		double area = cv::contourArea(vtr_contours[ct]);
		if (area > area_largest_contour)
		{
			index_of_largest_area = ct;
			area_largest_contour = area;
		}

	}
	largest_contour = vtr_contours[index_of_largest_area];

}
}
}