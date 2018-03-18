#include "stdafx.h"
#include "RANSAC.h"
// для коммита
cv::Vec4f TotalLeastSquares(
	std::vector<cv::Point>& nzPoints,
	std::vector<int> ptOnLine)
{
	//if there are enough inliers calculate model
	float x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
	float dx2, dy2, dxy;
	float t;
	for (size_t i = 0; i < nzPoints.size(); ++i)
	{
		x += ptOnLine[i] * nzPoints[i].x;
		y += ptOnLine[i] * nzPoints[i].y;
		x2 += ptOnLine[i] * nzPoints[i].x * nzPoints[i].x;
		y2 += ptOnLine[i] * nzPoints[i].y * nzPoints[i].y;
		xy += ptOnLine[i] * nzPoints[i].x * nzPoints[i].y;
		w += ptOnLine[i];
	}

	x /= w;
	y /= w;
	x2 /= w;
	y2 /= w;
	xy /= w;

	//Covariance matrix
	dx2 = x2 - x * x;
	dy2 = y2 - y * y;
	dxy = xy - x * y;

	t = (float)atan2(2 * dxy, dx2 - dy2) / 2;
	cv::Vec4f line;
	line[0] = (float)cos(t);
	line[1] = (float)sin(t);

	line[2] = (float)x;
	line[3] = (float)y;

	return line;
}

SLine LineFitRANSAC(
	float t,//distance from main line
	float p,//chance of hitting a valid pair
	float e,//percentage of outliers
	int T,//number of expected minimum inliers 
	std::vector<cv::Point>& nzPoints)
{
	int s = 2;//number of points required by the model
	int N = (int)ceilf(log(1 - p) / log(1 - pow(1 - e, s)));//number of independent trials

	N = N * 10;//----------

	std::vector<SLine> lineCandidates;
	std::vector<int> ptOnLine(nzPoints.size());//is inlier
	cv::RNG rng((uint64)-1);
	SLine line;
	for (int i = 0; i < N; i++)
	{
		//pick two points
		int idx1 = (int)rng.uniform(0, (int)nzPoints.size());
		int idx2 = (int)rng.uniform(0, (int)nzPoints.size());
		cv::Point p1 = nzPoints[idx1];
		cv::Point p2 = nzPoints[idx2];

		//points too close - discard
		if (cv::norm(p1 - p2) < t)
		{
			continue;
		}

		//line equation ->  (y1 - y2)X + (x2 - x1)Y + x1y2 - x2y1 = 0 
		float a = static_cast<float>(p1.y - p2.y);
		float b = static_cast<float>(p2.x - p1.x);

		double k = a / b;////////////////////////////////////

		float c = static_cast<float>(p1.x*p2.y - p2.x*p1.y);
		
		//normalize them
		float scale = 1.f / sqrt(a*a + b*b);
		a *= scale;
		b *= scale;
		c *= scale;
		//float k = a / b; // ----------------
		//count inliers
		int numOfInliers = 0;
		for (size_t i = 0; i < nzPoints.size(); ++i)
		{
			cv::Point& p0 = nzPoints[i];
			float rho = abs(a*p0.x + b*p0.y + c);
			bool isInlier = rho  < t;
			if (isInlier) numOfInliers++;
			ptOnLine[i] = isInlier;
		}

		if (numOfInliers < T)
		{
			continue;
		}

		line.k = k;
		line.params = TotalLeastSquares(nzPoints, ptOnLine);
		line.numOfValidPoints = numOfInliers;
		lineCandidates.push_back(line);
	}

	int bestLineIdx = 0;
	int bestLineScore = 0;
	for (size_t i = 0; i < lineCandidates.size(); i++)
	{
		if (lineCandidates[i].numOfValidPoints > bestLineScore)
		{
			bestLineIdx = i;
			bestLineScore = lineCandidates[i].numOfValidPoints;
		}
	}

	if (lineCandidates.empty())
	{
		return SLine();
	}
	else
	{
		return lineCandidates[bestLineIdx];
	}
}

