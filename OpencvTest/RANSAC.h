#pragma once
#include "stdafx.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

struct SLine
{
	SLine() :
		numOfValidPoints(0),
		params(-1.f, -1.f, -1.f, -1.f)
	{}
	cv::Vec4f params;//(cos(t), sin(t), X0, Y0)
	int numOfValidPoints;

	float A = 0.0;
	float B = 0.0;
	float C = 0.0;

	float x0 = params[2];
	float y0 = params[3];
	float k = atan(params[1] / params[0]);
};


SLine LineFitRANSAC(
	float t,//distance from main line
	float p,//chance of hitting a valid pair
	float e,//percentage of outliers
	int T,//number of expected minimum inliers 
	std::vector<cv::Point>& nzPoints);
