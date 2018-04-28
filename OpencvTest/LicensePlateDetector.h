#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <Windows.h>
#include "common.h"
#include "PlateTracker.h"
#include "Vanisher.h"

#define DEGREE_IN_RADIAN 57.295779513

#define VANISH_RANSAC_RADIUS 300

class LicensePlateDetector
{

public:

	PlateTracker tracker;

	cv::Mat colorImage;
	cv::Mat preColorImage;

	std::vector<LINE> roadDirectionLines;
	std::vector<LINE> roadPerpendicularLines;


	cv::Point2f roadDirectionVP;
	cv::Point2f roadPerpendicularVP;
	cv::Point2f zenithVP;

	Vanisher xVanisher;
	

// Methods
public:

	LicensePlateDetector();
	~LicensePlateDetector();

	bool MainThread();
	bool ProcessVideo(const char* filename);

	int LoadImageFromFile(const char* filename);
	bool ShowImage(cv::Mat *image, const char *wndName, double scale);
	void PrintVanishVector(cv::Point vanishPoint, int lenght, int number, int R, int G, int B);

	float GetAngleBetweenVector(cv::Vec3f v1, cv::Vec3f v2);	//TODO: vectors
	cv::Point2f GetThirdVanishPoint(float Beta, cv::Point2f pp, cv::Point2f vp1, cv::Point2f vp2);
	cv::Vec3f GetLambdas(cv::Point2f pp, cv::Point2f vp1, cv::Point2f vp2, cv::Point2f vp3);
	cv::Vec3f GetRotatation(cv::Point2f vp1, cv::Point2f vp2);

	void GetCarsContours();
	void PrintCarBound(std::vector<cv::Point> contour, cv::Mat *image);

	void PrintIntersections(std::vector<cv::Point2f> intersecions, cv::Point2f vanish);
	cv::Point2f FindVanishPoint(std::vector<cv::Point2f> intersecions, float sigma);

	bool GetVanishLineFromPlate(LICENSE_PLATE *plate, LINE* line);
	bool GetVanishLineFromCar(LICENSE_PLATE *plate, LINE* line, double angle);


	void ProcessRoadDirectionLines(std::vector<LINE> lines);
	void ProcessRoadPerpendicularLines(std::map<float, LINE> *lines);


	
	bool FindLinesIntersections2(std::vector<LINE> lines1, std::vector<LINE> lines2, std::vector<cv::Point2f> *result);
	int FindVanishPointRANSAC(std::vector<cv::Point2f> intersections, cv::Point2f *resultPoint, float radius);


	
	bool LicensePlateDetector::FindLinesIntersections(std::vector<LINE> lines, std::vector<cv::Point2f> *result);

	


	int LicensePlateDetector::ProcessPlates(const char* filename);

	void LicensePlateDetector::ComputeRotationMatrix(float focal_lenght, cv::Point2f v1, cv::Point2f v2, cv::Point2f v3);









	
	// This method finds vanish point by minimal sum of distances from point to lines 
	int LicensePlateDetector::FindVanishPointMinDistances( std::vector<LINE> lines, // Vanishing lines
														   cv::Point2f *resultPoint // Result vanishing point
	);

	// This method computes focal lenght from two vanishing points
	float LicensePlateDetector::ComputeFocalLenght( cv::Point2f vp1, // Horizontal vanishing point
													cv::Point2f vp2 // Depth vanihins point
	);

	// This method computes third vector from two vanishing points
	cv::Point2f LicensePlateDetector::ComputeThirdVanishingPoint( cv::Point2f vp1, // Horizontal vanishing point
																  cv::Point2f vp2, // Depth vanihins point
																  float focal // Focal lenght
	);

};

