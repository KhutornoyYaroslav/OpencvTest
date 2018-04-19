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

#define DEGREE_IN_RADIAN 57.295779513f

class LicensePlateDetector
{
public:


	struct LINE_CANDIDATE {

		LINE line1;
		LINE line2;
	};

	cv::Mat colorImage;



	cv::CascadeClassifier plateClassifierCascade;
	std::vector<LICENSE_PLATE> plateVector;

	std::vector<LINE> directionLines;
	std::vector<PLATES_TRACK*> plateTracks;

	std::vector<LINE> dirVanishLines;
	std::vector<LINE> horVanishLines;

	std::vector<LINE> upLines;
	std::vector<LINE> downLines;



	PlateTracker tracker;


// Methods
public:

	LicensePlateDetector();
	~LicensePlateDetector();

	bool MainThread();
	bool ProcessVideo(const char* filename);

	int LoadImageFromFile(const char* filename);
	bool ShowImage(cv::Mat *image, const char *wndName, double scale);
	void PrintVanishVector(cv::Point vanishPoint, int lenght, int number, int R, int G, int B);



	bool FilterVanishLines(std::vector<LINE> *lines);

	bool GetVanishLinesFromPlate(LICENSE_PLATE *plate, std::vector<LINE> *lines);
	bool GetVanishLinesFromCarTrack(PLATES_TRACK *track, std::vector<LINE> *lines);

	
	bool FindLinesIntersections2(std::vector<LINE> lines1, std::vector<LINE> lines2, std::vector<cv::Point2f> *result);
	bool FindVanishPointRANSAC2(std::vector<cv::Point2f> intersections, cv::Point2f *resultPoint, float radius);


	int LicensePlateDetector::FindProbablyPlateWidth(LICENSE_PLATE *plate);
	
	int LicensePlateDetector::FindLinesIntersections(std::vector<LINE> lines, std::vector<cv::Point2f> *result);

	


	int LicensePlateDetector::ProcessPlates(const char* filename);

	void LicensePlateDetector::ComputeRotationMatrix(float focal_lenght, cv::Point2f v1, cv::Point2f v2, cv::Point2f v3);






	// This method finds two line intersection
	int LicensePlateDetector::FindTwoLineIntersection( LINE l1, // First line
													   LINE l2, // Second line
													   cv::Point2f  *intersection // Point of lines intersection
	);

	// This method finds vanish point by RANSAC algorithm
	int LicensePlateDetector::FindVanishPointRANSAC( std::vector<LINE> lines, // Vanishing lines
													 cv::Point2f *resultPoint, // Result vanishing point
												     float radius // Maximal radius from result point to others points in pixels
	);
	
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

