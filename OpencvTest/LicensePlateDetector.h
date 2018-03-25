#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "RANSAC.h"

#include <Windows.h>

class LicensePlateDetector
{

// Fields
public:

	enum PLATE_STATE {

		inprocess,
		finished,
	};

	struct LINE {

		CvPoint p1;
		CvPoint p2;
		cv::Vec4f params = { -1.f, -1.f, -1.f, -1.f };//(cos(t), sin(t), X0, Y0)
		int numOfValidPoints = 0;
	};

	struct PLATE_OBJECT {

		IplImage* plateImage = 0;
		PLATE_STATE state = PLATE_STATE::finished;
		time_t timestamp = 0;

		int width = 0;
		int height = 0;
		int x_coord = 0;
		int y_coord = 0;

		LINE  lineH[2];
		LINE  vanishLineH;
		int lineHcount = 0;
		//double aH_err = 0.0;
		//double aV_err = 0.0;
		//double dH_err = 0.0;
		//double dV_err = 0.0;
		//
		//LINE  lineV[2];
	};



	struct LINE_CANDIDATE {

		LINE line1;
		LINE line2;
	};


	IplImage* colorImage = 0;
	cv::CascadeClassifier plateClassifierCascade;

	double probablyWidth = 0.0;

	std::vector<LINE> horizontalLineVector;
	std::vector<LINE_CANDIDATE> horizontalLineCandidateVector;
	std::vector<LINE> horizontalLineVector2;
	std::vector<PLATE_OBJECT> plateVector;

// Methods
public:
	LicensePlateDetector();
	~LicensePlateDetector();

	int LicensePlateDetector::LoadImageFromFile(const char* filename);
	int LicensePlateDetector::ShowColorImage(double scale);
	int LicensePlateDetector::ShowImage(IplImage* image, double scale = 1.0);
	IplImage* LicensePlateDetector::ConvertColorImage();

	int LicensePlateDetector::FindProbablyPlateWidth(PLATE_OBJECT plate);
	int LicensePlateDetector::FindPlateHorizontalLines(PLATE_OBJECT plate);
	int LicensePlateDetector::FilterPlateHorizontalLines(PLATE_OBJECT* plate);

	int LicensePlateDetector::PrintPlateHorizontalLines(PLATE_OBJECT plate);
	int LicensePlateDetector::PrintPlateVanishHorLine(PLATE_OBJECT plate);
	int LicensePlateDetector::RotateGrayImage(double angle = 0.0);



	

	int LicensePlateDetector::ProcessPlates(const char* filename);



	// This method creates and loads Haar cascade from xml-file
	int LicensePlateDetector::InitClassifierCascade(const char* filename);

	// This method searches for region with license plate and push it to plateVector
	int LicensePlateDetector::FindPlateROI( CvRect ROI, // Searh area
											CvSize minSize, // Minimal size of Haar searching window
											CvSize maxSize, // Maximum size of Haar searching window
											double scaleFactor = 1.1, // Haar cascade scale factor is from 1.1 to 2.0 (1.1 - best quality with low speed)
											double xboundScale = 0.1, // X-axis plate bound scaling in percents from 0 to 0.2
											double yboundScale = 0.1  // Y-axis plate bound scaling in percents from 0 to 0.4
	);

};

