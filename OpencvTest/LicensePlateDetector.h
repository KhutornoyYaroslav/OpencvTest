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


#define PLATE_TRACKS_VECTOR_SIZE 10
#define DEGREE_IN_RADIAN 57.295779513f



class LicensePlateDetector
{
public:

	struct PLATE_OBJECT {

		// Plate ROI data
		IplImage* plateImage = 0;
		time_t time = 0;
		int plateROIwidth = 0;
		int plateROIheight = 0;
		int plateROIleftX = 0;
		int plateROIleftY = 0;

		// Plate size data
		int plateWidth = 0;
		int plateHeight = 0;

		// Lines data
		LINE horLines[2];
		LINE vertLines[2];
		int horLinesCount = 0;
		int vertLinesCount = 0;

		// Vanishes
		LINE horVanishLine;
		bool isHorVanishLine = false;

		// Tracking
		int owner_id = -1;
		float owner_weight = 0.0;
		float dirAngle = 0.0;
		//float dy = 0.0;
	};

	struct PLATE_TRACK {

		enum class STATE {

			ready,
			inprocess,
			finished
		};

		LINE DirectionVanishLine;
		unsigned int lost_count = 0;
		int id;
		STATE state = STATE::ready;
		std::vector<PLATE_OBJECT> points;
	};

	struct LINE_CANDIDATE {

		LINE line1;
		LINE line2;
	};


	IplImage* colorImage = 0;
	cv::CascadeClassifier plateClassifierCascade;
	std::vector<PLATE_OBJECT> plateVector;
	std::vector<PLATE_TRACK*> plateTracks;

	std::vector<LINE> dirVanishLines;
	std::vector<LINE> horVanishLines;

	unsigned int track_id = 0;


// Methods
public:
	LicensePlateDetector();
	~LicensePlateDetector();

	int LicensePlateDetector::RotateGrayImage(double angle = 0.0);


	int LicensePlateDetector::LoadImageFromFile(const char* filename);
	int LicensePlateDetector::ShowColorImage(double scale);
	int LicensePlateDetector::ShowImage(IplImage* image, double scale = 1.0);
	IplImage* LicensePlateDetector::ConvertColorImage();

	int LicensePlateDetector::FindProbablyPlateWidth(PLATE_OBJECT *plate);
	int LicensePlateDetector::FindPlateHorizontalLines(PLATE_OBJECT *plate);
	int LicensePlateDetector::FilterPlateHorizontalLines(std::vector<LINE> lines, PLATE_OBJECT* plate);

	int LicensePlateDetector::PrintVanishVectors(CvPoint vanishPoint, int number);
	int LicensePlateDetector::PrintVanishVectors2(CvPoint vanishPoint, int number);
	int LicensePlateDetector::PrintVanishVectors3(CvPoint vanishPoint, int number);

	int LicensePlateDetector::GetLineFromTwoPoints(CvPoint2D32f firstPoint, CvPoint2D32f secondPoint, LINE* line);

	

	int LicensePlateDetector::FindLinesIntersections(std::vector<LINE> lines, std::vector<CvPoint2D32f> *result);	
	int LicensePlateDetector::ProcessPlates(const char* filename);

	// ProcessPlate
	PLATE_TRACK* LicensePlateDetector::CreateNewTrack();
	int LicensePlateDetector::CleanTrack(PLATE_TRACK* track);
	int LicensePlateDetector::ProcessTracks(std::vector<PLATE_OBJECT> *new_plates);
	int LicensePlateDetector::FinishedTrackProccess();
	void LicensePlateDetector::PrintTracks(PLATE_TRACK::STATE state);


	
	void LicensePlateDetector::ComputeRotationMatrix(float focal_lenght, CvPoint2D32f v1, CvPoint2D32f v2, CvPoint2D32f v3);

	int LicensePlateDetector::PrintTrackDirectionVanishLine(PLATE_TRACK track,
		int topYcoord, 
		int bottomYcoord
	);



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

	// This method fits line in points by RANSAC algorithm
	int LicensePlateDetector::LineFitRANSAC( double t, // Distance from main line
											 double p, // Chance of hitting a valid pair
											 double e, // Percentage of outliers
											 int T, // Number of expected minimum inliers 
											 std::vector<cv::Point>& nzPoints, // Vector with points
											 LINE *line // Result line
	);

	// This method finds two line intersection
	int LicensePlateDetector::FindTwoLineIntersection( LINE l1, // First line
													   LINE l2, // Second line
													   CvPoint2D32f *intersection // Point of lines intersection
	);

	// This method finds vanish point by RANSAC algorithm
	int LicensePlateDetector::FindVanishPointRANSAC( std::vector<LINE> lines, // Vanishing lines
												     CvPoint2D32f *resultPoint, // Result vanishing point
												     float radius // Maximal radius from result point to others points in pixels
	);
	
	// This method finds vanish point by minimal sum of distances from point to lines 
	int LicensePlateDetector::FindVanishPointMinDistances( std::vector<LINE> lines, // Vanishing lines
														   CvPoint2D32f *resultPoint // Result vanishing point
	);

	// This method prints plate horizontal lines
	int LicensePlateDetector::PrintPlateHorizontalLines( PLATE_OBJECT plate, // Plate with lines
													     int leftXcoord, // Left line point coordinate in pixels
													     int rightXcoord // Right line point coordinate in pixels
	);

	// This method prints plate horizontal vanish line
	int LicensePlateDetector::PrintPlateVanishHorLine( PLATE_OBJECT plate, // Plate with horizontal vansih line
													   int leftXcoord, // Left line point coordinate in pixels
													   int rightxcoord // Right line point coordinate in pixels
	);

	// This method computes focal lenght from two vanishing points
	float LicensePlateDetector::ComputeFocalLenght( CvPoint2D32f vp1, // Horizontal vanishing point
													CvPoint2D32f vp2 // Depth vanihins point
	);

	// This method computes third vector from two vanishing points
	CvPoint2D32f LicensePlateDetector::ComputeThirdVanishingPoint( CvPoint2D32f vp1, // Horizontal vanishing point
																   CvPoint2D32f vp2, // Depth vanihins point
																   float focal // Focal lenght
	);

};

