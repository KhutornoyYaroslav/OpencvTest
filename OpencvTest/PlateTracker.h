#pragma once

#include "common.h"
#include <vector>

#define DEGREE_IN_RADIAN 57.295779513f

#define TRACKS_VECTOR_INIT_SIZE 10
#define TRACKS_MAX_DELTA_COORD_KOEF 1.75f
#define TRACKS_MAX_DELTA_ANGLE 10
#define TRACKS_CMP_COORD_KOEF 0.8f
#define TRACKS_CMP_ANGLE_KOEF 0.2f
#define TRACKS_CMP_MAX_WEIGHT 1.0f

#define TRACKS_LOST_COUNT_MAX 20
#define TRACKS_POINTS_COUNT_MIN 10
#define TRACKS_TRACK_LENGHT_MIN 300

#define CAR_IMAGE_RESIZE 5.0

class PlateTracker
{

private:

	struct TRACKED_PLATE 
	{
		LICENSE_PLATE plate;
		bool isOwner = false;
		unsigned int ownerID = 0;
		float ownerWeight = 0.0;
		float dirAngle = 0.0;
	};

	struct TRACK 
	{
		enum class STATE { ready, inprocess, finished };
		enum class DIRECTION { none, incomming, outcomming };

		unsigned int lostCount = 0;
		unsigned int id = 0;
		LINE directionVanishLine;
		STATE state = STATE::ready;
		std::vector<TRACKED_PLATE> points;

		DIRECTION GetDirection() 
		{
			if (points.size() < 2)
				return DIRECTION::none;

			if (points.back().plate.plateRect.y < points.front().plate.plateRect.y)
				return DIRECTION::outcomming;
			else
				return DIRECTION::incomming;
		}
	};

	unsigned int trackCount = 0;
	std::vector<TRACK*> plateTracks;
	cv::CascadeClassifier plateClassifierCascade;
	
public:

	PlateTracker();
	~PlateTracker();

	// This method creates and loads Haar cascade from xml-file
	bool InitClassifierCascade(const char* filename);

	// This is main function
	bool ProcessImage(cv::Mat grayImage);

	// This method returns finished track table
	bool GetFinishedTrackTable(unsigned int *IDs, unsigned int *count, unsigned int IDsSize);

	// This method returns finished track by ID
	bool GetTrack(unsigned int ID, PLATES_TRACK *track);

	// This method returns stat of tracks
	void GetTracksStat(int *empty, int *inprocess, int *finished);

	// This method prints tracks
	void PrintTracks(cv::Mat *image);

	//  NEW
	cv::Rect calculateCarRect(TRACKED_PLATE *plate); // TODO: сделать с направлением двжиения

private:

	// This method get empty new track from tracks vector
	TRACK* GetNewTrack();

	// This method clear track
	bool CleanTrack(TRACK* track);

	// This method separates new plates between tracks or creates new track
	void ProcessTracks(std::vector<TRACKED_PLATE> *newPlates);

	// This method computes finished track and checks that track is valid
	void FinishedTrackProccess();

	// This method searches for license plate in roi and push it to vector
	bool FindPlateROI( const cv::Mat image,							// Source gray image
					   cv::Rect searchArea,							// Searh area
					   cv::Size windowSize,							// Size of Haar searching window		
					   std::vector<TRACKED_PLATE> *platesInRoi,		// Vector in which new plates will save
					   double xboundScale = 0.1,					// X-axis plate bound scaling in percents from 0 to 0.2
					   double yboundScale = 0.1						// Y-axis plate bound scaling in percents from 0 to 0.4					
	);	
};

