#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"

#define DEGREE_IN_RADIAN 57.295779513
#define PI 3.1415926535

#define ROTHERS_ANGLE_K 0.3
#define ROTHERS_ANGLE_MAX 10.0
#define ROTHERS_LENGHT_K 0.7



class Vanisher
{

public:

	enum class VanishType { x, y };

private:

	struct Histogram
	{
		struct Column
		{
			int low;
			int high;
			std::vector<cv::Point2f> points;
			int size() { return points.size(); }
		};

		std::vector<Column> slots;
		int size() { return slots.size(); }
	};

	Histogram hist;
	VanishType vtype;
	bool isProcess = false;
	cv::Point2f vanishPoint;


	std::vector<LINE> allLines;
	std::vector<cv::Point2f> intersections;

public:		
	cv::Mat *image;

public:
	Vanisher();
	~Vanisher();
	bool ComputeVanishingPoint(std::vector<LINE> input_lines, cv::Point2f *result_point, VanishType vtype); // MAIN FUNC
	bool ComputeVanishingPoint2(std::vector<LINE> input_lines, cv::Point2f *result_point, VanishType vtype); // MAIN FUNC

	bool ComputeRothersVP(std::vector<LINE> lines, cv::Point2f *vp);

	void ClearHist();
	cv::Mat PrintVanishPoints(int w, int h);

private:

	void CreateHist(int low, int high, int slot_number);
	void AddPointsToHist(std::vector<cv::Point2f> points);
	bool GetMaxSlotSize(int *result);
	bool GetProbability(int slot, float *result);

	bool GetLinesInersections(std::vector<LINE> input_lines, std::vector<cv::Point2f> *intersections, int step, bool groups = false);


	bool ProcessHist(int *slot, float sigma); //TODO: по названию не понятно, что делает этот метод

	bool GetVanishPointRANSAC(std::vector<cv::Point2f> intersections, cv::Point2f *resultPoint, float radius);



};

