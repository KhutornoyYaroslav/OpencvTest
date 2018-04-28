#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


class LINE {

	float A, B, C;
	cv::Point point1;
	cv::Point point2;

public:

	LINE() : A(0.0), B(1.0), C(0.0) {};
	LINE(cv::Point2f p1, cv::Point2f p2) { setendpoints(p1, p2); };

	void setendpoints(cv::Point2f p1, cv::Point2f p2) 
	{
		A = p1.y - p2.y;
		B = p2.x - p1.x;
		C = (p1.x * p2.y) - (p2.x * p1.y);

		point1 = p1;
		point2 = p2;
	};

	cv::Point getfirstpoint() { return point1; };
	cv::Point getsecondpoint() { return point2; };

	float k() { return -(A / B); };
	float b() { return -(C / B); };
	float y(float x) { return (x * k() + b()); };
	float x(float y) { return (y - b()) / k(); };

	float dist2point(cv::Point2f p) 
	{ 
		return abs(A*p.x + B*p.y + C) / sqrt(A*A + B*B); 
	};

	float angle2line(LINE l)
	{
		return acos((A*l.A + B*l.B) / (sqrt(A*A + B*B) * sqrt(l.A*l.A + l.B*l.B)));
	};

	bool intersection(LINE line, cv::Point2f *point)
	{
		float d = (this->A * line.B) - (this->B * line.A);
		if (abs(d) < 1e-9) return false;

		point->x = -((this->C * line.B) - (this->B * line.C)) / d;
		point->y = -((this->A * line.C) - (this->C * line.A)) / d;

		return true;
	};	

	float lenght()
	{
		float dx = point1.x - point2.x;
		float dy = point1.y - point2.y;
		return sqrt(dx*dx + dy*dy);
	};

	cv::Point2f midpoint()
	{
		float x = (point1.x + point2.x) / 2;
		float y = (point1.y + point2.y) / 2;
		return cv::Point2f(x, y);
	};
};



struct LICENSE_PLATE {

	// Plate ROI data
	cv::Mat plateImage;
	cv::Rect plateRect;

	// Car ROI data
	cv::Mat carImage;
	cv::Rect carRect;

	// Plate size data
	int plateWidth = 0;
	int plateHeight = 0;
};

struct PLATES_TRACK {

	LINE directionVanishLine;
	std::vector<LICENSE_PLATE> points;
};






class histogram {

	struct column {

		float minLimit;
		float maxLimit;
		std::vector<float> points;

		column(float min, float max)
		{ 
			minLimit = min;
			maxLimit = max;
		};
	};


	std::vector<column> hist;



	

public:







	float probabillity(float point) 
	{
		if (hist.empty()) return 0.0;

		int max = 0;
		int count = 0;

		for (auto h : hist)
		{
			if (h.points.size() > max)
			{
				max = h.points.size();
			}
		}

		if (max == 0) return 0.0;

		for (auto h : hist)
		{
			if (point > h.minLimit && point <= h.maxLimit)
			{
				count = h.points.size();
				break;
			}
		}

		return static_cast<float>(count) / static_cast<float>(max);
	};

};

int LineFitRANSAC(double t, double p, double e, int T, std::vector<cv::Point>& nzPoints, LINE *line);