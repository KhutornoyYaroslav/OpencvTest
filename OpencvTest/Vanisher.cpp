
#include "Vanisher.h"

bool SortLinesY(LINE a, LINE b)
{
	return (a.b() < b.b());
}

Vanisher::Vanisher()
{
}

Vanisher::~Vanisher()
{
}

void Vanisher::ClearHist()
{
	isProcess = false;
	hist.slots.clear();
}

void Vanisher::CreateHist(int low, int high, int slot_number)
{
	ClearHist();

	int range = abs(high - low);
	int delta = range / slot_number;

	for (int c = 0; c < slot_number; c++)
	{
		Histogram::Column new_column;
		new_column.low = delta * c;
		new_column.high = delta * (c + 1);
		hist.slots.push_back(new_column);
	}
}

void Vanisher::AddPointsToHist(std::vector<cv::Point2f> points)
{
	if (hist.size() == 0) return;
	if (points.empty()) return;

	for (auto s = hist.slots.begin(); s != hist.slots.end(); s++)
	{
		for (auto p : points)
		{
			float val;

			if (vtype == VanishType::x)
			{
				val = p.x;
			}

			if (vtype == VanishType::y)
			{
				val = p.y;
			}

			if (val >= s->low && val < s->high)
			{
				s->points.push_back(p);
			}
		}
	}
}

bool Vanisher::GetMaxSlotSize(int *result)
{
	if (hist.size() == 0) return false;
	int max = 0;

	for (auto s = hist.slots.begin(); s != hist.slots.end(); s++)
	{
		if (s->size() > max) max = s->size();
	}

	if (max < 1) return false;

	*result = max;
	return true;
}

bool Vanisher::GetProbability(int slot, float *result)
{
	if (slot < 0) return false;
	if (hist.size() <= slot) return false;

	int slot_size = hist.slots.at(slot).size();
	int max_size = 0;
	if (!GetMaxSlotSize(&max_size)) return false;

	*result = static_cast<float>(slot_size) / static_cast<float>(max_size);

	return true;
}

bool Vanisher::GetLinesInersections(std::vector<LINE> input_lines, std::vector<cv::Point2f> *intersections, int step, bool groups)
{
	if (input_lines.empty()) return false;
	if (intersections == nullptr) return false;

	std::vector<LINE> vanish_lines;
	cv::Point2f intersection;

	std::sort(input_lines.begin(), input_lines.end(), SortLinesY);


	auto l1 = input_lines.begin();
	vanish_lines.push_back(*l1);

	for (auto l2 = input_lines.begin(); l2 != input_lines.end(); l2++)
	{
		float dY = l2->b() - l1->b();
		if (dY >= step)// && l2->b() < 2800 && l2->b() > 2500)
		{
			vanish_lines.push_back(*l2);
			l1 = l2;
		}
	}

	//int mid = vanish_lines.size() / 2;
	// l1 0... < gsize
	// l2 gisze ... < vanish_lines.size()
	//for (auto l : vanish_lines)
	//{
		//cv::line(*image, cv::Point(0, l.y(0) / 10 + 500 / 2), cv::Point(4000, l.y(40000) / 10 + 500 / 2), CV_RGB(0, 0, 255), 1);
	//}


	for (auto l1 = vanish_lines.begin(); l1 != vanish_lines.end(); l1++) {
		for (auto l2 = vanish_lines.begin(); l2 != vanish_lines.end(); l2++) {

			if (l1 <= l2) continue;

			if (l1->intersection(*l2, &intersection))
			{
				intersections->push_back(intersection);
			}
		}
	}

	//for (int l1 = 0; l1 < mid; l1++) {
	//	for (int l2 = mid; l2 < vanish_lines.size(); l2++) {

	//		if(vanish_lines.at(l1).intersection(vanish_lines.at(l2), &intersection))
	//			intersections->push_back(intersection);
	//	}
	//}

	return true;
}



bool Vanisher::ProcessHist(int *slot, float sigma) //tODO: пока что только в правую сторону будет искать
{
	if (slot == nullptr) return false;
	if (hist.size() == 0) return false;

	int result = 0;
	float p = 0.0;

	for (int s = 0; s < hist.size(); s++)
	{
		p = 0.0;
		if(!GetProbability(s, &p)) continue;

		if (p > sigma)
		{
			result = s;
		}
	}

	GetProbability(result, &p);
	printf("++++ probability = %.3f\n", p);

	*slot = result;
	return true;
}

bool Vanisher::GetVanishPointRANSAC(std::vector<cv::Point2f> intersections, cv::Point2f *resultPoint, float radius)
{
	if (intersections.empty()) return false;
	if (resultPoint == nullptr) return false;
	if (radius < 1) return false;

	int result_index = 0;
	int number_max = 0;
	int number = 0;

	for (int i = 0; i < intersections.size(); i++) {

		float x1 = intersections.at(i).x;
		float y1 = intersections.at(i).y;
		number = 0;

		for (int j = 0; j < intersections.size(); j++) {

			if (j == i) continue;

			float x2 = intersections.at(j).x;
			float y2 = intersections.at(j).y;

			float dist = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
			if (dist < radius)
				number++;
		}

		if (number > number_max) {

			result_index = i;
			number_max = number;
		}
	}

	if (number_max == 0) return false;

	resultPoint->x = intersections.at(result_index).x;
	resultPoint->y = intersections.at(result_index).y;

	return true;
}

cv::Mat Vanisher::PrintVanishPoints(int w, int h) {

	if (image == nullptr)
	{
		image = new cv::Mat(h, w, CV_8UC3, CV_RGB(200, 200, 200));
	}


	for (auto s : hist.slots)
	{
		for (auto p : s.points)
		{
			p.x = p.x / 10;
			p.y = (p.y / 10) + h / 2;
			cv::circle(*image, p, 2, CV_RGB(255, 0, 0), 2);
		}
	}

	cv::Point2f vp;
	vp.x = vanishPoint.x / 10;
	vp.y = (vanishPoint.y / 10) + h / 2;
	cv::circle(*image, vp, 5, CV_RGB(0, 255, 0), 5);

	return *image;
}

bool Vanisher::ComputeVanishingPoint(std::vector<LINE> input_lines, cv::Point2f *result_point, VanishType van_type)
{
	if (input_lines.empty()) return false;
	if (result_point == nullptr) return false;

	cv::Point2f result;
	std::vector<cv::Point2f> intersections;

	if (isProcess == false)
	{
		CreateHist(0, 200000, 500);
		vtype = van_type;
		isProcess = true;		
	}

	if (hist.size() == 0) return false;

	
	if (!GetLinesInersections(input_lines, &intersections, 150)) return false;

	AddPointsToHist(intersections);
	int slot = 0;
	if (!ProcessHist(&slot, 0.4)) return false;
	if (!GetVanishPointRANSAC(hist.slots[slot].points, &result, 200)) return false; // parameter

	
	vanishPoint = result;
	*result_point = result;

	return true;
}

bool Vanisher::ComputeVanishingPoint2(std::vector<LINE> input_lines, cv::Point2f *result_point, VanishType vtype) {

	if (input_lines.empty()) return false;
	if (result_point == nullptr) return false;

	for (auto l : input_lines)
	{
		this->allLines.push_back(l);
	}


	for (auto l1 = allLines.begin(); l1 != allLines.end(); l1++) {
		for (auto l2 = allLines.begin(); l2 != allLines.end(); l2++) {

			if (l1 <= l2) continue;
			cv::Point2f inter;
			if (l1->intersection(*l2, &inter))
			{
				intersections.push_back(inter);
			}
		}
	}


	if (image == nullptr)
	{
		image = new cv::Mat(500, 4000, CV_8UC3, CV_RGB(200, 200, 200));
	}

	for (auto i : intersections)
	{
		i.x = i.x / 10;
		i.y = (i.y / 10) + 500 / 2;
		cv::circle(*image, i, 2, CV_RGB(255, 0, 0), 2);
	}

	double scale = 0.5;
	cv::Mat scaleImage;
	cv::resize(*image, scaleImage, cvSize(image->cols * scale, image->rows * scale));
	cv::imshow("dsd", scaleImage);
	scaleImage.release();



	cv::Point2f result;

	if (!GetVanishPointRANSAC(intersections, &result, 200)) return false; // parameter

	vanishPoint = result;
	*result_point = result;


	cv::Point2f vp;
	vp.x = vanishPoint.x / 10;
	vp.y = (vanishPoint.y / 10) + 500 / 2;
	cv::circle(*image, vp, 5, CV_RGB(0, 255, 0), 5);

	printf("vanish.x = %.1f |", vanishPoint.x);
	printf("vanish.y = %.1f\n", vanishPoint.y);

	return true;
}

bool Vanisher::ComputeRothersVP(std::vector<LINE> lines, cv::Point2f *vp) {

	if (lines.empty()) return false;
	if (vp == nullptr) return false;

	cv::Point2f inter;
	std::vector<cv::Point2f> intersections;

	for (auto l1 = lines.begin(); l1 != lines.end(); l1++)
	{
		for (auto l2 = lines.begin(); l2 != lines.end(); l2++)
		{
			if (l1 <= l2) continue;
			if (l1->intersection(*l2, &inter)) 
				intersections.push_back(inter);	
		}
	}

	float max_lenght = 0.0;
	for (auto l : lines)
	{
		if (l.lenght() > max_lenght)
		{
			max_lenght = l.lenght();
		}
	}

	float max_dangle = ROTHERS_ANGLE_MAX * PI / 180.0;
	float max_weight = 0.0;
	cv::Point2f max_point;

	for (auto point : intersections)
	{
		float weight = 0.0;

		for (auto l : lines)
		{
			LINE line2vp(l.midpoint(), point);
			float dangle = abs(l.angle2line(line2vp));
			if (dangle > PI / 2) dangle = PI - dangle; 
			float w = ROTHERS_ANGLE_K * (1 - dangle / max_dangle) + ROTHERS_LENGHT_K * (l.lenght() / max_lenght);
			weight = weight + w;
		}

		if (weight > max_weight)
		{
			max_weight = weight;
			max_point = point;
		}
	}

	*vp = max_point;

	return true;
}