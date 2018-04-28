
#include "LicensePlateDetector.h"
#include "Zebra.h"

bool sort_func(LINE a, LINE b)
{
	return (a.y(0) < b.y(0));
}

bool sort_vec_size(std::vector<cv::Point> &a, std::vector<cv::Point> &b)
{
	return (a.size() < b.size());
}


// ****************** GOOD METHODS *****************

LicensePlateDetector::LicensePlateDetector() 
{
}

LicensePlateDetector::~LicensePlateDetector()
{
}

int LicensePlateDetector::LoadImageFromFile(const char* filename) {

	if (!colorImage.empty()) {

		colorImage.copyTo(preColorImage);
		colorImage.release();
	}
	
	colorImage = cv::imread(filename);

	if (colorImage.empty()) {

		printf("Error while load image from file. File not found.\n");
		return -1;
	}
	else {

		return 0;
	}
}

bool LicensePlateDetector::ShowImage(cv::Mat *image, const char *wndName, double scale) {

	if (image->empty()) {

		printf("Error while showing image. There is no image.\n");
		return false;
	}

	if (!wndName) {

		printf("Error while showing image. There is no window name.\n");
		return false;
	}

	cv::Mat scaleImage;
	cv::resize(*image, scaleImage, cvSize(image->cols * scale, image->rows * scale));
	cv::imshow(wndName, scaleImage);
	scaleImage.release();
	return true;
}





int LicensePlateDetector::FindVanishPointMinDistances(std::vector<LINE> lines, cv::Point2f *resultPoint) {

	float sum = 0.0;
	float min_sum = 10000000.0;
	int result_index = 0;
	std::vector<cv::Point2f> intersections;

	if (lines.empty()) return -1;
	if (FindLinesIntersections(lines, &intersections) < 0) return -1;

	for (int i = 0; i < intersections.size(); i++) {

		float x = intersections.at(i).x;
		float y = intersections.at(i).y;
		sum = 0.0;

		for (int j = 0; j < lines.size(); j++) {

			//TODO: упроситить код. дистанцию через линию сделать (структкура метод)
			//float A = lines.at(j).A;
			//float B = lines.at(j).B;
			//float C = lines.at(j).C;

			//if (A == 0.0 && B == 0.0) continue;

			//float dist = abs(A*x + B*y + C) / sqrt(A*A + B*B);

			float dist = lines.at(j).dist2point(cv::Point2f(x, y));

			dist = dist / 1000;
			sum = sum + dist;
		}

		if (sum < min_sum) {

			min_sum = sum;
			result_index = i;
		}
	}

	resultPoint->x = intersections.at(result_index).x;
	resultPoint->y = intersections.at(result_index).y;
	intersections.clear();

	return 0;
}

float LicensePlateDetector::ComputeFocalLenght(cv::Point2f vp1, cv::Point2f vp2) {

	float dist = 0.0;
	float dist1 = 0.0;
	float dist2 = 0.0;
	cv::Point2f PrincipalPoint;
	LINE horizont(vp1, vp2);

	PrincipalPoint.x = static_cast<float>(colorImage.cols / 2);
	PrincipalPoint.y = static_cast<float>(colorImage.rows / 2);

	dist = horizont.dist2point(PrincipalPoint);

	float dist1_pre = sqrt((PrincipalPoint.x - vp1.x)*(PrincipalPoint.x - vp1.x) + (PrincipalPoint.y - vp1.y)*(PrincipalPoint.y - vp1.y));
	dist1 = sqrt(dist1_pre*dist1_pre - dist*dist);
	
	float dist2_pre = sqrt((PrincipalPoint.x - vp2.x)*(PrincipalPoint.x - vp2.x) + (PrincipalPoint.y - vp2.y)*(PrincipalPoint.y - vp2.y));
	dist2 = sqrt(dist2_pre*dist2_pre - dist*dist);

	//dist1 = sqrt((PrincipalPoint.x - vp1.x)*(PrincipalPoint.x - vp1.x) + (PrincipalPoint.y - vp1.y)*(PrincipalPoint.y - vp1.y));//TODO: не принципал...
	//dist2 = sqrt((PrincipalPoint.x - vp2.x)*(PrincipalPoint.x - vp2.x) + (PrincipalPoint.y - vp2.y)*(PrincipalPoint.y - vp2.y));//TODO: не принципал...

	//return sqrt((dist1 * dist2) - dist*dist);
	return sqrt(dist1_pre*dist2_pre - dist*dist); //TODO: все же, как правильно?
}

cv::Point2f LicensePlateDetector::ComputeThirdVanishingPoint(cv::Point2f vp1, cv::Point2f vp2, float focal) {

	cv::Point2f thirdPoint;
	cv::Point2f PrincipalPoint;

	PrincipalPoint.x = static_cast<float>(colorImage.cols / 2);
	PrincipalPoint.y = static_cast<float>(colorImage.rows / 2);
	thirdPoint.x = ((vp1.y - PrincipalPoint.y) * focal) - (focal * (vp2.y - PrincipalPoint.y));
	thirdPoint.y = (focal * (vp2.x - PrincipalPoint.x)) - ((vp1.x - PrincipalPoint.x) * focal);

	return thirdPoint;
}


// ****************** METHODS IN PROCESS *****************

bool LicensePlateDetector::ProcessVideo(const char* filename)
{
	cv::VideoCapture capture;

	//CvCapture* capture = cvCreateFileCapture(filename);
	//if (!capture) return false;

	if (!capture.open(filename))
	{
		printf("Error while executing ProcessVideo(). Video file not found.\n");
		return false;
	}
	//IplImage* frame = 0;

	while (true)
	{

		colorImage.copyTo(preColorImage);

		if (!capture.read(this->colorImage)) break;	

		//frame = cvQueryFrame(capture);
		//if (!frame) break;

		//colorImage = cv::cvarrToMat(frame);

		this->MainThread();
	}

	//cvReleaseImage(&frame);
	return true;
}

bool LicensePlateDetector::MainThread()
{
	//this->tracker.InitClassifierCascade("haarcascade_russian_plate_number.xml");
	//this->tracker.ProcessImage(this->colorImage);
	//this->tracker.PrintTracks(&colorImage);
	//this->ShowImage(&colorImage, "Main color image", 1.0);
	//cvWaitKey(100);
	printf("lolol\n");
	this->GetCarsContours();


	return true;
}






bool LicensePlateDetector::GetVanishLineFromPlate(LICENSE_PLATE *plate, LINE* line)
{
	if (plate == nullptr) return false;
	if (line == nullptr) return false;

	LINE foundLine;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat grayImg;
	int mid = plate->plateRect.tl().x + plate->plateRect.width / 2;	
	int Ymax = 0;

	cv::cvtColor(plate->plateImage, grayImg, CV_BGR2GRAY);
	cv::blur(grayImg, grayImg, cv::Size(3, 3));
	cv::threshold(grayImg, grayImg, 0, 256, CV_THRESH_BINARY + CV_THRESH_OTSU);
	cv::Sobel(grayImg, grayImg, 0, 0, 2, 1);
	cv::findContours(grayImg, contours, hierarchy, 0, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	grayImg.release();

	std::sort(contours.begin(), contours.end(), sort_vec_size);
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < contours.back().size() - contours.back().size() / 5) continue;
		//if (contours[i].size() < 220) continue;

		int maxX = 0;
		int minX = plate->plateImage.cols;

		for (int j = 0; j < contours[i].size(); j++)
		{
			if (contours[i][j].x > maxX) maxX = contours[i][j].x;
			if (contours[i][j].x < minX) minX = contours[i][j].x;
		}
		
		if (maxX - minX < plate->plateRect.width / 1.5) continue;
		
		for (int j = 0; j < contours.at(i).size(); j++)
		{
			contours.at(i).at(j).x = contours.at(i).at(j).x + plate->plateRect.x;
			contours.at(i).at(j).y = contours.at(i).at(j).y + plate->plateRect.y;
		}
		
		if (LineFitRANSAC(1.0, 0.8, 0.8, 0.8*contours.at(i).size(), contours.at(i), &foundLine) > -1)
		{
			if (foundLine.y(mid) > Ymax)
			{
				Ymax = foundLine.y(mid);
			}
		}	
	}

	*line = foundLine;

	if (Ymax > 0)
		return true;
	else
		return false;
}

bool LicensePlateDetector::GetVanishLineFromCar(LICENSE_PLATE *plate, LINE* line, double angle) {

	if (line == nullptr) return false;
	if (plate == nullptr) return false;
	if (plate->carImage.empty()) return false;

	cv::Mat gray;
	cv::cvtColor(plate->carImage, gray, CV_BGR2GRAY);
	cv::blur(gray, gray, cv::Size(5, 5));
	cv::Canny(gray, gray, 50, 200);


	//std::vector<std::vector<cv::Point> > contours;
	//std::vector<cv::Vec4i> hierarchy;
	//findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	//for (int i = 0; i< contours.size(); i++)
	//{
	//	cv::drawContours(plate->carImage, contours, i, CV_RGB(255, 0,0), 1, 8, hierarchy, 0, cv::Point());
	//}


	std::vector<cv::Vec2f> lines;
	int lenght = plate->carRect.width / 5;
	cv::HoughLines(gray, lines, 1, CV_PI / 180, lenght, 0, 0, angle - 0.1, angle + 0.1); // TODO: 0.2 ?
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));

		cv::line(plate->carImage, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);

		cv::Point2f p1((pt1.x + plate->carRect.x), (pt1.y + plate->carRect.y));
		cv::Point2f p2((pt2.x + plate->carRect.x), (pt2.y + plate->carRect.y));
		LINE l(p1, p2);
		*line = l;
	}

	//std::vector<cv::Vec4i> lines;
	//cv::HoughLinesP(gray, lines, 1, CV_PI / 180, 25, 120, 20);
	//for (size_t i = 0; i < lines.size(); i++)
	//{
	//	cv::Vec4i l = lines[i];
	//	cv::line(plate->carImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
	//}


	//ShowImage(&plate->carImage, "lol", 1.0);
	//ShowImage(&gray, "gray", 1.0);
	//cvWaitKey(0);

	return true;
}


void LicensePlateDetector::ProcessRoadDirectionLines(std::vector<LINE> lines) {

	if (lines.empty()) return;

	cv::Point2f vanishPoint;
	std::vector<cv::Point2f> intersections;

	if (!FindLinesIntersections(lines, &intersections)) return; // TODO: нужна ли эта функци€ ?

	int pointNumber = FindVanishPointRANSAC(intersections, &vanishPoint, VANISH_RANSAC_RADIUS);

	float dX = (vanishPoint.x - roadDirectionVP.x);
	float dY = (vanishPoint.y - roadDirectionVP.y);
	float delta = sqrt(dX*dX + dY*dY);

	if (vanishPoint == roadDirectionVP) return;

	this->roadDirectionVP = vanishPoint;
	printf("--->>>> directVP. Delta: %.3f | Num: %i\n", delta, pointNumber);
}

void LicensePlateDetector::PrintIntersections(std::vector<cv::Point2f> intersecions, cv::Point2f vanish) {

	if (intersecions.empty()) return;

	cv::Mat img(2050 * 5, 40480, CV_8UC3, RGB(255, 255, 255));

	for (auto i : intersecions)
	{
		i.x = i.x / 1;
		i.y = i.y / 1 + 2 * 2050;
		cv::circle(img, i, 50, CV_RGB(255, 0, 0), 50);
	}

	vanish.y = vanish.y + 2 * 2050;
	cv::circle(img, vanish, 90, CV_RGB(0, 255, 0), 90);

	ShowImage(&img, "points", 0.04);
	cvWaitKey(1);
	img.release();
}


cv::Point2f LicensePlateDetector::FindVanishPoint(std::vector<cv::Point2f> intersecions, float sigma) {

	cv::Point2f resultPoint;

	std::vector<float> points;

	for (auto i : intersecions)
	{
		float x = i.x;
		points.push_back(x);
	}
	
	//histogram hist;
	//hist.create(-50000.0, 50000.0, 1000.0);
	//hist.addPoints(points);

	float left = 0.0;
	float right = 0.0;
	
	for (float f = -50000.0; f < 50000.0; f = f + 1000.0)
	{
		//printf("hist prob: %.5f\n", hist.probabillity(f));
		//if (hist.probabillity(f + 1) > 0.6)
		//{
			//left = hist.col(f + 1).minLimit;
			//right = hist.col(f + 1).maxLimit;
		//}
	}


	//this->FindVanishPointRANSAC()


	

	
	return resultPoint;
}




void LicensePlateDetector::ProcessRoadPerpendicularLines(std::map<float, LINE> *lines) {

	// ѕроверить, есть ли достаточное количество и в нужном диапазоне
	// ѕроредить и pop из вектора
	// ѕрофильтровать на средний угол
	// –азделить на два диапазона
	// Ќайти пересечени€ между лини€ми из двух диапазонов
	// —охранить пересечени€ в вектор или найти ваниш поинт и сохранить ее в вектор
	// Ќайти одну ваниш среди найденых ваниш
	std::vector<LINE> filterLines;

	static int size = 0;
	if (lines->size() == size) return;
	size = lines->size();

	int step = colorImage.rows / 40; // parameter

	if (abs(lines->rbegin()->first - lines->begin()->first) < colorImage.rows / 2) return;
	if (lines->size() < 20) return;

	auto first = lines->begin();
	filterLines.push_back(first->second);


	for (auto second = lines->begin(); second != lines->end(); second++)
	{
		float dY = second->first - first->first;
		if (dY >= step)
		{
			filterLines.push_back(second->second);
			first = second;
		}
	}

	//lines->clear(); // TODO:

	
	static std::vector<cv::Point2f> intersections;


	cv::Point2f vanishPoint;
	if (!FindLinesIntersections(filterLines, &intersections)) return; // TODO: нужна ли эта функци€ ?



	int pointNumber = FindVanishPointRANSAC(intersections, &vanishPoint, VANISH_RANSAC_RADIUS);

	vanishPoint =this->FindVanishPoint(intersections, 0.5);
	//this->PrintIntersections(intersections, vanishPoint);
	//cvWaitKey(0);

	float dX = (vanishPoint.x - roadPerpendicularVP.x);
	float dY = (vanishPoint.y - roadPerpendicularVP.y);
	float delta = sqrt(dX*dX + dY*dY);



	if (vanishPoint == roadPerpendicularVP) return;
	this->roadPerpendicularVP = vanishPoint;
	//printf("--->>>> perpVP. Delta: %.3f | Num: %i\n", delta, pointNumber);




	//for (auto line : filterLines)
	//{
		//cv::line(colorImage, cv::Point(0, line.y(0)), cv::Point(colorImage.cols, line.y(colorImage.cols)), CV_RGB(255, 0, 0), 2);
	//}
	//printf("Perp lines number: %i\n", lines->size());

	//ShowImage(&colorImage, "dfdf", 0.5);
	//cvWaitKey(0);

}





int LicensePlateDetector::ProcessPlates(const char* filename) {

	if (this->LoadImageFromFile(filename) < 0) return -1;
	

	tracker.InitClassifierCascade("haarcascade_russian_plate_number.xml");
	tracker.ProcessImage(colorImage);


	unsigned int tracks[10];
	unsigned int count = 0;

	if (tracker.GetFinishedTrackTable(tracks, &count, 10))
	{
		for (int c = 0; c < count; c++)
		{
			PLATES_TRACK track;
			if (tracker.GetTrack(tracks[c], &track))
			{
				roadDirectionLines.push_back(track.directionVanishLine);

				for (auto plate : track.points)
				{
					LINE line;
					if (GetVanishLineFromPlate(&plate, &line))
					{
						roadPerpendicularLines.push_back(line);
					}
					//GetVanishLineFromCar(&plate, &line, 1.57);
					//roadPerpendicularLines.push_back(line);
				}
			}
		}	
	}
	

	//this->ProcessRoadDirectionLines(roadDirectionLines);


	//xVanisher.ComputeVanishingPoint(roadPerpendicularLines, &roadPerpendicularVP, Vanisher::VanishType::x);
	//roadPerpendicularLines.clear();
	
	static int size = 0;
	if (size != roadPerpendicularLines.size())
	{
		size = roadPerpendicularLines.size();
		xVanisher.ComputeRothersVP(roadPerpendicularLines, &roadPerpendicularVP);
		xVanisher.ComputeRothersVP(roadDirectionLines, &roadDirectionVP);
	}


	//printf("VP.x = %i | VP.y = %i\n", roadPerpendicularVP.x, roadPerpendicularVP.y);
	//cv::Mat image = xVanisher.PrintVanishPoints(4000, 500);
	//ShowImage(&image, "points", 0.5);

	//this->roadPerpendicularVP.x = 12500;
	//this->roadPerpendicularVP.y = 0;

	float focal = this->ComputeFocalLenght(this->roadDirectionVP, this->roadPerpendicularVP);	
	this->zenithVP = this->ComputeThirdVanishingPoint(this->roadDirectionVP, this->roadPerpendicularVP, focal);
	this->ComputeRotationMatrix(focal, this->roadDirectionVP, this->roadPerpendicularVP, zenithVP);

	this->PrintVanishVector(this->roadDirectionVP, 200, 3, 255, 0, 0);
	this->PrintVanishVector(this->roadPerpendicularVP, 250, 3, 0, 255, 0);
	this->PrintVanishVector(this->zenithVP, 250, 3, 0, 0, 255);
	
	ShowImage(&colorImage, "main", 0.45);
	cvWaitKey(1);
	//this->GetCarsContours();

	return 0;
}




void LicensePlateDetector::PrintVanishVector(cv::Point vanishPoint, int lenght, int number, int R, int G, int B) {

	//printf("vanishPoint: x = %i; y = %i\n", vanishPoint.x, vanishPoint.y);

	int delta_x = static_cast<int>(colorImage.cols / (number + 1));
	int delta_y = static_cast<int>(colorImage.rows / (number + 1));

	for (int x = delta_x; x < delta_x * (number + 1); x = x + delta_x) {
		for (int y = delta_y; y < delta_y * (number + 1); y = y + delta_y) {

			LINE vector(cv::Point2f(x, y), cv::Point2f(vanishPoint.x, vanishPoint.y));

			int eps = 5;
			float x2 = 0;
			int step = 1;

			if (vanishPoint.x > x) {
				x2 = colorImage.cols;
				step = step * -1;
			}

			while( (x - x2)*(x - x2) + (y - vector.y(x2))*(y - vector.y(x2)) - lenght*lenght > eps)
			{
				x2 = x2 + step;
			}

			cv::arrowedLine(colorImage, cv::Point(x, y), cv::Point(x2, vector.y(x2)), CV_RGB(R, G, B), 4);
			cv::circle(colorImage, cv::Point(x, y), 8, CV_RGB(255, 255, 255), 5, 8, 0);
		}
	}
}

bool LicensePlateDetector::FindLinesIntersections(std::vector<LINE> lines, std::vector<cv::Point2f> *result) { //TODO: сделать проверку того, что ваниш лайн есть

	cv::Point2f intersection;

	if (lines.empty()) return false;

	for (int i = 0; i < lines.size(); i++) {

		LINE l1 = lines.at(i);

		for (int j = 0; j < lines.size(); j++) {

			if (j <= i) continue;

			LINE l2 = lines.at(j);

			if(l1.intersection(l2, &intersection))
			{
				result->push_back(intersection);
			}
		}
	}

	if (result->empty()) return false;

	return true;
}

void LicensePlateDetector::ComputeRotationMatrix(float focal_lenght, cv::Point2f v1, cv::Point2f v2, cv::Point2f v3) {

	cv::Mat_<float> R = cv::Mat_<float>(cv::Size(3, 3), 0);

	// X column
	R(0, 0) = v1.x / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);
	R(1, 0) = v1.y / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);
	R(2, 0) = focal_lenght / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);

	// Y column
	R(0, 1) = v2.x / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);
	R(1, 1) = v2.y / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);
	R(2, 1) = focal_lenght / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);

	// Z column
	R(0, 2) = (R(1, 0) * R(2, 1) - R(2, 0) * R(1, 1));
	R(1, 2) = (R(2, 0) * R(0, 1) - R(0, 0) * R(2, 1));
	R(2, 2) = (R(0, 0) * R(1, 1) - R(1, 0) * R(0, 1));



	//double cosYangle = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
	//double rotXangle = atan2(R(2, 1), R(2, 2));
	//double rotZangle = atan2(R(1, 0), R(0, 0));
	//double rotYangle = atan2(-R(2, 0), cosYangle);

	char text[256];
	
	float focal_mm = focal_lenght * 0.0035;	
	float aoH = DEGREE_IN_RADIAN *	2 * atanf(9.93 / (2 * focal_mm));
	float aoV = DEGREE_IN_RADIAN * 2 * atanf(8.7 / (2 * focal_mm));

	sprintf_s(text, sizeof(text), "Yaw: %.1f Pitch: %.1f Roll: %.1f FL: %.1f AoH: %.1f AoV: %1.f",
		90 - DEGREE_IN_RADIAN * GetRotatation(v1, v2)[1],
		DEGREE_IN_RADIAN * GetRotatation(v1, v2)[0],
		DEGREE_IN_RADIAN * GetRotatation(v1, v2)[2],
		focal_mm,
		aoH,
		aoV);

	cv::putText(colorImage, text, cv::Point(100, colorImage.rows - 100), 2, 2.25, CV_RGB(255, 200, 45) , 3);
}




bool LicensePlateDetector::FindLinesIntersections2(std::vector<LINE> lines1, std::vector<LINE> lines2, std::vector<cv::Point2f> *result)
{

	cv::Point2f intersection;

	if (lines1.empty()) return false;
	if (lines2.empty()) return false;

	for (int i = 0; i < lines1.size(); i++) {

		LINE l1 = lines1.at(i);

		for (int j = 0; j < lines2.size(); j++) {

			if (j <= i) continue;

			LINE l2 = lines2.at(j);

			if (l1.intersection(l2, &intersection)) {
			
				result->push_back(intersection);
			}
		}
	}

	if (result->empty()) return false;

	return true;
}

int LicensePlateDetector::FindVanishPointRANSAC(std::vector<cv::Point2f> intersections, cv::Point2f *resultPoint, float radius) {

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

	//if (number_max == 0) return false;

	resultPoint->x = intersections.at(result_index).x;
	resultPoint->y = intersections.at(result_index).y;
	intersections.clear();

	return number_max;
}



float LicensePlateDetector::GetAngleBetweenVector(cv::Vec3f v1, cv::Vec3f v2) {

	return acos(v1.dot(v2) / (cv::norm(v1) * cv::norm(v2)));
}

cv::Vec3f LicensePlateDetector::GetRotatation(cv::Point2f vp1, cv::Point2f vp2) {

	float Beta = 1.0;
	cv::Point2f pp(colorImage.cols / 2, colorImage.rows / 2);
	cv::Point2f vp3 = GetThirdVanishPoint(Beta, pp, vp1, vp2);
	vp3 = ComputeThirdVanishingPoint(vp1, vp2, ComputeFocalLenght(vp1, vp2));
	cv::Mat_<float> R = cv::Mat_<float>(3,3);

	R(0, 0) = GetLambdas(pp, vp1, vp2, vp3)[0] * (vp1.x - pp.x) / ComputeFocalLenght(vp1, vp2);
	R(0, 1) = GetLambdas(pp, vp1, vp2, vp3)[1] * (vp2.x - pp.x) / ComputeFocalLenght(vp1, vp2);
	R(0, 2) = GetLambdas(pp, vp1, vp2, vp3)[2] * (vp3.x - pp.x) / ComputeFocalLenght(vp1, vp2);

	R(1, 0) = GetLambdas(pp, vp1, vp2, vp3)[0] * (vp1.y - pp.y) / (Beta * ComputeFocalLenght(vp1, vp2));
	R(1, 1) = GetLambdas(pp, vp1, vp2, vp3)[1] * (vp2.y - pp.y) / (Beta * ComputeFocalLenght(vp1, vp2));
	R(1, 2) = GetLambdas(pp, vp1, vp2, vp3)[2] * (vp3.y - pp.y) / (Beta * ComputeFocalLenght(vp1, vp2));

	R(2, 0) = GetLambdas(pp, vp1, vp2, vp3)[0];
	R(2, 1) = GetLambdas(pp, vp1, vp2, vp3)[1];
	R(2, 2) = GetLambdas(pp, vp1, vp2, vp3)[2];

	//printf("R00 = %.1f | R01 = %.1f | R02 = %.1f \n", R(0, 0) , R(0, 1), R(0, 2));
	//printf("R10 = %.1f | R11 = %.1f | R12 = %.1f \n", R(1, 0), R(1, 1), R(1, 2));
	//printf("R20 = %.1f | R21 = %.1f | R22 = %.1f \n", R(2, 0), R(2, 1), R(2, 2));


	// roll
	float roll = GetAngleBetweenVector(cv::Vec3f(R(0, 2), R(1, 2), 0), cv::Vec3f(0, R(1,2), 0)); // ROLL ? norm ?
	//float roll = GetAngleBetweenVector(cv::Vec3f(R(0, 0), R(1,0), 0), cv::Vec3f(R(0, 0), 0, 0)); // ROLL ? norm ?

	// pitch
	//float pitch = GetAngleBetweenVector(cv::Vec3f(0, R(1, 2), R(2, 2)), cv::Vec3f(0, R(1, 2), 0)); // Slope?
	float pitch = GetAngleBetweenVector(cv::Vec3f(0, R(1, 1), R(2, 1)), cv::Vec3f(0, 0, R(2, 1))); // Slope?

	// yaw
	float yaw = GetAngleBetweenVector(cv::Vec3f(R(0,1), 0, R(2,1)), cv::Vec3f(0, 0, R(2, 1))); // Rot
	//float yaw = GetAngleBetweenVector(cv::Vec3f(R(0, 0), 0, R(2, 0)), cv::Vec3f(R(0, 0), 0, 0)); // ROT !!!!

	return (cv::Vec3f(pitch, yaw, roll));
}

cv::Point2f LicensePlateDetector::GetThirdVanishPoint(float Beta, cv::Point2f pp, cv::Point2f vp1, cv::Point2f vp2) {

	float u3, v3, e, f, a, b, c, d;

	a = vp2.y - pp.y;
	b = vp2.x - pp.x;
	c = vp1.x - pp.x;
	d = vp1.y - pp.y;

	e = a / (Beta * b);
	f = (Beta * c) / d;

	u3 = (-e*vp1.y + e*f*vp2.x + e*vp2.y + vp1.x) / (1 + e*f);
	v3 = ((Beta * c * (vp2.x - u3)) / d) + vp2.y;

	return cv::Point2f(u3, v3);
}

cv::Vec3f LicensePlateDetector::GetLambdas(cv::Point2f pp, cv::Point2f vp1, cv::Point2f vp2, cv::Point2f vp3) {

	float L1 = ((pp.y-vp3.y)*(vp2.x-vp3.x) - (pp.x-vp3.x)*(vp2.y-vp3.y)) / ((vp1.y-vp3.y)*(vp2.x-vp3.x) - (vp1.x-vp3.x)*(vp2.y-vp3.y));	
	float L2 = ((vp1.y - vp3.y)*(pp.x - vp3.x) - (vp1.x - vp3.x)*(pp.y - vp3.y)) / ((vp1.y - vp3.y)*(vp2.x - vp3.x) - (vp1.x - vp3.x)*(vp2.y - vp3.y));
	float L3 = 1 - L1 + L2;

	return cv::Vec3f(sqrt(L1), sqrt(L2), sqrt(L3));
}









void LicensePlateDetector::GetCarsContours() {

	if (colorImage.empty()) return;
	if (preColorImage.empty()) return;

	cv::Mat image;
	cv::Mat gray;
	cv::Mat pregray;
	cv::Mat motion;

	colorImage.copyTo(image);
	cv::cvtColor(colorImage, gray, CV_BGR2GRAY);
	cv::cvtColor(preColorImage, pregray, CV_BGR2GRAY);


	//cv::blur(gray, gray, cv::Size(3, 3));
	//cv::blur(pregray, pregray, cv::Size(3, 3));

	cv::absdiff(gray, pregray, motion);
	//cv::threshold(motion, motion, 0, 255, cv::THRESH_OTSU);	
	//cv::dilate(motion, motion, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(13, 13)), cv::Point(-1, -1), 1);

	cv::blur(motion, motion, cv::Size(7, 7));

	cv::Canny(motion, motion, 50, 200);

	std::vector<cv::Vec2f> lines;
	cv::HoughLines(motion, lines, 1, CV_PI / 180, 100, 0, 0, 1.4, 1.7);

	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));

		LINE line(pt1, pt2);


		cv::line(image, cv::Point(0, line.y(0)), cv::Point(image.cols, line.y(image.cols)), cv::Scalar(0, 0, 255), 1, CV_AA);
	}












	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(motion, contours, hierarchy, 1, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	
	for (size_t i = 0; i< contours.size(); i++)
	{		
		if (contours[i].size() < 800) continue;
		
		std::vector<cv::Point> result;

		convexHull(contours[i], result);

		//cv::drawContours(image, contours, (int)i, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0, cv::Point());
		//cv::polylines(image, result, true, CV_RGB(0, 0, 255), 3);

		PrintCarBound(result, &image);
	}

	ShowImage(&image, "main", 0.45);
	ShowImage(&motion, "motion", 0.4);
	cvWaitKey(1);

}

void LicensePlateDetector::PrintCarBound(std::vector<cv::Point> contour, cv::Mat *image) {

	//cv::Point2f vp1(21000, 1500);
	//cv::Point2f vp2(50, -500);
	//cv::Point2f vp3(colorImage.cols / 2 + 300, -50000);

	//cv::Point2f vp1(10000, 300);
	//cv::Point2f vp2(-800, -500);
	//cv::Point2f vp3(colorImage.cols / 2 + 300, -50000);


	cv::Point2f vp1 = this->roadPerpendicularVP;
	cv::Point2f vp2 = this->roadDirectionVP;
	cv::Point2f vp3 = this->ComputeThirdVanishingPoint(vp1, vp2, this->ComputeFocalLenght(vp1, vp2));



	//TOP point
	int Ymax = 10000;
	cv::Point Ptop;
	for (auto p : contour)
	{
		if (p.y < Ymax)
		{
			Ymax = p.y;
			Ptop = p;
		}
	}

	//BOOTOM point
	int Ymin = 0;
	cv::Point Pbot;
	for (auto p : contour)
	{
		if (p.y > Ymin)
		{
			Ymin = p.y;
			Pbot = p;
		}
	}

	//LEFT point
	int Xmin = 10000;
	cv::Point Pleft;
	for (auto p : contour)
	{
		if (p.x < Xmin)
		{
			Xmin = p.x;
			Pleft = p;
		}
	}

	//RIGHT point
	int Xmax = 0;
	cv::Point Pright;
	for (auto p : contour)
	{
		if (p.x > Xmax)
		{
			Xmax = p.x;
			Pright = p;
		}
	}


	// Horizontal lines
	LINE hor_back_top(Ptop, vp1);
	LINE hor_front_bottom(Pbot, vp1);
	LINE hor_front_top; 
	LINE hor_back_bottom;

	// Directional lines
	LINE dir_left_bottom(Pleft, vp2);
	LINE dir_left_top;
	LINE dir_right_top;
	LINE dir_right_bottom;

	// Vertical lines
	LINE vert_left_back(Pleft, vp3);
	LINE vert_right_front(Pright, vp3);
	LINE vert_left_front;
	LINE vert_right_back;

	// Intersection points
	cv::Point2f p1, p2, p3, p4, p5, p6, p7, p8;



	vert_left_back.intersection(dir_left_bottom, &p5);
	hor_back_bottom = LINE(p5, vp1);

	vert_right_front.intersection(hor_front_bottom, &p2);
	dir_right_bottom = LINE(p2, vp2);

	dir_left_bottom.intersection(hor_front_bottom, &p1);
	vert_left_front = LINE(p1, vp3);

	hor_back_top.intersection(vert_left_back, &p8);
	dir_left_top = LINE(p8, vp2);

	vert_left_front.intersection(dir_left_top, &p4);
	hor_front_top = LINE(p4, vp1);

	hor_front_top.intersection(vert_right_front, &p3);
	dir_right_top = LINE(p3, vp2);

	hor_back_top.intersection(dir_right_top, &p7);
	vert_right_back = LINE(p7, vp3);

	hor_back_bottom.intersection(dir_right_bottom, &p6);




	float dx = p1.x - p5.x;
	float dy = p1.y - p5.y;
	float dlina = sqrt(dx*dx + dy*dy);
	//if (dlina > 350 || dlina < 50) return; // dlina

	dx = p1.x - p4.x;
	dy = p1.y - p4.y;
	float height = sqrt(dx*dx + dy*dy);
	//if (height > 500 || height < 50) return; // height

	dx = p1.x - p2.x;
	dy = p1.y - p2.y;
	float width = sqrt(dx*dx + dy*dy);
	//if (width > 500 || width < 50) return; // width

	if (width / height > 2) return;
	if (height / width > 2) return;
	//if (dlina / height > 1.3) return;
	//if (dlina / width > 1.3) return;


	cv::line(*image, p1, p2, CV_RGB(205, 0, 0), 4);
	cv::line(*image, p1, p5, CV_RGB(205, 0, 0), 4);
	cv::line(*image, p1, p4, CV_RGB(205, 0, 0), 4);

	cv::line(*image, p3, p2, CV_RGB(205, 0, 0), 4);
	cv::line(*image, p3, p4, CV_RGB(205, 0, 0), 4);
	cv::line(*image, p3, p7, CV_RGB(205, 0, 0), 4);

	cv::line(*image, p8, p5, CV_RGB(205, 0, 0), 4);
	cv::line(*image, p8, p4, CV_RGB(205, 0, 0), 4);
	cv::line(*image, p8, p7, CV_RGB(205, 0, 0), 4);

	cv::line(*image, p6, p7, CV_RGB(0, 55, 0), 4);
	cv::line(*image, p6, p5, CV_RGB(0, 55, 0), 4);
	cv::line(*image, p6, p2, CV_RGB(0, 55, 0), 4);
}