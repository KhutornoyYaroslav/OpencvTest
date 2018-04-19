
#include "LicensePlateDetector.h"
#include "Zebra.h"

bool sort_func(LINE a, LINE b)
{
	return (a.y(1000) < b.y(1000));
}


// ****************** GOOD METHODS *****************

LicensePlateDetector::LicensePlateDetector() 
{
}

LicensePlateDetector::~LicensePlateDetector()
{
}

int LicensePlateDetector::LoadImageFromFile(const char* filename) {

	if (!colorImage.empty()) colorImage.release();
	
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

int LicensePlateDetector::FindTwoLineIntersection(LINE l1, LINE l2, cv::Point2f  *intersection) {

	float zn = (l1.A * l2.B) - (l1.B * l2.A);
	if (abs(zn) < 1e-9)
		return -1;

	float x = -((l1.C * l2.B) - (l1.B * l2.C)) / zn;
	float y = -((l1.A * l2.C) - (l1.C * l2.A)) / zn;
	
	intersection->x = x;
	intersection->y = y;
	return 0;
}

int LicensePlateDetector::FindVanishPointRANSAC(std::vector<LINE> lines, cv::Point2f *resultPoint, float radius) {

	int result_index = 0;
	int number_max = 0;
	int number = 0;
	std::vector<cv::Point2f> intersections;

	if (lines.empty()) return -1;
	if (FindLinesIntersections(lines, &intersections) < 0) return -1;

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

	if (number_max == 0) return -1;

	resultPoint->x = intersections.at(result_index).x;
	resultPoint->y = intersections.at(result_index).y;
	intersections.clear();

	return 0;
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
			float A = lines.at(j).A;
			float B = lines.at(j).B;
			float C = lines.at(j).C;

			if (A == 0.0 && B == 0.0) continue;

			float dist = abs(A*x + B*y + C) / sqrt(A*A + B*B);

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

	return sqrt((dist1 * dist2) - dist*dist);
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
	//cv::VideoCapture capture;

	CvCapture* capture = cvCreateFileCapture(filename);
	if (!capture) return false;

	//if (!capture.open(filename))
	//{
		//printf("Error while executing ProcessVideo(). Video file not found.\n");
		//return false;
	//}
	IplImage* frame = 0;

	while (true)
	{
		//if (!capture.read(this->colorImage)) break;	

		frame = cvQueryFrame(capture);
		if (!frame) break;

		colorImage = cv::cvarrToMat(frame);

		this->MainThread();
	}

	cvReleaseImage(&frame);
	return true;
}

bool LicensePlateDetector::MainThread()
{
	this->tracker.InitClassifierCascade("haarcascade_russian_plate_number.xml");
	this->tracker.ProcessImage(this->colorImage);
	this->tracker.PrintTracks(&colorImage);
	this->ShowImage(&colorImage, "Main color image", 1.0);
	cvWaitKey(100);

	return true;
}


int LicensePlateDetector::FindProbablyPlateWidth(LICENSE_PLATE *plate) {

	int delta = 0;
	int delta_max = -100;
	int x_max = 0;
	int x_min = 0;
	int offset = 0;
	int* arr = 0;
	double offset_koef = 4; // чем меньше, тем ближе к краям стартовая точка (меньше анализируемая область)
	double koef = 4;


	cv::Mat gray = plate->plateImage;

	//cvErode(gray, gray, 0, 2);
	//cvDilate(gray, gray, 0, 2);
	cv::erode(gray, gray, (0, 2));
	cv::dilate(gray, gray, (0, 2));

	//Считаем горизонтальную гистограмму
	arr = new int[gray.cols];
	for (int x = 0; x < gray.cols; x++) {

		arr[x] = 0;
		for (int y = 0; y < gray.rows; y++) {

			//arr[x] = arr[x] + CV_IMAGE_ELEM(gray, uchar, y, x);
			arr[x] = arr[x] + gray.at<uchar>(y, x);//CV_MAT_ELEM(gray, uchar, y, x);
		}
	}

	//Проходим от середины изображения к краям и находим вертикальные границы номера
	offset = gray.cols / offset_koef;

	for (int x = gray.cols / 2 + offset; x < gray.cols - 1; x++) {

		delta = abs(arr[x] - arr[x + 1]);
		if (delta > delta_max - (delta_max / koef)) {

			x_max = x;
			delta_max = delta;
		}
	}

	delta_max = -100;
	for (int x = gray.cols / 2 - offset; x > 0; x--) {

		delta = abs(arr[x] - arr[x - 1]);
		if (delta > delta_max - (delta_max / koef)) {

			x_min = x;
			delta_max = delta;
		}
	}

	plate->plateWidth = abs(x_max - x_min);


	//cvLine(this->colorImage, CvPoint(x_min + plate->plateROIleftX, 0 + plate->plateROIleftY), CvPoint(x_min + plate->plateROIleftX, gray->height + plate->plateROIleftY), CV_RGB(0, 255, 0), 5, CV_AA, 0);
	//cvLine(this->colorImage, CvPoint(x_max + plate->plateROIleftX, 0 + plate->plateROIleftY), CvPoint(x_max + plate->plateROIleftX, gray->height + plate->plateROIleftY), CV_RGB(0, 255, 0), 5, CV_AA, 0);

	///////////////////////////////////////
	//cvLine(plate->plateImage, CvPoint(x_min, 0 ), CvPoint(x_min , gray->height ), CV_RGB(255, 255, 255), 1, CV_AA, 0);
	//cvLine(plate->plateImage, CvPoint(x_max, 0 ), CvPoint(x_max , gray->height ), CV_RGB(255, 255, 255), 1, CV_AA, 0);
	//this->ShowImage(plate->plateImage, 4.0);
	//cvWaitKey(0);
	///////////////////////////////////////
	//ShowImage(&gray, "gray plate", 5.0);

	//Освобождаем ресурсы
	gray.release();
	delete[] arr;
	return 0;
}

bool LicensePlateDetector::GetVanishLinesFromPlate(LICENSE_PLATE *plate, std::vector<LINE> *lines)
{

	std::vector<cv::Vec2f> findedLines;

	cv::Mat grayImg;
	cv::cvtColor(plate->plateImage, grayImg, CV_BGR2GRAY);
	cv::blur(grayImg, grayImg, cv::Size(3, 3));
	cv::threshold(grayImg, grayImg, 0, 256, CV_THRESH_BINARY + CV_THRESH_OTSU);
	cv::Sobel(grayImg, grayImg, 0, 0, 2, 1);



	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(grayImg, contours, hierarchy, 0, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	/// Draw contours
	for (int i = 0; i< contours.size(); i++)
	{
		if (contours.at(i).size() < 155) continue;

		//cv::drawContours(plate->plateImage, contours, i, CV_RGB(255, 0, 0), 1, 8, hierarchy, 0, cv::Point());
		LINE lineR;
		//LineFitRANSAC(1.0, 0.8, 0.8, 0.8*contours.at(i).size(), contours.at(i), &lineR);
		//cv::line(plate->plateImage, cv::Point(0, lineR.y(0)), cv::Point(plate->plateImage.cols, lineR.y(plate->plateImage.cols)), cv::Scalar(0, 0, 255), 1, CV_AA);


		for (int j = 0; j < contours.at(i).size(); j++)
		{
			contours.at(i).at(j).x = contours.at(i).at(j).x + plate->plateRect.x;
			contours.at(i).at(j).y = contours.at(i).at(j).y + plate->plateRect.y;
		}


		//
		

		
		if (LineFitRANSAC(1.0, 0.8, 0.8, 0.8*contours.at(i).size(), contours.at(i), &lineR) > -1)
		{
			//lines->push_back(lineR);
			if ( lineR.y(colorImage.cols) < 1.0 * colorImage.rows / 2.0) upLines.push_back(lineR);
			if ( lineR.y(colorImage.cols) >= 1.0 * colorImage.rows / 2.0) downLines.push_back(lineR);
		}

		

		
		
	}




	//this->FindProbablyPlateWidth(plate);

	//findedLines.clear();
	//cv::HoughLines(grayImg, findedLines, 1, CV_PI / 180, plate->plateWidth * 0.8, 0.0, 0.0, 1.4, 1.8);

	//for (size_t i = 0; i < findedLines.size(); i++)
	//{
	//	float rho = findedLines[i][0], theta = findedLines[i][1];
	//	cv::Point pt1, pt2;
	//	double a = cos(theta), b = sin(theta);
	//	double x0 = a*rho, y0 = b*rho;
	//	pt1.x = cvRound(x0 + 1000 * (-b));
	//	pt1.y = cvRound(y0 + 1000 * (a));
	//	pt2.x = cvRound(x0 - 1000 * (-b));
	//	pt2.y = cvRound(y0 - 1000 * (a));

	//	cv::line(plate->plateImage, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);

	//	pt1.x = pt1.x + plate->plateRect.x;
	//	pt1.y = pt1.y + plate->plateRect.y;
	//	pt2.x = pt2.x + plate->plateRect.x;
	//	pt2.y = pt2.y + plate->plateRect.y;

	//	LINE line(pt1, pt2);


	//	//curY = line.y(colorImage.cols);
	//	//if (abs(curY - preY) > deltaY && curY > preY) {

	//	//preY = curY;
	//	if ( line.y(colorImage.cols) < colorImage.rows / 3.0) upLines.push_back(line);
	//	if ( line.y(colorImage.cols) >= 1.0 * colorImage.rows / 3.0) downLines.push_back(line);

	//	lines->push_back(line);
	//	//lines->push_back(line);
	//	cv::line(colorImage, cv::Point(0, line.y(0)), cv::Point(colorImage.cols, line.y(colorImage.cols)), cv::Scalar(0, 0, 255), 1, CV_AA);
	//	////}			
	//}

	//ShowImage(&plate->plateImage, "color car", 10.0);
	//ShowImage(&grayImg, "gray car", 10.0);
	//ShowImage(&colorImage, "main image", 0.5);
	//cvWaitKey(0);

	grayImg.release();

	return true;
}

bool LicensePlateDetector::GetVanishLinesFromCarTrack(PLATES_TRACK *track, std::vector<LINE> *lines)
{
	std::vector<cv::Vec2f> findedLines;
	float preY = 0;
	float curY = 0;
	//float deltaY = 100;


	for (auto p : track->points)
	{
		if (p.carImage.empty()) continue;

		cv::Mat grayImg;

		cv::cvtColor(p.carImage, grayImg, CV_BGR2GRAY);
		cv::blur(grayImg, grayImg, cv::Size(3, 3));
		cv::threshold(grayImg, grayImg, 0, 256, CV_THRESH_BINARY + CV_THRESH_OTSU);
		cv::Sobel(grayImg, grayImg, 0, 0, 2, 1);

		//FindProbablyPlateWidth(&p);
		//if (p.plateWidth == 0) p.plateWidth = 50;

		//findedLines.clear();
		//cv::HoughLines(grayImg, findedLines, 1, CV_PI / 90, p.plateWidth * 1.0, 0.0, 0.0, 1.3, 1.8);

		//for (size_t i = 0; i < findedLines.size(); i++)
		//{
		//	float rho = findedLines[i][0], theta = findedLines[i][1];
		//	cv::Point pt1, pt2;
		//	double a = cos(theta), b = sin(theta);
		//	double x0 = a*rho, y0 = b*rho;
		//	pt1.x = cvRound(x0 + 1000 * (-b));
		//	pt1.y = cvRound(y0 + 1000 * (a));
		//	pt2.x = cvRound(x0 - 1000 * (-b));
		//	pt2.y = cvRound(y0 - 1000 * (a));

		//	cv::line(p.carImage, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);

		//	pt1.x = pt1.x + p.carRect.x;
		//	pt1.y = pt1.y + p.carRect.y;
		//	pt2.x = pt2.x + p.carRect.x;
		//	pt2.y = pt2.y + p.carRect.y;

		//	LINE line(pt1, pt2);
		//	
		//	if ( line.y(colorImage.cols) < 1.0 * colorImage.rows / 3.0) upLines.push_back(line);
		//	if ( line.y(colorImage.cols) >= 1.0 * colorImage.rows / 3.0) downLines.push_back(line);

		//	cv::line(colorImage, cv::Point(0, line.y(0)), cv::Point(colorImage.cols, line.y(colorImage.cols)), cv::Scalar(0, 0, 255), 1, CV_AA);
		//	
		//	lines->push_back(line);
		//}


		//ShowImage(&p.carImage, "color car", 2.0);
		//ShowImage(&colorImage, "main", 0.5);
		//ShowImage(&grayImg, "gray car", 1.0);
		//cvWaitKey(0);


		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours(grayImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		/// Draw contours
		for (int i = 0; i< contours.size(); i++)
		{
			//cv::drawContours(p.carImage, contours, i, CV_RGB(255, 0, 0), 1, 8, hierarchy, 0, cv::Point());

			for (int j = 0; j < contours.at(i).size(); j++)
			{
				contours.at(i).at(j).x = contours.at(i).at(j).x + p.carRect.x;
				contours.at(i).at(j).y = contours.at(i).at(j).y + p.carRect.y;
			}


			
			//if (contours.at(i).size() < 45) continue;

			LINE lineR;
			LineFitRANSAC(2.0, 0.8, 0.8, 0.5*contours.at(i).size(), contours.at(i), &lineR);



			lines->push_back(lineR);
			cv::line(p.carImage, cv::Point(0, lineR.y(0)), cv::Point(colorImage.cols, lineR.y(colorImage.cols)), cv::Scalar(0, 0, 255), 1, CV_AA);
		}

		
		//ShowImage(&p.carImage, "madfgdfgin", 2.0);
		//ShowImage(&grayImg, "gray", 2.0);
		cvWaitKey(0);
		grayImg.release();
	}

	


	//ShowImage(&colorImage, "main", 0.5);
	//cvWaitKey(0);
		
	return true;
}



bool LicensePlateDetector::FilterVanishLines(std::vector<LINE> *lines)
{
	if (lines->empty()) return false;

	std::sort(lines->begin(), lines->end() , sort_func);

	float angleAver = 0.0;
	for (auto it = lines->begin(); it != lines->end(); it++)
	{

		angleAver = angleAver + abs(it->k());
	}
	angleAver = angleAver / lines->size();

	

	for (auto it = lines->begin(); it != lines->end(); it++)
	{

		printf("--->>> angleAver = %.3f\n", DEGREE_IN_RADIAN*atan(it->k()));

		if (DEGREE_IN_RADIAN*atan(it->k()) < -3.4 ||
			DEGREE_IN_RADIAN*atan(it->k()) > 3.4)
		{
			it = lines->erase(it);
			it = lines->begin();
		}
	}



	int curY = 0;
	int delta = 30;
	int preY = 0;

	for (auto it = lines->begin(); it != lines->end(); it++)
	{
		curY = it->y(1000);

		if ((curY - preY) < delta)
		{
			it = lines->erase(it);
			it = lines->begin();
			preY = 0;
		}
		else
		{
			preY = curY;
		}

	}


	return true;
}



int LicensePlateDetector::ProcessPlates(const char* filename) {

	int res = 0;

	res = this->LoadImageFromFile(filename);
	if (res < 0) return -2;

	tracker.InitClassifierCascade("haarcascade_russian_plate_number.xml");
	tracker.ProcessImage(colorImage);

	static std::vector<LINE> lines;

	unsigned int tracks[10];
	unsigned int count = 0;

	if (tracker.GetFinishedTrackTable(tracks, &count, 10))
	{
		for (int c = 0; c < count; c++)
		{
			PLATES_TRACK *track = new PLATES_TRACK;
			if (tracker.GetTrack(tracks[c], track))
			{
				plateTracks.push_back(track);

				directionLines.push_back(track->directionVanishLine);

				for (auto p : track->points)
				{
					
					GetVanishLinesFromPlate(&p, &lines);
					
				}

				
				
				//GetVanishLinesFromCarTrack(track, &lines);
				//FilterVanishLines(&lines);

				delete track;
			}
			else
			{
				delete track;
			}
		}	
	}
	
	static cv::Point2f point;
	static cv::Point2f point2;
	static cv::Point2f W;
	static float focal;
	static int size = 0;
	static std::vector<LINE> lines1;
	static std::vector<LINE> lines2;

	if (directionLines.size() != size)
	{
		size = directionLines.size();

		//this->FindVanishPointMinDistances(directionLines, &point2);
		this->FindVanishPointRANSAC(directionLines, &point2, 200);

		lines1.clear();
		lines2.clear();
		int mins = min(upLines.size(), downLines.size());

		printf("___>>>mins = %i\n", mins);

		for (int i = 0; i < mins; i++)
		{
			if (mins > 30) break;
			lines1.push_back(upLines.at(i));
			lines2.push_back(downLines.at(i));
		}

		
		static std::vector<cv::Point2f> inters;
		this->FindLinesIntersections2(lines1, lines2, &inters);
		this->FindVanishPointRANSAC2(inters, &point, 200);


		//upLines.clear(); 
		//downLines.clear();
		//this->FindVanishPointMinDistances(lines, &point);
		//this->FindVanishPointRANSAC(lines, &point, 200);
	}


	for (auto l : directionLines)
	{
		cv::line(colorImage, cv::Point(0, l.y(0)), cv::Point(colorImage.cols, l.y(colorImage.cols)), CV_RGB(255, 255, 0), 1);
	}

	//for (auto l : lines)
	//{
		//cv::line(colorImage, cv::Point(0, l.y(0)), cv::Point(colorImage.cols, l.y(colorImage.cols)), CV_RGB(0, 255, 0), 1);
	//}
	for (auto l : lines1)
	{
		cv::line(colorImage, cv::Point(0, l.y(0)), cv::Point(colorImage.cols, l.y(colorImage.cols)), CV_RGB(255, 0, 0), 1);
	}
	for (auto l : lines2)
	{
		cv::line(colorImage, cv::Point(0, l.y(0)), cv::Point(colorImage.cols, l.y(colorImage.cols)), CV_RGB(255, 0, 0), 1);
	}
	//tracker.PrintTracks(&colorImage);


	focal = ComputeFocalLenght(point, point2);
	ComputeRotationMatrix(focal, point, point2, cv::Point(0, 0));
	W = ComputeThirdVanishingPoint(point, point2, focal);

	PrintVanishVector(point, 250, 3, 0, 255, 0);
	PrintVanishVector(point2, 200, 3, 255, 0, 0);
	PrintVanishVector(W, 250, 3, 0, 0, 255);

	ShowImage(&colorImage, "main", 0.6);
	cvWaitKey(1);


	
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

int LicensePlateDetector::FindLinesIntersections(std::vector<LINE> lines, std::vector<cv::Point2f> *result) { //TODO: сделать проверку того, что ваниш лайн есть

	cv::Point2f intersection;

	if (lines.empty()) return -1;

	for (int i = 0; i < lines.size(); i++) {

		LINE l1 = lines.at(i);

		for (int j = 0; j < lines.size(); j++) {

			if (j <= i) continue;

			LINE l2 = lines.at(j);

			if (FindTwoLineIntersection(l1, l2, &intersection) >= 0) {

				result->push_back(intersection);
			}
		}
	}

	if (result->empty()) return -1;

	return 0;
}

void LicensePlateDetector::ComputeRotationMatrix(float focal_lenght, cv::Point2f v1, cv::Point2f v2, cv::Point2f v3) {

	// X column
	float R11 = v1.x / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);
	float R21 = v1.y / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);
	float R31 = focal_lenght / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);

	// Y column
	float R12 = v2.x / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);
	float R22 = v2.y / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);
	float R32 = focal_lenght / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);

	// Z column
	float R13 = (R21 * R32 - R31 * R22);
	float R23 = (R31 * R12 - R11 * R32);
	float R33 = (R11 * R22 - R21 * R12);

	//printf("R11 = %.3f | R12 = %.3f | R13 = %.3f\n", R11, R12, R13);
	//printf("R21 = %.3f | R22 = %.3f | R23 = %.3f\n", R21, R22, R23);
	//printf("R31 = %.3f | R32 = %.3f | R33 = %.3f\n", R31, R32, R33);

	double cosYangle = sqrt(R11 * R11 + R21 * R21);
	double rotXangle = atan2(R32, R33);
	double rotZangle = atan2(R21, R11);
	double rotYangle = atan2(-R31, cosYangle);


	char text[256];
	
	sprintf_s(text, sizeof(text), "Rotate: %.1f Sl: %.1f Roll: %.1f Focal: %.1f", 
		DEGREE_IN_RADIAN * rotYangle,
		90 - DEGREE_IN_RADIAN * rotXangle, 
		DEGREE_IN_RADIAN * rotZangle,
		focal_lenght);

	cv::putText(colorImage, text, cv::Point(200, 1600), 2, 2.0, CV_RGB(255, 200, 45) , 3);
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

			if (FindTwoLineIntersection(l1, l2, &intersection) >= 0) {

				result->push_back(intersection);
			}
		}
	}

	if (result->empty()) return false;

	return true;
}



bool LicensePlateDetector::FindVanishPointRANSAC2(std::vector<cv::Point2f> intersections, cv::Point2f *resultPoint, float radius) {

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

	if (number_max == 0) return -1;

	resultPoint->x = intersections.at(result_index).x;
	resultPoint->y = intersections.at(result_index).y;
	intersections.clear();

	return 0;
}