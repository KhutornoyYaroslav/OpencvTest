
#include "LicensePlateDetector.h"
#include "Zebra.h"
// ****************** GOOD METHODS *****************

LicensePlateDetector::LicensePlateDetector() {

	for (int i = 0; i < PLATE_TRACKS_VECTOR_SIZE; i++)
	{
		PLATE_TRACK *track = new PLATE_TRACK;
		plateTracks.push_back(track);
	}
}

LicensePlateDetector::~LicensePlateDetector()
{
}

int LicensePlateDetector::LoadImageFromFile(const char* filename) {

	if (this->colorImage) cvReleaseImage(&this->colorImage);

	this->colorImage = cvLoadImage(filename, 1);
	if (this->colorImage == NULL) {

		printf("Error while load image from file. File not found.\n");
		return -1;
	}
	else {

		return 0;
	}
}

int LicensePlateDetector::ShowColorImage(double scale)
{
	if (this->colorImage == NULL) {

		printf("Error while show color image. There is no color image.\n");
		return -1;
	}
	else {

		IplImage *scaleimg = cvCreateImage(cvSize(this->colorImage->width * scale, this->colorImage->height * scale), this->colorImage->depth, this->colorImage->nChannels);
		cvResize(this->colorImage, scaleimg);
		cvShowImage("Color Image", scaleimg);
		cvReleaseImage(&scaleimg);
		return 0;
	}
}

int LicensePlateDetector::ShowImage(IplImage* image, double scale) {

	if (image == NULL) {

		printf("Error while showing image. There is no image.\n");
		return -1;
	}
	else {

		IplImage *scaleimg = cvCreateImage(cvSize(image->width * scale, image->height * scale), image->depth, image->nChannels);
		cvResize(image, scaleimg);
		cvShowImage("Image", scaleimg);
		cvReleaseImage(&scaleimg);
		return 0;
	}
}

IplImage* LicensePlateDetector::ConvertColorImage()
{
	if (this->colorImage == NULL) {

		printf("Error while convert to gray image. There is no color image.\n");
		return NULL;
	}
	else {

		IplImage* binaryImage = cvCreateImage(cvGetSize(this->colorImage), IPL_DEPTH_8U, 1);
		cvConvertImage(this->colorImage, binaryImage, CV_BGR2GRAY);

		return binaryImage;
	}
}

int LicensePlateDetector::FindTwoLineIntersection(LINE l1, LINE l2, CvPoint2D32f *intersection) {

	float zn = (l1.A * l2.B) - (l1.B * l2.A);
	if (abs(zn) < 1e-9)
		return -1;

	float x = -((l1.C * l2.B) - (l1.B * l2.C)) / zn;
	float y = -((l1.A * l2.C) - (l1.C * l2.A)) / zn;

	intersection->x = x;
	intersection->y = y;
	return 0;
}

int LicensePlateDetector::LineFitRANSAC(double t, double p, double e, int T, std::vector<cv::Point>& nzPoints, LINE *line) {

	LINE tmpLine;
	int bestLineIdx = 0;
	int bestLineScore = 0;
	std::vector<LINE> lineCandidates;
	std::vector<int> lineValidPoints;
	cv::RNG rng((uint64)-1);
	int s = 2;
	int N = (int)ceilf(log(1 - p) / log(1 - pow(1 - e, s)));

	for (int i = 0; i < N; i++)
	{
		int idx1 = (int)rng.uniform(0, (int)nzPoints.size());
		int idx2 = (int)rng.uniform(0, (int)nzPoints.size());
		cv::Point p1 = nzPoints[idx1];
		cv::Point p2 = nzPoints[idx2];

		if (cv::norm(p1 - p2) < t) continue;

		//line equation ->  (y1 - y2)X + (x2 - x1)Y + x1y2 - x2y1 = 0 
		double a = static_cast<double>(p1.y - p2.y);
		double b = static_cast<double>(p2.x - p1.x);
		double c = static_cast<double>(p1.x*p2.y - p2.x*p1.y);

		//normalize them
		double scale = 1.0 / sqrt(a*a + b*b);
		a *= scale;
		b *= scale;
		c *= scale;

		tmpLine.A = a;
		tmpLine.B = b;
		tmpLine.C = c;

		//count inliers
		int numOfInliers = 0;
		for (size_t i = 0; i < nzPoints.size(); ++i)
		{
			cv::Point& p0 = nzPoints[i];
			double rho = abs(a*p0.x + b*p0.y + c);
			bool isInlier = rho  < t;
			if (isInlier) numOfInliers++;
		}

		if (numOfInliers < T) continue;

		lineCandidates.push_back(tmpLine);
		lineValidPoints.push_back(numOfInliers);
	}

	for (size_t i = 0; i < lineCandidates.size(); i++)
	{
		if (lineValidPoints.at(i) > bestLineScore)
		{
			bestLineIdx = i;
			bestLineScore = lineValidPoints.at(i);
		}
	}

	if (lineCandidates.empty()) {

		return -1;
	}
	else {

		*line = lineCandidates[bestLineIdx];
		lineCandidates.clear();
		lineValidPoints.clear();
		return 0;
	}
}

int LicensePlateDetector::FindPlateROI(CvRect ROI, CvSize minSize, CvSize maxSize, double scaleFactor, double xboundScale, double yboundScale) {

	IplImage* grayImage = 0;
	std::vector<cv::Rect> symbols;
	PLATE_OBJECT plateRoi;
	int x_offset = 0;
	int y_offset = 0;

	if (this->colorImage == NULL) {

		printf("Error while executing FindPlateROI(). There is no source color Image.\n");
		return -1;
	}

	if (plateClassifierCascade.empty()) {

		printf("Error while executing FindPlateROI(). Classifier cascade is not loaded.\n");
		return -1;
	}

	if (xboundScale < 0) {

		printf("Warning while executing FindPlateROI(). Argument xboundScale is less than 0. Its will set to 0,01.\n");
		xboundScale = 0.01;
	}

	if (xboundScale > 0.2) {

		printf("Warning while executing FindPlateROI(). Argument xboundScale is too high. Its will set to 0,2.\n");
		xboundScale = 0.2;
	}

	if (yboundScale < 0) {

		printf("Warning while executing FindPlateROI(). Argument yboundScale is less than 0. Its will set to 0,05.\n");
		yboundScale = 0.05;
	}

	if (yboundScale > 0.4) {

		printf("Warning while executing FindPlateROI(). Argument yboundScale is too high. Its will set to 0,4.\n");
		yboundScale = 0.4;
	}

	if (scaleFactor < 1.1) {

		printf("Warning while executing FindPlateROI(). Argument scaleFactor is less than 1,1. Its will set to 1,1.\n");
		scaleFactor = 1.1;
	}

	if (scaleFactor > 2.0) {

		printf("Warning while executing FindPlateROI(). Argument scaleFactor is morse than 2,0. Its will set to 2,0.\n");
		scaleFactor = 2.0;
	}

	if (minSize.height <= 0 || minSize.width <= 0) {

		printf("Error while executing FindPlateROI(). Argument minWnd is less or equal zero.\n");
		return -1;
	}

	if (maxSize.height <= 0 || maxSize.width <= 0) {

		printf("Error while executing FindPlateROI(). Argument maxWnd is less or equal zero.\n");
		return -1;
	}

	if (ROI.x + ROI.width > this->colorImage->width || ROI.width < 1 || ROI.x < 0) {

		printf("Error while executing FindPlateROI(). ROI width is out of frame border.\n");
		return -1;
	}

	if (ROI.y + ROI.height > this->colorImage->height || ROI.height < 1 || ROI.y < 0) {

		printf("Error while executing FindPlateROI(). ROI height is out of frame border.\n");
		return -1;
	}

	grayImage = cvCreateImage(cvGetSize(this->colorImage), IPL_DEPTH_8U, 1);
	cvConvertImage(this->colorImage, grayImage, CV_BGR2GRAY);

	//Detect plates
	cvSetImageROI(grayImage, ROI);
	//----- Debug code -----
	//cvDrawRect(this->colorImage, CvPoint(ROI.x, ROI.y), CvPoint(ROI.x + ROI.width, ROI.y + ROI.height), CV_RGB(0, 200, 255), 5, 8, 0);
	//CvFont font;
	//cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 2.0, 2.0, 0, 2, CV_AA);
	//cvPutText(this->colorImage, "Region of interest", CvPoint(ROI.x + 20, ROI.y - 20), &font, CV_RGB(0, 200, 255));
	//--- End Debug code ---
	cv::Mat matImage = cv::cvarrToMat(grayImage);
	plateClassifierCascade.detectMultiScale(matImage, symbols, scaleFactor, 3, 0, minSize, maxSize); // parameters
	cvResetImageROI(grayImage);

	if (symbols.size() == 0) {

		cvReleaseImage(&grayImage);
		symbols.clear();
		return -1;
	}

	for (auto& p : symbols) {

		x_offset = static_cast<int>((static_cast<double>(p.width) * xboundScale));
		y_offset = static_cast<int>((static_cast<double>(p.height) * yboundScale));

		plateRoi.plateROIleftX = ROI.x + p.x - x_offset;
		plateRoi.plateROIleftY = ROI.y + p.y - y_offset;
		plateRoi.plateROIwidth = p.width + 2 * x_offset;
		plateRoi.plateROIheight = p.height + 2 * y_offset;
		
		if (plateRoi.plateROIleftX < 0 || plateRoi.plateROIleftY < 0 ||
			plateRoi.plateROIleftX + plateRoi.plateROIwidth > this->colorImage->width ||
			plateRoi.plateROIleftY + plateRoi.plateROIheight > this->colorImage->height) continue;

		cvSetImageROI(grayImage, CvRect(plateRoi.plateROIleftX, plateRoi.plateROIleftY, plateRoi.plateROIwidth, plateRoi.plateROIheight));
		plateRoi.plateImage = cvCreateImage(CvSize(plateRoi.plateROIwidth, plateRoi.plateROIheight), IPL_DEPTH_8U, 1);
		cvCopy(grayImage, plateRoi.plateImage);
		cvResetImageROI(grayImage);


		//----- Debug code -----
		//cvDrawRect(this->colorImage, CvPoint(plateRoi.plateROIleftX, plateRoi.plateROIleftY),
		//CvPoint(plateRoi.plateROIleftX + plateRoi.plateROIwidth, plateRoi.plateROIleftY + plateRoi.plateROIheight),
		// CV_RGB(0, 255, 125), 2, 8, 0);
		//std::cout << "X: " << plateRoi.x_coord << " Y: " << plateRoi.y_coord 
		//	<< " Width: " << plateRoi.width << " Height: " << plateRoi.height << std::endl;
		//IplImage *scaleimg = cvCreateImage(cvSize(plateRoi.width * 4, plateRoi.height * 4), plateRoi.plateImage->depth, plateRoi.plateImage->nChannels);
		//cvResize(plateRoi.plateImage, scaleimg);
		//cvShowImage("plate ROI", scaleimg);
		//printf("plateRoiImage sizes: w = %i, h = %i.\n", plateRoi.plateImage->width, plateRoi.plateImage->height);
		//--- End Debug code ---

		this->plateVector.push_back(plateRoi);
	}

	cvReleaseImage(&grayImage);
	symbols.clear();
	return 0;
}

int LicensePlateDetector::InitClassifierCascade(const char* filename) {

	bool isload = plateClassifierCascade.load(filename);
	if (isload == false)
	{
		printf("Error while InitClassifierCascade(). No xml-file was found.\n");
		return -1;
	}

	return 0;
}

int LicensePlateDetector::FindVanishPointRANSAC(std::vector<LINE> lines, CvPoint2D32f *resultPoint, float radius) {

	int result_index = 0;
	int number_max = 0;
	int number = 0;
	std::vector<CvPoint2D32f> intersections;

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

int LicensePlateDetector::FindVanishPointMinDistances(std::vector<LINE> lines, CvPoint2D32f *resultPoint) {

	float sum = 0.0;
	float min_sum = 10000000.0;
	int result_index = 0;
	std::vector<CvPoint2D32f> intersections;

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

int LicensePlateDetector::PrintPlateHorizontalLines(PLATE_OBJECT plate, int leftXcoord, int rightXcoord) {

	CvPoint p1;
	CvPoint p2;

	if (this->colorImage == NULL) {

		printf("Error while showing plate horizontal lines. There is no color Image.\n");
		return -1;
	}

	if (plate.horLinesCount == 0) {

		printf("Error while showing plate horizontal lines. There is no horizontal lines.\n");
		return -1;
	}

	if (leftXcoord < 0) leftXcoord = 0;
	if (rightXcoord > this->colorImage->width) rightXcoord = this->colorImage->width;

	for (int i = 0; i < plate.horLinesCount; i++) {

		p1.x = leftXcoord;
		p1.y = static_cast<int>(plate.horLines[i].y(leftXcoord));

		p2.x = rightXcoord;
		p2.y = static_cast<int>(plate.horLines[i].y(static_cast<float>(rightXcoord)));

		cvLine(this->colorImage, p1, p2, CV_RGB(0, 0, 255), 2, CV_AA, 0);
	}

	return 0;
}

int LicensePlateDetector::PrintPlateVanishHorLine(PLATE_OBJECT plate, int leftXcoord, int rightXcoord) {

	CvPoint p1;
	CvPoint p2;

	if (this->colorImage == NULL) {

		printf("Error while showing plate horizontal vanish line. There is no color Image.\n");
		return -1;
	}

	if (plate.isHorVanishLine == false) {

		printf("Error while showing plate horizontal vanish line. There is no vanish line.\n");
		return -1;
	}

	if (leftXcoord < 0) leftXcoord = 0;
	if (rightXcoord > this->colorImage->width) rightXcoord = this->colorImage->width;

	p1.x = leftXcoord;
	p1.y = static_cast<int>(plate.horVanishLine.y(leftXcoord));

	p2.x = rightXcoord;
	p2.y = static_cast<int>(plate.horVanishLine.y(static_cast<float>(rightXcoord)));

	cvLine(this->colorImage, p1, p2, CV_RGB(250, 50, 50), 2, CV_AA, 0);

	return 0;
}

float LicensePlateDetector::ComputeFocalLenght(CvPoint2D32f vp1, CvPoint2D32f vp2) {

	float dist = 0.0;
	float dist1 = 0.0;
	float dist2 = 0.0;
	CvPoint2D32f PrincipalPoint;
	LINE horizont(vp1, vp2);

	PrincipalPoint.x = static_cast<float>(colorImage->width / 2);
	PrincipalPoint.y = static_cast<float>(colorImage->height / 2);

	dist = horizont.dist2point(PrincipalPoint);

	float dist1_pre = sqrt((PrincipalPoint.x - vp1.x)*(PrincipalPoint.x - vp1.x) + (PrincipalPoint.y - vp1.y)*(PrincipalPoint.y - vp1.y));
	dist1 = sqrt(dist1_pre*dist1_pre - dist*dist);
	
	float dist2_pre = sqrt((PrincipalPoint.x - vp2.x)*(PrincipalPoint.x - vp2.x) + (PrincipalPoint.y - vp2.y)*(PrincipalPoint.y - vp2.y));
	dist2 = sqrt(dist2_pre*dist2_pre - dist*dist);

	//dist1 = sqrt((PrincipalPoint.x - vp1.x)*(PrincipalPoint.x - vp1.x) + (PrincipalPoint.y - vp1.y)*(PrincipalPoint.y - vp1.y));//TODO: не принципал...
	//dist2 = sqrt((PrincipalPoint.x - vp2.x)*(PrincipalPoint.x - vp2.x) + (PrincipalPoint.y - vp2.y)*(PrincipalPoint.y - vp2.y));//TODO: не принципал...

	return sqrt((dist1 * dist2) - dist*dist);
}

CvPoint2D32f LicensePlateDetector::ComputeThirdVanishingPoint(CvPoint2D32f vp1, CvPoint2D32f vp2, float focal) {

	CvPoint2D32f thirdPoint;
	CvPoint2D32f PrincipalPoint;

	PrincipalPoint.x = static_cast<float>(colorImage->width / 2);
	PrincipalPoint.y = static_cast<float>(colorImage->height / 2);
	thirdPoint.x = ((vp1.y - PrincipalPoint.y) * focal) - (focal * (vp2.y - PrincipalPoint.y));
	thirdPoint.y = (focal * (vp2.x - PrincipalPoint.x)) - ((vp1.x - PrincipalPoint.x) * focal);

	return thirdPoint;
}

// ****************** METHODS IN PROCESS *****************


int LicensePlateDetector::FindPlateHorizontalLines(PLATE_OBJECT *plate) {

	std::vector<LINE> horizontalLineVector;// NEW

	IplImage* binaryImage = 0;
	CvMemStorage* contours_storage = 0;
	CvSeq* contours = 0;
	std::vector<cv::Point> points;
	CvSeqReader seq_reader;
	CvMat kernel_matrix;
	LINE line;
	//SLine ransacLine;
	float sharp_kernel[9] = { -0.1f, -0.1f, -0.1f, -0.1f, 2.0f, -0.1f, -0.1f, -0.1f, -0.1f }; // parameter

	//Cleaning and initing
	horizontalLineVector.clear();
	points.clear();	
	binaryImage = cvCloneImage(plate->plateImage);

	//Image sharpening, adaptive treshloding, x-axis smoothing and canny edge detector
	kernel_matrix = cvMat(3, 3, CV_32FC1, sharp_kernel);
	cvFilter2D(binaryImage, binaryImage, &kernel_matrix, cvPoint(-1, -1));
	//cvAdaptiveThreshold(binaryImage, binaryImage, 255, 0, 0, 13, 1.0); // parameter
	cvThreshold(binaryImage, binaryImage, 0, 128, CV_THRESH_BINARY + CV_THRESH_OTSU);
	cvSmooth(binaryImage, binaryImage, CV_GAUSSIAN, 43, 1); // parameter
	cvCanny(binaryImage, binaryImage, 185, 255, 3); // parameter

	//Contours finding
	contours_storage = cvCreateMemStorage(0);
	cvFindContours(binaryImage, contours_storage, &contours, sizeof(CvContour), 0, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

	for (CvSeq* current = contours; current != NULL; current = current->h_next) {

		//TODO: что-то придумать здесь нормальное
		if (current->total < 185) continue; // parameter 185

		//Reading of all points from current contour and saving its to vector.
		points.clear();
		cvStartReadSeq(current, &seq_reader, -1);
		for (int i = 0; i < current->total; i++) {

			cv::Point point;
			CV_READ_SEQ_ELEM(point, seq_reader);
			point.x = point.x + plate->plateROIleftX;//NEW
			point.y = point.y + plate->plateROIleftY;//NEW
			points.push_back(point);
		}

		//Find RANSAC fitting lines 
		if (LineFitRANSAC(1.5, 0.85, 0.85, current->total * 0.85, points, &line) < 0) continue; // parameter

		horizontalLineVector.push_back(line);
	}
	
	if (horizontalLineVector.size() == 0) return -1;
	//printf("horizontalLineVector.size() = %i\n", horizontalLineVector.size());

	if (this->FilterPlateHorizontalLines(horizontalLineVector, plate) < 0) return -1;
	
	return 0;
}

int LicensePlateDetector::FilterPlateHorizontalLines(std::vector<LINE> lines, PLATE_OBJECT* plate) {

	IplImage* binaryImage = 0;
	float x_mid = 0.0;
	float yi = 0.0;
	float yy = 0.0;

	float dist = 0.0;
	float angle = 0.0;
	float min_angle = 100.0;
	int count = 0;
	LINE line_i, line_y;
	double black_min = 100000;
	double white_max = -100000;
	LINE_CANDIDATE tmp_line_candidate;
	int blacks = 0;
	int whites = 0;
	int white_count = 0;
	int black_count = 0;

	if (lines.size() < 2) {
	
		printf("Error while filtering plate horizontal lines. Number of input lines is less than two.\n");
		return -1;
	}

	//Cleaning and init
	x_mid = static_cast<float>(this->colorImage->width / 2);
	binaryImage = cvCloneImage(plate->plateImage);

	//Get best line pair from all lines
	for (int i = 0; i < lines.size(); i++) {	

		line_i = lines.at(i);
		yi = (line_i.k() * x_mid) + line_i.b();

		for (int y = 0; y < lines.size(); y++) {

			if (y <= i) continue;

			line_y = lines.at(y);
			yy = line_y.k() * x_mid + line_y.b();

			//dist = abs(abs(yy - yi) - this->probablyWidth); //TODO: заменить тут надо этот пробабли ширину
			double probablyWidth = static_cast<double>(plate->plateWidth)  * 0.21538461f;
			dist = abs(abs(line_i.b() - line_y.b()) - probablyWidth);
			angle = abs(line_i.k() - line_y.k());
			

			//Coarse filter
			if (dist < 8.0 && angle < 0.005) { // parameter

				//float y_max = line_y.params[3];
				//float y_min = line_i.params[3];
				//if (line_i.params[3] > line_y.params[3]) {

					//y_max = line_i.params[3];
					//y_min = line_y.params[3];
				//}

				/*
				float y_max = line_y.b();
				float y_min = line_i.b();

				if (line_i.b() > line_y.b()) {

					y_max = line_i.b();
					y_min = line_y.b();
				}


				blacks = 0;
				whites = 0;
				white_count = 0;
				black_count = 0;

				//Soft filter			
				for (double y_0 = y_min; y_0 < y_max; y_0++) {			

					whites = 0;
					blacks = 0;

					//for (int xx = 0; xx < this->binaryImage->width; xx++) { // parameter
					for (int xx = binaryImage->width / 8; xx < binaryImage->width - binaryImage->width / 8; xx++) {

						//int yy = static_cast<int>(ki * (static_cast<float>(xx) - line_i.params[2]) + y_0);	
						int yy = static_cast<int>(ki * static_cast<double>(xx) + y_0);
						if (yy > binaryImage->height || yy < 0) {
							
							whites = 0;
							blacks = 0;
							break;
						}
												
						int elem =  CV_IMAGE_ELEM(binaryImage, uchar, yy, xx); // ?!
						if (elem == 255)
							whites++;
						else
							blacks++;
					}

					if (whites > (blacks * 0.7f)) // parameter
						white_count++;
					else
						black_count++;					
				}

			
				
				//if (angle < 0.01 && dist < 10.0 && black_count < black_min) { // parameter
				if (angle < 0.01 && dist < 5.0) {

					white_max = white_count;
					black_min = black_count;
					min_angle = min_angle;

					tmp_line_candidate.line1 = line_i;
					tmp_line_candidate.line2 = line_y;
					//horizontalLineCandidateVector.push_back(tmp_line_candidate);

					//printf("white_count = %i. black_count = %i\n", white_count, black_count);
					printf("dists[%i][%i].dist = %.5f | angle = %.5f\n", i, y, dist, angle);
					count = count + 2;
				}				
				*/

				tmp_line_candidate.line1 = line_i;
				tmp_line_candidate.line2 = line_y;
			}		
		}
	}

	plate->horLines[0] = tmp_line_candidate.line1;
	plate->horLines[1] = tmp_line_candidate.line2;
	plate->horLinesCount = 2;

	//TODO: подумать алгоритм выбора итоговой ваниш лайн. ћожет среднеарифметическое?

	if (plate->horLines[0].b() > plate->horLines[1].b()) 
		plate->horVanishLine = plate->horLines[0];
	else
		plate->horVanishLine = plate->horLines[1];

	plate->isHorVanishLine = true;


	return 0;
}

int LicensePlateDetector::RotateGrayImage(double angle) {

	//CvMat* rot_mat = 0;
	//CvPoint2D32f center;

	//rot_mat = cvCreateMat(2, 3, CV_32FC1);
	//center = cvPoint2D32f(this->binaryImage->width / 2, this->binaryImage->height / 2);
	//cv2DRotationMatrix(center, angle, 1, rot_mat);

	//// = cvCreateImage(cvSize(this->grayImage->width, this->grayImage->height),
	//						//					 this->grayImage->depth, this->grayImage->nChannels);

	//cvWarpAffine(this->binaryImage, this->binaryImage, rot_mat);
	//cvReleaseMat(&rot_mat);

	return 0;
}

int LicensePlateDetector::FindProbablyPlateWidth(PLATE_OBJECT *plate) {

	int delta = 0;
	int delta_max = -100;
	int x_max = 0;
	int x_min = 0;
	int offset = 0;
	int* arr = 0;
	double offset_koef = 4; // чем меньше, тем ближе к кра€м стартова€ точка (меньше анализируема€ область)
	double koef = 4;

	IplImage* gray = cvCloneImage(plate->plateImage);
	cvErode(gray, gray, 0, 2);
	cvDilate(gray, gray, 0, 2);

	//—читаем горизонтальную гистограмму
	arr = new int[gray->width];
	for (int x = 0; x < gray->width; x++) {

		arr[x] = 0;
		for (int y = 0; y < gray->height; y++) {

			arr[x] = arr[x] + CV_IMAGE_ELEM(gray, uchar, y, x);
		}
	}

	//ѕроходим от середины изображени€ к кра€м и находим вертикальные границы номера
	offset = gray->width / offset_koef;

	for (int x = gray->width / 2 + offset; x < gray->width - 1; x++) {

		delta = abs(arr[x] - arr[x + 1]);
		if (delta > delta_max - (delta_max / koef)) {

			x_max = x;
			delta_max = delta;
		}
	}

	delta_max = -100;
	for (int x = gray->width / 2 - offset; x > 0; x--) {

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


	//ќсвобождаем ресурсы
	cvReleaseImage(&gray);
	delete[] arr;
	return 0;
}





int LicensePlateDetector::ProcessPlates(const char* filename) {

	int res = 0;

	res = this->LoadImageFromFile(filename);
	if (res < 0) return -2;

	res = this->InitClassifierCascade("haarcascade_russian_plate_number.xml");
	if (res < 0) return -1;

	this->plateVector.clear(); // TODO:

	
	res = this->FindPlateROI(CvRect(400,
		200,
		1300,
		300),
		cv::Size(60 * 1, 20 * 1), cv::Size(60 * 1, 20 * 1), 1.4, 0.1, 0.1);

	res = this->FindPlateROI(CvRect(550,
		500,
		1400,
		300),
		cv::Size(60 * 1.5, 20 * 1.5), cv::Size(60 * 1.5, 20 * 1.5), 1.4, 0.1, 0.1);

	res = this->FindPlateROI(CvRect(700,
			800,
			1500,
			800),
			cv::Size(60 * 2, 20 * 2), cv::Size(60 * 2, 20 * 2), 1.99, 0.1, 0.1);


	res = this->FindPlateROI(CvRect(900,
		1600,
		1500,
		400),
		cv::Size(60 * 3, 20 * 3), cv::Size(60 * 3, 20 * 3), 1.99, 0.1, 0.1);

	/*res = this->FindPlateROI(CvRect(700,
		600,
		1700,
		700),
		cv::Size(60 * 0.3, 20 * 0.3), cv::Size(60 * 2.0, 20 * 2.0), 1.3, 0.1, 0.1);*/

	//res = this->FindPlateROI(CvRect(700,
	//	900,
	//	1600,
	//	1100),
	//	cv::Size(60 * 2, 20 * 2), cv::Size(60 * 2, 20 * 2), 1.4, 0.1, 0.1);



	this->ProcessTracks(&plateVector);
	this->FinishedTrackProccess();
	//this->PrintTracks(PLATE_TRACK::STATE::inprocess);
	
	
	static CvPoint2D32f point;
	static CvPoint2D32f point2;
	static CvPoint2D32f W;
	static float focal;
	static int size = 0;

	printf("____________________ vector directional size = %i\n", dirVanishLines.size());
	printf("____________________ vector horizontal size = %i\n", horVanishLines.size());

	if (dirVanishLines.size() != size)
	{
		size = dirVanishLines.size();
		

		//this->FindVanishPointMinDistances(horVanishLines, &point);
		//this->FindVanishPointRANSAC(horVanishLines, &point, 300);
		//this->FindVanishPointMinDistances(dirVanishLines, &point2);
		//this->FindVanishPointRANSAC(dirVanishLines, &point2, 200);

		//point.x = 14000;
		//point.y = -50;

		//focal = ComputeFocalLenght(point, point2);
	//	W = ComputeThirdVanishingPoint(point, point2, focal);

		printf("\n*** FOCAL = %.3f ***\n", focal);

		//W.x = colorImage->width / 2 - 350;
		//W.y = 15000.0;


		//this->ComputeRotationMatrix(focal, point, point2, W);
	}

	//this->ComputeRotationMatrix(focal, point, point2, W);

	//FindZebraContour(colorImage);

	//this->PrintVanishVectors(CvPoint(static_cast<int>(point.x), static_cast<int>(point.y)), 3);
	//this->PrintVanishVectors2(CvPoint(static_cast<int>(point2.x), static_cast<int>(point2.y)), 4);
	//this->PrintVanishVectors3(CvPoint(static_cast<int>(W.x), static_cast<int>(W.y)), 3);
	


	this->ShowColorImage(0.5);
	cvWaitKey(1);

	//CvPoint2D32f p(0, 0);
	//CvPoint2D32f l1(0, colorImage->height);
	//CvPoint2D32f l2(colorImage->width, colorImage->height);
	//LINE line(l1, l2);
	//printf("--------- dist to point: %.3f\n", line.dist2point(p));
	
	return 0;
}








int LicensePlateDetector::PrintVanishVectors(CvPoint vanishPoint, int number) {

	printf("Vanishing point: x = %i, y = %i.\n", vanishPoint.x, vanishPoint.y);

	// –исуем результат в виде вектора
	int num = number + 1;
	int offset_x = static_cast<int>(this->colorImage->width / 10);
	int delta_x = static_cast<int>((this->colorImage->width - offset_x) / num);
	int offset_y = static_cast<int>(this->colorImage->height / 10);
	int delta_y = static_cast<int>((this->colorImage->height - offset_y) / num);

	for (int x = delta_x; x < delta_x * num; x = x + delta_x) {
		for (int y = delta_y; y < delta_y * num; y = y + delta_y) {


			// Ќайти уравнение пр€мой по двум точкам
			// (y1 - y2)x + (x2 - x1)y + (x1y2 - x2y1) = 0
			// y = ((y2 - y1)x - (x1y2 - x2y1))/(x2 - x1);
			int yy = 0;
			int xx = 0;

			int x2 = vanishPoint.x;
			int y2 = vanishPoint.y;

			xx= x + 350; // TODO: как едино указать длину стрелки
			yy = static_cast<int>(((((y2 - y) * xx) - (x * y2 - x2 * y)) / (x2 - x)));

			cvLine(this->colorImage, CvPoint(x, y), CvPoint(xx, yy), CV_RGB(255, 0, 0), 4, CV_AA, 0);
			cvCircle(this->colorImage, CvPoint(x, y), 8, CV_RGB(255, 100, 0), 5, 8, 0);
		}
	}

	return 0;
}

int LicensePlateDetector::PrintVanishVectors2(CvPoint vanishPoint, int number) {

	printf("Vanishing point: x = %i, y = %i.\n", vanishPoint.x, vanishPoint.y);

	// –исуем результат в виде вектора
	int num = number + 1;
	int offset_x = static_cast<int>(this->colorImage->width / 10);
	int delta_x = static_cast<int>((this->colorImage->width - offset_x) / num);
	int offset_y = static_cast<int>(this->colorImage->height / 10);
	int delta_y = static_cast<int>((this->colorImage->height - offset_y) / num);

	for (int x = delta_x; x < delta_x * num; x = x + delta_x) {
		for (int y = delta_y; y < delta_y * num; y = y + delta_y) {


			// Ќайти уравнение пр€мой по двум точкам
			// (y1 - y2)x + (x2 - x1)y + (x1y2 - x2y1) = 0
			// y = ((y2 - y1)x - (x1y2 - x2y1))/(x2 - x1);
			int yy = 0;
			int xx = 0;

			int x2 = vanishPoint.x;
			int y2 = vanishPoint.y;

			yy = y - 150;
			xx = static_cast<int>(((yy - y) * (x2 - x) / (y2 - y)) + x);

			cvLine(this->colorImage, CvPoint(x, y), CvPoint(xx, yy), CV_RGB(0, 255, 0), 4, CV_AA, 0);
			cvCircle(this->colorImage, CvPoint(x, y), 8, CV_RGB(255, 100, 0), 5, 8, 0);
		}
	}

	return 0;
}

int LicensePlateDetector::PrintVanishVectors3(CvPoint vanishPoint, int number) {

	printf("Vanishing point: x = %i, y = %i.\n", vanishPoint.x, vanishPoint.y);

	// –исуем результат в виде вектора
	int num = number + 1;
	int offset_x = static_cast<int>(this->colorImage->width / 10);
	int delta_x = static_cast<int>((this->colorImage->width - offset_x) / num);
	int offset_y = static_cast<int>(this->colorImage->height / 10);
	int delta_y = static_cast<int>((this->colorImage->height - offset_y) / num);

	for (int x = delta_x; x < delta_x * num; x = x + delta_x) {
		for (int y = delta_y; y < delta_y * num; y = y + delta_y) {


			// Ќайти уравнение пр€мой по двум точкам
			// (y1 - y2)x + (x2 - x1)y + (x1y2 - x2y1) = 0
			// y = ((y2 - y1)x - (x1y2 - x2y1))/(x2 - x1);
			int yy = 0;
			int xx = 0;

			int x2 = vanishPoint.x;
			int y2 = vanishPoint.y;

			yy = y + 300;
			xx = static_cast<int>(((yy - y) * (x2 - x) / (y2 - y)) + x);

			cvLine(this->colorImage, CvPoint(x, y), CvPoint(xx, yy), CV_RGB(0, 0, 255), 4, CV_AA, 0);
			cvCircle(this->colorImage, CvPoint(x, y), 8, CV_RGB(255, 100, 0), 5, 8, 0);
		}
	}

	return 0;
}


int LicensePlateDetector::FindLinesIntersections(std::vector<LINE> lines, std::vector<CvPoint2D32f> *result) { //TODO: сделать проверку того, что ваниш лайн есть

	CvPoint2D32f intersection;

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



LicensePlateDetector::PLATE_TRACK* LicensePlateDetector::CreateNewTrack() {

	PLATE_TRACK *tmp = 0;

	for (auto t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::ready) 
		{
			tmp = t;
			break;
		}
	}

	if (!tmp)
	{
		tmp = new PLATE_TRACK;
		plateTracks.push_back(tmp);
	}
		
	tmp->state = PLATE_TRACK::STATE::inprocess;
	tmp->id = track_id++;

	return tmp;
}

int LicensePlateDetector::CleanTrack(PLATE_TRACK* track) {

	if (!track)
		return -1;

	track->points.clear();
	track->lost_count = 0;
	track->state = PLATE_TRACK::STATE::ready;

	return 0;
}


int LicensePlateDetector::ProcessTracks(std::vector<PLATE_OBJECT> *new_plates) {

	// «аполн€ем треки новыми точками
	for(auto *t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::ready ||
			t->state == PLATE_TRACK::STATE::finished) continue;

		PLATE_OBJECT trPlate = t->points.back();
		float tpx = trPlate.plateROIleftX + (trPlate.plateROIwidth / 2);
		float tpy = trPlate.plateROIleftY + (trPlate.plateROIheight / 2);

		float minWeight = 9999;
		int minIndex = -1;

		for (int i = 0; i < new_plates->size(); i++)
		{			
			auto newPlate = &new_plates->at(i);
			float npx = newPlate->plateROIleftX + (newPlate->plateROIwidth / 2);
			float npy = newPlate->plateROIleftY + (newPlate->plateROIheight / 2);

			float dirAngle = 0.0;
			if ((npx - tpx) == 0) dirAngle = 90.0;
			dirAngle = DEGREE_IN_RADIAN * atan((npy - tpy) / (npx - tpx));
			float dCoord = sqrt((tpx - npx)*(tpx - npx) + (tpy - npy)*(tpy - npy));
			float dAngle = 0.0;

			if (trPlate.dirAngle != 0.0f)
				dAngle = abs(dirAngle - trPlate.dirAngle);
			
			float radius = static_cast<float>(newPlate->plateROIwidth) * 1.5f; // radius - max delta distance

			cvCircle(this->colorImage, CvPoint(npx, npy), radius, CV_RGB(255, 100, 0), 5, 8, 0); // debug

			float compare_weight = ((0.85f * (dCoord / radius)) + (0.15f * (dAngle / 10))); // 10 - max delta angle

			printf("CmpW = %.2f\n", compare_weight);
			if (compare_weight > 1.0) continue;

			if (compare_weight < minWeight)
			{
				minWeight = compare_weight;
				minIndex = i;
			}
		}

		if (minIndex > -1) 
		{
			if (new_plates->at(minIndex).owner_id != -1)
			{
				if (new_plates->at(minIndex).owner_weight > minWeight)
				{
					new_plates->at(minIndex).owner_id = t->id;
					new_plates->at(minIndex).owner_weight = minWeight;
				}
			}
			else
			{
				new_plates->at(minIndex).owner_id = t->id;
				new_plates->at(minIndex).owner_weight = minWeight;
			}
		}
	}
	
	for (auto *t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::ready ||
			t->state == PLATE_TRACK::STATE::finished) continue;

		bool isAdded = false;

		for(int i = 0; i < new_plates->size(); i++)
		{
			if (t->id == new_plates->at(i).owner_id)
			{
				t->points.push_back(new_plates->at(i));
				isAdded = true;
			}
		}

		if (!isAdded)
			t->lost_count++;
	}



	// ќтладочна€ информаци€
	for (auto *t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::ready ||
			t->state == PLATE_TRACK::STATE::finished) continue;

		printf("TrID: %i, LC: %i, PC: %i\n", t->id, t->lost_count, t->points.size());

	}
	printf("\n");






	// ѕровер€ем, не закончилс€ ли трек
	for (auto t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::ready ||
			t->state == PLATE_TRACK::STATE::finished) continue;

		if (t->lost_count >= 40) // parameter
		{
			if (t->points.size() >= 20) // parameter
			{
				t->state = PLATE_TRACK::STATE::finished;
			}
			else
			{
				CleanTrack(t);
			}
		}
	}

	// —оздаем новые треки дл€ точек, которые не подход€т остальным трекам
	for (int i = 0; i < new_plates->size(); i++)
	{
		PLATE_OBJECT p = new_plates->at(i);

		if (p.owner_id < 0)
		{
			PLATE_TRACK *new_track = CreateNewTrack(); //TODO: GetNewTrack()

			if (!new_track)
			{
				printf("Error while executing ProcessTracks(). Cant create new PLATE_TRACK.\n");
			}
			else
			{
				new_track->points.push_back(p);
			}
		}
	}

	return 0;
}


int LicensePlateDetector::FinishedTrackProccess() {

	for (auto t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::finished)
		{
			// ќбработка линии направлени€ движени€
			std::vector<cv::Point> points;
			points.clear();

			for (auto p : t->points)
			{
				int x = static_cast<int>(p.plateROIleftX + p.plateROIwidth / 2);
				int y = static_cast<int>(p.plateROIleftY + p.plateROIheight / 2);
				points.push_back(cv::Point(x, y));
			}

			// ≈сли траектори€ крива€
			// TODO: какой тут первый параметр выбрать? может зависит от размера номера?
			if (LineFitRANSAC(10.0, 0.6, 0.6, 0.9 * t->points.size(), points, &t->DirectionVanishLine) < 0) 
			{
				CleanTrack(t);
				continue;
			}

			// ≈сли траектори€ пр€ма€, обрабатываем дальше
			dirVanishLines.push_back(t->DirectionVanishLine);


			// ќбработка горизонтальных линий
			for (auto p : t->points)
			{
				//this->FindProbablyPlateWidth(&p);
				//this->FindPlateHorizontalLines(&p);
				//this->PrintPlateVanishHorLine(p, 0, colorImage->width);
				//horVanishLines.push_back(p.horVanishLine);					
			}

			//this->PrintTrackDirectionVanishLine(*t, 0, colorImage->height);
			//this->PrintTracks(PLATE_TRACK::STATE::finished);
		
			//this->ShowColorImage(0.5);//
			//cvWaitKey(2500);
			CleanTrack(t);
		}
	}

	return 0;
}









void LicensePlateDetector::PrintTracks(PLATE_TRACK::STATE state) {

	int p = 0;
	int r = 0;
	int f = 0;

	for (auto t : plateTracks)
	{
		if (t->state == PLATE_TRACK::STATE::finished) f++;
		if (t->state == PLATE_TRACK::STATE::ready) r++;
		if (t->state == PLATE_TRACK::STATE::inprocess) p++;
	}
	//printf("***** Tracks info *****\nready = %i | inprocess: %i | finished: %i\n", r, p, f);
	

	for (auto t : plateTracks)
	{
		if (t->state == state)
		{		
			for (auto p : t->points)
			{
				int x = static_cast<int>(p.plateROIleftX + p.plateROIwidth / 2);
				int y = static_cast<int>(p.plateROIleftY + p.plateROIheight / 2);
				cvCircle(this->colorImage, CvPoint(x, y), 8, CV_RGB(255, 100, 0), 5, 8, 0);
				//this->PrintPlateVanishHorLine(p, 0, this->colorImage->width);

				CvFont font;
				cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 2.0, 0, 3, CV_AA);
				char text[255];
				sprintf_s(text, sizeof(text), "id: %i", t->id);
				cvPutText(this->colorImage, text, CvPoint(x + 20, y - 20), &font, CV_RGB(255, 200, 200));
			}
		}
	}
}











int LicensePlateDetector::PrintTrackDirectionVanishLine(PLATE_TRACK track, int topYcoord, int bottomYcoord) {

	CvPoint p1;
	CvPoint p2;

	if (this->colorImage == NULL) {

		printf("Error while showing track direction vanish line. There is no color Image.\n");
		return -1;
	}

	if (track.state != PLATE_TRACK::STATE::finished) {

		printf("Error while showing track direction vanish line. Track is no finished.\n");
		return -1;
	}

	if (topYcoord < 0) topYcoord = 0;
	if (bottomYcoord > this->colorImage->height) bottomYcoord = this->colorImage->height;

	p1.y = topYcoord;
	p1.x = static_cast<int>(track.DirectionVanishLine.x(p1.y));

	p2.y = bottomYcoord;
	p2.x = static_cast<int>(track.DirectionVanishLine.x(p2.y));

	cvLine(this->colorImage, p1, p2, CV_RGB(50, 250, 50), 2, CV_AA, 0);

	return 0;

}











void LicensePlateDetector::ComputeRotationMatrix(float focal_lenght, CvPoint2D32f v1, CvPoint2D32f v2, CvPoint2D32f v3) {


	// X column
	float R11 = v1.x / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);
	float R21 = v1.y / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);
	float R31 = focal_lenght / sqrt((v1.x * v1.x) + (v1.y * v1.y) + focal_lenght);

	// Y column
	float R12 = v2.x / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);
	float R22 = v2.y / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);
	float R32 = focal_lenght / sqrt((v2.x * v2.x) + (v2.y * v2.y) + focal_lenght);

	// Z column
	/*float R13 = v3.x / sqrt((v3.x * v3.x) + (v3.y * v3.y) + focal_lenght);
	float R23 = v3.y / sqrt((v3.x * v3.x) + (v3.y * v3.y) + focal_lenght);
	float R33 = focal_lenght / sqrt((v3.x * v3.x) + (v3.y * v3.y) + focal_lenght);*/

	//R12 = R12 / R32;
	//R22 = R22 / R32;
	//R32 = R32 / R32;
	//float norm1 = sqrt(R11*R11 + R21*R21 + R31*R31);
	//float norm2 = sqrt(R12*R12 + R22*R22 + R32*R32);

	//R11 = R11 / norm1;
	//R21 = R21 / norm1;
	//R31 = R31 / norm1;

	//R12 = R12 / norm2;
	//R22 = R22 / norm2;
	//R32 = R32 / norm2;

	// Z column
	float R13 = (R21 * R32 - R31 * R22);
	float R23 = (R31 * R12 - R11 * R32);
	float R33 = (R11 * R22 - R21 * R12);

	//R21 = R21 / R11;
	//R31 = R31 / R11;
	//R11 = R11 / R11;

	//R12 = R12 / R32;
	//R22 = R22 / R32;
	//R32 = R32 / R32;

	//R13 = R13 / R33;
	//R23 = R23 / R33;
	//R33 = R33 / R33;




	/*float R13 = v3.x / sqrt((v3.x * v3.x) + (v3.y * v3.y) + focal_lenght);
	float R23 = v3.y / sqrt((v3.x * v3.x) + (v3.y * v3.y) + focal_lenght);
	float R33 = focal_lenght / sqrt((v3.x * v3.x) + (v3.y * v3.y) + focal_lenght);*/




	printf("R11 = %.3f | R12 = %.3f | R13 = %.3f\n", R11, R12, R13);
	printf("R21 = %.3f | R22 = %.3f | R23 = %.3f\n", R21, R22, R23);
	printf("R31 = %.3f | R32 = %.3f | R33 = %.3f\n", R31, R32, R33);

	//printf("***** ZYX *****\n");
	double cosYangle = sqrt(R11 * R11 + R21 * R21);
	double rotXangle = atan2(R32, R33);
	double rotZangle = atan2(R21, R11);
	double rotYangle = atan2(-R31, cosYangle);

	//double cosYangle = sqrt(R11 * R11 + R12 * R12);
	//double rotXangle = atan2(R23, R33);
	//double rotZangle = atan2(R12, R11);
	//double rotYangle = atan2(-R13, cosYangle);





	//printf("ay = %.2f | ", DEGREE_IN_RADIAN * rotYangle);
	//printf("ax = %.2f | ", 90 - DEGREE_IN_RADIAN * rotXangle);
	//printf("az = %.2f \n ", DEGREE_IN_RADIAN * rotZangle);

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 2.0, 3.0, 0, 6, CV_AA);
	char text[255];
	sprintf_s(text, sizeof(text), "Rotate: %.1f Sl: %.1f Roll: %.1f Focal: %.1f", DEGREE_IN_RADIAN * rotYangle,
		90 - DEGREE_IN_RADIAN * rotXangle, 
		DEGREE_IN_RADIAN * rotZangle,
		focal_lenght);
	//sprintf_s(text, sizeof(text), "Rotate: %.1f Roll: %.1f", DEGREE_IN_RADIAN * rotYangle, DEGREE_IN_RADIAN * rotZangle);

	cvPutText(this->colorImage, text, CvPoint(200, 1600), &font, CV_RGB(255, 200, 45));
}