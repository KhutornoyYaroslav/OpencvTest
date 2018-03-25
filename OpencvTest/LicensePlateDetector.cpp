#include "stdafx.h"
#include "LicensePlateDetector.h"


LicensePlateDetector::LicensePlateDetector()
{
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

int LicensePlateDetector::FindPlateHorizontalLines(PLATE_OBJECT plate) {

	IplImage* binaryImage = 0;
	CvMemStorage* contours_storage = 0;
	CvSeq* contours = 0;
	std::vector<cv::Point> points;
	CvSeqReader seq_reader;
	CvMat kernel_matrix;
	LINE line;
	SLine ransacLine;
	float sharp_kernel[9] = { -0.1f, -0.1f, -0.1f, -0.1f, 2.0f, -0.1f, -0.1f, -0.1f, -0.1f }; // parameter

	//Cleaning and initing
	horizontalLineVector.clear();
	points.clear();	
	binaryImage = cvCloneImage(plate.plateImage);

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
	
	//int total_max = -10;
	//for (CvSeq* current = contours; current != NULL; current = current->h_next) {
	//	
	//	if (current->total > total_max) total_max = current->total;
	//}
	int valid_max = -10;
	
	for (CvSeq* current = contours; current != NULL; current = current->h_next) {

		//Reading of all points from current contour and saving its to vector.
		points.clear();
		cvStartReadSeq(current, &seq_reader, -1);
		for (int i = 0; i < current->total; i++) {

			cv::Point point;
			CV_READ_SEQ_ELEM(point, seq_reader);
			points.push_back(point);
		}
		//printf("points size = %i\n", points.size());
		
		//Filter short contours	
		//if (current->total < (total_max - (total_max/4))) continue; // parameter
		if (current->total < 100) continue; // parameter 185

		//Find RANSAC fitting lines 
		ransacLine = LineFitRANSAC(1.5, 0.9, 0.9, current->total * 0.8, points); // parameter
		if (ransacLine.params[0] == -1 ||
			ransacLine.params[1] == -1 ||
			ransacLine.params[2] == -1 ||
			ransacLine.params[3] == -1) continue;		

		//Push line to vector
		line.numOfValidPoints = ransacLine.numOfValidPoints;
		line.params = ransacLine.params;
		horizontalLineVector.push_back(line);
	}
	

	if (this->horizontalLineVector.size() == 0) return -1;

	return 0;
}

int LicensePlateDetector::FilterPlateHorizontalLines(PLATE_OBJECT* plate) {

	IplImage* binaryImage = 0;
	float x = 0.0;
	float ki = 0.0;
	float yi = 0.0;
	float ky = 0.0;
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

	if (this->horizontalLineVector.size() == 0) {
	
		printf("Error while filtering plate horizontal lines. No lines is found.\n");
		return -1;
	}

	//Cleaning and init
	horizontalLineCandidateVector.clear();
	//this->horizontalLineVector2.clear();////////
	x = static_cast<float>(this->colorImage->width / 2);
	binaryImage = cvCloneImage(plate->plateImage);

	//Get best line pair from all lines
	for (int i = 0; i < this->horizontalLineVector.size(); i++) {		
		for (int y = 0; y < this->horizontalLineVector.size(); y++) {

			if (y <= i) continue;

			line_i = this->horizontalLineVector.at(i);
			line_y = this->horizontalLineVector.at(y);

			ki = atan(line_i.params[1] / line_i.params[0]);
			yi = ki * (x - line_i.params[2]) + line_i.params[3]; 

			ky = atan(line_y.params[1] / line_y.params[0]);
			yy = ky * (x - line_y.params[2]) + line_y.params[3];

			dist = abs(abs(yy - yi) - this->probablyWidth);
			angle = abs(ky - ki);
			
			//Coarse filter
			if (dist < 10.0 && angle < 0.01) { // parameter

				float y_max = line_y.params[3];
				float y_min = line_i.params[3];
				if (line_i.params[3] > line_y.params[3]) {

					y_max = line_i.params[3];
					y_min = line_y.params[3];
				}
				
				blacks = 0;
				whites = 0;
				white_count = 0;
				black_count = 0;

				//Soft filter			
				for (float y_0 = y_min; y_0 < y_max; y_0++) {			

					whites = 0;
					blacks = 0;

					//for (int xx = 0; xx < this->binaryImage->width; xx++) { // parameter
					for (int xx = binaryImage->width / 8; xx < binaryImage->width - binaryImage->width / 8; xx++) {

						int yy = static_cast<int>(ki * (static_cast<float>(xx) - line_i.params[2]) + y_0);					
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

				
				
				if (angle < 0.01 && dist < 10.0 && black_count < black_min) { // parameter

					white_max = white_count;
					black_min = black_count;
					min_angle = min_angle;

					tmp_line_candidate.line1 = line_i;
					tmp_line_candidate.line2 = line_y;
					//horizontalLineCandidateVector.push_back(tmp_line_candidate);

					printf("white_count = %i. black_count = %i\n", white_count, black_count);
					printf("dists[%i][%i].dist = %.5f | angle = %.5f\n", i, y, dist, angle);

					this->horizontalLineVector2.push_back(line_i);
					this->horizontalLineVector2.push_back(line_y);	

					count = count + 2;
				}				
				
			}		
		}
	}

	plate->lineHcount = count;
	plate->lineH[0] = tmp_line_candidate.line1;
	plate->lineH[1] = tmp_line_candidate.line2;

	if (count != 0) { //TODO:

		if(tmp_line_candidate.line1.params[3] > tmp_line_candidate.line2.params[3]) 
			plate->vanishLineH = tmp_line_candidate.line1;
		else 
			plate->vanishLineH = tmp_line_candidate.line2;
		
	}

	//this->horizontalLineVector2.push_back(tmp_line_candidate.line1);
	//this->horizontalLineVector2.push_back(tmp_line_candidate.line2);
	printf("count of good line pairs= %i\n", count);
	return 0;
}

int LicensePlateDetector::PrintPlateHorizontalLines(PLATE_OBJECT plate) {

	LINE tmp_line;

	if (this->colorImage == NULL) {

		printf("Error while showing plate horizontal lines. There is no color Image.\n");
		return -1;
	}

	for (int i = 0; i < plate.lineHcount; i++) {

		float sin = plate.lineH[i].params[1];
		float cos = plate.lineH[i].params[0];
		float k = atan(sin / cos);

		float x0 = plate.lineH[i].params[2] + plate.x_coord;
		float y0 = plate.lineH[i].params[3] + plate.y_coord;

		float x = 0;
		float y = k * (x - x0) + y0;
		tmp_line.p1.x = static_cast<int>(x);
		tmp_line.p1.y = static_cast<int>(y);

		x = this->colorImage->width;
		y = k * (x - x0) + y0;
		tmp_line.p2.x = static_cast<int>(x);
		tmp_line.p2.y = static_cast<int>(y);

		cvLine(this->colorImage, tmp_line.p1, tmp_line.p2, CV_RGB(255, 0, 55), 2, CV_AA, 0);
	}

	return 0;
}

int LicensePlateDetector::PrintPlateVanishHorLine(PLATE_OBJECT plate) {

	LINE tmp_line;

	if (this->colorImage == NULL) {

		printf("Error while showing plate horizontal lines. There is no color Image.\n");
		return -1;
	}

	if (plate.vanishLineH.params[0] == -1) return -1;

	float sin = plate.vanishLineH.params[1];
	float cos = plate.vanishLineH.params[0];
	float k = atan(sin / cos);

	float x0 = plate.vanishLineH.params[2] + plate.x_coord;
	float y0 = plate.vanishLineH.params[3] + plate.y_coord;

	float x = 0;
	float y = k * (x - x0) + y0;
	tmp_line.p1.x = static_cast<int>(x);
	tmp_line.p1.y = static_cast<int>(y);

	x = this->colorImage->width;
	y = k * (x - x0) + y0;
	tmp_line.p2.x = static_cast<int>(x);
	tmp_line.p2.y = static_cast<int>(y);

	cvLine(this->colorImage, tmp_line.p1, tmp_line.p2, CV_RGB(255, 0, 55), 2, CV_AA, 0);
	
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

int LicensePlateDetector::FindProbablyPlateWidth(PLATE_OBJECT plate) {

	int delta = 0;
	int delta_max = -100;
	int x_max = 0;
	int x_min = 0;
	int offset = 0;
	int* arr = 0;
	double offset_koef = 3;
	double koef = 4;

	//Преобразуем исходное изображение в оттенки серого
	//IplImage* gray = cvCreateImage(cvGetSize(this->colorImage), IPL_DEPTH_8U, 1);
	//cvConvertImage(this->colorImage, gray, CV_BGR2GRAY);
	IplImage* gray = cvCloneImage(plate.plateImage);
	//Предобработка изображения
	cvErode(gray, gray, 0, 2);
	cvDilate(gray, gray, 0, 2);

	//Считаем горизонтальную гистограмму
	arr = new int[gray->width];
	for (int x = 0; x < gray->width; x++) {

		arr[x] = 0;
		for (int y = 0; y < gray->height; y++) {

			arr[x] = arr[x] + CV_IMAGE_ELEM(gray, uchar, y, x);
		}
	}

	//Проходим от середины изображения к краям и находим вертикальные границы номера
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

	this->probablyWidth = static_cast<double>(abs(x_max - x_min)) / 520.0f * 112.0f;
	printf("this->probablyWidth = %.2f\n", this->probablyWidth);
	//cvLine(this->colorImage, CvPoint(x_min, 0), CvPoint(x_min, gray->height), CV_RGB(55, 155, 55), 1, CV_AA, 0);
	//cvLine(this->colorImage, CvPoint(x_max, 0), CvPoint(x_max, gray->height), CV_RGB(55, 155, 55), 1, CV_AA, 0);

	//Освобождаем ресурсы
	cvReleaseImage(&gray);
	delete[] arr;
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

	if (minSize.height <= 0 || minSize.width <= 0 ) {

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
	cvDrawRect(this->colorImage, CvPoint(ROI.x, ROI.y), CvPoint(ROI.x + ROI.width, ROI.y + ROI.height), CV_RGB(0, 200, 255), 15, 8, 0);
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
		
		plateRoi.x_coord = ROI.x + p.x - x_offset;
		plateRoi.y_coord = ROI.y + p.y - y_offset;
		plateRoi.width = p.width + 2 * x_offset;
		plateRoi.height = p.height + 2 * y_offset;
		plateRoi.state = PLATE_STATE::inprocess;

		if (plateRoi.x_coord < 0 || plateRoi.y_coord < 0 ||
			plateRoi.x_coord + plateRoi.width > this->colorImage->width ||
			plateRoi.y_coord + plateRoi.height > this->colorImage->height) continue;

		cvSetImageROI(grayImage, CvRect(plateRoi.x_coord, plateRoi.y_coord, plateRoi.width, plateRoi.height));	
		plateRoi.plateImage = cvCreateImage(CvSize(plateRoi.width, plateRoi.height), IPL_DEPTH_8U, 1);
		cvCopy(grayImage, plateRoi.plateImage);
		cvResetImageROI(grayImage);
		

		//----- Debug code -----
		cvDrawRect(this->colorImage, CvPoint(plateRoi.x_coord, plateRoi.y_coord),
									 CvPoint(plateRoi.x_coord + plateRoi.width, plateRoi.y_coord + plateRoi.height),
									 CV_RGB(0, 255, 125), 2, 8, 0);
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


int LicensePlateDetector::ProcessPlates(const char* filename) {

	int res = 0;

	res = this->LoadImageFromFile(filename);
	if (res < 0) return -1;

	res = this->InitClassifierCascade("haarcascade_russian_plate_number.xml");
	if (res < 0) return -1;

	//res = this->FindPlateROI(CvRect(this->colorImage->width / 2,
	//	this->colorImage->height / 2,
	//	this->colorImage->width - this->colorImage->width /2  - 1,
	//	this->colorImage->height - this->colorImage->height / 2 - 1),
	//	cv::Size(60 * 2, 20 * 2), cv::Size(60 * 3, 20 * 3), 1.2, 0.001, 0.2);

	res = this->FindPlateROI(CvRect(this->colorImage->width / 3,
		0,
		this->colorImage->width / 3 * 2,
		this->colorImage->height),
		cv::Size(60 * 0.5, 20 * 0.5), cv::Size(60 * 3, 20 * 3), 1.1, 0.001, 0.2);
	if (res < 0) return -1;

	for (int i = 0; i < this->plateVector.size(); i++) {

		if (plateVector.at(i).state == PLATE_STATE::finished) continue;

		this->FindProbablyPlateWidth(plateVector.at(i));
		this->FindPlateHorizontalLines(plateVector.at(i));
		this->FilterPlateHorizontalLines(&plateVector[i]);
		plateVector.at(i).state = PLATE_STATE::finished;

		//this->PrintPlateVanishHorLine(plateVector.at(i));
		//this->PrintPlateHorizontalLines(plateVector.at(i));
		//this->ShowImage(plateVector.at(i).plateImage, 5.0);
		//this->ShowColorImage(0.5);		
	}



	

	return 0;
}