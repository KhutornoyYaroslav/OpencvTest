
#include "Zebra.h"


double FindLineAngle2(CvPoint p1, CvPoint p2) {

	if (p1.y == p2.y) return 90.0 * 3.14 / 180.0;

	if (p1.x == p2.x) return 0.0 * 3.14 / 180.0;

	int delta_x = abs((p1.x) - (p2.x));
	int delta_y = abs((p1.y) - (p2.y));

	double tgA = delta_x / delta_y;

	return  atan(tgA) * 180.0 / 3.14;
}

void FindLines2(CvSeq* contour, CvPoint* kern4, int step = 2) {

	const float treshold = 40.0;
	float delta = 0.0;
	float current_angle = 0.0;
	float pre_angle = 0.0;
	int count = 0;
	int index = 0;
	CvPoint* v_points;
	CvSeqReader reader;

	v_points = new CvPoint[contour->total];
	cvStartReadSeq(contour, &reader, -1);

	//—читываем все точки контура
	for (int i = 0; i < contour->total; i++) {

		CV_READ_SEQ_ELEM(v_points[i], reader);
	}

	printf("contour->total = %i\n", contour->total);
	//—читаем углы
	for (int i = 0; i < contour->total; i++) {

		//if(i == (contour->total - step)) step = 

		current_angle = FindLineAngle2(v_points[i], v_points[i + step]);

		delta = abs(current_angle - pre_angle);

		if (delta > treshold) {

			count++;
			index = i;

		}
		else {

			count = 0;
		}


		printf("v_points[%i].x = %i; v_points[%i].y = %i; ", i, v_points[i].x, i, v_points[i].y);
		printf("delta[%i] = %.2f; ", i, delta);
		printf("current_angle[%i] = %.2f; ", i, current_angle);
		printf("count = %i; index = %i\n", count, index);

		pre_angle = current_angle;

	}
}


void FindLines3(CvSeq* contour, CvPoint* kern4) {

	const float treshold = 40.0;
	float delta = 0.0;
	float current_angle = 0.0;
	float pre_angle = 0.0;
	int count = 0;
	int index = 0;
	CvPoint* v_points;
	CvSeqReader reader;

	v_points = new CvPoint[contour->total];
	cvStartReadSeq(contour, &reader, -1);

	//—читываем все точки контура
	for (int i = 0; i < contour->total; i++) {

		CV_READ_SEQ_ELEM(v_points[i], reader);
	}

	printf("contour->total = %i\n", contour->total);
	//—читаем углы
	int step = 1;
	for (int i = 0; i < contour->total - step; i++) {

		//current_angle = FindLineAngle2(v_points[i], v_points[i + 1]);
		//delta = abs(current_angle - pre_angle);

		int delta_y = abs(v_points[i].y - v_points[i + step].y);
		int delta_x = abs(v_points[i].x - v_points[i + step].x);
		printf("delta_y = %i | ", delta_y);
		printf("delta_x = %i\n", delta_x);
		/*if (delta > treshold) {

			count++;
			index = i;

		}
		else {

			count = 0;
		}*/


		//printf("v_points[%i].x = %i; v_points[%i].y = %i; ", i, v_points[i].x, i, v_points[i].y);
		//printf("delta[%i] = %.2f; ", i, delta);
		//printf("current_angle[%i] = %.2f; ", i, current_angle);
		//printf("count = %i; index = %i\n", count, index);

		pre_angle = current_angle;

	}
}

void rotate2(IplImage* _image, double _angle = 90)
{
	// матрицы трансформации
	CvMat* rot_mat = cvCreateMat(2, 3, CV_32FC1);
	// вращение относительно центра изображени€
	CvPoint2D32f center = cvPoint2D32f(_image->width / 2, _image->height / 2);
	double angle = _angle;
	double scale = 1;
	cv2DRotationMatrix(center, angle, scale, rot_mat);

	IplImage* Temp = 0;
	Temp = cvCreateImage(cvSize(_image->width, _image->height), _image->depth, _image->nChannels);

	// выполн€ем вращение
	cvWarpAffine(_image, Temp, rot_mat);

	// сохран€ем результат
	cvCopy(Temp, _image);

	cvReleaseImage(&Temp);
	cvReleaseMat(&rot_mat);
}

void DrawRotatedRect(IplImage * iplSrc, CvBox2D rect, CvScalar color, int thickness, int line_type = 8, int shift = 0)
{
	CvPoint2D32f boxPoints[4];
	cvBoxPoints(rect, boxPoints);
	cvLine(iplSrc, cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y), cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y), color, thickness, line_type, shift);
	cvLine(iplSrc, cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y), cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y), color, thickness, line_type, shift);
	cvLine(iplSrc, cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y), cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y), color, thickness, line_type, shift);
	cvLine(iplSrc, cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y), cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y), color, thickness, line_type, shift);
}

void WndTreshold(IplImage* gray, int wsize) {

	for (int i = 0; i < gray->height; i = i + wsize) {
		for (int j = 0; j < gray->width; j = j + wsize) {

			cvSetImageROI(gray, cvRect(j, i, wsize, wsize));

			cvSmooth(gray, gray, CV_GAUSSIAN, 7, 7);

			//cvErode(gray, gray, 0, 2);
			//cvDilate(gray, gray, 0, 2);

			cvAdaptiveThreshold(gray, gray, 255, 0, 1, 25, 1.0);

			cvResetImageROI(gray);
		}
	}

}

int BoxFilter(CvBox2D b, IplImage* image) {

	float width = 0.0;
	float height = 0.0;
	float relation = 0.0;
	
	if (b.size.width > b.size.height) {

		width = b.size.width;
		height = b.size.height;
	}
	else {

		width = b.size.height;
		height = b.size.width;
	}

	relation = width / height;

	//‘ильтры по размерам
	if (width < image->width / 100) return -1;
	if (height < image->height / 100) return -1;
	if (relation < 2) return -1;
	if (relation > 6) return -1;
	if (width > image->width / 8) return -1;
	if (height > image->height / 8) return -1;

	//‘ильтры по гистограмме

	return 0;
}

double inline LineLenght(CvPoint p1, CvPoint p2) {

	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


int FourPointsContourFilter(CvSeq* contour) {

	CvPoint* v_points;
	CvSeqReader reader;

	struct LINE_ {

		CvPoint p1;
		CvPoint p2;
	} lines[4];

	//‘ильтр по количеству углов
	if (contour->total != 4) return -1;

	v_points = new CvPoint[contour->total];
	cvStartReadSeq(contour, &reader, -1);

	//—читываем все точки контура
	for (int i = 0; i < contour->total; i++) {

		CV_READ_SEQ_ELEM(v_points[i], reader);	

		//printf("points[i].x = %i | points[i].y = %i ;\n", v_points[i].x, v_points[i].y);
	}

	double len[3];
	int max_index = 0;
	//дл€ первой точки i = 0
	len[0] = LineLenght(v_points[0], v_points[1]);
	len[1] = LineLenght(v_points[0], v_points[2]);
	len[2] = LineLenght(v_points[0], v_points[3]);

	max_index = (len[0] > len[1]) ? 1 : 2;
	max_index = (len[max_index - 1] > len[2]) ? max_index : 3;
	
	

	//длина 1
	//lines[0].p1 = v_points[0];
	//lines[0].p2 = ;
	//ширина 1
	//lines[1].p1 = v_points[0];
	//lines[1].p2 = ;


	//printf("len1 = %.2f\n", len[0]);
	//printf("len2 = %.2f\n", len[1]);
	//printf("len3 = %.2f\n", len[2]);
	//printf("max = %i\n", max_index);

	return 0;
}


void DrawContourPoints(CvSeq* contour, IplImage* image) {

	CvPoint* v_points;
	CvSeqReader reader;

	v_points = new CvPoint[contour->total];
	cvStartReadSeq(contour, &reader, -1);

	//—читываем все точки контура
	for (int i = 0; i < contour->total; i++) {

		CV_READ_SEQ_ELEM(v_points[i], reader);

		cvDrawCircle(image, v_points[i], 2, CV_RGB(255, 0, 0), 12, 8, 0);
	}
}



void FindZebraContour(IplImage* image) {

	IplImage* gray = 0;
	CvSeq* contours = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvMemStorage* storage2 = cvCreateMemStorage(0);
	// создаем пустое изображение в оттенках серого
	gray = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	// конвертируем исходное изображение в градации серого
	cvConvertImage(image, gray, CV_BGR2GRAY);

	//WndTreshold(gray, 100);
	cvSmooth(gray, gray, CV_GAUSSIAN, 3, 3);
	//cvAdaptiveThreshold(gray, gray, 255, 0, 1, 25, 1.0);

	cvThreshold(gray, gray, 145, 255, CV_THRESH_OTSU);

	cvCanny(gray, gray, 185, 255, 3);

	cvFindContours(gray, storage, &contours, sizeof(CvContour), 0, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
	//contours = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 3, 1);


	



	CvMemStorage* hull_storage = cvCreateMemStorage();
	CvSeq* retHulls = NULL;
	for (CvSeq* i = contours; i != NULL; i = i->h_next) {

		//if (i->total > 320 || i->total < 300) continue;

		retHulls = cvConvexHull2(i, hull_storage, CV_CLOCKWISE, 0);
		CvSeq* defect = NULL;
		defect = cvConvexityDefects(i, retHulls, NULL); // reuse storage of the contour
		retHulls = cvConvexHull2(i, hull_storage, CV_CLOCKWISE, 1);

		CvBox2D b = cvMinAreaRect2(retHulls);
		if (BoxFilter(b, image) == -1) continue;

		//DrawRotatedRect(image, b, CV_RGB(255, 20, 120), 1);
		//CvRect rect = cvBoundingRect(i, 0);
		//cvDrawRect(image, CvPoint(rect.x, rect.y), CvPoint(rect.x + rect.width, rect.y + rect.height), CV_RGB(0, 255, 0), 1, 8, 0);

		retHulls = cvApproxPoly(retHulls, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 4, 1);

		
		if (retHulls->total != 4) continue;
		//printf("retHulls->total = %i\n", retHulls->total);

		//if (FourPointsContourFilter(retHulls) == -1) continue;

		DrawContourPoints(retHulls, image);


		cvDrawContours(image, retHulls, CV_RGB(0, 255, 0), CV_RGB(0, 255, 0), 0, 3, 8);

		//CvPoint kern4[120];
		//FindLines2(retHulls, kern4, 1);
	}

	





	//**************************************************************
	//double resize = 1.0;
	//if (image->width > 1280) resize = 0.5;
	//if (image->width < 720) resize = 2.0;
	//if (image->width < 400) resize = 4.0;
	
	//IplImage *lol = cvCreateImage(cvSize(gray->width * resize, gray->height * resize), gray->depth, gray->nChannels);
	//cvResize(gray, lol);
	//cvShowImage("gray", lol);


	//IplImage *lol2 = cvCreateImage(cvSize(image->width * resize, image->height * resize), image->depth, image->nChannels);
	//cvResize(image, lol2);
	//cvShowImage("original", lol2);

}