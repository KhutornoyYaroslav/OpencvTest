
#include "PlateTracker.h"

PlateTracker::PlateTracker()
{
	for (int i = 0; i < TRACKS_VECTOR_INIT_SIZE; i++)
	{
		TRACK *track = new TRACK;
		plateTracks.push_back(track);
	}
}

PlateTracker::~PlateTracker()
{

}

PlateTracker::TRACK* PlateTracker::GetNewTrack()
{
	TRACK *track = 0;

	for (auto t : plateTracks)
	{
		if (t->state == TRACK::STATE::ready)
		{
			track = t;
			break;
		}
	}

	if (track == nullptr)
	{
		track = new TRACK;
		plateTracks.push_back(track);
	}

	track->state = TRACK::STATE::inprocess;
	track->id = trackCount++;

	return track;
}

bool PlateTracker::CleanTrack(TRACK* track)
{
	if (!track)
		return false;

	for (auto p : track->points)
	{		
		p.plate.plateImage.release();
	}

	track->points.clear();
	track->lostCount = 0;
	track->state = TRACK::STATE::ready;

	return true;
}

bool PlateTracker::InitClassifierCascade(const char* filename)
{
	bool isload = plateClassifierCascade.load(filename);
	if (isload == false)
	{
		printf("Error while InitClassifierCascade(). No xml-file was found.\n");
		return false;
	}

	return true;
}

bool PlateTracker::FindPlateROI(const cv::Mat image, cv::Rect searchArea, cv::Size windowSize, std::vector<TRACKED_PLATE> *platesInRoi, double xboundScale, double yboundScale)
{
	if (image.empty())
	{
		printf("Error while executing FindPlateROI(). There is no source gray image.\n");
		return false;
	}

	if (platesInRoi == nullptr)
	{
		printf("Error while executing FindPlateROI(). Vector pointer is null.\n");
		return false;
	}
		
	if (searchArea.height <= 0 || searchArea.width <= 0) 
	{
		printf("Error while executing FindPlateROI(). Search area height or width is less or equal zero.\n");
		return false;
	}

	if (searchArea.x + searchArea.width > image.cols || searchArea.width < 1 || searchArea.x < 0)
	{
		printf("Error while executing FindPlateROI(). Search area width is out of image border.\n");
		return false;
	}

	if (searchArea.y + searchArea.height > image.rows || searchArea.height < 1 || searchArea.y < 0)
	{
		printf("Error while executing FindPlateROI(). Search area height is out of image border.\n");
		return false;
	}

	if (windowSize.height <= 0 || windowSize.width <= 0)
	{
		printf("Error while executing FindPlateROI(). Window size is less or equal than zero.\n");
		return false;
	}

	if (xboundScale < 0)
	{
		printf("Warning while executing FindPlateROI(). Argument xboundScale is less than zero. Its will set to 0,01.\n");
		xboundScale = 0.01;
	}

	if (xboundScale > 0.2)
	{
		printf("Warning while executing FindPlateROI(). Argument xboundScale is too high. Its will set to 0,2.\n");
		xboundScale = 0.2;
	}

	if (yboundScale < 0)
	{
		printf("Warning while executing FindPlateROI(). Argument yboundScale is less than zero. Its will set to 0,05.\n");
		yboundScale = 0.05;
	}

	if (yboundScale > 0.4)
	{
		printf("Warning while executing FindPlateROI(). Argument yboundScale is too high. Its will set to 0,4.\n");
		yboundScale = 0.4;
	}

	std::vector<cv::Rect> result;
	cv::Mat img = image(searchArea);
	plateClassifierCascade.detectMultiScale(img, result, 1.5, 3, 0, windowSize, windowSize);
	img.release();

	if (result.size() == 0) return true;
	
	for (auto p : result)
	{
		TRACKED_PLATE newPlate;

		int x_offset = static_cast<int>((static_cast<double>(p.width) * xboundScale));
		int y_offset = static_cast<int>((static_cast<double>(p.height) * yboundScale));

		newPlate.plate.plateRect.x = searchArea.x + p.x - x_offset;
		newPlate.plate.plateRect.y = searchArea.y + p.y - y_offset;
		newPlate.plate.plateRect.width = p.width + 2 * x_offset;
		newPlate.plate.plateRect.height = p.height + 2 * y_offset;

		if (newPlate.plate.plateRect.x < 0 || newPlate.plate.plateRect.y < 0 ||
			newPlate.plate.plateRect.x + newPlate.plate.plateRect.width > image.cols ||
			newPlate.plate.plateRect.y + newPlate.plate.plateRect.height > image.rows) continue;
		
		image(cv::Rect(newPlate.plate.plateRect.x,
						newPlate.plate.plateRect.y,
						newPlate.plate.plateRect.width,
						newPlate.plate.plateRect.height)).copyTo(newPlate.plate.plateImage);
		
		cv::Rect carRect = calculateCarRect(&newPlate); // TODO: сделать с направлением движения

		if (!(carRect.x < 0 || carRect.y < 0 ||
			carRect.x + carRect.width > image.cols ||
			carRect.y + carRect.height > image.rows))
		{
			image(carRect).copyTo(newPlate.plate.carImage);
		}

		newPlate.plate.carRect = carRect;
		platesInRoi->push_back(newPlate);	
	}

	return true;
}

void PlateTracker::ProcessTracks(std::vector<TRACKED_PLATE> *newPlates)
{
	// Fill tracks by new plates
	for (auto *t : plateTracks)
	{
		if (t->state == TRACK::STATE::ready ||
			t->state == TRACK::STATE::finished) continue;

		TRACKED_PLATE trPlate = t->points.back();
		float tpx = trPlate.plate.plateRect.x + (trPlate.plate.plateRect.width / 2);
		float tpy = trPlate.plate.plateRect.y + (trPlate.plate.plateRect.height / 2);

		float minWeight = 9999;
		int minIndex = -1;

		for (size_t i = 0; i < newPlates->size(); i++)
		{
			auto *newPlate = &newPlates->at(i);
			float npx = newPlate->plate.plateRect.x + (newPlate->plate.plateRect.width / 2);
			float npy = newPlate->plate.plateRect.y + (newPlate->plate.plateRect.height / 2);

			float dirAngle = 0.0;
			if ((npx - tpx) == 0) dirAngle = 90.0;
			dirAngle = DEGREE_IN_RADIAN * atan((npy - tpy) / (npx - tpx));
			float dCoord = sqrt((tpx - npx)*(tpx - npx) + (tpy - npy)*(tpy - npy));
			float dAngle = 0.0;

			if (trPlate.dirAngle != 0.0f) dAngle = abs(dirAngle - trPlate.dirAngle);

			TRACK::DIRECTION dir;
			(tpy > npy) ? (dir = TRACK::DIRECTION::outcomming) : (dir = TRACK::DIRECTION::incomming);
			if (t->GetDirection() != TRACK::DIRECTION::none && t->GetDirection() != dir) continue;
	
			float dCoordMax = static_cast<float>(newPlate->plate.plateRect.width) * TRACKS_MAX_DELTA_COORD_KOEF;
			float compare_weight = (TRACKS_CMP_COORD_KOEF * (dCoord / dCoordMax) +
				(TRACKS_CMP_ANGLE_KOEF * (dAngle / TRACKS_MAX_DELTA_ANGLE)));

			if (compare_weight > TRACKS_CMP_MAX_WEIGHT) continue;

			if (compare_weight < minWeight)
			{
				minWeight = compare_weight;
				minIndex = (int)i;
			}
		}

		if (minIndex > -1)
		{
			if (newPlates->at(minIndex).isOwner == true)
			{
				if (newPlates->at(minIndex).ownerWeight > minWeight)
				{
					newPlates->at(minIndex).ownerID = t->id;
					newPlates->at(minIndex).ownerWeight = minWeight;
				}
			}
			else
			{
				newPlates->at(minIndex).ownerID = t->id;
				newPlates->at(minIndex).ownerWeight = minWeight;
				newPlates->at(minIndex).isOwner = true;
			}
		}
	}

	for (auto *t : plateTracks)
	{
		if (t->state == TRACK::STATE::ready ||
			t->state == TRACK::STATE::finished) continue;

		bool isAdded = false;

		for (auto p : *newPlates)
		{
			if (p.isOwner == false) continue;
			if (t->id == p.ownerID)
			{
				float dX = abs(p.plate.plateRect.x - t->points.back().plate.plateRect.x);
				float dY = abs(p.plate.plateRect.y - t->points.back().plate.plateRect.y);
				
				if (dX > static_cast<float>(t->points.back().plate.plateRect.width / 5) ||
					dY > static_cast<float>(t->points.back().plate.plateRect.height / 3))
				{
					t->points.push_back(p);
				}
			
				isAdded = true;
			}
		}

		if (!isAdded)
			t->lostCount++;
	}

	// Lost tracks checking
	for (auto t : plateTracks)
	{
		if (t->state == TRACK::STATE::ready ||
			t->state == TRACK::STATE::finished) continue;

		if (t->lostCount >= TRACKS_LOST_COUNT_MAX)
		{
			if (t->points.size() >= TRACKS_POINTS_COUNT_MIN)
			{
				t->state = TRACK::STATE::finished;
			}
			else
			{
				CleanTrack(t);
			}
		}
	}

	// New tracks creating for plates with no owner
	for(auto p : *newPlates)
	{
		if (p.isOwner == false)
		{
			auto *new_track = GetNewTrack();

			if (new_track != nullptr)
				new_track->points.push_back(p);			
		}
	}

}

void PlateTracker::FinishedTrackProccess() 
{
	for (auto t : plateTracks)
	{
		if (t->state == TRACK::STATE::finished)
		{
			std::vector<cv::Point> points;
			points.clear();
	
			float dY = t->points.back().plate.plateRect.y - t->points.front().plate.plateRect.y;
			float dX = t->points.back().plate.plateRect.x - t->points.front().plate.plateRect.x;
			float trackLenght = sqrt(dX*dX + dY*dY);

			if (trackLenght < TRACKS_TRACK_LENGHT_MIN)
			{
				CleanTrack(t);
				continue;
			}

			for (auto p : t->points)
			{
				int x = static_cast<int>(p.plate.plateRect.x + p.plate.plateRect.width / 2);
				int y = static_cast<int>(p.plate.plateRect.y + p.plate.plateRect.height / 2);
				points.push_back(cv::Point(x, y));
			}

			size_t middle = static_cast<size_t>(t->points.size() / 2);
			double dist = static_cast<double>(t->points.at(middle).plate.plateRect.width) / 10.0;

			if (LineFitRANSAC(dist, 0.6, 0.6, 0.85 * t->points.size(), points, &t->directionVanishLine) < 0)
			{
				CleanTrack(t);
				continue;
			}
		}
	}
}

bool PlateTracker::GetFinishedTrackTable(unsigned int *IDs, unsigned int *count, unsigned int IDsSize)
{
	for (auto t : plateTracks)
	{
		if (t->state == TRACK::STATE::finished)
		{
			*(IDs++) = t->id;
			(*count)++;

			if (*count >= IDsSize)
				break;
		}
	}

	return *count > 0;
}

bool PlateTracker::GetTrack(unsigned int ID, PLATES_TRACK *track)
{
	if (track == nullptr) return false;

	for (auto t : plateTracks)
	{
		if (t->state != TRACK::STATE::finished) continue;

		if (t->id == ID)
		{
			for (auto p : t->points)
			{
				track->points.push_back(p.plate);
			}

			track->directionVanishLine = t->directionVanishLine;

			CleanTrack(t);
			return true;
		}
	}
}

bool PlateTracker::ProcessImage(cv::Mat grayImage)
{
	bool result = true;
	std::vector<TRACKED_PLATE> newPlates;

	if (grayImage.empty())
	{
		return false;
	}

	result = FindPlateROI(grayImage, cv::Rect(300, 200, 1900, 300 + 15), cv::Size(60, 20), &newPlates);
	result = FindPlateROI(grayImage, cv::Rect(300, 500, 1900, 300 + 15 * 1.45), cv::Size(60 * 1.45, 20 * 1.45), &newPlates);
	result = FindPlateROI(grayImage, cv::Rect(300, 800, 1900, 300 + 15 * 2.0), cv::Size(60 * 2.0, 20 * 2.0), &newPlates);
	result = FindPlateROI(grayImage, cv::Rect(300, 1100, 1900, 300 + 15 * 2.5), cv::Size(60 * 2.5, 20 * 2.5), &newPlates);
	result = FindPlateROI(grayImage, cv::Rect(300, 1400, 1900, 300 + 15 * 2.85), cv::Size(60 * 2.5, 20 * 2.85), &newPlates);

	if (result == true) {

		ProcessTracks(&newPlates);
		FinishedTrackProccess();
	}

	return result;
}

void PlateTracker::GetTracksStat(int *empty, int *inprocess, int *finished)
{
	(*empty) = 0;
	(*inprocess) = 0;
	(*finished) = 0;

	for (auto t : plateTracks)
	{
		if (t->state == TRACK::STATE::ready) (*empty)++;
		if (t->state == TRACK::STATE::inprocess) (*inprocess)++;
		if (t->state == TRACK::STATE::finished) (*finished)++;
	}
}
	
void PlateTracker::PrintTracks(cv::Mat *image)
{
	for (auto t : plateTracks)
	{
		if (t->state != TRACK::STATE::ready)
		{
			for (auto p : t->points)
			{
				int x = static_cast<int>(p.plate.plateRect.x + p.plate.plateRect.width / 2);
				int y = static_cast<int>(p.plate.plateRect.y + p.plate.plateRect.height / 2);

				char text[255];
				sprintf_s(text, sizeof(text), "%i", t->id);

				if (t->state == TRACK::STATE::finished)
				{
					cv::circle(*image, cv::Point(x, y), 8, CV_RGB(0, 200, 0), 5);
				}
				else
				{
					cv::circle(*image, cv::Point(x, y), 8, CV_RGB(200, 50, 50), 5);
					cv::putText(*image, text, cv::Point(x + 20, y - 20), 2, 1.0, CV_RGB(200, 255, 200), 2);
				}			
			}
		}
	}
}

cv::Rect PlateTracker::calculateCarRect(TRACKED_PLATE *plate)
{
	const int width = 640;
	const int height = 480;

	const float k = static_cast<float>(width) / static_cast<float>(height);

	cv::Rect newR = plate->plate.plateRect;

	const int oldCenterX = (newR.tl().x + newR.br().x) / 2;
	const int oldCenterY = (newR.tl().y + newR.br().y) / 2;

	const double lol = 0.0; // TODO: сделать с направлением движения

	newR.width = newR.width * CAR_IMAGE_RESIZE;
	newR.height = newR.width / k;
	newR.x = oldCenterX - newR.width / 2 - lol * width / 4;
	newR.y = oldCenterY - newR.height / 2 - newR.height / 4;

	return newR;
}