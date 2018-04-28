#include "common.h"

int LineFitRANSAC(double t, double p, double e, int T, std::vector<cv::Point>& nzPoints, LINE *line) {

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

		LINE tmpLine(p1, p2);
		int numOfInliers = 0;

		for (size_t i = 0; i < nzPoints.size(); ++i)
		{
			cv::Point& p0 = nzPoints[i];
			double rho = static_cast<double>(tmpLine.dist2point(p0));
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


