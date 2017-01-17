#ifndef ICAMERA_H_
#define ICAMERA_H_

#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <vector>

#include "UserConfig.h"
#include "GlobalStructsDef.h"
#include "ocam_functions.h"

using namespace cv;
using namespace std;

class Camera_i
{
protected:
	Mat m_frame_RGB;
	Mat m_frame_GRAY;
	Mat m_tableMask;
	unsigned int numFrames;
	unsigned int currentFrameIndex;

	Vector<Stat> m_stats;
	Vector<Track> m_tracks;
	Vector<unsigned int> m_assignedTracksIndexes;
	Vector<unsigned int> m_unAssignedTracksIndexes;
	vector<unsigned int> m_lostTrackIndexes;
	Vector<Stat> m_unAssignedStats;


	void removeSmallBlobs(cv::Mat& im, double size);
	void imadjust(Mat& frame, double contrast, int brightness);
	Stats regionStats(vector<vector<Point> > contours);
	bool isRobot(const Stat stat);
	void create_sBox(const Stat stat, Rect& sBox);
	void create_biggerBox(const Stat stat, Rect& sBox);
	void drawDetection(const Stat stat, Rect sBox, Point pt);
	vector<int> find(Stats stats); 
	void assignCandidatesToTracks();
	vector<unsigned int> findCandidateTrackIndexesForStat(unsigned int stat_index);
	vector<unsigned int> findMinimalCandidateIndexes(vector<unsigned int> candidateTrackIndexes);
	void removeSelectedCandidateFromTracks(Stat statToRemove);
	unsigned int findClosestTrackIndex(vector<unsigned int> minimalTrackIndexes, unsigned int stat_index);
	vector<unsigned int> sortTracksAccordingCandidatesNum();
	int findClosestCandidateIndex(unsigned int track_index);
	bool isTrackLost(Track track);
	void createNewTrack(Stat stat);
	double polyval(double* pol, int pol_len, double val);

public:
	bool cameraInit();
	void initializeTracks();
	bool captureFrame(unsigned int* nFrames, double* realTime);
	Mat  detectColor(Mat& frameHSV);                 /* returns imborder */
	bool detectTable(Mat imborder, int attempts);   /* returns true if succeded. Changes m_frame to hold table only */
	void detectRobots(int addRobots);				/* Detects Robots*/
	void predictNewLocationsOfTracks(void);
	void displayTrackingResults(SharedData sharedData);
	vector<double> getTracksDistances();
	Vector<Point2d> getTracksLocations();
	Vector<Point2d> getTracksPositions();
	bool isCandidateCentroidsExists();
	void detectionToTrackAssignment();
	void updateAssignedTracks();
	void updateUnassignedTracks();
	void deleteLostTracks(unsigned int* nLost);
	void createNewTracks();
	bool isTracksExists();
	bool isStatExists();
	void initKalmanFilter(Track* track);
	bool isUnassignedTracksExists();
	void cleanPreviousFrameData();
	void measureDistances();
};

#endif /* ICAMERA_H_ */