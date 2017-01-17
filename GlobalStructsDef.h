/*****************************************************************************
*
*	GlobalStructsDef module deffine global structs and enums.
*
******************************************************************************/


#ifndef GLOBAL_STRUCTS_DEF_H_
#define GLOBAL_STRUCTS_DEF_H_

#include <opencv\cv.hpp>
#include <opencv\highgui.h>

using namespace cv;

typedef struct SystemParameters
{
	bool addRobots;				/* Flag to add robots to the tracker */
	double scale;				/* Resize images (imsize/scale = int) */
	unsigned int redetectDelay;  // redetect required after redetectDelay frames
	unsigned int invisiblilityThreshold;
	double visiblilityThreshold;
	unsigned int ageThreshold;
} SYSTEM_PARAMETERS;


typedef enum HsvColors
{
	H_MIN,
	S_MIN,
	V_MIN,
	H_MAX,
	S_MAX,
	V_MAX,
	NUMBER_HSV_PARAMS
} HSV_COLORS;

typedef struct SharedData
{
	int id;
	Vector<Point2d> positions;
	Vector<Point2d> locations;
	vector<double> distances;
	double theta;
	long timestamp;
} SHARED_DATA;


typedef struct HsvParameters
{
	double contrastValue;
	int brightnessValue;
	// todo optimize - int to unsigned char?
	int hsvColors[NUMBER_HSV_PARAMS];

} HSV_PARAMETERS;

typedef struct DetectionParameters
{
	/* TABLE DETECTION */
	Mat seTable;		    /* morphological structuring element */
	double filledPart;		/* part of the filled table area in the image */
	double minAreaTable;	/* Minimum BLOB area in Table detection(pixels) */
	double parameterZ;

	/* ROBOT DETECTION */
	double bwThreshold;		 /* Black and white transformation threshold */
	double minEccentricity;  /* Minimum Object Eccentricity threshold */
	double maxEccentricity;  /* Maximum Object Eccentricity threshold */
	double minSolidity;		 /* Minimum Object Solidity threshold */
	double maxSolidity;		 /* Maximum Object Solidity threshold */
	double minAreaRobot;	 /* Minimum BLOB area in Robot detection(pixels) */
	Mat seRobot;			 /* Create morphological structuring element for Robot detection */
	Mat seErode;			 /* Create morphological erode element for Robot detection */

} DETECTION_PARAMETERS;


typedef struct KalmanFilterParameters
{
	
} KALMAN_FILTER_PARAMETERS;


struct Stat						  /* Region statistics*/
{
	Point2d centroid;
	Rect bbox;
	double area;
	double eccentricity;
	double solidity;	
};

struct Stats
{
	vector<Stat> stat_array;
};

typedef struct Track
{
	unsigned int id;
    Point2d centroid;
    Rect bbox;
	Rect biggerBox;
    Rect sbox;
    Point2d location;  
	Point2d position;
    double distance;	
    KalmanFilter kalmanFilter;
	Vector<Stat> candidateStats;
    unsigned int age;
    unsigned int totalVisibleCount;
    unsigned int consecutiveInvisibleCount;
} TRACK;


/* globals defenitions */
extern HSV_PARAMETERS				hsvParameters;
extern DETECTION_PARAMETERS			detectionParameters;
extern SYSTEM_PARAMETERS			systemParameters;
extern KALMAN_FILTER_PARAMETERS		kalmanFilterParameters;
extern VideoCapture					videoCapture;


#endif /* GLOBAL_STRUCTS_DEF_H_ */