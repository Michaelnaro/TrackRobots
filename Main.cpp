#include <opencv\cv.hpp>
#include <opencv\highgui.h>
#include <iostream>

// TODO - change to relative path
// #include "C:\Program Files\IDS\uEye\Develop\include\uEye.h"
#include "Init.h"
#include "Camera_i.h"
#include <random>

#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

unsigned int nLost = 0; // number of lost robots 
unsigned int nFrames = 0;
unsigned int nFrameLost = 50000; // prevent nFrames == nFrames + systemparams.redetectDelay without loosing any target

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}
string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars(){
	//create window for trackbars


	namedWindow("Something!",0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar( "Brightness", "Something!", &(hsvParameters.brightnessValue), 100, on_trackbar );
	createTrackbar( "Contrast", "Something!", (int*)(&(hsvParameters.contrastValue)), 10, on_trackbar );
	createTrackbar( "H_MIN", "Something!", &(hsvParameters.hsvColors[H_MIN]), 255, on_trackbar );
	createTrackbar( "S_MIN", "Something!", &(hsvParameters.hsvColors[S_MIN]), 255, on_trackbar );
	createTrackbar( "V_MIN", "Something!", &(hsvParameters.hsvColors[V_MIN]), 255, on_trackbar );
	createTrackbar( "H_MAX", "Something!", &(hsvParameters.hsvColors[H_MAX]), 255, on_trackbar );
	createTrackbar( "S_MAX", "Something!", &(hsvParameters.hsvColors[S_MAX]), 255, on_trackbar );
	createTrackbar( "V_MAX", "Something!", &(hsvParameters.hsvColors[V_MAX]), 255, on_trackbar );
}

bool isReDetectionRequired()
{
	bool res = false;
	res = ((systemParameters.addRobots == true) || (nLost > 0) || (nFrames == nFrameLost + systemParameters.redetectDelay)) ? true : false;
	return res;
}

// TODO - implement using real sensor
double getThetaFromGyro()
{
	double res = 0;
	res = rand() % 360;
	return res;
}


using namespace std;

#define MAX_SAVED_DATA_LENGTH 1000

int main( int argc, char** argv )
{	
	bool isSuccessfulInit = false;
	bool endvideoflag = false;
	bool redetectionrequired = true;
	bool tableFound = false;
	Camera_i camera;
	Mat imBorder;
	Mat frameHSV;
	long timerFrame = 0;
	double elapsedTime = 0;	
	double realTime = 0;
	SHARED_DATA sharedData;
	SHARED_DATA saveData[MAX_SAVED_DATA_LENGTH];
	unsigned int savedDataIndex = 0;
	ofstream logFile;
	bool candidateCentroidsExists = false;
	bool trackExists = false;
	bool statExists = false;
	logFile.open ("logFile.txt");

	//createTrackbars();

	/* initialize ALL system global structs. Using UserConfig values when applicable. */
	isSuccessfulInit = system_init(camera);
	
	if(isSuccessfulInit == true)
	{
		while(endvideoflag == false)
		{
			try
			{
				// Timer for 1 frame calculations
				timerFrame = clock(); 

				// Read next frame
				endvideoflag = camera.captureFrame(&nFrames, &realTime);

				// must be the first command for the current frame after frame
				camera.cleanPreviousFrameData();
			
				// check if redetection is required - redetection is a havy process and shall be not performed if not essential
				redetectionrequired = isReDetectionRequired();
				if (redetectionrequired == true)
				{
					/* detectborders - hsv transform to detect red collor on table border */
					imBorder = camera.detectColor(frameHSV);
        
					/* table detection - retry till detection successeded  */
					int attemptsCount = 1;
					while((tableFound == false) && (attemptsCount < 5))
					{
						tableFound = camera.detectTable(imBorder, attemptsCount);
						attemptsCount++;
					}

					if (attemptsCount > 2)
					{
						cout << "attemptsCount is: " << attemptsCount << endl;
					}

					systemParameters.addRobots = true;
				}

				/* ROBOT DETECTION*/
				camera.detectRobots(systemParameters.addRobots);

				// Kalman filter prediction step
				camera.predictNewLocationsOfTracks();
    
				//candidateCentroidsExists = camera.isCandidateCentroidsExists();
				trackExists = camera.isTracksExists();
				statExists = camera.isStatExists();

				if((statExists == true) && (nFrames <= 5))
				{
					camera.initializeTracks();
				}
				// Assign target detections to tracks (If there were any detections)
				else if((trackExists == true) || (statExists == true))
				{
					camera.detectionToTrackAssignment();

					// Update assigned and unassigned tracks
					camera.updateAssignedTracks();
					camera.updateUnassignedTracks();
				}

				camera.deleteLostTracks(&nLost);

				if(systemParameters.addRobots == true)
				{
					// Create new tracks
					camera.createNewTracks();
					// Turn down addTargets triger (New targets were added)
					systemParameters.addRobots = false;
				
					// Save a frame number for re-detection if target is lost,
					// and was not found on the following frame
					// One of the tracks.centroid is NaN. Except the first frame
					bool unassignedTracksExists = camera.isUnassignedTracksExists();
					if((nFrames > 1) && (unassignedTracksExists == true))
					{
						// number of frame at which the track was lost
							nFrameLost = nFrames;
					}
				}

				if(nFrames > 5) // TODO - change 5 to deffine in all places in the code!
				{
					camera.measureDistances(); // ocam_model
				}

				// Save shared data 
				sharedData.distances		=	camera.getTracksDistances();
				sharedData.locations		=   camera.getTracksLocations();
				sharedData.positions		=   camera.getTracksPositions();
				sharedData.theta			=	getThetaFromGyro();  // TODO - FIXME: Read camera angle from an external sensor
				sharedData.timestamp		=	timerFrame;

				elapsedTime = _difftime64(clock(), timerFrame);
				// Display current frame number and computation time
				// TODO - disp(['Frame = ', num2str(nFrames), '. Computation time = ', num2str(elapsedTime)]);
				systemParameters.addRobots = false;
		
				// Save estimated data to file - TODO change to SHARED_DATA to class and overload operator <<
				logFile << "-----------------------------" << endl;
				logFile << "Frame Number: " << nFrames << endl;
				logFile << "Time[mSec]: " << timerFrame << endl;
				logFile << "Angle: " << sharedData.theta << endl;
				logFile << "Robots data: " << endl;

				for(int robotInd = 0; robotInd < sharedData.distances.size(); robotInd++)
				{
					logFile << "Robot Index: " << robotInd << endl;
					logFile << "Robot Distance: " << sharedData.distances[robotInd] << endl;
					logFile << "Robot Position: " << sharedData.positions[robotInd] << endl;
					logFile << "Robot Location: " << sharedData.locations[robotInd] << endl;
				}

				// for Debug only:
				if(savedDataIndex < MAX_SAVED_DATA_LENGTH)
				{
					saveData[savedDataIndex] = sharedData;
					savedDataIndex++;
				}

				// Display results including sharedData
				camera.displayTrackingResults(sharedData);
			}
			catch(...)
			{
				printf("Error!!");
			}
		}
	}
	else
	{
		// initialization failed - error code reported as stored in errno
		return 1;
	}

	
	logFile.close();
	
	return 0;
}
