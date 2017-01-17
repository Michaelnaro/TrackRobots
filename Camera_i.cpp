#include "Camera_i.h"
#include "math.h"

static Stat m_stats_debug[20];
static Track m_tracks_debug[20];

void Camera_i::updateUnassignedTracks() 
{
	for (int visibleRobotIndex = 0; visibleRobotIndex < m_unAssignedTracksIndexes.size(); visibleRobotIndex++)
	{
		unsigned int trackIndexToUpdate = m_unAssignedTracksIndexes[visibleRobotIndex];

		Mat_<float> measurement(2,1); 
		measurement.setTo(Scalar(0));
		measurement(0) = (float) m_tracks[trackIndexToUpdate].location.x;
		measurement(1) = (float) m_tracks[trackIndexToUpdate].location.y;
		Mat estimated = m_tracks[trackIndexToUpdate].kalmanFilter.correct(measurement);

		Point2d estimatedCentroid(estimated.at<float>(0),estimated.at<float>(1));
		m_tracks[trackIndexToUpdate].location = estimatedCentroid;

		m_tracks[trackIndexToUpdate].centroid = estimatedCentroid;
		
		m_tracks[trackIndexToUpdate].age++;
		m_tracks[trackIndexToUpdate].consecutiveInvisibleCount++;
	}
}

void Camera_i::updateAssignedTracks() 
{
	for (int visibleRobotIndex = 0; visibleRobotIndex < m_assignedTracksIndexes.size(); visibleRobotIndex++)
	{
		unsigned int trackIndexToUpdate = m_assignedTracksIndexes[visibleRobotIndex];

		Mat_<float> measurement(2,1); 
		measurement.setTo(Scalar(0));
		measurement(0) = (float) m_tracks[trackIndexToUpdate].centroid.x;
		measurement(1) = (float) m_tracks[trackIndexToUpdate].centroid.y;
		Mat estimated = m_tracks[trackIndexToUpdate].kalmanFilter.correct(measurement);

		Point2d estimatedCentroid(estimated.at<float>(0),estimated.at<float>(1));
		m_tracks[trackIndexToUpdate].location = estimatedCentroid;

		m_tracks[trackIndexToUpdate].age++;
		m_tracks[trackIndexToUpdate].totalVisibleCount++;
		m_tracks[trackIndexToUpdate].consecutiveInvisibleCount = 0;
	}
}

bool Camera_i::isStatExists() 
{
	bool res = false;

	if(m_stats.empty() == true)
	{
		res = false;
	}
	else
	{
		res = true;
	}
	
	return res;
}


bool Camera_i::isTracksExists() 
{
	bool res = false;

	if(m_tracks.empty() == true)
	{
		res = false;
	}
	else
	{
		res = true;
	}
	
	return res;
}

bool Camera_i::isTrackLost(Track track)
{
	bool res = false;
	double visibility = 0;

	visibility = ((double)track.totalVisibleCount) / track.age;
	if((track.consecutiveInvisibleCount > systemParameters.invisiblilityThreshold) && (visibility < systemParameters.visiblilityThreshold))
	{
		res = true;
	}
	else
	{
		res = false;
	}

	return res;
}

void Camera_i::deleteLostTracks(unsigned int* nLost) 
{
	bool trackLost = false;

	for (unsigned int track_index = 0; track_index < m_unAssignedTracksIndexes.size(); track_index++)
	{
		unsigned int currentUnassignedTrackIndex = m_unAssignedTracksIndexes[track_index];
		Track currentUnassignedTrack = m_tracks[currentUnassignedTrackIndex];
		trackLost = isTrackLost(currentUnassignedTrack);
		if(trackLost == true)
		{
			printf("\n Track %d is lost! \n", currentUnassignedTrack.id);
			m_lostTrackIndexes.push_back(currentUnassignedTrack.id);

			// already a lost track and not just unassigned!
			// remove from the unassigned vector
			unsigned int last_unassigned_track_index = m_unAssignedTracksIndexes[m_unAssignedTracksIndexes.size() - 1];
			// move the last track index to the index we want to remove from vector
			m_unAssignedTracksIndexes[track_index] = last_unassigned_track_index;
			// remove last element
			m_unAssignedTracksIndexes.pop_back();
		}
	}
	*nLost = m_lostTrackIndexes.size();
}

/*
* The function was called because robot were lost - if we are going to assign tracks we should assign the unassigned tracks before the lost tracks
* Algorithm:
*	1) match every unassigned track with the closest stat to it.
*	2) match every lost track with the closest stat to it.
*	3) if there is more unassigned stats - create new tracks
*/

void Camera_i::createNewTracks() 
{
	for(unsigned int stat_ind = 0; stat_ind < m_unAssignedStats.size(); stat_ind++)
	{
		Stat unassigned_stat = m_unAssignedStats[stat_ind];
		double minimum_distance = -1; // -1 means minimum_distance was not yet updated
		int closestCandidateIndex = -1;

		if(m_unAssignedTracksIndexes.empty() == false)
		{
			// 1) match every unassigned track with the closest stat to it.
			for(int track_ind = 0; track_ind < m_unAssignedTracksIndexes.size(); track_ind++)
			{
				unsigned int unassigned_track_index = m_unAssignedTracksIndexes[track_ind];
				Track unassigned_track = m_tracks[unassigned_track_index];
				double distance = norm(unassigned_track.location - unassigned_stat.centroid);
				if((distance < minimum_distance) || (minimum_distance < 0))
				{
					minimum_distance = distance;
					closestCandidateIndex = unassigned_track_index;
				}
			}

			m_tracks[closestCandidateIndex].centroid = unassigned_stat.centroid;
			m_tracks[closestCandidateIndex].bbox = unassigned_stat.bbox;
			create_sBox(unassigned_stat, m_tracks[closestCandidateIndex].sbox);
			create_biggerBox(unassigned_stat, m_tracks[closestCandidateIndex].biggerBox);
			m_tracks[closestCandidateIndex].candidateStats.clear();
			m_tracks[closestCandidateIndex].consecutiveInvisibleCount = 0;
			m_tracks[closestCandidateIndex].distance = 0;
			m_tracks[closestCandidateIndex].location = unassigned_stat.centroid;
			m_tracks[closestCandidateIndex].position = Point2d();
			// ? - initKalmanFilter(&(m_tracks[closestCandidateIndex]));
		}
		else if(m_lostTrackIndexes.empty() == false)
		{
			// 2) match every lost track with the closest stat to it.
			for(int track_ind = 0; track_ind < m_lostTrackIndexes.size(); track_ind++)
			{
				unsigned int lost_track_index = m_lostTrackIndexes[track_ind];
				Track lost_track = m_tracks[lost_track_index];
				double distance = norm(lost_track.location - unassigned_stat.centroid);
				if((distance < minimum_distance) || (minimum_distance < 0))
				{
					minimum_distance = distance;
					closestCandidateIndex = lost_track_index;
				}
			}

			m_tracks[closestCandidateIndex].centroid = unassigned_stat.centroid;
			m_tracks[closestCandidateIndex].bbox = unassigned_stat.bbox;
			create_sBox(unassigned_stat, m_tracks[closestCandidateIndex].sbox);
			create_biggerBox(unassigned_stat, m_tracks[closestCandidateIndex].biggerBox);
			m_tracks[closestCandidateIndex].age = 0;
			m_tracks[closestCandidateIndex].candidateStats.clear();
			m_tracks[closestCandidateIndex].consecutiveInvisibleCount = 0;
			m_tracks[closestCandidateIndex].distance = 0;
			m_tracks[closestCandidateIndex].location = unassigned_stat.centroid;
			m_tracks[closestCandidateIndex].totalVisibleCount = 0;
			m_tracks[closestCandidateIndex].position = Point2d();
			initKalmanFilter(&(m_tracks[closestCandidateIndex]));

			// not a lost track anymore
			for(int i = 0; i < m_lostTrackIndexes.size(); i++)
			{
				unsigned int current_lost_track_index = m_lostTrackIndexes[i];
				if(current_lost_track_index == closestCandidateIndex)
				{
					// remove from the lost vector
					unsigned int last_lost_track_index = m_lostTrackIndexes[m_lostTrackIndexes.size() - 1];
					// move the last track index to the index we want to remove from vector
					m_lostTrackIndexes[i] = last_lost_track_index;
					// remove last element
					m_lostTrackIndexes.pop_back();
					break;
				}
			}
		}
		else
		{
			// 3) if there is more unassigned stats - create new tracks
			createNewTrack(unassigned_stat);
		}
	}
}

void Camera_i::createNewTrack(Stat stat)
{
	Track newTrack;

	newTrack.id = m_tracks.size();
	newTrack.centroid = stat.centroid;
	newTrack.bbox = stat.bbox;
	create_sBox(stat, newTrack.sbox);
	create_biggerBox(stat, newTrack.biggerBox);
	initKalmanFilter(&newTrack);
	newTrack.age = 0;
	newTrack.candidateStats.clear();
	newTrack.consecutiveInvisibleCount = 0;
	newTrack.distance = 0;
	newTrack.location = stat.centroid;
	newTrack.totalVisibleCount = 0;
	newTrack.position = Point2d();
	m_tracks.push_back(newTrack);
}

void Camera_i::assignCandidatesToTracks()
{
	// find for each stat to which tracks its related
	for (unsigned int current_track_index = 0; current_track_index < m_tracks.size(); current_track_index++)
	{
		m_tracks[current_track_index].candidateStats.clear();

		for(unsigned int stat_index = 0; stat_index < m_stats.size(); stat_index++)
		{
			// check if centroid of stat j is inside search box of track i
			if(m_tracks[current_track_index].sbox.contains(m_stats[stat_index].centroid) == true)
			{
				// add stat j to track i's candidates
				m_tracks[current_track_index].candidateStats.push_back(m_stats[stat_index]);
				#ifdef DEBUG
					debug_array[current_track_index] = m_tracks[current_track_index];
				#endif
			}
		}
	}
}

#define MAX_POSSIBLE_TRACKS 20

vector<unsigned int> Camera_i::findMinimalCandidateIndexes(vector<unsigned int> candidateTrackIndexes)
{
	unsigned int minimumCandidatesPerTrack = 0;
	unsigned int candidatesInTrack = 0;
	vector<unsigned int> minimalTrackIndexes;

	minimumCandidatesPerTrack = MAX_POSSIBLE_TRACKS;
	for(unsigned int current_track_index = 0; current_track_index < candidateTrackIndexes.size(); current_track_index++)
	{
		candidatesInTrack = m_tracks[candidateTrackIndexes[current_track_index]].candidateStats.size();
		if((candidatesInTrack < minimumCandidatesPerTrack) 
			&& (candidatesInTrack > 0))
		{
			minimumCandidatesPerTrack = (unsigned int) (candidatesInTrack);
			minimalTrackIndexes.clear();
			minimalTrackIndexes.push_back(candidateTrackIndexes[current_track_index]);
		}
		else if(candidatesInTrack == minimumCandidatesPerTrack)
		{
			minimalTrackIndexes.push_back(candidateTrackIndexes[current_track_index]);
		}
	}

	// TODO - check if data of minimalTrackIndexes is still valid after the function ends!
	return minimalTrackIndexes;
}

vector<unsigned int> Camera_i::findCandidateTrackIndexesForStat(unsigned int stat_index)
{
	vector<unsigned int> candidateTrackIndexes;

	for(unsigned int track_index = 0; track_index < m_tracks.size(); track_index++)
	{	
		for(unsigned int current_candidate_index = 0; current_candidate_index < m_tracks[track_index].candidateStats.size(); current_candidate_index++)
		{
			if(m_tracks[track_index].candidateStats[current_candidate_index].centroid == m_stats[stat_index].centroid)
			{
				candidateTrackIndexes.push_back(track_index);
				break;
			}
		}
	}	
	
	return candidateTrackIndexes;
}

void Camera_i::removeSelectedCandidateFromTracks(Stat statToRemove)
{
	// remove selected candidate from other tracks so no one else will pick it too
	for(unsigned int other_track_index = 0; other_track_index < m_tracks.size(); other_track_index++)
	{	
		for(unsigned int current_candidate_index = 0; current_candidate_index < m_tracks[other_track_index].candidateStats.size(); current_candidate_index++)
		{
			if(m_tracks[other_track_index].candidateStats[current_candidate_index].centroid == statToRemove.centroid)
			{
				// move the last element to cell number l in candidateStats and pop its previous instance from the vector
				m_tracks[other_track_index].candidateStats[current_candidate_index] = 
					m_tracks[other_track_index].candidateStats[m_tracks[other_track_index].candidateStats.size() - 1];
				m_tracks[other_track_index].candidateStats.pop_back();
			}
		}
	}	
}

int Camera_i::findClosestCandidateIndex(unsigned int track_index)
{
	double minimum_distance = -1; // -1 means minimum_distance was not yet updated
	int closestCandidateIndex = -1;

	for(unsigned int j = 0; j < m_tracks[track_index].candidateStats.size(); j++)
	{
		double distance = norm(m_tracks[track_index].location - m_tracks[track_index].candidateStats[j].centroid);
		if((distance < minimum_distance) || (minimum_distance < 0))
		{
			minimum_distance = distance;
			closestCandidateIndex = j;
		}
	}

	return closestCandidateIndex;
}


vector<unsigned int> Camera_i::sortTracksAccordingCandidatesNum()
{
	vector<unsigned int> sortedTracks;
	unsigned int sortedTracksArray[MAX_POSSIBLE_TRACKS];

	for(int i = 0; i < m_tracks.size(); i++)
	{
		sortedTracksArray[i] = i;
	}

	// min sort
	for(int i = 0; i < m_tracks.size(); i++)
	{
		unsigned int min = m_stats.size();
		unsigned int min_index = 0;
		for(int j = i + 1; j < m_tracks.size(); j++)
		{
			if(m_tracks[j].candidateStats.size() < min)
			{
				min_index = j;
				min = m_tracks[j].candidateStats.size();
			}
		}
		unsigned int temp = sortedTracksArray[i];
		sortedTracksArray[i] = sortedTracksArray[min_index];
		sortedTracksArray[min_index] = temp;
	}

	for(int i = 0; i < m_tracks.size(); i++)
	{
		sortedTracks.push_back(sortedTracksArray[i]);
	}

	return sortedTracks;
}

void Camera_i::cleanPreviousFrameData()
{
	m_assignedTracksIndexes.clear();
	m_unAssignedTracksIndexes.clear();
	// m_lostTrackIndexes.clear();
	m_unAssignedStats.clear();
}

void Camera_i::initializeTracks()
{
	for(int stat_ind = 0; stat_ind < m_stats.size(); stat_ind++)
	{
		createNewTrack(m_stats[stat_ind]);
	}
}
	

void Camera_i::detectionToTrackAssignment()
{
	#define DEBUG
	#ifdef DEBUG
		Track debug_array[MAX_POSSIBLE_TRACKS]; 
	#endif

	vector<unsigned int> minimalTrackIndexes;
	vector<unsigned int> candidateTrackIndexes;

	// step 0
	assignCandidatesToTracks();

	// step 1 - sort tracks according to candidates number
	vector<unsigned int> sortedTracks = sortTracksAccordingCandidatesNum();

	// step 2 - assigne stats to tracks according distance from kalmanFilter position in the sequence of step's 1 sort.
	// if no stats in track's search box - tag it as unassigned track
	for(int i = 0; i < m_tracks.size(); i++)
	{	
		unsigned int track_index = sortedTracks[i];
		Track temp1 = m_tracks[track_index];
		unsigned int temp2 = m_tracks[track_index].candidateStats.size();

		if(m_tracks[track_index].candidateStats.size() > 0)
		{
			// Find closest stat to the current track
			int closestCandidateIndex = findClosestCandidateIndex(track_index);
			Stat candidateStat = m_tracks[track_index].candidateStats[closestCandidateIndex];
			m_tracks[track_index].centroid = candidateStat.centroid;
			m_tracks[track_index].bbox = candidateStat.bbox;
			create_biggerBox(candidateStat, m_tracks[track_index].biggerBox);
			create_sBox(candidateStat, m_tracks[track_index].sbox);

			// tag it as assigned track
			m_assignedTracksIndexes.push_back(track_index);
			// step 3 - remove the selected candidate from other tracks so no one else will pick it too
			removeSelectedCandidateFromTracks(candidateStat); 
		}
		else
		{
			bool lost = false;

			for(unsigned int current_lost_track_index = 0; current_lost_track_index < m_lostTrackIndexes.size(); current_lost_track_index++)
			{
				if(track_index == m_lostTrackIndexes[current_lost_track_index])
				{
					// track is assigned
					lost = true;
					break;
				}
			}
			if(lost == false)
			{
				// unassigned track but also track that is not yet lost
				m_unAssignedTracksIndexes.push_back(track_index);
			}
		}
	}

	// step 4 - stats that weren't selected by any track will be taged as unassigned stats/ centroids
	for(int i = 0; i < m_stats.size(); i++)
	{
		bool assigned = false;
		for(int j = 0; j < m_assignedTracksIndexes.size(); j++)
		{
			unsigned int current_assigned_track_undex = m_assignedTracksIndexes[j];
			Track current_assigned_track = m_tracks[current_assigned_track_undex];
			if(current_assigned_track.centroid == m_stats[i].centroid)
			{
				assigned = true;
			}
		}
		if(assigned == false)
		{
			m_unAssignedStats.push_back(m_stats[i]);
		}
	}
}

bool Camera_i::isCandidateCentroidsExists()
{
	bool res = false;
	if(m_stats.size() > 0)
	{
		res = true;
	}
	else
	{
		res = false;
	}

	return res;
}

vector<double> Camera_i::getTracksDistances()
{
	vector<double> distancesVector;
	for(int i = 0; i < m_tracks.size(); i++)
	{
		distancesVector.push_back(m_tracks[i].distance);
	}

	return distancesVector;
}

Vector<Point2d> Camera_i::getTracksLocations()
{
	Vector<Point2d> locationsVector;
	for(int i = 0; i < m_tracks.size(); i++)
	{
		locationsVector.push_back(m_tracks[i].location);
	}

	return locationsVector;
}


Vector<Point2d> Camera_i::getTracksPositions()
{
	Vector<Point2d> positionsVector;
	for(int i = 0; i < m_tracks.size(); i++)
	{
		positionsVector.push_back(m_tracks[i].position);
	}

	return positionsVector;
}

void Camera_i::displayTrackingResults(SharedData sharedData)
{
	string str1, str2;
	char charStr[100] = {0};

	for(int i = 0; i < m_assignedTracksIndexes.size(); i++)
	{
		Track track = m_tracks[m_assignedTracksIndexes[i]];
		circle(m_frame_RGB, track.centroid, 1, RED, 2);	// centroid
		str1 = std::to_string(track.id);
		putText(m_frame_RGB, str1 , Point2d(track.centroid.x + 5, track.centroid.y+5), 1, 1, RED);
		if(track.age > 40)
		{
			circle(m_frame_RGB, track.location, 1, YELLOW, 2);	// kalman
		}
		rectangle( m_frame_RGB, track.bbox, YELLOW, 2);	// bounding box
		
		rectangle( m_frame_RGB, track.sbox, MAGENTA, 2);	// search box
		putText(m_frame_RGB, "bBox", Point2d(track.bbox.x, track.bbox.y-5), 1, 1, YELLOW);
		putText(m_frame_RGB, "sBox", Point2d(track.sbox.x, track.sbox.y-5), 1, 1, MAGENTA);
		putText(m_frame_RGB, str1 , Point2d(track.centroid.x + 5, track.centroid.y+5), 1, 1, RED);
	}
	
	str1 = std::to_string(currentFrameIndex);
	putText(m_frame_RGB, "Frame Number: " + str1, Point2d(5, 15), 1, 1, YELLOW);
	
	str1 = std::to_string(sharedData.timestamp);
	putText(m_frame_RGB, "Time: " + str1, Point2d(5, 30), 1, 1, YELLOW);

	str1 = std::to_string(sharedData.theta);
	putText(m_frame_RGB, "Angle: " + str1, Point2d(5, 45), 1, 1, YELLOW);
	
	putText(m_frame_RGB, "Robots data: ", Point2d(5, 60), 1, 1, YELLOW);
	
	for(int robotInd = 0; robotInd < sharedData.distances.size(); robotInd++)
	{
		str1 = std::to_string(robotInd);
		putText(m_frame_RGB, "Robot Index: " + str1, Point2d(5, 75 + robotInd * 65), 1, 1, YELLOW);
		
		str1 = std::to_string(sharedData.distances[robotInd]);
		
		str1 += "." +  std::to_string(((int)(sharedData.distances[robotInd] * 1000)) % 1000);
		putText(m_frame_RGB, "Robot Distance: " + str1, Point2d(5, 90 + robotInd * 65), 1, 1, YELLOW);

		str1 = std::to_string(sharedData.locations[robotInd].x);
		str2 = std::to_string(sharedData.locations[robotInd].y);
		putText(m_frame_RGB, "Robot Location: (" + str1 + ", " + str2 + ")", Point2d(5, 105 + robotInd * 65), 1, 1, YELLOW);

		str1 = (sharedData.positions[robotInd].x >= 0) ? std::to_string(abs(sharedData.positions[robotInd].x)) : "-" + (string) charStr;
		str1 += "." + std::to_string(abs(((int)(sharedData.positions[robotInd].x * 1000)) % 1000));

		str2 = (sharedData.positions[robotInd].y >= 0) ? std::to_string(abs(sharedData.positions[robotInd].y)) : "-" + (string) charStr;
		str2 += "." + std::to_string(abs(((int)(sharedData.positions[robotInd].y * 1000)) % 1000));
		putText(m_frame_RGB, "Robot Position: (" + str1 + ", " + str2 + ")", Point2d(5, 120 + robotInd * 65), 1, 1, YELLOW);
	}
	try
	{
		imshow("Draws Bounding and Search boxes", m_frame_RGB);
		waitKey(10);
	}
	catch(...)
	{

	}
}

bool Camera_i::isUnassignedTracksExists()
{
	bool res = false;
	res = (m_unAssignedTracksIndexes.empty() == false) ? true : false;
	return res;
}

bool Camera_i::cameraInit()
{
	numFrames = 0;
	currentFrameIndex = 0;


	#if (VIDEO_INPUT == CAMERA)

		//default capture width and height
		const int FRAME_WIDTH = 1280;
		const int FRAME_HEIGHT = 1024;

		// init camera
		//HIDS hCam = (HIDS) 0;				// open next camera
		//is_InitCamera (&hCam, NULL);		// init camera - no window handle required	

		//if ( hCam != 0 )
		//{
			/* capture picture */
		//	if( is_FreezeVideo( hCam, IS_WAIT ) == IS_SUCCESS )
				/* display picture */
			//	is_RenderBitmap( hCam, 0, NULL, IS_RENDER_NORMAL );
		//}
		//open capture object at location zero (default location for webcam)
		videoCapture.open(0);
		//set height and width of capture frame
		videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
		videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	#else
		bool isOpen = videoCapture.open(VIDEO_FILE_PATH);
		if(isOpen == false)
		{
			// error ocured in openning the file
			printf("\n\n\n\t\t@@@@@@@@ Fatal error: can not open video file! @@@@@@\n\n\n");
			exit(1);
		}

		numFrames = (unsigned int)videoCapture.get(CV_CAP_PROP_FRAME_COUNT);
	#endif

	return true;
}

void Camera_i::imadjust(Mat& frame, double contrast, int brightness)
{
	Mat new_image = Mat::zeros( frame.size(), frame.type() );

	// Do the operation new_image(i,j) = alpha*image(i,j) + beta
	for( int y = 0; y < frame.rows; y++ )
	{ 
		for( int x = 0; x < frame.cols; x++ )
		{ 
			for( int c = 0; c < frame.channels(); c++ )
			{
					new_image.at<Vec3b>(y,x)[c] =
						saturate_cast<uchar>( contrast * ( frame.at<Vec3b>(y,x)[c] ) + brightness );
			}
		}
	}

	frame = new_image;
 }


void Camera_i::removeSmallBlobs(cv::Mat& im, double size)
	/**
 * Replacement for Matlab's bwareaopen()
 * Input image must be 8 bits, 1 channel, black and white (objects)
 * with values 0 and 255 respectively
 */
{
    // Only accept CV_8UC1
    if (im.channels() != 1 || im.type() != CV_8U)
	{
        return;
	}
    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(im.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = cv::contourArea(contours[i]);

        // Remove small objects by drawing the contour with black color
        if (area > 0 && area <= size)
		{
            cv::drawContours(im, contours, i, CV_RGB(0,0,0), -1);
		}
    }
}

#define FRAME_SAMPLING_RATE 5

bool Camera_i::captureFrame(unsigned int* currentFramesIndex, double* realTime)
{
	bool endvideoflag = false;
	static int i = 0;
	//store image to matrix
	// videoCapture.read(m_frame_RGB);
	// TODO - print real time
	
	Mat temp;

	for(int i = 0; i < FRAME_SAMPLING_RATE && currentFrameIndex < numFrames; i++)
	{
		videoCapture.read(m_frame_RGB);
		currentFrameIndex++;
		*currentFramesIndex = currentFrameIndex;
	}

	*realTime = videoCapture.get(CV_CAP_PROP_POS_MSEC);
	
	//for(int i = 0; i < m_assignedTracksIndexes.size(); i++)
	//{
	//	Track currentTrack1 = m_tracks[m_assignedTracksIndexes[i]];

	//	for(int j = i + 1; j < m_assignedTracksIndexes.size(); j++)
	//	{
	//		Track currentTrack2 = m_tracks[m_assignedTracksIndexes[j]];
	//		Rect currentIntersection = currentTrack1.biggerBox & currentTrack2.biggerBox;
	//		if((currentIntersection.width > 0) || (currentIntersection.height > 0))
	//		{
	//			rectangle(m_frame_RGB, currentIntersection, BLACK, -1);	// bigger boxes
	//		}
	//	}
	//}
	

	//imshow("bw_frame", m_frame_RGB);
	//waitKey(1);

	/* RGB to Gray scale and Multiply the frame with the mask */
	cvtColor(m_frame_RGB, m_frame_GRAY, CV_BGR2GRAY);
	
	// imshow("CapturedImage", m_frame_GRAY);
	// waitKey(1);

	/* offline- recording */

	
	

	#if (VIDEO_INPUT == AVI_FILE)
	{
		if(currentFrameIndex >= numFrames)
		{
			endvideoflag = true;
		}
	}
	#endif

	cout << "\rindexFrames= " << currentFrameIndex;

	return endvideoflag;

}


Mat Camera_i::detectColor(Mat& frameHSV)
{
	Mat thresholdImage, m_frame_RGB_resized ;
	 // imshow("before resize",m_frame_RGB);
	// waitKey(1);	
	resize(m_frame_RGB, m_frame_RGB_resized, Size(),  1 / systemParameters.scale, 1 / systemParameters.scale);
	// imshow("resized RGB frame",m_frame_RGB_resized);
	// waitKey(1);	
	imadjust(m_frame_RGB_resized, hsvParameters.contrastValue, hsvParameters.brightnessValue);
     // imshow("adjusted RGB frame",m_frame_RGB_resized);
	// waitKey(1);	
	//convert frame from BGR to HSV colorspace
	cvtColor(m_frame_RGB_resized, frameHSV, COLOR_BGR2HSV);
	 // imshow("frameHSV", frameHSV);
	// waitKey(1);	
	//filter HSV image between values and store filtered image to threshold matrix
	inRange(frameHSV, Scalar(hsvParameters.hsvColors[H_MIN], hsvParameters.hsvColors[S_MIN], hsvParameters.hsvColors[V_MIN]),
		Scalar(hsvParameters.hsvColors[H_MAX], hsvParameters.hsvColors[S_MAX], hsvParameters.hsvColors[V_MAX]), thresholdImage);
	
	// imshow("Thresholded Image", thresholdImage);
	// waitKey(1);	
	return thresholdImage;
}


bool Camera_i::detectTable(Mat imborder, int attempts)
{
	Mat bw_dilate, bw_inv, bw_fill, tableMask;
	Rect filledSize;
	/* init Rect struct */
	filledSize.x = 0;
	filledSize.y = 0;
	filledSize.width = imborder.cols;
	filledSize.height = imborder.rows;

	bool tableFound = false;
	int attemptsCount = 0;

	/* Choose morphological structuring element */
	if(attempts == 1)
	{
		// DO_NOTHING. Use default morfological element 
	}
	else
	{
		/* Create bigger morphological structuring element */
		// TODO conversion from 'double' to 'int', possible loss of data
		int coefficient = (int) (((1 + 0.5 * attempts) * TABLE_MORPH_ELEMENT_SIZE)/ systemParameters.scale);
		Size ksize = Size(coefficient, coefficient); 
		detectionParameters.seTable = getStructuringElement(MORPH_ELLIPSE , ksize);
	}
	
	try
	{
		/* Dilate image */
		dilate(imborder, bw_dilate, detectionParameters.seTable);
		 // imshow("bw_dilate", bw_dilate);
		// waitKey(1);	
		/* Image inverse */
		bitwise_not( bw_dilate, bw_inv );
		 // imshow("bw_inv", bw_inv);
		// waitKey(1);	
		/* Fill holes 
		* bw_dilate - input image,  Point(bw_dilate.cols/2 ,bw_dilate.rows/2) ? Starting point: image center
		* 255 - New value of the repainted domain pixels (White color)
		* filledSize Optional output parameter set by the function to the minimum bounding rectangle of the repainted domain
		*/
		floodFill( bw_dilate, Point( bw_dilate.cols/2 ,bw_dilate.rows/2 ), 255, &filledSize );
		 // imshow("bw_dilate_filled", bw_dilate);
		// waitKey(1);	
		/* floodFill not suppose to fill all the frame */
		if (filledSize.width * filledSize.height < detectionParameters.filledPart * (imborder.cols * imborder.rows))
		{
			tableFound = true;
		}

		/* Take pixels that are in the image inverse and in filled image */
		bitwise_and( bw_dilate, bw_inv, tableMask );
		 // imshow("tableMask", tableMask);
		// waitKey(1);	
		/* Remove small objects from binary image */
		removeSmallBlobs( tableMask, detectionParameters.minAreaTable );
		 // imshow("Remove small objects", tableMask);
		// waitKey(1);	
		/* Dilate image to fill holes*/
		for (int i = 0; i < 1; i++)
		{
			dilate( tableMask, tableMask, detectionParameters.seTable );
		}
		// imshow("tableMask", tableMask);
		// waitKey(1);	
		/* Close image */
		morphologyEx(tableMask,tableMask,MORPH_CLOSE,detectionParameters.seTable);
		
		// imshow("closed image", tableMask);
		// waitKey(1);	
		/* Resize the image mask back to the full resolution */
		resize(tableMask, m_tableMask, Size(), systemParameters.scale, systemParameters.scale);
		// imshow("resized back", m_tableMask);
		// waitKey(1);	
		// imshow("Table Detection", m_frame_GRAY);
		// waitKey(20);
		//// waitKey(20);

	}
	catch(...)
	{
		cout << "failed in current frame: detectTable" << endl;
	}
	//// waitKey(1);
	return tableFound;
}


Stats Camera_i::regionStats(vector<vector<Point> > contours)
{
	Stats stats;
	Stat stat;

	// centroid      stats.centroid = calculateCentroid(contours, idx);
	// TODO check if NULL is appropriate value to return (NULL - set to (0.0, 0.0) )

	for( int idx = 0; idx < contours.size(); idx++ )
	{
		// Measure properties of image regions
		try
		{
			if(contours[idx].size() < 5)
			{
				continue;
			}
			//stat = contours.at(idx);
			stat.centroid = Point2d(NULL);
			Moments m = moments(contours[idx]);

			if (m.m00 != 0.0)
			{
				double cx = m.m10 / m.m00;
				double cy = m.m01 / m.m00;
				stat.centroid = Point2d(cx, cy);		
			}
		
			// bounding box
			stat.bbox = boundingRect(contours[idx]);

			//area
			stat.area = contourArea(contours[idx]);

			//eccentricity = sqrt( 1 - (ma/MA)^2) --- ma= minor axis --- MA= major axis
			RotatedRect tempEllipse = fitEllipse(contours[idx]);

			float axisRatio = pow((tempEllipse.size.width / tempEllipse.size.height),2);
			stat.eccentricity = sqrt( 1 - axisRatio );
			
			// solidity = contour area / convex hull area
			vector<vector<Point> >hull( contours.size() );
			convexHull(contours[idx], hull[idx]);
			stat.solidity = contourArea(contours[idx]) / float(contourArea(hull[idx]));
			stats.stat_array.push_back(stat);
		}
		catch(Exception& e)
		{
			cout << e.what() << endl;
			continue;
		}
	}

	
    return stats;
}


bool Camera_i::isRobot(const Stat stat)
{
	bool res = false;
	// TODO - optimize
	//Mat bw_frame = m_frame_GRAY > 128;
	//Scalar colour = bw_frame.at<uchar>(Point(stat.centroid.x, stat.centroid.y));
	bool temp1 = stat.eccentricity > detectionParameters.minEccentricity;
	bool temp2 = stat.eccentricity < detectionParameters.maxEccentricity;
	bool temp3 = stat.solidity > detectionParameters.minSolidity;
	bool temp4 = stat.solidity < detectionParameters.maxSolidity;
	bool temp5 = stat.area > detectionParameters.minAreaRobot;


	if((stat.eccentricity > detectionParameters.minEccentricity) && (stat.eccentricity < detectionParameters.maxEccentricity) &&
		(stat.solidity > detectionParameters.minSolidity) && (stat.solidity < detectionParameters.maxSolidity) &&
		(stat.area > detectionParameters.minAreaRobot))
	{
		res = true;
	}
	else
	{
		res =  false;
	}

	return res;
}


void Camera_i::create_sBox(const Stat stat, Rect& sBox)
{
	// Create search boxes by enlarging the bounding boxes 
	int resize = (stat.bbox.width + stat.bbox.height) / 2;
	sBox.x = stat.bbox.x - resize;
	sBox.y = stat.bbox.y - resize;
	sBox.width =  stat.bbox.width + 2 * resize;
	sBox.height = stat.bbox.height + 2 * resize;
}

void Camera_i::create_biggerBox(const Stat stat, Rect& sBox)
{
	// Create search boxes by enlarging the bounding boxes 
	int resize = (stat.bbox.width + stat.bbox.height) / 2;
	sBox.x = stat.bbox.x - (resize / 3) - 1;
	sBox.y = stat.bbox.y - (resize / 3) - 1;
	sBox.width =  stat.bbox.width + (2 * resize / 3) + 1;
	sBox.height = stat.bbox.height + (2 * resize/ 3) + 1;
}

/*
%	Function transforms from the camera (pixels) coordinates 
%	to the real world coordinates (m)
%	The transformation is done based on David Scaramuzza 
%	OCamCalib: Omnidirectional Camera Calibration Toolbox for Matlab
%	https://sites.google.com/site/scarabotix/ocamcalib-toolbox
*/

double Camera_i::polyval(double* pol, int length_pol, double val)
{
	double zp  = pol[0];
	double r_i = 1;
	int i;
 
	for (i = 1; i < length_pol; i++)
	{
		r_i *= val;
		zp  += r_i*pol[i];
	}

	return zp;
}

double* reverseArray(double* arr, int arr_len)
{
	double* reversed_arr = (double*) malloc(arr_len * sizeof(double));

	for(int i = 0; i < arr_len; i++)
	{
		reversed_arr[i] = arr[arr_len - (i + 1)];
	}

	return reversed_arr;
}

void Camera_i::measureDistances()
{
	// TODO - move to init module!
	struct ocam_model ocam; // our ocam_models for the  catadioptric camera
	get_ocam_model(&ocam, "./calib_results_catadioptric.txt"); 
	// TODO - take from relevant file instead of hardcoded
	ocam.xc = 423.6670;
	ocam.yc = 431.7341;
	ocam.pol[0] = -1.533201461972195e+02;
	ocam.pol[1] = 0;
	ocam.pol[2] = 0.001913356589100;
	ocam.pol[3] = -2.150965186281575e-06;
	ocam.pol[4] = 1.911730571142784e-09;

	Point2d occamModelCenter(ocam.xc, ocam.yc);
	
	// TODO - check if relevant for lost tracks also
	for(int current_track_index = 0; current_track_index < m_tracks.size(); current_track_index++)
	{
		// Move the ref. system to the image center:
		Point2d currentTrackCentroid = m_tracks[current_track_index].centroid;
		Point2d c_center = currentTrackCentroid - occamModelCenter; // [x;y] x-right, y-up
		c_center.y *= -1;											// directions: x-Right, y-Up

		// Tranformation
		double c_center_dist = sqrt(pow(c_center.x, 2) + pow(c_center.y, 2));
		double z = polyval(ocam.pol, ocam.length_pol, c_center_dist);
		double scale = detectionParameters.parameterZ /z;


		// Real world targets positions 
		Point2d r = c_center * scale;
		m_tracks[current_track_index].position = r;
		m_tracks[current_track_index].distance = norm(r);
	}
}


vector<int> Camera_i::find(Stats stats)
{
	vector<int> idx;

	for(int i = 0; i < stats.stat_array.size(); i++)
	{
		/* idx = find(([stats.Eccentricity] > e_threshold(1)) .* ([stats.Eccentricity] < e_threshold(2)) .* ...
		([stats.Solidity] > sol_threshold(1)) .* ([stats.Solidity] < sol_threshold(2)) .* ...
		([stats.Area] > minArea)); */
		if(isRobot(stats.stat_array[i]))
		{
			idx.push_back(i);
		}
	}

	return idx;
}

void Camera_i::initKalmanFilter(Track* track)
{
	track->kalmanFilter = KalmanFilter(4, 2, 0);
	track->kalmanFilter.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	track->kalmanFilter.statePre.at<float>(0) = (float)track->centroid.x;
	track->kalmanFilter.statePre.at<float>(1) = (float)track->centroid.y;
	track->kalmanFilter.statePre.at<float>(2) = 0;
	track->kalmanFilter.statePre.at<float>(3) = 0;
	setIdentity(track->kalmanFilter.measurementMatrix);
	setIdentity(track->kalmanFilter.processNoiseCov, Scalar::all(9));
	setIdentity(track->kalmanFilter.measurementNoiseCov, Scalar::all(8));
	setIdentity(track->kalmanFilter.errorCovPost, Scalar::all(.1));
}

void Camera_i::detectRobots(int addRobots)
{
	Vector<Rect> biggerBoxes;
	Vector<Rect> bboxes;
	Vector<Rect> sboxes;
	Vector<Point2d> centroids;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Rect> bboxesPrev;
	vector<int> detected;
	vector<int> idx;
	Mat bw_frame;
	
	Stats stats;
	bool detectedRobot = false;
	Rect sBox;

	// TODO configure camera parameters to remove imadjust
	try
	{
		// imshow("m_frame_GRAY", m_frame_GRAY);
		// waitKey(20);
		// imshow("m_tableMask", m_tableMask);
		// waitKey(20);
		m_stats.clear();
		// TODO check if necessary?
		bitwise_and(m_frame_GRAY, m_tableMask, m_frame_GRAY);

		// Convert a frame from grayscale to BW
		bw_frame = m_frame_GRAY > 128;
		//threshold(m_frame_GRAY, bw_frame, 110, 255, CV_THRESH_BINARY);
		// imshow("BW frame", bw_frame);
		// waitKey(20);

		// Morphologically close image
		morphologyEx(bw_frame,bw_frame,MORPH_CLOSE,detectionParameters.seRobot);           /* Morphologically close image */
		// imshow("closed image", bw_frame);
		// waitKey(20);

		// Erode image to remove possible connections between targets
		erode(bw_frame,bw_frame,detectionParameters.seErode);							   /* Erode image to remove possible connections between targets */
		// imshow("Erode image", bw_frame);
		// waitKey(2);


		// Find connected components in binary image. Needed to find region properties (bwconncomp - in matlab)
		findContours(bw_frame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point2d(0,0));
		//findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			
		// Measure properties of image regions
		stats = regionStats(contours);
		
		// Indexes of suitable regions.
		// Taking into account eccentricity, solidity and size of each region
		idx = find(stats);

		// Save centroids and bounding boxes of the detected regions
		for (int i = 0; i < idx.size(); i++)
		{
			m_stats.push_back(stats.stat_array[idx[i]]);

			// TODO - remove
			m_stats_debug[i] = stats.stat_array[idx[i]];
		}

		for(int i = 0; i < m_tracks.size(); i++)
		{
			m_tracks_debug[i] = m_tracks[i];
		}
	}
	catch(const exception &e)
	{
		cout << "failed in current frame: detectRobots: " << e.what() << endl;
		//systemParameters.addRobots = true;
	}
}

void Camera_i::predictNewLocationsOfTracks()
{
	// Kalman filter prediction step
	// Search for the targets in the area it was previously seen
	for (int visibleRobotIndex = 0; visibleRobotIndex < m_tracks.size(); visibleRobotIndex++)
	{
		//Predict the current location of the track.
		Mat prediction = m_tracks[visibleRobotIndex].kalmanFilter.predict();
	}

}
