/*
 * PatternRec.h
 *
 *  Created on: 2014-8-13
 *      Author: frankxie
 */

#ifndef PATTERNREC_H_
#define PATTERNREC_H_

#include "../HelperFunctions.h"
#include "../autopilot/DroneController.h"
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvd/thread.h>

#include "tum_ardrone/filter_state.h"
#include "sensor_msgs/Image.h"
#include <pthread.h>
#include "boost/thread.hpp"
#include <string>

class EstimationNode;


class PatternRec:private CVD::Thread
{
public:
	PatternRec(EstimationNode *nde);
	~PatternRec();

	void startSystem();
	void stopSystem();
	void getDroneState();
	void currentImage(const sensor_msgs::ImageConstPtr img);
	bool currentImgProc();
	void calcDistance3d(int minmaxPoints[][2]);
	void prSetReached();
	void flyToTarget();
	void flyBackward(short tag);	//0:normal; 1:urgent
	bool currentSafe();

private:
	EstimationNode *nde;	//communication
	cv::Mat currentFrame,processFrame;
	int imgCenPtU,imgCenPtV,focalP;
	int borderDis;
	float refDiagnol; //mm
	float safeDis;	//m
	float backDis;	//m
	float safeBackDis;	//m
	float maxAxisDis;	//m

	TooN::Vector<3> delta;	//mm

	void run();

	boost::condition_variable current_frame_signal;
	pthread_mutex_t current_frame_signal_mutex;
	pthread_mutex_t hover_turn_mutex;
	tum_ardrone::filter_state currentPose;

	bool keepRunning;
	bool currentImageAvailable;
	bool targetAvailable;
	bool targetReached;
	bool finalFlag;

	unsigned char redLowTh,redHighTh,generalRedTh,lowTh;

	short countBack;
};

#endif /* PATTERNREC_H_ */
