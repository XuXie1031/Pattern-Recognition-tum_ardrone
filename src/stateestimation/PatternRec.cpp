/*
 * PatternRec.cpp
 *
 *  Created on: 2014-8-13
 *      Author: frankxie
 */

#include "PatternRec.h"
#include "EstimationNode.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

int counter=0;
PatternRec::PatternRec(EstimationNode *nde)
{
	this->nde=nde;
	//imgCenPtU=323,imgCenPtV=177,focalP=440;
	imgCenPtU=320,imgCenPtV=160,focalP=440; // in pixels
	borderDis=10;		//pixel frame
	refDiagnol=120.0;	//measured (mm)
	safeDis=0.25;	//(m)
	backDis=0.30;	//(m)
	safeBackDis=0.50;	//(m)
	maxAxisDis=0.80;	//(m)

	countBack=0;
	//current_frame_signal_mutex = PTHREAD_MUTEX_INITIALIZER;
	cv::namedWindow("view");
	cvStartWindowThread();
	cv::namedWindow("process");
	cvStartWindowThread();
	/*cv::namedWindow("initHSV");
	cvStartWindowThread();
	cv::namedWindow("processHSV");
	cvStartWindowThread();*/
	currentImageAvailable=false;
	targetAvailable=false;
	targetReached=false;
	finalFlag=false;

	//startSystem();
}

PatternRec::~PatternRec()
{
	//delete nde;
}

void PatternRec::startSystem()
{
	keepRunning=true;
	start();
	ROS_INFO("Rec_start!");
}

void PatternRec::stopSystem()
{
	keepRunning=false;
	cv::destroyWindow("view");
	cv::destroyWindow("process");
	/*cv::destroyWindow("initHSV");
	cv::destroyWindow("processHSV");*/
	current_frame_signal.notify_all();
	join();
}

void PatternRec::run()
{
	std::cout << "Waiting for Pattern_start" << std::endl;
	while(!currentImageAvailable)
		usleep(100000);	// sleep 100ms
	currentImageAvailable = false;
	while(!currentImageAvailable)
		usleep(100000);

	ros::Rate looprate(5);
	while(keepRunning)
	{
		if(currentImageAvailable)
		{
			ROS_INFO("PatternRec_Running");
			currentFrame.copyTo(processFrame);
			targetAvailable=currentImgProc();
			cv::imshow("process",processFrame);
			//cv::imwrite("process.bmp",processFrame);
			if(targetAvailable)	//maybe adjust within the scope!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			{
				countBack=0;
				if(!targetReached)		//fly to the target
				{
					ROS_INFO("fly to the target");
					getDroneState();
					flyToTarget();
					ros::Rate innerRate(10);
					while(!targetReached)
					{
						//ROS_INFO("flying to the target");
						innerRate.sleep();
					}
				}
				if(finalFlag)
				{
					if(currentSafe())
					{
						ros::Rate hoverRate(1);
						ROS_INFO("hover in front of target");
						//nde->publishCommand("c hover");
						hoverRate.sleep();
					}
					else
					{
						ROS_INFO("keep safe");
						targetReached=false;
						getDroneState();
						flyBackward(1);
						ros::Rate innerRate(10);
						while(!targetReached)
						{
							//ROS_INFO("moving backward");
							innerRate.sleep();
						}
						targetReached=false;	//should fly to the target now
					}
					finalFlag=false;
				}
				targetReached=false;
			}
			else
			{
				if(countBack<5)
				{
					ROS_INFO("fly backward");
					targetReached=false;
					getDroneState();
					flyBackward(0);
					ros::Rate innerRate(10);
					while(!targetReached)
					{
						//ROS_INFO("moving backward");
						innerRate.sleep();
					}
					countBack++;
				}
				else
				{
					ROS_INFO("hover and take turn");
					nde->publishCommand("c turn");
					ros::Rate hoverRate(1);
					hoverRate.sleep();
				}
				targetReached=false;	//should fly to the target now
				finalFlag=false;
			}

		}
		//looprate.sleep();
		ROS_INFO("one turn is over");
	}

	//stopSystem();
}

void PatternRec::getDroneState()
{
	currentPose=nde->s;
	ROS_INFO("get currentPose: x=%f y=%f z=%f yaw=%f",currentPose.x,currentPose.y,currentPose.z,currentPose.yaw);
}

void PatternRec::prSetReached()
{
	targetReached=true;
}

void PatternRec::currentImage(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	cv_ptr->image.copyTo(currentFrame);
	currentImageAvailable = true;
	cv::imshow("view",currentFrame);
	/*std::stringstream ss;
	ss<<"frame"<<counter<<".bmp";
	cv::imwrite(ss.str(),currentFrame);
	counter++;
	ros::Rate rate(1);
	rate.sleep();*/
	current_frame_signal.notify_all();
}

bool PatternRec::currentSafe()
{
	int countGenR=0;
	for(int i=0;i<currentFrame.rows;i++)
	{
		for(int j=0;j<currentFrame.cols;j++)//"bgr"
		{
			if(currentFrame.data[3*(i*currentFrame.cols+j)]<lowTh&&currentFrame.data[3*(i*currentFrame.cols+j)+1]<lowTh)
			{
				if(currentFrame.data[3*(i*currentFrame.cols+j)+2]>generalRedTh)
					countGenR++;
			}
		}
	}

	if(countGenR>currentFrame.rows*currentFrame.cols*2/3||countGenR<10)
		return false;
	return true;
}

void PatternRec::calcDistance3d(int minmaxPoints[][2])
{
	cv::Point cenPt;
	float diag1P=sqrt(pow(minmaxPoints[0][0]-minmaxPoints[1][0],2)+pow(minmaxPoints[0][1]-minmaxPoints[1][1],2)),
		  diag2P=sqrt(pow(minmaxPoints[2][0]-minmaxPoints[3][0],2)+pow(minmaxPoints[2][1]-minmaxPoints[3][1],2)),
		  diagP;

	if(diag1P>diag2P)
	{
		cenPt.x=(minmaxPoints[0][0]+minmaxPoints[1][0])/2;cenPt.y=(minmaxPoints[0][1]+minmaxPoints[1][1])/2;
		diagP=diag1P;
	}
	else
	{
		cenPt.x=(minmaxPoints[2][0]+minmaxPoints[3][0])/2;cenPt.y=(minmaxPoints[2][1]+minmaxPoints[3][1])/2;
		diagP=diag2P;
	}

	ROS_INFO("central Red: u=%d, v=%d",cenPt.x,cenPt.y);
	cv::circle(processFrame,cenPt,5.0,cv::Scalar(255,0,0));

	int deltaXP=cenPt.x-imgCenPtU, deltaZP=-(cenPt.y-imgCenPtV),deltaYP=focalP;
	delta[0]=refDiagnol*deltaXP/diagP;delta[1]=refDiagnol*deltaYP/diagP;delta[2]=refDiagnol*deltaZP/diagP;
	ROS_INFO("estimated distance:deltax=%f,deltay=%f,deltaz=%f mm",delta[0],delta[1],delta[2]);
}

bool PatternRec::currentImgProc()
{
	redLowTh=150;redHighTh=255;generalRedTh=130;lowTh=60;	//should have a further adjust; pay more attention!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	int countR=0,countGenR=0;
	//int centralU,centralV;
	int minmaxPoints[4][2]; //minR,maxR,minC,maxC;(x,y)
	minmaxPoints[0][1]=processFrame.rows-1;minmaxPoints[1][1]=0;
	minmaxPoints[2][0]=processFrame.cols-1;minmaxPoints[3][0]=0;
	cv::Mat binSrc(processFrame.rows,processFrame.cols,CV_8UC1);
	//int minR=255,maxR=0;

	/*cv::Mat floatSrc,processHSV(processFrame.rows,processFrame.cols,CV_32FC3);
	processFrame.convertTo(floatSrc,CV_32FC3,1.0/255.0);
	cv::cvtColor(floatSrc,processHSV,CV_BGR2HSV);
	std::vector<cv::Mat> hsvCn;
	cv::split(processHSV,hsvCn);

	float lowHTh=3.35,highHTh=3.85;//3.35,3.85
	for(int i=0;i<processHSV.rows;i++)
	{
		for(int j=0;j<processHSV.cols;j++)
		{
			if(currentFrame.data[3*(i*currentFrame.cols+j)+2]>generalRedTh)
				countGenR++;
			if(hsvCn.at(0).at<float>(i,j)>lowHTh&&hsvCn.at(0).at<float>(i,j)<highHTh)
				hsvCn.at(0).at<float>(i,j)=255;
			else
				hsvCn.at(0).at<float>(i,j)=0;
			//ROS_INFO("h=%f",hsvCn.at(0).at<float>(pt));
		}
	}
	cv::imshow("initHSV",hsvCn.at(0));
	cv::imwrite("initProcess.bmp",hsvCn.at(0));
	for(int i=0;i<processHSV.rows;i++)
	{
		for(int j=0;j<processHSV.cols;j++)
		{
			int medianSize=5,add=0;//(size=5,weight=3)
			if(i<(medianSize-1)/2||i>processHSV.rows-1-(medianSize-1)/2||j<(medianSize-1)/2||j>processHSV.cols-1-(medianSize-1)/2)
			{
				hsvCn.at(0).at<float>(i,j)=0;
				continue;
			}
			float medianP=0.0;
			for(int fi=i-(medianSize-1)/2;fi<=i+(medianSize-1)/2;fi++)
			{
				for(int fj=j-(medianSize-1)/2;fj<=j+(medianSize-1)/2;fj++)
				{
					if(abs(fi-i)<=1&&abs(fj-j)<=1)
						medianP+=3.1*hsvCn.at(0).at<float>(fi,fj);
					else
						medianP+=hsvCn.at(0).at<float>(fi,fj);
				}
			}
			medianP/=(medianSize*medianSize);
			if(medianP>127.0)
			{
				hsvCn.at(0).at<float>(i,j)=255;
				countR++;
				if(i<minmaxPoints[0][1])
				{	minmaxPoints[0][1]=i;minmaxPoints[0][0]=j;}
				if(i>minmaxPoints[1][1])
				{	minmaxPoints[1][1]=i;minmaxPoints[1][0]=j;}
				if(j<minmaxPoints[2][0])
				{	minmaxPoints[2][0]=j;minmaxPoints[2][1]=i;}
				if(j>minmaxPoints[3][0])
				{	minmaxPoints[3][0]=j;minmaxPoints[3][1]=i;}
			}
			else
				hsvCn.at(0).at<float>(i,j)=0;
		}
	}
	cv::imshow("processHSV",hsvCn.at(0));
	cv::imwrite("afterProcess.bmp",hsvCn.at(0));*/
	for(int i=0;i<processFrame.rows;i++)
	{
		for(int j=0;j<processFrame.cols;j++)//"bgr"
		{
			if(processFrame.data[3*(i*processFrame.cols+j)]<lowTh&&processFrame.data[3*(i*processFrame.cols+j)+1]<lowTh)
			{
				if(processFrame.data[3*(i*processFrame.cols+j)+2]>redLowTh&&processFrame.data[3*(i*processFrame.cols+j)+2]<redHighTh)
					binSrc.data[(i*processFrame.cols+j)]=255;
				else
					binSrc.data[(i*processFrame.cols+j)]=0;
				if(currentFrame.data[3*(i*currentFrame.cols+j)+2]>generalRedTh)
					countGenR++;
			}
			else
				binSrc.data[(i*processFrame.cols+j)]=0;
		}
	}
	if(countGenR>currentFrame.rows*currentFrame.cols*2/3)
		return false;
	for(int i=0;i<processFrame.rows;i++)
	{
		for(int j=0;j<processFrame.cols;j++)
		{
			if(i==0||i==processFrame.rows-1||j==0||j==processFrame.cols-1)
			{
				binSrc.data[(i*processFrame.cols+j)]=0;
				continue;
			}
			if(binSrc.data[(i*processFrame.cols+j)]==255)
			{
				if(binSrc.data[((i-1)*processFrame.cols+j-1)]==255||binSrc.data[((i-1)*processFrame.cols+j)]==255||binSrc.data[((i-1)*processFrame.cols+j+1)]==255||
				   binSrc.data[(i*processFrame.cols+j-1)]==255||binSrc.data[(i*processFrame.cols+j+1)]==255||
				   binSrc.data[((i+1)*processFrame.cols+j-1)]==255||binSrc.data[((i+1)*processFrame.cols+j)]==255||binSrc.data[((i+1)*processFrame.cols+j+1)]==255)
				{
					countR++;
					if(i<minmaxPoints[0][1])
					{	minmaxPoints[0][1]=i;minmaxPoints[0][0]=j;}
					if(i>minmaxPoints[1][1])
					{	minmaxPoints[1][1]=i;minmaxPoints[1][0]=j;}
					if(j<minmaxPoints[2][0])
					{	minmaxPoints[2][0]=j;minmaxPoints[2][1]=i;}
					if(j>minmaxPoints[3][0])
					{	minmaxPoints[3][0]=j;minmaxPoints[3][1]=i;}
				}
				else
					binSrc.data[(i*processFrame.cols+j)]=0;
			}
		}
	}
	/*//erosion
	cv::Mat erosionSrc(processFrame.rows,processFrame.cols,CV_8UC1);
	for(int i=0;i<processFrame.rows;i++)
	{
		for(int j=0;j<processFrame.cols;j++)
		{
			if(i==0||i==processFrame.rows-1||j==0||j==processFrame.cols-1)
			{
				erosionSrc.data[(i*processFrame.cols+j)]=0;
				continue;
			}
			if(binSrc.data[(i*processFrame.cols+j)]==255&&binSrc.data[((i-1)*processFrame.cols+j-1)]==255&&binSrc.data[((i-1)*processFrame.cols+j)]==255&&binSrc.data[((i-1)*processFrame.cols+j+1)]==255&&
			binSrc.data[(i*processFrame.cols+j-1)]==255&&binSrc.data[(i*processFrame.cols+j+1)]==255&&
			binSrc.data[((i+1)*processFrame.cols+j-1)]==255&&binSrc.data[((i+1)*processFrame.cols+j)]==255&&binSrc.data[((i+1)*processFrame.cols+j+1)]==255)
				erosionSrc.data[(i*processFrame.cols+j)]=255;
			else
				erosionSrc.data[(i*processFrame.cols+j)]=0;
		}
	}
	//dilation
	cv::Mat dilationSrc(processFrame.rows,processFrame.cols,CV_8UC1);
	for(int i=0;i<processFrame.rows;i++)
	{
		for(int j=0;j<processFrame.cols;j++)
		{
			if(i==0||i==processFrame.rows-1||j==0||j==processFrame.cols-1)
			{
				dilationSrc.data[(i*processFrame.cols+j)]=0;
				continue;
			}
			if(erosionSrc.data[(i*processFrame.cols+j)]==255||erosionSrc.data[((i-1)*processFrame.cols+j-1)]==255||erosionSrc.data[((i-1)*processFrame.cols+j)]==255||erosionSrc.data[((i-1)*processFrame.cols+j+1)]==255||
			erosionSrc.data[(i*processFrame.cols+j-1)]==255||erosionSrc.data[(i*processFrame.cols+j+1)]==255||
			erosionSrc.data[((i+1)*processFrame.cols+j-1)]==255||erosionSrc.data[((i+1)*processFrame.cols+j)]==255||erosionSrc.data[((i+1)*processFrame.cols+j+1)]==255)
			{
				dilationSrc.data[(i*processFrame.cols+j)]=255;
				countR++;
				if(i<minmaxPoints[0][1])
				{	minmaxPoints[0][1]=i;minmaxPoints[0][0]=j;}
				if(i>minmaxPoints[1][1])
				{	minmaxPoints[1][1]=i;minmaxPoints[1][0]=j;}
				if(j<minmaxPoints[2][0])
				{	minmaxPoints[2][0]=j;minmaxPoints[2][1]=i;}
				if(j>minmaxPoints[3][0])
				{	minmaxPoints[3][0]=j;minmaxPoints[3][1]=i;}
			}
			else
				dilationSrc.data[(i*processFrame.cols+j)]=0;
		}
	}*/
	ROS_INFO("countR=%d,countGenR=%d",countR,countGenR);
	if(countR<=0)		//
		return false;
	if(minmaxPoints[0][1]<borderDis||processFrame.rows-minmaxPoints[1][1]<borderDis||minmaxPoints[2][0]<borderDis||processFrame.cols-minmaxPoints[3][0]<borderDis)
		return false;

	cv::circle(processFrame,cv::Point(minmaxPoints[0][0],minmaxPoints[0][1]),5.0,cv::Scalar(255,0,0));
	cv::circle(processFrame,cv::Point(minmaxPoints[1][0],minmaxPoints[1][1]),5.0,cv::Scalar(255,0,0));
	cv::circle(processFrame,cv::Point(minmaxPoints[2][0],minmaxPoints[2][1]),5.0,cv::Scalar(255,0,0));
	cv::circle(processFrame,cv::Point(minmaxPoints[3][0],minmaxPoints[3][1]),5.0,cv::Scalar(255,0,0));
	calcDistance3d(minmaxPoints);

	return true;
}

void PatternRec::flyToTarget()
{
	std::stringstream ss;
	float targetX,
		   targetY,
		   targetZ;
	if(fabs(delta[0]/1000)>maxAxisDis)
	{
		if(delta[0]>0)
			targetX=currentPose.x+maxAxisDis;
		else
			targetX=currentPose.x-maxAxisDis;
	}
	else
		targetX=currentPose.x+delta[0]/1000;

	if((delta[1]/1000)<safeDis)
		targetY=currentPose.y;
	else if((delta[1]/1000)-safeDis>maxAxisDis)
		targetY=currentPose.y+maxAxisDis;
	else
		targetY=currentPose.y+(delta[1]/1000)-safeDis;

	if(fabs(delta[2]/1000)>maxAxisDis)
	{
		if(delta[2]>0)
			targetZ=currentPose.z+maxAxisDis;
		else
			targetZ=currentPose.z-maxAxisDis;
	}
	else
		targetZ=currentPose.z+delta[2]/1000;

	if(fabs(delta[0]/1000)<maxAxisDis&&(delta[1]/1000-safeDis<maxAxisDis)&&fabs(delta[2]/1000)<maxAxisDis)
		finalFlag=true;
	else
		finalFlag=false;

	ss<<"c goto"<<" "<<targetX<<" "<<targetY<<" "<<targetZ<<" "<<0.0;
	std::string tmp=ss.str();
	ROS_INFO("flyto: %s",tmp.c_str());
	nde->publishCommand(ss.str());
}

void PatternRec::flyBackward(short tag)
{
	std::stringstream ss;
	float targetX=currentPose.x,
		   targetY,
		   targetZ=currentPose.z;

	if(tag==0)
		targetY=currentPose.y-backDis;
	else
		targetY=currentPose.y-safeBackDis;

	ss<<"c goto"<<" "<<targetX<<" "<<targetY<<" "<<targetZ<<" "<<0.0;
	std::string tmp=ss.str();
	ROS_INFO("backto: %s",tmp.c_str());
	nde->publishCommand(ss.str());
}
