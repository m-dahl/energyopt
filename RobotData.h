#pragma once
#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include<vector>
#include <iostream> //for reading text
#include <fstream>  //for reading text
#include <string>   //for reading text
#include <sstream>
#include <algorithm>

using std::vector;
using std::fstream; 
using std::string;
using std::stringstream;
using std::max;
using std::abs;
using std::min;


struct RobotRawData
{
	//----------------------------------------------- [ Robot's info ]

	vector<double> velocityLimits_;
	vector<double> accelerationLimits_;
	vector<double> jerkLimits_;
	vector<vector<double>> weights_; 

	//----------------------------------------------- [Path's info ]

	vector<vector<double>> path_; //<sampleId, joints angles>

	vector<double> starting_points_;

	double samplingFrequency_;
	double costScaleFactor_;

	double makespan_;

	double timeToleranceMax_;
	double timeToleranceMin_;
	double epsilonT_;

};

class RobotData
{
  public:
	  
	  RobotData(); 
	  RobotData(RobotRawData& data);
	  
	  RobotData& loadData(); // For reading from a text file

	  int getPathLength() const;
	  int getNbJoints() const;

	//  int getZoneExitSampleNo() const;
	//  int getZoneEnterSampleNo() const;

	  double getVelocityLimit(int jointId) const;
	  double getAccelerationLimit(int jointId) const;
	  double getJerkLimit(int jointId) const;

	  const vector<double>& getWeightsOnJoint(int jointId) const;
	  double getAngle(int sampleId, int jointId) const;

	  double getSamplingFrequency() const;
	  double getScaleFactor() const;
	  double getTimeToleranceMax() const;
	  double getTimeToleranceMin() const;
	  double getMakespan() const;

	  double getStartingPoint(int sampleId) const;
	  double getSeenVelocity(int jointId) const;
	  double getSeenAcceleration(int jointId) const;
	  double getSeenJerk(int jointId) const;
	  double getCurrentJerk(int sampleId,int jointId) const;

  private:
	  
	  //----------------------------------------------- [ Robot's info ]
	  vector<double> velocityLimits_;	
	  vector<double> accelerationLimits_;
	  vector<double> jerkLimits_;
	  vector<vector<double>> currentJerk_;
	  vector<vector<double>> weights_;	
	 
	  //----------------------------------------------- [Path's info ]
	  vector<vector<double>> path_; //<sampleId, joints angles>
	  
	  vector<double> starting_points_; 
	  vector<double> maxSeenVeocity_;
	  vector<double> maxSeenAcceleration_;
	  vector<double> maxSeenJerk_;
	  
	  double samplingFrequency_;
	  double costScaleFactor_;
	  
	  double makespan_;
	  
	  double timeToleranceMax_;
	  double timeToleranceMin_;
	  double epsilonT_;

	  void caclMaxVelAccel_();
};


#endif