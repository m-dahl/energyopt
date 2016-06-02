#include "RobotData.h"


RobotData::RobotData()
{
	samplingFrequency_ = -1;
	makespan_ = -1;
	timeToleranceMax_ = -1;
	timeToleranceMin_ = -1;
	//nbJoints_ = -1;
	epsilonT_ = -1;
	costScaleFactor_ = -1;
}


RobotData::RobotData(RobotRawData& data)
{
	velocityLimits_     = data.velocityLimits_;
	accelerationLimits_ = data.accelerationLimits_;
	jerkLimits_         = data.jerkLimits_;
	
	starting_points_   = data.starting_points_;
	samplingFrequency_ = data.samplingFrequency_;
	makespan_          = data.makespan_;
	path_              = data.path_;
	
	timeToleranceMax_ = data.timeToleranceMax_;
	timeToleranceMin_ = data.timeToleranceMin_;
	costScaleFactor_  = data.costScaleFactor_  ;
	epsilonT_          = data.epsilonT_;
	weights_          = data.weights_;

	caclMaxVelAccel_();
}


RobotData& RobotData::loadData()
{
	int nbJoints_ = 6; //Read from jason
	
	fstream file;
	file.open("path.dat"); //read the corresponding jason for this robot
	//path = kb-9-9-2015 mov
	
	string line;

	while (getline(file, line))
	{
		if (!line.empty())
		{
			vector<double>  pose = vector<double>(nbJoints_);
			stringstream ss(line);
			for (int j = 0; j<nbJoints_; j++)
			{
				ss >> pose[j];
			}
			path_.push_back(pose);
		}
	}
	file.close();

	

	file.open("startingPoints.dat");
	line.clear();
	while (getline(file, line))
	{
		if (!line.empty())
		{
			double t;
			stringstream ss(line);

			ss >> t;
			starting_points_.push_back(t);
		}
	}
	file.close();


	//-------------

	velocityLimits_.push_back(83.888750);
	velocityLimits_.push_back(74.307833);
	velocityLimits_.push_back(60.084167);
	velocityLimits_.push_back(75.983333);
	velocityLimits_.push_back(78.566250);
	velocityLimits_.push_back(120.321667);

	accelerationLimits_.push_back(474.256944);
	accelerationLimits_.push_back(343.715278);
	accelerationLimits_.push_back(325.902778);
	accelerationLimits_.push_back(484.097222);
	accelerationLimits_.push_back(514.562500);
	accelerationLimits_.push_back(716.250000);

	jerkLimits_.push_back(5434.027778);
	jerkLimits_.push_back(4528.935185);
	jerkLimits_.push_back(2418.981481);
	jerkLimits_.push_back(19911.458333);
	jerkLimits_.push_back(8859.375000);
	jerkLimits_.push_back(6342.013889);

	weights_.push_back(vector<double>(6));
	weights_[0][0] = 24.000000;
	weights_[0][1] = 20.000000;
	weights_[0][2] = 16.000000;
	weights_[0][3] = 12.000000;
	weights_[0][4] = 8.000000;
	weights_[0][5] = 4.000000;
		
	samplingFrequency_ = 0.012;
	timeToleranceMin_ = 0.001;
	timeToleranceMax_ = 0.1;
	costScaleFactor_ = 0.001;
	epsilonT_ = 0.001;

	makespan_ = 23.004;

	return *this;
}


int RobotData::getPathLength() const
{
	return static_cast<int>(path_.size());
}


int RobotData::getNbJoints() const
{
	return static_cast<int>(path_[0].size());
}


double RobotData::getVelocityLimit(int jointId) const
{
	return velocityLimits_[jointId];
}


double RobotData::getAccelerationLimit(int jointId) const
{
	return accelerationLimits_[jointId];
}


double RobotData::getJerkLimit(int jointId) const
{
	return jerkLimits_[jointId];
}


const vector<double>& RobotData::getWeightsOnJoint(int jointId) const
{
	return weights_[jointId]; 
}


double RobotData::getAngle(int sampleId, int jointId) const
{
	return path_[sampleId][jointId];
}


double RobotData::getSamplingFrequency() const
{
	return samplingFrequency_;
}


double RobotData::getScaleFactor() const
{
	return costScaleFactor_;
}


double RobotData::getTimeToleranceMax() const
{
	return timeToleranceMax_;
}


double RobotData::getTimeToleranceMin() const
{
	return timeToleranceMin_;
}


double RobotData::getMakespan() const
{
	return makespan_;
}


double RobotData::getStartingPoint(int sampleId) const
{
	return starting_points_[sampleId];
}

double RobotData::getSeenVelocity(int jointId) const
{
	return maxSeenVeocity_[jointId];
}

double RobotData::getSeenAcceleration(int jointId) const
{
	return maxSeenAcceleration_[jointId];
}

double RobotData::getSeenJerk(int jointId) const
{
	return maxSeenJerk_[jointId];
}

double RobotData::getCurrentJerk(int sampleId, int jointId) const
{
	return currentJerk_[sampleId][jointId];
}

void RobotData::caclMaxVelAccel_()
{
	// Calculate v0
	vector<vector<double>> v0OnJoints;
	for (int j = 0; j < this->getNbJoints(); j++)
	{
		vector<double> v0ThisJoint;
		v0ThisJoint.push_back(0.);
		
		for (int i = 0; i < this->getPathLength() - 1; i++) //Obs! "-1"
		{
			double dt = this->getStartingPoint(i + 1) - this->getStartingPoint(i);
			double v0 = (this->getAngle(i + 1, j) - this->getAngle(i, j)) / dt;

			if (v0 != v0)
			{
				throw "NaN Error: Devision by zero when calculating v0.";
			}
			v0ThisJoint.push_back(v0);
		}
		v0OnJoints.push_back(v0ThisJoint);

	}

	// Calculate a0
	vector<vector<double>> a0OnJoints;
	for (int j = 0; j < this->getNbJoints(); j++)
	{
		vector<double> a0ThisJoint;
		a0ThisJoint.push_back(0.);

		for (int i = 0; i < this->getPathLength() - 1; i++) //Obs! "-1"
		{
			double dt = this->getStartingPoint(i + 1) - this->getStartingPoint(i);
			double a0 = (v0OnJoints[j][i + 1] - v0OnJoints[j][i]) / dt;

			a0ThisJoint.push_back(a0);
			if (a0 != a0)
			{
				throw "NaN Error: Devision by zero when calculating a0.";
			}
		}
		a0OnJoints.push_back(a0ThisJoint);
	}

	// Calculate jerk0
	vector<vector<double>> jerk0OnJoints;
	for (int j = 0; j < this->getNbJoints(); j++)
	{
		vector<double> jerk0ThisJoint;
		jerk0ThisJoint.push_back(0.);

		for (int i = 0; i < this->getPathLength() - 1; i++) //Obs! "-1"
		{
			double dt = this->getStartingPoint(i + 1) - this->getStartingPoint(i);
			double jerk0 = (a0OnJoints[j][i + 1] - a0OnJoints[j][i]) / dt;

			jerk0ThisJoint.push_back(jerk0);
			if (jerk0 != jerk0)
			{
				throw "NaN Error: Devision by zero when calculating a0.";
			}
		}
		jerk0OnJoints.push_back(jerk0ThisJoint);
	}

	for (int i = 0; i < this->getPathLength() - 1; i++) //Obs! "-1"
	{
		vector<double> jerkRow;
		for (int j = 0; j < this->getNbJoints(); j++)
		{
			jerkRow.push_back(jerk0OnJoints[j][i]);
		}
		currentJerk_.push_back(jerkRow);
	
	}

	





	// Get the maximums

	for (int j = 0; j < this->getNbJoints(); j++)
	{
		double maxV = *max_element(v0OnJoints[j].begin(), v0OnJoints[j].end());
		double minV = *min_element(v0OnJoints[j].begin(), v0OnJoints[j].end());

		double maxA = *max_element(a0OnJoints[j].begin(), a0OnJoints[j].end());
		double minA = *min_element(a0OnJoints[j].begin(), a0OnJoints[j].end());

		double maxJerk = *max_element(jerk0OnJoints[j].begin(), jerk0OnJoints[j].end());
		double minJerk = *min_element(jerk0OnJoints[j].begin(), jerk0OnJoints[j].end());


		// max V
		if (abs(minV) >= abs(maxV))
		{
			maxSeenVeocity_.push_back(abs(minV));
		}
		else
		{
			maxSeenVeocity_.push_back(maxV);
		}

		// max A
		if (abs(minA) >= abs(maxA))
		{
			maxSeenAcceleration_.push_back(abs(minA));
		}
		else
		{
			maxSeenAcceleration_.push_back(maxA);
		}

		// max jerk
		if (abs(minJerk) >= abs(maxJerk))
		{
			maxSeenJerk_.push_back(abs(minJerk));
		}
		else
		{
			maxSeenJerk_.push_back(maxJerk);
		}
		
	}

}
