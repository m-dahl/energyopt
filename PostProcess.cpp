#include "PostProcess.h"

PostProcess::PostProcess()
{
	model_ = 0;
	robotDataVec_ = 0;
	timeVars_ = 0;
}


void PostProcess::setupModel(Model* model, vector<RobotData>* robotDataVec,
	vector <vector<FloatVar>>* timeVars)
{
	model_ = model;
	robotDataVec_ = robotDataVec;
	timeVars_ = timeVars;
}

void PostProcess::interpolate()
{
	// Setup t0 and P0;
	int nbRobots = static_cast<int> (robotDataVec_->size());
	
	vector<vector<double>> initialTimeAllRobots;
	vector<vector<double>> standardTimeAllRobots;


	// Get initial times; 
	// Create standard Time.
	for (int r = 0; r < nbRobots; r++)
	{
		vector<double> initialTimeOneRobot;
		vector<double> standardTimeOneRobot;
		
		int nbTimeVar = static_cast<int>(timeVars_->operator[](r).size());
		
		double value = -1;
		int index = -1;
	
		for (int i = 0; i < nbTimeVar; i++)
		{
			index = timeVars_->operator[](r).operator[](i).getLocalIndex();
			value = (model_->getStartingPoints().find(index))->second;

			initialTimeOneRobot.push_back(value);
		}
		initialTimeAllRobots.push_back(initialTimeOneRobot);

		// Standard time vector with equidstance samples (0.012)
		double finalTime = model_->getSol(timeVars_->operator[](r)[nbTimeVar - 1]);
		double currentTime = 0;

		while (currentTime < finalTime)
		{
			standardTimeOneRobot.push_back(currentTime);
			currentTime += robotDataVec_->operator[](r).getSamplingFrequency();
		}
		standardTimeAllRobots.push_back(standardTimeOneRobot);
	}


	// Get Optimal Times.
	for (int r = 0; r < nbRobots; r++)
	{
		vector<double> optimalTimeOneRobot;
		for (int i = 0; i < robotDataVec_->operator[](r).getPathLength(); i++)
		{
			optimalTimeOneRobot.push_back(model_->getSol(timeVars_->operator[](r)[i]));
		}
		optimalTimeAllRobots_.push_back(optimalTimeOneRobot);
	}


	// Interpolate using initial recorder times
/*	for (int r = 0; r < nbRobots; r++)
	{
		vector<vector<double>> allInterpolatedJointAngles;
		for (int j = 0; j < robotDataVec_->operator[](r).getNbJoints(); j++)
		{
			vector<double> jointAngles;
			for (int i = 0; i < robotDataVec_->operator[](r).getPathLength(); i++)
			{
				jointAngles.push_back(robotDataVec_->operator[](r).getAngle(i, j));
			}
			
			tk::spline s;
			s.set_points(optimalTimeAllRobots_[r], jointAngles );

			int newNbSamples = static_cast<int>(optimalTimeAllRobots_[r].size());
			vector<double> oneInterpolatedJointAngles;
			for (int i = 0; i < newNbSamples; i++)
			{
				oneInterpolatedJointAngles.push_back(s(initialTimeAllRobots[r][i]));
			}

			allInterpolatedJointAngles.push_back(oneInterpolatedJointAngles);
		}
		
		interpolatedAnglesAllRobots_.push_back(allInterpolatedJointAngles);
	}
*/
	// Interpolate using standard time (0.012)
	for (int r = 0; r < nbRobots; r++)
	{
		vector<vector<double>> allInterpolatedJointAngles;
		for (int j = 0; j < robotDataVec_->operator[](r).getNbJoints(); j++)
		{
			vector<double> jointAngles;
			for (int i = 0; i < robotDataVec_->operator[](r).getPathLength(); i++)
			{
				jointAngles.push_back(robotDataVec_->operator[](r).getAngle(i, j));
			}

			tk::spline s;
			s.set_points(optimalTimeAllRobots_[r], jointAngles);

			
			int newNbSamples = static_cast<int>(standardTimeAllRobots[r].size());
			vector<double> oneInterpolatedJointAngles;
			for (int i = 0; i < newNbSamples; i++)
			{
				oneInterpolatedJointAngles.push_back(s(standardTimeAllRobots[r][i]));
			}

			allInterpolatedJointAngles.push_back(oneInterpolatedJointAngles);
		}

		interpolatedAnglesAllRobots_.push_back(allInterpolatedJointAngles);
	}


	// Tutorial
	// Post process

	/*std::vector<double> X(5), Y(5);
	X[0] = 0.1; X[1] = 0.4; X[2] = 1.2; X[3] = 1.8; X[4] = 2.0;
	Y[0] = 0.1; Y[1] = 0.7; Y[2] = 0.6; Y[3] = 1.1; Y[4] = 0.9;

	tk::spline s;
	s.set_points(X, Y);    // currently it is required that X is already sorted

	double x = 1.5;

	printf("spline at %f is %f\n", x, s(x));
	*/	
}


void PostProcess::writeAsci()
{
	int nbRobots = static_cast<int> (robotDataVec_->size());
	
	for (int r = 0; r < nbRobots; r++)
	{
		std::ofstream file;
		char name[128];
		sprintf(name, "sol_r_%d.txt", r);
		file.open(name);

		for (int i = 0; i < robotDataVec_->operator[](r).getPathLength(); i++)
		{
			for (int j = 0; j < robotDataVec_->operator[](r).getNbJoints(); j++)
			{
				file << interpolatedAnglesAllRobots_[r][j][i]<<" ";
			}
			file << std::endl;
		}
		file.close();
	}

}


vector<vector<vector<double>>>& PostProcess::getTrajectories()
{
	return interpolatedAnglesAllRobots_;
}


vector<vector<double>>& PostProcess::getOptimalTimes()
{
	return optimalTimeAllRobots_;
}
