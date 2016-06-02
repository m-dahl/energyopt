#pragma once

#ifndef POST_PROCESS_H
#define POST_PROCESS_H

#include<map>

#include "Model.h"
#include "RobotData.h"
#include "spline.h"

using std::map;


class PostProcess
{
  public:

	  PostProcess();
	 
	  void setupModel(Model* model, vector<RobotData>* RData, vector <vector<FloatVar>>* timeVars);

	  void interpolate();

	  void writeAsci();

	  vector<vector<vector<double>>>& getTrajectories();
	  vector<vector<double>>& getOptimalTimes();

	  
  private:

	  Model* model_;

	  vector<RobotData>* robotDataVec_;
	  vector <vector<FloatVar>>* timeVars_;

	  vector<vector<vector<double>>> interpolatedAnglesAllRobots_;
	  vector<vector<double>> optimalTimeAllRobots_;  
};


#endif


