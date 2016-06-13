/* I3 num oo. 

Genreal robotic cell model using MODALA.
Started 10 Sep 2015 by Sarmad Riazi

*/

#include "IO.h"
#include "Model.h"
#include "RobotData.h"
#include "ZoneData.h"
#include "PostProcess.h"


using std::endl;

void initializeModel(Model& model,vector<RobotData>& R,
                     PostProcess& postProcess, DataFactory& dataFactory)
{
	
	//########################################
	//	#  Initialize robots' information
	//########################################

	int nbRobots = dataFactory.getNbRobots();
	
	for (int i = 0; i < nbRobots; i++)
	{
		R.push_back( RobotData(dataFactory.getRobotData(i)) );
	}

	double makespanMax = 0;
	for (int i = 0; i < nbRobots; i++)
	{
		if (makespanMax <= R[i].getMakespan())
		{
			makespanMax = R[i].getMakespan();
		}
	}

	// Shared zones
	int nbSharedZones = dataFactory.getNbSharedZones();
	vector<SharedZone> szVec;
	for (int i = 0; i < nbSharedZones; i++)
	{
		szVec.push_back(SharedZone(dataFactory.getSharedZone(i)));
	}

	// Preserved zones
	int nbPreservedZones = dataFactory.getNbPreservedZones();
	vector<PreservedZone> pzVec;

	for (int i = 0; i < nbPreservedZones; i++)
	{
		pzVec.push_back(PreservedZone(dataFactory.getPreservedZone(i)));
	}
	

	//########################################
	//	#  Load Data
	//########################################
/*	
	
	int nbRobots = 1; // Get from Jason file
	
	vector<RobotData> R;
	for (int i = 0; i < nbRobots; i++)
	{
		R.push_back(RobotData());
		R[i].loadData();
		//R[i].loadData( send in the corresponding jasonFile )
	}

	double makespanMax = 0;
	for (int i = 0; i < nbRobots; i++)
	{
		if (makespanMax <= R[i].getMakespan())
		{
			makespanMax = R[i].getMakespan();
		}
	}


	// "Don't touch zones"
	int nbPreservedZones = 0; //Jason
	vector<PreservedZone> pzVec;

	for (int i = 0; i < nbPreservedZones; i++)
	{
		int robotId = 0; //Jason
		int startPoint = 5; //Jason
		int endPoint = 100; //Jason

		pzVec.push_back(PreservedZone(robotId, startPoint, endPoint));
	}


	// Shared zones

	int nbSharedZones = 0; //Jason
	vector<SharedZone> szVec;

	for (int i = 0; i < nbSharedZones; i++)
	{
		szVec.push_back(SharedZone());

		szVec[i].addRobot(0, 0, 10, 50); // The second argument: R[0] has the priority.
	}
*/	
	//########################################
	//	#  MODEL & Variables
	//########################################
	

	vector <vector<FloatVar>> t; // Time
	vector <vector<vector<FloatVar>>> v; // Velocity
	vector <vector<vector<FloatVar>>> a; // Acceleration

	// Velocity
	for (int r = 0; r < nbRobots; r++)
	{
		vector<vector<FloatVar>> thisRobotMatrix;
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			vector<FloatVar> thisRobotRow;
			for (int i = 0; i < R[r].getPathLength(); i++)
			{
				thisRobotRow.push_back(FloatVar());

				model.addFloatVar(thisRobotRow[i]);
			}

			thisRobotMatrix.push_back(thisRobotRow);
		}

		v.push_back(thisRobotMatrix);
	}

	// Acceleration
	for (int r = 0; r < nbRobots; r++)
	{
		vector<vector<FloatVar>> thisRobotMatrix;
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			vector<FloatVar> thisRobotRow;
			for (int i = 0; i < R[r].getPathLength(); i++)
			{
				thisRobotRow.push_back(FloatVar());

				model.addFloatVar(thisRobotRow[i]);
			}

			thisRobotMatrix.push_back(thisRobotRow);
		}

		a.push_back(thisRobotMatrix);
	}

	// Time
	for (int r = 0; r < nbRobots; r++)
	{
		vector<FloatVar> thisRobotRow;
		for (int i = 0; i < R[r].getPathLength(); i++)
		{
			thisRobotRow.push_back(FloatVar(0., R[r].getMakespan())); 
			//Time has lowerbound 0, and upper bound of makespan

			model.addFloatVar(thisRobotRow[i]);
		}

		t.push_back(thisRobotRow);
	}

	//########################################
	//	#  Constraitns
	//########################################

	//#---------------------------------------
	//#  Velocity
	//#---------------------------------------

	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
			{
				Expression con;

				con.plus(v[r][j][i + 1]).multiply(t[r][i + 1]);
				con.plus(-1, v[r][j][i + 1]).multiply(t[r][i]);
				con.plus(-1 * R[r].getAngle(i + 1, j));
				con.plus(R[r].getAngle(i, j));

				model.addConstraint(con, "=");
			}
		}
	}

    //#---------------------------------------
	//#  Acceleration
    //#---------------------------------------

	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			for (int i = 0; i < R[r].getPathLength()-1; i++) //Obs! "-1"
			{
				Expression con;
				con.plus(a[r][j][i + 1]).multiply(t[r][i + 1]);
				con.plus(-1, a[r][j][i + 1]).multiply(t[r][i]);
				con.plus(-1, v[r][j][i + 1]);
				con.plus(v[r][j][i]);

				model.addConstraint(con, "=");
			}
		}
	}


	//#---------------------------------------
	//#  Rolling Time
	//#---------------------------------------

	// TicTac Constraint
	// t[r,i+1] - t[r,i] >= epsilon_t;

	for (int r = 0; r < nbRobots; r++)
	{
		for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
		{
			Expression con;
			con.plus(R[r].getTimeToleranceMin());
			con.plus(t[r][i]);
			con.plus(-1, t[r][i + 1]);

			model.addConstraint(con, "<=");
		}
	}

	// TicTac Upperbound
	// t[r,i+1] - t[r,i] <= t_step_length_UB;

	for (int r = 0; r < nbRobots; r++)
	{
		for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
		{
			Expression con;

			con.plus(t[r][i + 1]);
			con.plus(-1, t[r][i]);
			con.plus(-1*R[r].getTimeToleranceMax());
			
			model.addConstraint(con, "<=");
		}
	}
	
	//#---------------------------------------
	//#  Total time / Robot time
	//#---------------------------------------

	// t[r, pathLength[r] ] <= makespan_total;
		
	for (int r = 0; r < nbRobots; r++)
	{
		Expression con;

		con.plus(t[r][R[r].getPathLength()-1]);
		con.plus(-1 * makespanMax);

		model.addConstraint(con,"<=");
	}


	//#---------------------------------------
	//#  Jerk Control
	//#---------------------------------------

	// "Scientifically correct" version
	// Up:  a[r,j,i+1]-a[r,j,i] <= jerkLimit[j] * (t[r,i+1]-t[r,i]);
	//Down: a[r,j,i+1]-a[r,j,i] >= - jerkLimit[j] * (t[r,i+1]-t[r,i]);
	int nbjerkConst = 0;
	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
			{
				double jLim = 0;
				jLim = R[r].getCurrentJerk(i, j);
				if(true)//(jLim>=0 & jLim >= 0.7*R[r].getJerkLimit(j)) || (jLim <= 0 & jLim <= -0.7*R[r].getJerkLimit(j)))
				
				{

					Expression cUp, cDown;

					cUp.plus(a[r][j][i + 1]).plus(-1, a[r][j][i]);
					cUp.plus(-1 * R[r].getJerkLimit(j), t[r][i + 1]);
					cUp.plus(R[r].getJerkLimit(j), t[r][i]);

					cDown.plus(-1 * R[r].getJerkLimit(j), t[r][i + 1]);
					cDown.plus(R[r].getJerkLimit(j), t[r][i]);
					cDown.plus(-1, a[r][j][i + 1]).plus(a[r][j][i]);

					model.addConstraint(cUp, "<=");
					model.addConstraint(cDown, "<=");

					nbjerkConst++;
				}
			}
		}
	}

	
	
	// "Computationally light" version"
	// Up:   a[r,j,i+1]-a[r,j,i] <= jerkLimit[j] * sampleFactor;
	// Down: a[r,j,i+1]-a[r,j,i] >= - jerkLimit[j] * sampleFactor;

	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
			{
				double jLim = 0;
				jLim = R[r].getCurrentJerk(i, j);
				if (true)//(jLim >= 0 & jLim >= 0.7*R[r].getJerkLimit(j)) || (jLim <= 0 & jLim <= -0.7*R[r].getJerkLimit(j)))
				{

					Expression cUp, cDown;

					cUp.plus(a[r][j][i + 1]).plus(-1, a[r][j][i]);
					cUp.plus(-1 * R[r].getJerkLimit(j) * R[r].getSamplingFrequency());

					cDown.plus(-1 * R[r].getJerkLimit(j) * R[r].getSamplingFrequency());
					cDown.plus(-1, a[r][j][i + 1]).plus(a[r][j][i]);

					model.addConstraint(cUp, "<=");
					model.addConstraint(cDown, "<=");
					nbjerkConst++;
				}
			}
		}
	}

	std::cout<<std::endl<< nbjerkConst++;

	//#---------------------------------------
	//#  Physical Limits
	//#---------------------------------------

	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			for (int i = 1; i < R[r].getPathLength()-1 ; i++) //
			{

				v[r][j][i].setLowerBound(model, -1 * R[r].getVelocityLimit(j))->setUpperBound(model, R[r].getVelocityLimit(j));

				a[r][j][i].setLowerBound(model, -1 * R[r].getAccelerationLimit(j))->setUpperBound(model, R[r].getAccelerationLimit(j));
			}
		}
	}


	//#---------------------------------------
	//#  Initial/final conditions 
	//#---------------------------------------

	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			int lastElementId = R[r].getPathLength() - 1;
			
			v[r][j][0].setLowerBound(model, 0.)->setUpperBound(model, 0.);

			a[r][j][0].setLowerBound(model, 0.)->setUpperBound(model, 0.);

			v[r][j][lastElementId].setLowerBound(model, 0.)->
				setUpperBound(model, 0.);

			a[r][j][lastElementId].setLowerBound(model, 0.)->
				setUpperBound(model, 0.);
		}
	}


	//#---------------------------------------
	//#  Shared zone constraints
	//#---------------------------------------

	for (int k = 0; k < nbSharedZones; k++)
	{
		int nbAccessQuary = szVec[k].getNbRobots();

		for (int i = 0; i < nbAccessQuary-1; i++) //NB! -1
		{
			int whoGoesFirst  = szVec[k].getPrecedenceVec()[i];
			int whoGoesSecond = szVec[k].getPrecedenceVec()[i + 1];

			int firstOneExitIndex = szVec[k].getZoneExitPoint(whoGoesFirst);
			int secondOneEnterIndex = szVec[k].getZoneEnterancePoint(whoGoesSecond);

			// Ex.  t1 goes first: (t1 + epsilon <= t2)
			Expression con;
			con.plus(t[whoGoesFirst][firstOneExitIndex]);
			con.plus(R[whoGoesFirst].getTimeToleranceMin());
			con.plus(-1, t[whoGoesSecond][secondOneEnterIndex] );
						
			model.addConstraint(con, "<=");
		} 
	}


	//#---------------------------------------
	//#  "Don't Touch" constraints
	//#---------------------------------------

	for (int k = 0; k < nbPreservedZones; k++)
	{
		int r = pzVec[k].getRobotId();
		int startPoint = pzVec[k].getStartPoint();
		int endPoint = pzVec[k].getEndPoint();

		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			for (int i = startPoint; i < endPoint-1; i++) //Obs! "-1"
			{
				// v_{i+1} = (p{i+1}-p{i})/samplingRate
				Expression con;
				
				con.plus(v[r][j][i + 1]).multiply(R[r].getSamplingFrequency());
				con.plus(-1 * R[r].getAngle(i + 1, j)).plus(R[r].getAngle(i, j));
				
				model.addConstraint(con, "=");
			}
		}	
	}


	//########################################
	//	#  Objective Function
	//########################################

	double scaleFactor = R[0].getScaleFactor();
	Expression obj;


	for (int r = 0; r < nbRobots; r++)
	{
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			double seenV = R[r].getSeenVelocity(j);
			double seenA = R[r].getSeenAcceleration(j);
			double seenJerk = R[r].getSeenJerk(j);

			if (seenV <= 1) seenV = 1;
			if (seenA <= 1) seenA = 1;
			if (seenJerk <= 1) seenJerk = 1;


			for (int i = 0; i < R[r].getPathLength()-1; i++) //
			{
				//	obj.plus(0);			
		/* /		
				obj.plus(a[r][j][i+1]).multiply(a[r][j][i+1]).
					multiply(v[r][j][i+1]).multiply(v[r][j][i+1]).
					multiply(t[r][i+1]).
					multiply(1 / pow(seenV, 2)).
					multiply(1 / pow(seenA, 2)).
					multiply(R[r].getWeightsOnJoint(0)[j]);

				obj.plus(-1,a[r][j][i+1]).multiply(a[r][j][i+1]).
					multiply(v[r][j][i+1]).multiply(v[r][j][i+1]).
					multiply(t[r][i]).
					multiply(1 / pow(seenV, 2)).
					multiply(1 / pow(seenA, 2)).
					multiply(R[r].getWeightsOnJoint(0)[j]); 
		*/
	//  Jerk terms


		/*		double multiplier = pow(1 / R[r].getSamplingFrequency(), 2) * 5*
					pow(1/ seenJerk,2);

				obj.plus(multiplier, a[r][j][i + 1], 2).multiply(t[r][i + 1]);
				obj.plus(multiplier, a[r][j][i], 2).multiply(t[r][i + 1]);
				obj.plus(multiplier, a[r][j][i + 1]).multiply(-2,a[r][j][i]).
					multiply(t[r][i + 1]);

				multiplier = -1 * multiplier;
				obj.plus(multiplier, a[r][j][i + 1], 2).multiply(t[r][i]);
				obj.plus(multiplier, a[r][j][i], 2).multiply(t[r][i]);
				obj.plus(multiplier, a[r][j][i + 1]).multiply(-2, a[r][j][i]).
					multiply(t[r][i]);
		*/		


		//acceleration		
				obj.plus(a[r][j][i]).multiply(a[r][j][i]).
					multiply(1 / pow(seenA, 2)).
					multiply(R[r].getWeightsOnJoint(0)[j]).
					multiply(R[r].getSamplingFrequency());
					

			// Acceleration^2
			/*for (int r = 0; r < nbRobots; r++)
			{
			for (int j = 0; j < R[r].getNbJoints(); j++)
			{
			for (int i = 0; i < R[r].getPathLength(); i++) //
			{
			//	obj.plus(0);
			obj.plus(scaleFactor, a[r][j][i]).multiply(scaleFactor, a[r][j][i]);
			obj.multiply(R[r].getWeightsOnJoint(0)[j]);
			}
			}
			}*/
	
			}
		}
	}

	model.addObjective(obj);


	//########################################
	//	#  Starting Points
	//########################################

	//Starting points.

	//t0
	for (int r = 0; r < nbRobots; r++)
	{	
		for (int i = 0; i < R[r].getPathLength(); i++) 
		{
			model.addStartingPoint(t[r][i], R[r].getStartingPoint(i) ) ;
		}
	}

	
	for (int r = 0; r < nbRobots; r++)
	{
		// Calculate v0
		vector<vector<double>> v0OnJoints;
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			vector<double> v0ThisJoint;
			v0ThisJoint.push_back(0.);

			for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
			{
				double dt = R[r].getStartingPoint(i + 1) - R[r].getStartingPoint(i);
				double v0 = (R[r].getAngle(i+1,j) - R[r].getAngle(i,j)) / dt;

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
		for (int j = 0; j < R[r].getNbJoints(); j++)
		{
			vector<double> a0ThisJoint;
			a0ThisJoint.push_back(0.);

			for (int i = 0; i < R[r].getPathLength() - 1; i++) //Obs! "-1"
			{
				double dt = R[r].getStartingPoint(i + 1) - R[r].getStartingPoint(i);
				double a0 = (v0OnJoints[j][i + 1] - v0OnJoints[j][i]) / dt;

				a0ThisJoint.push_back(a0);
				if (a0 != a0)
				{
					throw "NaN Error: Devision by zero when calculating a0.";
				}
			}
			a0OnJoints.push_back(a0ThisJoint);
		}

		//Add v0 and a0 to starting poitns.
		
		//for (int j = 0; j < R[r].getNbJoints(); j++)
		//{
		//	for (int i = 0; i < R[r].getPathLength() ; i++) 
		//	{
		//		model.addStartingPoint(v[r][j][i], v0OnJoints[j][i]);
		//		model.addStartingPoint(a[r][j][i], a0OnJoints[j][i]);	
		//	}	
		//}
		
	}

	
	//########################################
	//	#  Solve and post process
	//########################################
	
	model.solveByIpopt();
	
	postProcess.setupModel(&model, &R, &t);
	postProcess.interpolate();

}
	
	
