#include "IO.h"

#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <cstdio>

using namespace std;


DataFactory::DataFactory(string& jsonString)
{
	if (document_.Parse(jsonString.c_str()).HasParseError())
	{
		cout << "Something went wrong when parsing the string.";
	}

	parseRobotData(); //These parser functions will go in the constructor.
	parseSharedZoneData();
	parsePreservedZoneData();
}


DataFactory::DataFactory(char* filename)
{
	string fileNameString(filename);
	
	fstream file;
	file.open(fileNameString);

	string infoString;
	string line;

	while (getline(file, line))
	{
	if (!line.empty())
	{
	infoString.append(line);
	}
	}

	line.clear();

	if (document_.Parse(infoString.c_str()).HasParseError())
	{
	cout << "Something went wrong when parsing the string.";
	}

	parseRobotData(); //These parser functions will go in the constructor.
	parseSharedZoneData();
	parsePreservedZoneData();
}


void DataFactory::parseRobotData()
{
	SizeType nbRobots = document_["robots"].Size();

	for (SizeType r = 0; r < nbRobots; r++)
	{
		RobotRawData rawData;

		// Data members in rawData structure
		vector<double> velocityLimits;
		vector<double> accelerationLimits;
		vector<double> jerkLimits;
		vector<double> starting_points;
		vector<vector<double>> weights;
		vector<vector<double>> path;

		// Parse parameters' data.
		rawData.epsilonT_ = document_["robots"][r]["epsilonT"].GetDouble();
		rawData.makespan_ = document_["robots"][r]["makespan"].GetDouble();
		rawData.samplingFrequency_ = document_["robots"][r]["samplingRate"].GetDouble();
		rawData.costScaleFactor_ = document_["robots"][r]["costScaleFactor"].GetDouble();
		rawData.timeToleranceMax_ = document_["robots"][r]["timeToleranceMax"].GetDouble();
		rawData.timeToleranceMin_ = document_["robots"][r]["timeToleranceMin"].GetDouble();

		// Parse limits and starting points.
		SizeType nbJoints = document_["robots"][r]["velocityLimit"].Size();

		for (SizeType j = 0; j < nbJoints; j++)
		{
			velocityLimits.push_back(document_["robots"][r]["velocityLimit"][j].GetDouble());
			accelerationLimits.push_back(document_["robots"][r]["accelerationLimit"][j].GetDouble());
			jerkLimits.push_back(document_["robots"][r]["jerkLimit"][j].GetDouble());
		}

		rawData.velocityLimits_ = velocityLimits;
		rawData.accelerationLimits_ = accelerationLimits;
		rawData.jerkLimits_ = jerkLimits;
		
		// Parse path and starting points.
		SizeType nbSamples = document_["robots"][r]["trajectory"].Size();
		for (SizeType i = 0; i < nbSamples; i++)
		{
			starting_points.push_back(document_["robots"][r]["time"][i].GetDouble());
			
			vector<double> pose;
			for (SizeType j = 0; j < nbJoints; j++)
			{
				pose.push_back(document_["robots"][r]["trajectory"][i][j].GetDouble());
			}
			path.push_back(pose);
		}
		rawData.starting_points_ = starting_points;
		rawData.path_ = path;


		// Parse weights on terms in cost function.
		SizeType nbWeightTypes = document_["robots"][r]["weights"].Size();

		// I know that the first term is weight on the acceleration. 
		for (SizeType k = 0; k < nbWeightTypes; k++)
		{
			vector<double> firstTerm;

			for (SizeType j = 0; j < nbJoints; j++)
			{
				firstTerm.push_back(document_["robots"][r]["weights"][k][j].GetDouble());
			}
			weights.push_back(firstTerm);
		}
		rawData.weights_ = weights;

		this->robotVec_.push_back(rawData);
	}
}


void DataFactory::parseSharedZoneData()
{
	SizeType nbSharedZones = document_["sharedZones"].Size();

	for (SizeType i = 0; i < nbSharedZones; i++)
	{
		SharedZoneRaw rawData;
		vector<int> precedenceVec; // 'member of rawData';
		map<int, pair<int, int> > zoneBorderMap; // 'Ditto', <robotID, <enter, exit> >

		SizeType nbRobotsInZone = document_["sharedZones"][i].Size();
		for (SizeType r = 0; r < nbRobotsInZone; r++)
		{
			int robotId  = document_["sharedZones"][i][r]["robot"].GetInt();
			int entersAt = document_["sharedZones"][i][r]["entersAtSample"].GetInt();
			int exitsAt  = document_["sharedZones"][i][r]["exitsAtSample"].GetInt();

			precedenceVec.push_back(robotId);
			zoneBorderMap.insert(pair<int, pair<int, int>>(robotId,
				pair<int, int>(entersAt, exitsAt)));
		}

		rawData.precedenceVec_ = precedenceVec;
		rawData.zoneBorderMap_ = zoneBorderMap;

		this->sharedZoneVec_.push_back(rawData);
	}
}


void DataFactory::parsePreservedZoneData()
{
	SizeType nbPreservedZones = document_["preservedZones"].Size();

	for (SizeType i = 0; i < nbPreservedZones; i++)
	{
		PreservedZoneRaw rawData;
		
		rawData.robotId_    = document_["preservedZones"][i]["robot"].GetInt();
		rawData.startPoint_ = document_["preservedZones"][i]["entersAtSample"].GetInt();
		rawData.endPoint_   = document_["preservedZones"][i]["exitsAtSample"].GetInt();

		this->preservedZoneVec_.push_back(rawData);
	}
}


int DataFactory::getNbRobots() const
{
	return static_cast<int>(robotVec_.size());
}


int DataFactory::getNbSharedZones() const
{
	return static_cast<int>(sharedZoneVec_.size());
}


int DataFactory::getNbPreservedZones() const
{
	return static_cast<int>(preservedZoneVec_.size());
}


RobotRawData& DataFactory::getRobotData(int robotIndex)
{
	// Sanity check.
	if (robotIndex < this->getNbRobots())
	{
		return robotVec_[robotIndex];
	}
	else throw "Robot index is out of bound.";
}


SharedZoneRaw& DataFactory::getSharedZone(int zoneIndex)
{
	// Sanity check.
	if (zoneIndex < this->getNbSharedZones())
	{
		return sharedZoneVec_[zoneIndex];
	}
	else throw "Shared zone index is out of bound.";
}


PreservedZoneRaw& DataFactory::getPreservedZone(int zoneIndex)
{
	// Sanity check.
	if (zoneIndex < this->getNbPreservedZones())
	{
		return preservedZoneVec_[zoneIndex];
	}
	else throw "Preserved zone index is out of bound.";
}


ResultFactory::ResultFactory(PostProcess& p)
{
	trajectories_   = p.getTrajectories();
	optimizedTimes_ = p.getOptimalTimes();;
}


string& ResultFactory::getJsonString()
{
	
	return jsonString_;

}


void ResultFactory::makeString()
{
	int nbRobots = static_cast<int>(optimizedTimes_.size());
	
	char c[128];
	
	jsonString_.append("{ \"result\" : [ \n");
	for (int r = 0; r < nbRobots; r++)
	{	
		int nbSamples = static_cast<int>(this->optimizedTimes_[r].size());
		int nbJoints =  static_cast<int>(this->trajectories_[r].size());
	
		//Start of object for each robot
		jsonString_.append("{");
		
		  // Start of time array 
		  jsonString_.append(" \"optimizedTime\": [ \n");

		    // The value
		    for (int i = 0; i < nbSamples; i++)
		    {
				sprintf(c, "%f", optimizedTimes_[r][i] ); 
				jsonString_.append(c);

				if (i < nbSamples - 1)
				{
					jsonString_.append(", \n");
				}
				else
				{
					jsonString_.append("\n");
				}
		    }
		   
		  // End of time array
		  jsonString_.append("], \n");


		  // Start of trajectory array 
		  jsonString_.append(" \"interpolatedTrajectory\": [ \n");

		    for (int i = 0; i < nbSamples; i++)
		    {
				jsonString_.append("[ \n");

				for (int j = 0; j < nbJoints; j++)
				{
					sprintf(c, "%f", trajectories_[r][j][i]);
					jsonString_.append(c);

					if (j < nbJoints - 1)
					{
						jsonString_.append(", \n");
					}
					else
					{
						jsonString_.append("\n");
					}

				}

				// End of pose
				if (i < nbSamples - 1)
				{
					jsonString_.append("], \n");
				}
				else
				{
					jsonString_.append("] \n");
				}

		    }


		  // End of trajectory array
		  jsonString_.append("] \n");

		// End of object for each robot
		if (r == nbRobots - 1)
		{
			jsonString_.append("}");
		}
		else
		{
			jsonString_.append("},");
		} 
	}
	// End of Jason
	jsonString_.append("\t ] \n }");

	// Test the created json using the following lines:

		//Document document;
		//document.Parse(jsonString_.c_str());
		//StringBuffer sb;
		//PrettyWriter<StringBuffer> writer(sb);
		//document.Accept(writer);    // Accept() traverses the DOM and generates Handler events.
		//puts(sb.GetString());
}
