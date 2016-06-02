#pragma once

#ifndef INPUT_OUTPUT_H
#define INPUT_OUTPUT_H

#include <iostream> //for reading text
#include <fstream>  //for reading text
#include <string>   //for reading text
#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"


#include "RobotData.h"
#include "ZoneData.h"
#include "PostProcess.h"

using std::fstream;
using std::string;
using std::stringstream;

using namespace rapidjson;

class DataFactory
{
  public:

	  DataFactory(string& jsonString); //for now this iactually reads form a text file
	  DataFactory(char* fileName);

	  void parseRobotData();
	  void parseSharedZoneData();
	  void parsePreservedZoneData();
	  

	  int getNbRobots() const;
	  int getNbSharedZones() const;
	  int getNbPreservedZones() const;
	  
	  RobotRawData& getRobotData(int robotIndex);
	  SharedZoneRaw& getSharedZone(int zoneIndex);
	  PreservedZoneRaw& getPreservedZone(int zoneIndex);
	  
 private:

	  vector<RobotRawData> robotVec_;
	  vector<SharedZoneRaw> sharedZoneVec_;
	  vector<PreservedZoneRaw> preservedZoneVec_;
	  
	  Document document_;
};


class ResultFactory
{
  public:

	  ResultFactory(PostProcess& p);

	  string& getJsonString();
	  void makeString();

  private:
	  
	  vector<vector<vector<double>>> trajectories_;
	  vector<vector<double>> optimizedTimes_;
	  string jsonString_;
};





#endif