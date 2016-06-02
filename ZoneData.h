#pragma once

#include <vector>
#include <map>

using std::vector;
using std::pair;
using std::map;


// Class definition for "Shared" zones and "Don't Touch" zones.

struct PreservedZoneRaw
{
	int robotId_;
	int startPoint_;
	int endPoint_;
};


struct SharedZoneRaw
{
	vector<int> precedenceVec_;
	map<int, pair<int, int> > zoneBorderMap_;
};


class PreservedZone
{
  public:

	  PreservedZone(int robotId, int startPoint, int endPoint);
	  PreservedZone(PreservedZoneRaw& data);

	  int getRobotId() const;
	  int getStartPoint() const;
	  int getEndPoint() const;
	  
  private:

	  int robotId_;
	  int startPoint_;
	  int endPoint_;
};


class SharedZone
{
  public:

	  SharedZone();
	  SharedZone(SharedZoneRaw& data);
	  
	  SharedZone& addRobot(int robotId, int priority, int enterIndex, int exitIndex);
 		  
	  int getNbRobots() const;
	  const vector<int>&  getPrecedenceVec() const;
	  int getZoneEnterancePoint(int robotId) const;
	  int getZoneExitPoint(int robotId) const;

  private:

	  vector<int> precedenceVec_;

	  map<int, pair<int,int> > zoneBorderMap_; // <robotID, <enter, exit> >

};

