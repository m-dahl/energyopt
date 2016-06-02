#include "ZoneData.h"

SharedZone::SharedZone()
{

}


SharedZone::SharedZone(SharedZoneRaw& data)
{
	precedenceVec_ = data.precedenceVec_;
	zoneBorderMap_ = data.zoneBorderMap_;
}

SharedZone& SharedZone::addRobot(int robotId, int priority, int enterIndex, int exitIndex)
{
	
	zoneBorderMap_.insert( pair<int, pair<int,int>>(
		                 robotId, pair<int,int>(enterIndex, exitIndex) ) );

	precedenceVec_.push_back(priority);
	
	return *this;
}


int SharedZone::getNbRobots() const
{
	return static_cast<int> (precedenceVec_.size());
}


const vector<int>& SharedZone::getPrecedenceVec() const
{
	return precedenceVec_;
}


int SharedZone::getZoneEnterancePoint(int robotId) const
{
	return ( zoneBorderMap_.find(robotId) )-> second.first;
}


int SharedZone::getZoneExitPoint(int robotId) const
{
	return (zoneBorderMap_.find(robotId))->second.second;
}



PreservedZone::PreservedZone(int robotId, int startPoint, int endPoint)
{
	robotId_ = robotId;
	startPoint_ = startPoint;
	endPoint_ = endPoint;
}


PreservedZone::PreservedZone(PreservedZoneRaw& data)
{
	robotId_    = data.robotId_;
	startPoint_ = data.startPoint_;
	endPoint_   = data.endPoint_;
}


int PreservedZone::getRobotId() const
{
	return robotId_;
}


int PreservedZone::getStartPoint() const
{
	return startPoint_;
}

int PreservedZone::getEndPoint() const
{
	return endPoint_;
}
