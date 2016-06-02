#pragma once

#ifndef VARIABLES_H
#define VARIABLES_H

#include <vector>
#include <string>

#define BIG_INTEGER 1000000
#define BIG_FLOAT   1000000

using std::string;


namespace VariableTypes
{
	typedef int Type;
	enum { Unknown = -1, Float = 0, Integer = 1, Boolian = 2 };
};

class Model;

class BaseVar
{
  friend class Model;

  public:

	  BaseVar();
	 
	  int getLocalIndex() const; 
	  int getGlobalIndex() const;
	  VariableTypes::Type getType() const;
	  string getName() const;
	  BaseVar& setName(string name);
	  
	
  protected:

	  int localIndex_;
	  int globalIndex_;
	  bool active_;
	  string name_;
	  VariableTypes::Type type_;
	  
	  void setLocalIndex(int index);
	  void setGlobalIndex(int index);
	  void activate();

};



class FloatVar  : public BaseVar
{
  public:
	  
	  FloatVar();
	  FloatVar(double lowerBound);
	  FloatVar(double lowerBound, double upperBound);
	
	  double getLowerBound() const;
	  double getUpperBound() const;

	  bool hasLowerBound() const;
	  bool hasUpperBound() const;

	  FloatVar* setLowerBound(Model& env, double lb);
	  FloatVar* setUpperBound(Model& env, double ub);
	  
  private:

	  double lowerBound_;
	  double upperBound_;

	  bool hasLowerBound_;
	  bool hasUpperBound_;
};




class IntVar : public BaseVar
{
  public:

	IntVar();
	IntVar(int lowerBound);
	IntVar(int lowerBound, int upperBound);

	int getLowerBound() const;
	int getUpperBound() const;

	IntVar* setLowerBound(Model& env, int lb);
	IntVar* setUpperBound(Model& env, int ub);

private:

	int lowerBound_;
	int upperBound_;
};



class BoolVar : public BaseVar
{
public:

	BoolVar();
	BoolVar(bool sense);
	
	bool isTrue() const;
	
	BoolVar* setSense(Model& env, bool sense);
	
private:

	bool sense_;
};


#endif
