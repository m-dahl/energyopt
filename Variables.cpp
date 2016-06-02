#include "Variables.h"
#include "Model.h"

//----------------------------------------------- [ Variables ]
BaseVar::BaseVar()
{
	localIndex_  = -1;
	globalIndex_ = -1;
	active_ = false;
	type_ = VariableTypes::Unknown;
	name_ = "baseVar";
}


int BaseVar::getLocalIndex() const
{
	return localIndex_;	
}


int BaseVar::getGlobalIndex() const
{
	return globalIndex_;
}


void BaseVar::setLocalIndex(int index)
{
	localIndex_ = index;
}


void BaseVar::setGlobalIndex(int index)
{
	globalIndex_ = index;
}


void BaseVar::activate()
{
	active_ = true;
}


VariableTypes::Type BaseVar::getType() const
{
	return type_;
}


string BaseVar::getName() const
{
	return name_;
}


BaseVar& BaseVar::setName(string name)
{
	name_ = name;

	return *this;
}


//----------------------------------------------- [ FloatVar ]
FloatVar::FloatVar()
{
	lowerBound_ = -BIG_FLOAT;
	upperBound_ = BIG_FLOAT; 
	hasLowerBound_ = false;
	hasUpperBound_ = false;
	type_ = VariableTypes::Float;
	name_ = "y";
}


FloatVar::FloatVar (double lowerBound)
{
	lowerBound_ = lowerBound;
	upperBound_ = BIG_FLOAT;
	hasLowerBound_ = true;
	hasUpperBound_ = false;
	type_ = VariableTypes::Float;
	name_ = "y";
}


FloatVar::FloatVar (double lowerBound, double upperBound)
{
	lowerBound_ = lowerBound;
	upperBound_ = upperBound;
	hasLowerBound_ = true;
	hasUpperBound_ = true;
	type_ = VariableTypes::Float;
	name_ = "y";
}


double FloatVar::getLowerBound() const
{
	return lowerBound_;
}


double FloatVar::getUpperBound() const
{
	return upperBound_;
}


bool FloatVar::hasLowerBound() const
{
	return hasLowerBound_;
}


bool FloatVar::hasUpperBound() const
{
	return hasUpperBound_;
}


FloatVar* FloatVar::setLowerBound (Model& model, double lb)
{
	// If a copy is available in the Model, update it.
	if (active_)
	{
		model.floatVarVec_[this->localIndex_].lowerBound_ = lb;
		model.floatVarVec_[this->localIndex_].hasLowerBound_ = true;
	}

	lowerBound_ = lb;
	hasLowerBound_ = true;
	
	return this;
}


FloatVar* FloatVar::setUpperBound (Model& env, double ub)
{
	// If a copy is available in the Model, update it.
	if (active_)
	{
		env.floatVarVec_[this->localIndex_].upperBound_ = ub;
		env.floatVarVec_[this->localIndex_].hasUpperBound_ = true;
	}

	upperBound_ = ub;
	
	return this;
}



//----------------------------------------------- [ IntVar ]
IntVar::IntVar()
{
	lowerBound_ = -BIG_INTEGER;
	upperBound_ = BIG_INTEGER;
	type_ = VariableTypes::Integer;
	name_ = "z";
}


IntVar::IntVar(int lowerBound)
{
	lowerBound_ = lowerBound;
	upperBound_ = BIG_INTEGER;
	type_ = VariableTypes::Integer;
	name_ = "z";
}


IntVar::IntVar(int lowerBound, int upperBound)
{
	lowerBound_ = lowerBound;
	upperBound_ = upperBound;
	type_ = VariableTypes::Integer;
	name_ = "z";
}


int IntVar::getLowerBound() const
{
	return lowerBound_;
}


int IntVar::getUpperBound() const
{
	return upperBound_;
}


IntVar* IntVar::setLowerBound(Model& env, int lb)
{
	if (active_)
	{
		env.intVarVec_[this->localIndex_].lowerBound_ = lb;
	}
		
	lowerBound_ = lb;
	
	return this;
}


IntVar* IntVar::setUpperBound(Model& env, int ub)
{
	if (active_)
	{
		env.intVarVec_[this->localIndex_].upperBound_ = ub;
	}
	
	upperBound_ = ub;
	
	return this;
}



//----------------------------------------------- [ BoolVar ]

BoolVar::BoolVar()
{
	sense_ = false;
	type_ = VariableTypes::Boolian;
	name_ = "b";
}


BoolVar::BoolVar(bool sense)
{
	sense_ = sense;
	type_ = VariableTypes::Boolian;
	name_ = "b";
}


bool BoolVar::isTrue() const
{
	return sense_;
}


BoolVar* BoolVar::setSense(Model& env, bool sense)
{
	if (active_)
	{
		env.boolVarVec_[this->localIndex_].sense_ = sense;
	}
	
	sense_ = sense;
	
	return this;
}

