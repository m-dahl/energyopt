#include "Model.h"

Model::Model()
{
	nbAllVars_   = 0;
	nbFloatVars_ = 0;
	nbIntVars_   = 0;
	nbBoolVars_  = 0;
}


Model* Model::addFloatVar(FloatVar& var)
{
	processOneFloatVar(var);

	return this;
}

Model* Model::addFloatVar(vector<FloatVar>& varVec)
{
	
	for (vector<FloatVar>::iterator it = varVec.begin(); it != varVec.end() ; it++)
	{
		processOneFloatVar(*it);
	}

	return this;
}


Model* Model::addIntVar(IntVar& var)
{
	processOneIntVar(var);

	return this;
}


Model* Model::addIntVar(vector<IntVar>& varVec)
{
	for (vector<IntVar>::iterator it = varVec.begin(); it != varVec.end(); it++)
	{
		processOneIntVar(*it);
	}

	return this;
}


Model* Model::addBoolVar(BoolVar& var)
{
	processOneBoolVar(var);
	
	return this;
}


Model* Model::addBoolVar(vector<BoolVar>& varVec)
{
	for (vector<BoolVar>::iterator it = varVec.begin(); it != varVec.end(); it++)
	{
		processOneBoolVar(*it);
	}

	return this;
}


void Model::processOneFloatVar (FloatVar& var)
{
	var.activate();

	var.setLocalIndex (nbFloatVars_);
	var.setGlobalIndex(nbAllVars_);

	floatVarVec_.push_back (var);

	nbFloatVars_ ++;
	nbAllVars_ ++;
}


void Model::processOneIntVar(IntVar& var)
{
	var.activate();

	var.setLocalIndex (nbIntVars_);
	var.setGlobalIndex(nbAllVars_);

	intVarVec_.push_back(var);

	nbIntVars_ ++;
	nbAllVars_ ++;
}


void Model::processOneBoolVar(BoolVar& var)
{
	var.activate();
	var.setLocalIndex (nbBoolVars_);
	var.setGlobalIndex(nbAllVars_);

	boolVarVec_.push_back(var);

	nbBoolVars_++;
	nbAllVars_++;
}


Model* Model::addConstraint(Expression& expr, string type)
{
	int nbConstraints = static_cast<int> (constraintsVec_.size());
		
	Constraint con(expr);

	// A reified constraints has 2 more settings. See the constructor: 
	// Model::addConstraint(Expression& expr, string type, BoolVar& bVar, bool imply)
	con.setIndex(nbConstraints);
	con.activate();
	
	if (type == "=")
	{
		con.setType(ConstraintTypes::eq);
	}
	else if (type == "<=")
	{
		con.setType(ConstraintTypes::neq);
	}
	else throw "Unknown Constraint Type";
		
	constraintsVec_.push_back(con);

	return this;
}


Model* Model::addConstraint(Expression& expr, string type, BoolVar& bVar, bool imply)
{
	int nbConstraints = static_cast<int> (constraintsVec_.size());

	Constraint con(expr);

	con.setIndex(nbConstraints);
	con.activate();
	// Extra stuff for reified constraitns
	con.linkToBoolVar (bVar.getLocalIndex());
	con.setImplication(imply);
	
	if (type == "=")
	{
		con.setType(ConstraintTypes::eq);
	}
	else if (type == "<=")
	{
		con.setType(ConstraintTypes::neq);
	}
	else throw "Unknown Constraint Type";

	constraintsVec_.push_back(con);

	return this;
}

//Currently works only for NLP terms. To be extended to all kinds of terms
void Model::printTerm(Term & t) const
{
	// Printing subterm a*x^c: a, x, c.
	int nbSubTerms =  static_cast<int> (t.size());
	for (size_t i = 0; i < t.size(); i++)
	{
		double a = t[i].getCoefficient();

		// If more than one subterm, add "*".
		if (i < nbSubTerms && i > 0)
		{
		cout << " * ";
		}

		// Extra space and "+/-" for the first subterm
		if (a > 0 && i==0) 
		{
			cout << " + " << t[i].getCoefficient();
		}

		// Extra space and "+/-" for the first subterm
		else if (a < 0 && i == 0) 
		{
			cout << " - " << abs(t[i].getCoefficient());
		}

		// Next negative subterm: no space, but with "-"
		else if(a < 0 && i != 0) 
		{
			cout << "-" << abs(t[i].getCoefficient());
		}

		// Next positive term: no space, no sign
		else if (a > 0 && i != 0) 
		{
			cout << abs(t[i].getCoefficient());
		}
		
		// Print variable index if it exists (index != -1)	
		int tmpIndex = t[i].getIndex();
		if (tmpIndex != -1)
		{
			cout << floatVarVec_[tmpIndex].getName() << "[" << tmpIndex << "]";
			
			int power = t[i].getPower();
			if (power != 1) {
				cout << "^" << power;
			}	
		}

	}
}


void Model::prettyPrint(Constraint& con) const
{
	//Print Info on reified constraints
	if (con.getLinkedBoolVarIndex() != -1)
	{
		int bVarIndex = con.getLinkedBoolVarIndex();
		cout << boolVarVec_[bVarIndex].getName() << "[" << bVarIndex << "]";

		if (con.isImplied())
		{
			cout << " => ";
		}
		else
		{
			cout << " !=> ";
		}
	}
	
	// Print the expression
	this->prettyPrint(con.expression_);

	// Print "<=" or "="
	if (ConstraintTypes::eq == con.getType())
	{
		cout << "== 0 ";
	}
	else if(ConstraintTypes::neq == con.getType())
	{
		cout << "<= 0 ";
	}
	else
	{
		throw "Unknown Constraint Type.";
	}

}


void Model::prettyPrint(Expression& expr) const
{
	// Print the expression
	for (size_t i = 0; i < expr.leftHandSideExpr_.size(); i++)
	{
		printTerm(expr.leftHandSideExpr_[i]);
		cout << " ";
	}
}


Model* Model::addObjective(Expression& expr)
{
	objective_ = expr;

	return this;
}


Model* Model::addStartingPoint(FloatVar& var, double val)
{
	startingPointsMap_.insert( pair<int,double>(var.getLocalIndex(), val) );

	return this;
}


Model* Model::changeStartingPoint(FloatVar& var, double val)
{
	int index = var.getLocalIndex();

	startingPointsMap_.operator[](index) = val;

	/*for (int i = 0; i < this->getNbStartingPoints(); i++)
	{
		if (startingPointsVec_[i].first == index)
		{
			startingPointsVec_[i].second = val;
		}

	}*/

	return this;
}


Model* Model::addUnaryConstraint(UnaryConstraint& uCon)
{
	unaryConstraintVec_.push_back(uCon);

	return this;
}


int Model::getNbFloatVars() const
{
	return nbFloatVars_;
}


int Model::getNbBoolVars() const
{
	return nbBoolVars_;
}


int Model::getNbConstraints() const
{
	return static_cast<int> (constraintsVec_.size());
}


int Model::getNbUnaryConstraints() const
{
	return static_cast<int> (unaryConstraintVec_.size());
}


int Model::getNbStartingPoints() const
{
	return static_cast<int> (startingPointsMap_.size());
}


double Model::getSol(FloatVar& var) const
{
	return solutionVec_[var.getLocalIndex()];
}


const map<int, double>& Model::getStartingPoints() const
{
	return startingPointsMap_ ;
}


const FloatVar&  Model::getFloatVar(int index) const 
{
	return   floatVarVec_[index];
}


Constraint& Model::getConstraint(int index) 
{
	return constraintsVec_[index];
}


UnaryConstraint& Model::getUnaryConstraint(int index) 
{
	return unaryConstraintVec_[index];
}


Expression& Model::getObjective() 
{
	return objective_;
}
