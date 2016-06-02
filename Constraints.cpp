#include "Constraints.h"

//----------------------------------------------- [ Constraint ]

Constraint::Constraint(Expression& expr)
{
	expression_ = expr;

	index_ = -1;
	linkedBoolVarIndex_ = -1;
	implied_ = false;
	active_  = false;
	type_ = ConstraintTypes::Unknown;
}


Constraint* Constraint::setIndex(int index)
{
	index_ = index;

	return this;
}


Constraint* Constraint::linkToBoolVar(int index)
{
	linkedBoolVarIndex_ = index;

	return this;
}


Constraint* Constraint::setImplication(bool imply)
{
	implied_ = imply;

	return this;
}


Constraint* Constraint::activate()
{
	active_ = true;

	return this;
}


Constraint* Constraint::setType(ConstraintTypes::Type ctype)
{
	type_ = ctype;

	return this;
}


bool Constraint::isActive() const
{
	return active_;
}


bool Constraint::isImplied() const
{
	return implied_;
}


int Constraint::getIndex() const
{
	return index_;
}


int Constraint::getLinkedBoolVarIndex() const
{
	return linkedBoolVarIndex_;
}


ConstraintTypes::Type Constraint::getType() const
{
	return type_;
}


LeftHandSideExpression& Constraint::getExpression() 
{
	return this->expression_.getExpression();
}


int Constraint::getNbTerms() const
{
	return this->expression_.getNbTerms();
}


//----------------------------------------------- [ Sub Term ]

SubTerm::SubTerm(double coefficient, int index, int power)
{
	coefficient_ = coefficient;
	index_       = index;
	power_       = power;
}


double SubTerm::getCoefficient() const
{
	return coefficient_;
}


int SubTerm::getIndex() const
{
	return index_;
}


int SubTerm::getPower() const
{
	return power_;
}



//----------------------------------------------- [ Expression]
Expression::Expression()
{
	leftHandSideExpr_ = vector<Term>();
}


Expression& Expression::plus(double coefficient, const FloatVar& var, int power)
{
	// ax^b
	// Create an emty term, then create the first subterm and add it to the term.
	// Other subterms could be added to the term by calling the method multiply(...) 
	// Add the term to the expression.

	leftHandSideExpr_.push_back (Term());

	int termIndex = static_cast<int> (leftHandSideExpr_.size())-1;

	leftHandSideExpr_[termIndex].push_back (
		SubTerm(coefficient, var.getLocalIndex(), power) );
	
	return *this;
}


Expression& Expression::plus(const FloatVar& var, int power)
{
	//1x^b

	leftHandSideExpr_.push_back(Term());

	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back(SubTerm(1, var.getLocalIndex(), power));

	return *this;
}


Expression& Expression::plus(double coefficient, const FloatVar& var)
{
	//ax

	leftHandSideExpr_.push_back(Term());

	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back (SubTerm(coefficient, var.getLocalIndex(), 1) );

	return *this;
}


Expression& Expression::plus(double coefficient)
{
	leftHandSideExpr_.push_back(Term());

	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	// index=-1, when there is no variable in the subterm
	leftHandSideExpr_[termIndex].push_back (SubTerm(coefficient, -1 , 1));

	return *this;
}


Expression& Expression::plus(const FloatVar& var)
{
	leftHandSideExpr_.push_back(Term());

	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back(SubTerm(1, var.getLocalIndex(), 1));

	return *this;
}


Expression& Expression::multiply(double coefficient, const FloatVar& var, int power)
{
	// The last term is fetched from the expression.
	// Create the next subterm and store it together with other subterms of the last term.

	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back(
		SubTerm(coefficient, var.getLocalIndex(), power));

	return *this;
}


Expression& Expression::multiply(const FloatVar& var, int power)
{
	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back( SubTerm(1, var.getLocalIndex(), power));

	return *this;
}


Expression& Expression::multiply(double coefficient, const FloatVar& var)
{
	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back(SubTerm(coefficient, var.getLocalIndex(),1));

	return *this;
}


Expression& Expression::multiply(double coefficient)
{
	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back(SubTerm(coefficient, -1, 1));

	return *this;
}


Expression& Expression::multiply(const FloatVar& var)
{
	int termIndex = static_cast<int> (leftHandSideExpr_.size()) - 1;

	leftHandSideExpr_[termIndex].push_back(SubTerm(1, var.getLocalIndex(), 1));

	return *this;
}


int Expression::getNbTerms() const
{
	return static_cast<int> (leftHandSideExpr_.size());
}


LeftHandSideExpression& Expression::getExpression() 
{
	return leftHandSideExpr_;
}


Term& Expression::getTerm(int index)
{
	return leftHandSideExpr_[index];
}


//----------------------------------------------- [ Unary Constraint ]

UnaryConstraint& UnaryConstraint::addOperation(FloatVar& start, FloatVar& duration)
{
	operationsVec_.push_back(
		Operation(start.getLocalIndex(), DurPair(duration.getLocalIndex(), -1)));
	
	return *this;
}


UnaryConstraint& UnaryConstraint::addOperation(FloatVar& start, double duration)
{
	operationsVec_.push_back(
		Operation(start.getLocalIndex(), DurPair(-1, duration)));

	return *this;
}


const Operation& UnaryConstraint::getOperation(int operationId) const
{
	return operationsVec_[operationId];
}


int UnaryConstraint::getNbOperations() const
{
	return static_cast<int> (operationsVec_.size());
}


bool UnaryConstraint::hasConstantDuration(int operationId) const
{
	int durIndex = operationsVec_[operationId].second.first;

	if (durIndex == -1)
	{
		return true;
	}
	else
	{
		return false;
	}
}


int UnaryConstraint::getDurationVarIndex(int operationId) const
{
	return operationsVec_[operationId].second.first;
}


double UnaryConstraint::getDurationValue(int operationId) const
{
	return operationsVec_[operationId].second.second;
}


int UnaryConstraint::getStartVarIndex(int operationId) const
{
	return operationsVec_[operationId].first;
}








