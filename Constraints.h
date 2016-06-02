#pragma once

#ifndef CONSTRAINTS
#define CONSTRAITNS

#include<string> 
#include<iostream>


#include "Variables.h"




using std::vector;
using std::pair;
using std::cout; 

namespace ConstraintTypes
{
	typedef int Type;
	enum { Unknown = -1, neq = 0, eq = 1 };
};


//----------------------------------------------- [ Sub_term ]
// Ex. (2x^2) is a subterm in the term (2x^2)*(y^3).

class SubTerm
{
  public:

	  SubTerm(double coefficient, int index, int power);

	  double getCoefficient() const;
	  int    getIndex() const;
	  int    getPower() const;

  private:

	  double coefficient_;
	  int    index_;
	  int    power_;
};


//----------------------------------------------- [ Term ]
typedef vector<SubTerm> Term;
typedef vector<Term> LeftHandSideExpression;

//----------------------------------------------- [ Expression ]
// This class can be written using templates, so that it accomodates for MILP
// formulations as well. Right now it accepts only FloatVar.

class Expression
{
  friend class Model;

  public:
	  
	  Expression();

	  Expression& plus(double coefficient, const FloatVar& var, int power);	//2x^3
	  Expression& plus(const FloatVar& var, int power);						//x^3
	  Expression& plus(double coefficient, const FloatVar& var);			//2x
	  Expression& plus(double coefficient);									//2
	  Expression& plus(const FloatVar& var);								//x


	  Expression& multiply(double coefficient, const FloatVar& var, int power);	//2x^3
	  Expression& multiply(const FloatVar& var, int power);						//x^3
	  Expression& multiply(double coefficient, const FloatVar& var);			//2x
	  Expression& multiply(double coefficient);									//2  
	  Expression& multiply(const FloatVar& var);								//x 

	  int getNbTerms() const;
	  LeftHandSideExpression& getExpression() ;
	  Term& getTerm(int index);


  private:

	  LeftHandSideExpression leftHandSideExpr_;
};


//----------------------------------------------- [ Constraint ]

class Constraint
{
  friend class Model;

  public:

	  Constraint(Expression& expr);

	  bool isActive() const;
	  bool isImplied() const;
	  int  getLinkedBoolVarIndex() const;
	  int getIndex() const;
	  ConstraintTypes::Type getType() const;

	  LeftHandSideExpression& getExpression() ;
	  int getNbTerms() const;

  private:

	int index_;
	int linkedBoolVarIndex_;
	bool implied_;
	bool active_;

	ConstraintTypes::Type type_;
	Expression expression_;

	Constraint* setIndex(int index);
	Constraint* linkToBoolVar(int index);
	Constraint* setImplication(bool imply);
	Constraint* activate();
	Constraint* setType(ConstraintTypes::Type ctype);

};
// ----------------------------------------------- [ Operation ]

//Operation = pair <int startVarIndex, <int durationVarIndex, double duration>>
typedef pair<int , pair<int , double > > Operation;
typedef pair<int, double> DurPair;

//----------------------------------------------- [ Unary Constraint ]

class UnaryConstraint
{
  public:

	  // Manipulators
	  UnaryConstraint& addOperation(FloatVar& start, FloatVar& duration);
	  UnaryConstraint& addOperation(FloatVar& start, double duration);
	  
	  // Accessors
	  const  Operation& getOperation(int operationId) const;

	  bool hasConstantDuration(int operationId) const;

	  int getNbOperations() const;
	  int getDurationVarIndex (int operationId) const;
	  int getStartVarIndex (int operationId) const;
	  double getDurationValue(int operationId) const;

  private:
	  vector<Operation> operationsVec_;

};









//Expression expr;
//expr. + (2, x, 3)-> * (3, y, 2);
//Constraint con(env, expr, '<=' ,0 )







#endif