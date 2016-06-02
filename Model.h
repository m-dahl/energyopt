//-----------------------------------------------
// I3 num 00.
// A simple Modeling Environment 
// Started: September 3 2015 By Sarmad Riazi
//-----------------------------------------------

#pragma once

#ifndef MODEL_H
#define MODEL_H

#include<map>
#include "Variables.h"
#include "Constraints.h"

using std::vector;
using std::map;



class Model
{
	friend class FloatVar;
	friend class IntVar;
	friend class BoolVar;
		
	public:
		
		Model();

		Model* addFloatVar(FloatVar& var);
		Model* addFloatVar(vector<FloatVar>& varVec);

		Model* addIntVar(IntVar& var);
		Model* addIntVar(vector<IntVar>& varVec);

		Model* addBoolVar(BoolVar& var);
		Model* addBoolVar(vector<BoolVar>& varVec);

		Model* addConstraint(Expression& expr, string type);
		Model* addConstraint(Expression& expr, string type, BoolVar& bVar, bool imply);

		Model* addUnaryConstraint(UnaryConstraint& uCon);

		Model* addObjective(Expression& expr);

		Model* addStartingPoint(FloatVar& var, double val);

		Model* changeStartingPoint(FloatVar& var, double val);

		
		// These need to be updated if I want to distinguishe Int constraints, etc.
		int getNbFloatVars() const;
		int getNbBoolVars() const; 
		int getNbConstraints() const;
		int getNbUnaryConstraints() const;
		int getNbStartingPoints() const;

		double getSol(FloatVar& var) const;

		const FloatVar&  getFloatVar(int index) const ;
		Constraint& getConstraint(int index) ;
		UnaryConstraint& getUnaryConstraint(int index)  ;
		Expression& getObjective() ;
		const map<int, double>& getStartingPoints() const;

		void prettyPrint(Constraint& con) const;
		void prettyPrint(Expression& expr) const;

		// This has public access for simplicity.
		vector<Constraint> constraintsVec_; 
		vector<UnaryConstraint> unaryConstraintVec_;
		Expression objective_;
		vector<double> solutionVec_;
		double objectiveValue_;

		int solveByIpopt();
		// getsolution(varVec)

	private:
		
		int nbAllVars_;
		int nbFloatVars_;
		int nbIntVars_;
		int nbBoolVars_;
				
		vector<FloatVar> floatVarVec_;
		vector<IntVar>   intVarVec_;
		vector<BoolVar>  boolVarVec_;
		map<int, double> startingPointsMap_; 
		
				
		void processOneFloatVar(FloatVar& var);
		void processOneIntVar(IntVar& var);
		void processOneBoolVar(BoolVar& var);
		
		void printTerm(Term & t) const;
		
};


#endif