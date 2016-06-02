#include "ipopt.h"


#include <algorithm>



#include "utility.h"

















void setupJacobian(std::vector<Element> & jacobian, vector<Term> & expression, int constraintIndex)

{



	vector<int> variables = getIncludedVariables(expression);



		for (int j = 0; j < variables.size(); j++)

		{

			Expression derivative;



			differentiate(expression, variables[j], derivative);



			jacobian.push_back(Element(constraintIndex, variables[j], derivative));



		}



}





void setupHessian(Element & jacobianElement, std::vector<std::pair<int,int>> & hessianIndices,

								std::vector<Element> & hessianElements)

{

		vector<int> variables = 

					getIncludedVariables(jacobianElement.expression_.getExpression());

		int jIndex = jacobianElement.elementIndex_;





		for (int i = 0; i < variables.size(); i++)

		{

			Expression derivative;



			differentiate(jacobianElement.expression_.getExpression(), 

										variables[i], derivative);

		

			



			// Make sure i < j

			int I; int J;

			int iIndex = variables[i];



			if ( iIndex > jIndex)

			{

				

				I = iIndex;

				J = jIndex;

			}

			else

			{

				I = jIndex;

				J = iIndex;

			}



			// Search if we have already included this hessian index.



			std::vector<std::pair<int,int>>::iterator hessianIndex =

						std::find(hessianIndices.begin(), hessianIndices.end(), 

									std::pair<int,int>(I,J));



			int position;



			if (  hessianIndex == hessianIndices.end() )

			{

				position = hessianIndices.size();

				std::pair<int,int> index(I,J);

				hessianIndices.push_back(index);

			}

			else

			{

				position = hessianIndex - hessianIndices.begin();

			}

			

			hessianElements.push_back(Element(jacobianElement.constraintIndex_, position, derivative));

			



		}

	





}





IpoptProblem::~IpoptProblem()

{

	delete relaxVector_;

};



IpoptProblem::IpoptProblem() : model_(NULL), relaxVector_(NULL)

{

};



IpoptProblem::IpoptProblem(Model * model) : model_(model), relaxVector_(NULL)

{

	initialize();

};



void IpoptProblem::initialize(Model * model)

{

	model_ = model;

	initialize();



}





void IpoptProblem::updateBooleanVector(int * b)

{



	for (size_t i = 0; i < implicationArray_.size(); i++)

	{

		int booleanValue = b[implicationArray_[i].second.first];

		bool imples = implicationArray_[i].second.second;

		int constraintIndex = implicationArray_[i].first;



		relaxVector_[constraintIndex] = true;





		if (booleanValue == 0 && !imples)

		{

			relaxVector_[constraintIndex] = false;

		}

		else if (booleanValue == 1 && imples)

		{

			relaxVector_[constraintIndex] = false;

		}



	

	}





};







void IpoptProblem::isolateBooleanConstraints()

{



	implicationArray_.clear();



	for (size_t i = 0; i < model_->getNbConstraints(); i++)

	{

		if ( model_->getConstraint(i).getLinkedBoolVarIndex() != -1 ) 

		{

			this->implicationArray_.push_back(std::pair<int, std::pair<int, bool>>

				(i,

				std::pair<int,bool>(model_->getConstraint(i).getLinkedBoolVarIndex()

								, model_->getConstraint(i).isImplied())));

		} 

	}





};



void IpoptProblem::initialize()

{

	/*

	 *	Initialize bool vector

	 */

	

	isolateBooleanConstraints();



	relaxVector_ = new bool[model_->getNbConstraints()];

	for (int i = 0; i < model_->getNbConstraints(); i++)

	{

		relaxVector_[i] = false;

	}



	/*

	 *	Meta data

	 */



	nbVar_ = model_->getNbFloatVars();

	nbCon_ = model_->getNbConstraints();



	/*

	 *	Jacobian!

	 */



	// Constraints



	for (int i = 0; i < model_->getNbConstraints(); i++)

	{

		setupJacobian(jacobian_, model_->getConstraint(i).getExpression(),i);

	

	}



	// Objective



	setupJacobian(objectiveJacobian_, model_->getObjective().getExpression(), -1);





	/*

	 *	Hessian!

	 */



	// Constraints



	for (int k = 0; k < jacobian_.size(); k++)

	{

		setupHessian(jacobian_[k], hessianIndices_, hessianElements_);

	}



	// Objective

	for (int k = 0; k < objectiveJacobian_.size(); k++)

	{

		setupHessian(objectiveJacobian_[k], hessianIndices_, objectiveHessianElements_);



	}

}





#include <cassert>

#include <iostream>



using namespace Ipopt;



// constructor

IpoptSolver::IpoptSolver(IpoptProblem * problem) : problem_(problem) 

{}



//destructor

IpoptSolver::~IpoptSolver()

{}



// returns the size of the problem

bool IpoptSolver::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,

                             Index& nnz_h_lag, IndexStyleEnum& index_style)

{



	n = problem_->nbVar_;

	m = problem_->nbCon_;



	nnz_jac_g = problem_->jacobian_.size();



	nnz_h_lag = problem_->hessianIndices_.size();



	// use the C style indexing (0-based)

	index_style = TNLP::C_STYLE;



  return true;

}



// returns the variable bounds

bool IpoptSolver::get_bounds_info(Index n, Number* x_l, Number* x_u,

                                Index m, Number* g_l, Number* g_u)

{



	if (problem_->lb_.size() == 0)

	{

		for (Index i=0; i<n; i++) {

			x_l[i] = problem_->model_->getFloatVar(i).getLowerBound();

			x_u[i] = problem_->model_->getFloatVar(i).getUpperBound();

		}

	}

	else

	{

		for (Index i=0; i<n; i++) {

			x_l[i] = problem_->lb_[i];

			x_u[i] = problem_->ub_[i];

		}

	}



	for (Index i=0; i<m; i++) {

		if (problem_->relaxVector_[i])

		{

			// Relax

			g_u[i] =  2e19;

			g_l[i] = -2e19;

		}

		else

		{

			g_u[i] = 0;

		

			if (problem_->model_->getConstraint(i).getType() == ConstraintTypes::eq)

			{

				g_l[i] = 0;

			}

			else

			{

				g_l[i] = -2e19;

			}

		}

	}



  return true;

}



// returns the initial point for the problem

bool IpoptSolver::get_starting_point(Index n, bool init_x, Number* x,

                                   bool init_z, Number* z_L, Number* z_U,

                                   Index m, bool init_lambda,

                                   Number* lambda)

{

  // Here, we assume we only have starting values for x, if you code

  // your own NLP, you can provide starting values for the dual variables

  // if you wish

  assert(init_x == true);

  assert(init_z == false);

  assert(init_lambda == false);



  //Initialize all with zero

  for (int i = 0; i < n; i++)

  {

	  x[i] = 0.;

  }



  // Change the ones with a supplied value.

  int nbStartingPoints = problem_->model_->getNbStartingPoints();



  if (nbStartingPoints >= 1)

  {

	  int varIndex = -1;

	  for (map<int, double>::const_iterator it = problem_->model_->getStartingPoints().begin(); it != problem_->model_->getStartingPoints().end(); it++) {



		  varIndex = it->first;

		  x[varIndex] = it->second;

	  }

  }

  return true;

}



// returns the value of the objective function

bool IpoptSolver::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)

{



  obj_value = evaluateExpression(problem_->model_->getObjective().getExpression(), x);





  return true;

}



// return the gradient of the objective function grad_{x} f(x)

bool IpoptSolver::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)

{

	for (int i = 0; i < n; i++){ grad_f[i] = 0; }

	



	for (int i = 0; i < problem_->objectiveJacobian_.size(); i++)

	{

		int index = problem_->objectiveJacobian_[i].elementIndex_;

		grad_f[index] = evaluateExpression(problem_->objectiveJacobian_[i].expression_.getExpression(), x);

	}





  return true;

}



// return the value of the constraints: g(x)

bool IpoptSolver::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)

{



	for (int i = 0; i < m; i++)

	{

		g[i] = evaluateExpression(problem_->model_->getConstraint(i).getExpression(), x);

	}



	return true;

}



// return the structure or values of the jacobian

bool IpoptSolver::eval_jac_g(Index n, const Number* x, bool new_x,

                           Index m, Index nele_jac, Index* iRow, Index *jCol,

                           Number* values)

{

  if (values == NULL) {

    // return the structure of the jacobian

	for (int i = 0; i < problem_->jacobian_.size(); i++)

	{

		iRow[i] = problem_->jacobian_[i].constraintIndex_;

		jCol[i] = problem_->jacobian_[i].elementIndex_;

	}

  

  }

  else {

    // return the values of the jacobian of the constraints



	for (int i = 0; i < problem_->jacobian_.size(); i++)

	{

		values[i] = evaluateExpression(problem_->jacobian_[i].expression_.getExpression(), x);

	}

  }



  return true;

}



//return the structure or values of the hessian

bool IpoptSolver::eval_h(Index n, const Number* x, bool new_x,

                       Number obj_factor, Index m, const Number* lambda,

                       bool new_lambda, Index nele_hess, Index* iRow,

                       Index* jCol, Number* values)

{



if (values == NULL) {

    // return the structure. This is a symmetric matrix, fill the lower left

    // triangle only.



	for (int i = 0; i < problem_->hessianIndices_.size(); i++)

	{

		iRow[i] = problem_->hessianIndices_[i].first;

        jCol[i] = problem_->hessianIndices_[i].second;

	}







}

else {

    

	// return the values. This is a symmetric matrix, fill the lower left

    // triangle only



	// Set all elements to zero



	for (int i = 0; i < problem_->hessianIndices_.size(); i++)

	{

		values[i] = 0;

	}

	



	 // fill the objective portion



	for (int i = 0; i < problem_->objectiveHessianElements_.size(); i++)

	{

     	double value = evaluateExpression(problem_->objectiveHessianElements_[i]

									  .expression_.getExpression(), x);



		int hessianIndex = problem_->objectiveHessianElements_[i].elementIndex_;



		values[hessianIndex] = obj_factor * value;

	}



 	for (int i = 0; i < problem_->hessianElements_.size(); i++)

	{



		double value = evaluateExpression(problem_->hessianElements_[i]

									  .expression_.getExpression(), x);



		int constraintIndex = problem_->hessianElements_[i].constraintIndex_;

		int hessianIndex = problem_->hessianElements_[i].elementIndex_;



		if (problem_->hessianIndices_[hessianIndex].first

				== problem_->hessianIndices_[hessianIndex].second)

		{

			values[hessianIndex] += lambda[constraintIndex]*value;

		}

		else

		{

			values[hessianIndex] += lambda[constraintIndex]*value/2;

		}

	}





  }



  return true;

}



void IpoptSolver::finalize_solution(SolverReturn status,

                                  Index n, const Number* x, const Number* z_L, const Number* z_U,

                                  Index m, const Number* g, const Number* lambda,

                                  Number obj_value,

				  const IpoptData* ip_data,

				  IpoptCalculatedQuantities* ip_cq)

{

	// Save the solution
	for (int i = 0; i < problem_->model_->getNbFloatVars(); i++)
	{
		problem_->model_->solutionVec_.push_back(x[i]);
	}

	problem_->model_->objectiveValue_ = obj_value;

	printf("\n\nObjective value\n");
	printf("f(x*) = %e\n", obj_value);





	problem_->objectiveValue = obj_value;

	problem_->solution_ = x;



}

