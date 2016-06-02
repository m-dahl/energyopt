/*----------------------------------------------------------------------------
 ADOL-C -- Automatic Differentiation by Overloading in C++
 File:     ADOL-C_NLP.cpp
 Revision: $$
 Contents: class myADOLC_NPL for interfacing with Ipopt
 
 Copyright (c) Andrea Walther
   
 This file is part of ADOL-C. This software is provided as open source.
 Any use, reproduction, or distribution of the software constitutes 
 recipient's acceptance of the terms of the accompanying license file.
 
 This code is based on the file  MyNLP.cpp contained in the Ipopt package
 with the authors:  Carl Laird, Andreas Waechter   
----------------------------------------------------------------------------*/

/** C++ Example NLP for interfacing a problem with IPOPT and ADOL-C.
 *  MyADOL-C_NLP implements a C++ example showing how to interface 
 *  with IPOPT and ADOL-C through the TNLP interface. This class 
 *  implements the Example 5.1 from "Sparse and Parially Separable
 *  Test Problems for Unconstrained and Equality Constrained
 *  Optimization" by L. Luksan and J. Vlcek ignoring sparsity.
 *
 *  no exploitation of sparsity !!
 *
 */
#include <cassert>

#include "ADOL-C_NLP.hpp"

using namespace Ipopt;

/* Constructor. */
MyADOLC_NLP::MyADOLC_NLP(Model* model)
{
	model_ = model;
}

MyADOLC_NLP::~MyADOLC_NLP(){}

bool MyADOLC_NLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = model_->getNbFloatVars();

	m = model_->getNbConstraints();

  // in this example the jacobian is dense. Hence, it contains n*m nonzeros
  nnz_jac_g = n*m;

  // the hessian is also dense and has n*n total nonzeros, but we
  // only need the lower left corner (since it is symmetric)
  nnz_h_lag = n*(n-1)/2+n;

  generate_tapes(n, m);

  // use the C style indexing (0-based)
  index_style = C_STYLE;

  return true;
}

bool MyADOLC_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{
	// Set Bound on the variables.
	for (Index i = 0; i<n; i++) {

		// Lower bound
		if (model_->getFloatVar(i).hasLowerBound())
		{
			x_l[i] = model_->getFloatVar(i).getLowerBound();
		}
		else
		{
			x_l[i] = -1e20;
		}

		// Upper bound
		if (model_->getFloatVar(i).hasUpperBound())
		{
			x_u[i] = model_->getFloatVar(i).getUpperBound();
		}
		else
		{
			x_u[i] = 1e20;
		}

	}

	// Set the bounds for the constraints (<= or ==)

	for (Index i = 0; i<m; i++)
	{
		int boolIndex = model_->getConstraint(i).getLinkedBoolVarIndex();

		ConstraintTypes::Type conType = model_->getConstraint(i).getType();

		// Index -1 on LinkedBoolVar of a constraint means it's an ordinary(non-reified)
		// constraitn and has noboolian variable associated with it.

		if (boolIndex == -1) // Ordinary
		{
			if (conType == ConstraintTypes::eq) // =
			{
				g_l[i] = 0;
				g_u[i] = 0;
			}
			else if (conType == ConstraintTypes::neq) // <=
			{
				g_l[i] = -1e20;
				g_u[i] = 0;
			}
			else throw "Invalid Constraint Type!";
		}
		else // Reified
		{
			if (model_->getConstraint(i).isImplied())
			{
				if (conType == ConstraintTypes::eq) // = 
				{
					g_l[i] = 0;
					g_u[i] = 0;
				}
				else if (conType == ConstraintTypes::neq) // <=
				{
					g_l[i] = -1e20;
					g_u[i] = 0;
				}
				else throw "Invalid Constraint Type!";
			}
			else  //Relax if not implied
			{
				g_l[i] = -1e20;
				g_u[i] = 1e20;
			}
		}
	}

  return true;
}

bool MyADOLC_NLP::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);

	//Initialize all with zero
	for (int i = 0; i < n; i++)
	{
		x[i] = 0.;
	}

	// Change the ones with a supplied value.
	int nbStartingPoints = model_->getNbStartingPoints();

	if (nbStartingPoints >= 1)
	{
		int varIndex = -1;
		for (map<int, double>::const_iterator it = model_->getStartingPoints().begin(); it != model_->getStartingPoints().end(); it++) {

			varIndex = it->first;
			x[varIndex] = it->second;
		}
	}

  return true;
}

template<class T> bool  MyADOLC_NLP::eval_obj(Index n, const T *x, T& obj_value)
{
	obj_value = 0.;
	int nbTerms = model_->getObjective().getNbTerms();

	for (int i = 0; i < nbTerms; i++)
	{
		// Initializing a term with 1 is mandatory because:
		// (1) The subterms cannot be multiplied with an empty term (muliplication by 0).
		// (2) No matter what the subterms are, multiplication with 1.0 yields the
		//     desired form of term.

		T term = 1.0;
		int nbSubTerms = static_cast<int>(model_->getObjective().getExpression()[i].size());

		// Multiply with the subterms
		for (int j = 0; j < nbSubTerms; j++)
		{
			int varIndex = model_->getObjective().getExpression()[i][j].getIndex();
			double varCoeff = model_->getObjective().getExpression()[i][j].getCoefficient();
			int varPower = model_->getObjective().getExpression()[i][j].getPower();

			// Check if the subterm j has a variable
			if (varIndex != -1)
			{
				term = term * varCoeff * pow(x[varIndex], varPower);
			}
			else // Constant term
			{
				term = term * varCoeff;
			}
		}

		obj_value += term;
	}

  return true;
}

template<class T> bool  MyADOLC_NLP::eval_constraints(Index n, const T *x, Index m, T* g)
{
	for (Index k = 0; k < m; k++)
	{
		int nbTerms = model_->getConstraint(k).getNbTerms();
		T expr = 0.0;

		for (int i = 0; i < nbTerms; i++)
		{

			T term = 1.0;
			int nbSubTerms = static_cast<int>(model_->getConstraint(k).getExpression()[i].size());

			// Multiply with the subterms
			for (int j = 0; j < nbSubTerms; j++)
			{
				int varIndex = model_->getConstraint(k).getExpression()[i][j].getIndex();
				double varCoeff = model_->getConstraint(k).getExpression()[i][j].getCoefficient();
				int varPower = model_->getConstraint(k).getExpression()[i][j].getPower();

				// Check if the subterm j has a variable
				if (varIndex != -1)
				{
					term = term * varCoeff * pow(x[varIndex], varPower);
				}
				else // Constant term
				{
					term = term * varCoeff;
				}
			}

			expr += term;
		}

		g[k] = expr;
	}

  return true;
}

//*************************************************************************
//
//
//         Nothing has to be changed below this point !!
//
//
//*************************************************************************


bool MyADOLC_NLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  eval_obj(n,x,obj_value);

  return true;
}

bool MyADOLC_NLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{

  gradient(tag_f,n,x,grad_f);

  return true;
}

bool MyADOLC_NLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{

  eval_constraints(n,x,m,g);

  return true;
}

bool MyADOLC_NLP::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{
  if (values == NULL) {
    // return the structure of the jacobian, 
    // assuming that the Jacobian is dense

    Index idx = 0;
    for(Index i=0; i<m; i++)
      for(Index j=0; j<n; j++)
	{
	  iRow[idx] = i;
	  jCol[idx++] = j;
	}
 }
  else {
    // return the values of the jacobian of the constraints

    jacobian(tag_g,m,n,x,Jac);

    Index idx = 0;
    for(Index i=0; i<m; i++)
      for(Index j=0; j<n; j++)
	  values[idx++] = Jac[i][j];

  }

  return true;
}

bool MyADOLC_NLP::eval_h(Index n, const Number* x, bool new_x,
                   Number obj_factor, Index m, const Number* lambda,
                   bool new_lambda, Index nele_hess, Index* iRow,
                   Index* jCol, Number* values)
{
  if (values == NULL) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    // the hessian for this problem is actually dense
    Index idx=0;
    for (Index row = 0; row < n; row++) {
      for (Index col = 0; col <= row; col++) {
        iRow[idx] = row;
        jCol[idx] = col;
        idx++;
      }
    }

    assert(idx == nele_hess);
  }
  else {
    // return the values. This is a symmetric matrix, fill the lower left
    // triangle only

    obj_lam[0] = obj_factor;
    for(Index i = 0; i<m ; i++)
      obj_lam[1+i] = lambda[i];

    set_param_vec(tag_L,m+1,obj_lam);
    hessian(tag_L,n,const_cast<double*>(x),Hess);

    Index idx = 0;

    for(Index i = 0; i<n ; i++)
      {
	for(Index j = 0; j<=i ; j++)
	  {
	    values[idx++] = Hess[i][j];
	  }
      }
  }

  return true;
}

void MyADOLC_NLP::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			      const IpoptData* ip_data,
			      IpoptCalculatedQuantities* ip_cq)
{

  printf("\n\nObjective value\n");
  printf("f(x*) = %e\n", obj_value);

// Memory deallocation for ADOL-C variables

  delete[] obj_lam;

  for(Index i=0;i<m;i++)
    delete[] Jac[i];
  delete[] Jac;

  for(Index i=0;i<n;i++)
    delete[] Hess[i];
  delete[] Hess;
}


//***************    ADOL-C part ***********************************

void MyADOLC_NLP::generate_tapes(Index n, Index m)
{
  Number *xp    = new double[n];
  Number *lamp  = new double[m];
  Number *zl    = new double[m];
  Number *zu    = new double[m];

  adouble *xa   = new adouble[n];
  adouble *g    = new adouble[m];
  double *lam   = new double[m];
  double sig;
  adouble obj_value;
  
  double dummy;

  Jac = new double*[m];
  for(Index i=0;i<m;i++)
    Jac[i] = new double[n];

  obj_lam   = new double[m+1];

  Hess = new double*[n];
  for(Index i=0;i<n;i++)
    Hess[i] = new double[i+1];

  get_starting_point(n, 1, xp, 0, zl, zu, m, 0, lamp);

  trace_on(tag_f);
    
    for(Index i=0;i<n;i++)
      xa[i] <<= xp[i];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

  trace_off();
  
  trace_on(tag_g);
    
    for(Index i=0;i<n;i++)
      xa[i] <<= xp[i];

    eval_constraints(n,xa,m,g);


    for(Index i=0;i<m;i++)
      g[i] >>= dummy;

  trace_off();

   trace_on(tag_L);
    
    for(Index i=0;i<n;i++)
      xa[i] <<= xp[i];
    for(Index i=0;i<m;i++)
      lam[i] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);
 
    for(Index i=0;i<m;i++)
      obj_value += g[i]*mkparam(lam[i]);

    obj_value >>= dummy;

  trace_off();

  delete[] xa;
  delete[] xp;
  delete[] g;
  delete[] lam;
  delete[] lamp;
  delete[] zu;
  delete[] zl;

}
