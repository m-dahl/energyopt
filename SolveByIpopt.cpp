#include "Model.h"

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#include "ADOL-C_sparseNLP.hpp"
//#include "ADOL-C_NLP.hpp"

// For OHGen
#include "ipopt.h"


using namespace Ipopt;

int Model::solveByIpopt()
{

	//#---------------------------------------
	//#  OHGen
	//#---------------------------------------
	
	IpoptProblem ipoptProblem(this);
	SmartPtr<TNLP> mynlp = new IpoptSolver(&ipoptProblem);
	SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
	

//	app->Options()->SetStringValue("print_options_documentation", "yes"); 

	//#---------------------------------------
	//#  ADOL-C
	//#---------------------------------------
/*
	// Create an instance of your nlp...
	SmartPtr<TNLP> mynlp = new MyADOLC_sparseNLP( this );
	// Create an instance of the IpoptApplication
	SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
*/
	//	SmartPtr<TNLP> mynlp = new MyADOLC_NLP(this);
	
	//#---------------------------------------
	//#  Other options
	//#---------------------------------------


	// Change some options
	//app->RethrowNonIpoptException(true);
	//app->Options()->SetNumericValue("tol", 1e-7);
	//app->Options()->SetStringValue("mu_strategy", "adaptive");
	//app->Options()->SetStringValue("output_file", "ipopt.out");
	//app->RethrowNonIpoptException(true);
  //   app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    //app->Options()->SetStringValue("derivative_test","second-order");
    //app->Options()->SetIntegerValue("max_iter", 60);

	// Initialize the IpoptApplication and process the options
	ApplicationReturnStatus status;
	status = app->Initialize();
	if (status != Solve_Succeeded) {
		printf("\n\n*** Error during initialization!\n");
		return (int)status;
	}

	status = app->OptimizeTNLP(mynlp);

	if (status == Solve_Succeeded) {
		// Retrieve some statistics about the solve
		Index iter_count = app->Statistics()->IterationCount();
		printf("\n\n*** The problem solved in %d iterations!\n", iter_count);

		Number final_obj = app->Statistics()->FinalObjective();
		printf("\n\n*** The final value of the objective function is %e.\n", final_obj);
	}

	return (int)status;
}