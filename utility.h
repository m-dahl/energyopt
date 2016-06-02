#ifndef hUtility
#define hUtility


#include "Model.h"

bool isLinear( Constraint & con );
vector<int> getIncludedVariables(vector<Term> & expression);
vector<int> getIncludedVariables(Constraint & constraint);

void differentiate(vector<Term> & expression, int index, Expression & derivative);
void differentiate(Constraint & constraint, int index, Expression & derivative);

double evaluateExpression(vector<Term> & expression, const double * x);

class Linearizer
{
public:

	
	Linearizer(Constraint constraint);

	Constraint linearize(const double * x);

	Constraint constraint_;
	std::vector<std::pair<int, Expression>> gradient_;


};


#endif
