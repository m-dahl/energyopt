#include "utility.h"
#include <algorithm>

bool isLinear( Constraint & con )
{
	
	vector<Term>::const_iterator termIterator = con.getExpression().begin();
	
	for ( size_t i = 0; i < con.getExpression().size(); i++ )
	{
		
		int numberOfSubTerms = con.getExpression()[i].size();

		if (numberOfSubTerms > 1)
		{
			return false;
		}

		int subTermPower = con.getExpression()[i][0].getPower();

		if (subTermPower != 1)
		{	
			return false;
		}

	}
	return true;
};


vector<int> getIncludedVariables(vector<Term> & expression)
{

	vector<int> variables;

	for (int i = 0; i < expression.size(); i++)
	{
	
		for (int j = 0; j < expression[i].size(); j++)
		{
			int variableIndex = expression[i][j].getIndex();

			if (variableIndex != -1)
			{
				
				if ( std::find(variables.begin(), variables.end(), 
									  variableIndex) == variables.end() )
				{
					variables.push_back(variableIndex);
				}	
			}
		}
	}

	return variables;
};


vector<int> getIncludedVariables(Constraint & constraint)
{
	return getIncludedVariables(constraint.getExpression());

};

void differentiate(vector<Term> & expression, int index, Expression & derivative)
{

	for (int i = 0; i < expression.size(); i++)
	{
		for (int j = 0; j < expression[i].size(); j++)
		{
			if ( expression[i][j].getIndex() == index)
			{
				
				derivative.getExpression().push_back(expression[i]);

				int index = expression[i][j].getIndex();
				double coefficient = expression[i][j].getCoefficient();
				int power = expression[i][j].getPower();

				int expressionLength = derivative.getExpression().size();

				// If differentiation removes a variable
				if ( power == 1 && index != -1)
				{

					derivative.getExpression()[expressionLength - 1][j] = 
								SubTerm(coefficient,-1,1);
				}
				// If constants (these should never come alone (since getVariables)
				else if( index == -1)
				{
					derivative.getExpression()[expressionLength - 1][j] = 
								SubTerm(coefficient,-1,1);
				}
				// Normal case, simple derivative
				else
				{
					derivative.getExpression()[expressionLength - 1][j] = 
								SubTerm(coefficient*power,index,power-1);
				}


			
			}

		}
	}


}


void differentiate(Constraint & constraint, int index, Expression & derivative)
{
	differentiate(constraint.getExpression(), index, derivative);
}


double evaluateExpression(vector<Term> & expression, const double * x)
{
	double value = 0;
	double subvalue;

	
	//	std::cout << "\n";

	for (int i = 0; i < expression.size(); i++)
	{

		subvalue = 1;

		for (int j = 0; j < expression[i].size(); j++)
		{
			int index = expression[i][j].getIndex();
			double coef = expression[i][j].getCoefficient();
			int power = expression[i][j].getPower();

			if (index != -1)
			{
				subvalue *= coef*pow(x[index],power);		
			}
			else
			{
				subvalue *= pow(coef,power);	
			}
		}
		 
		//std::cout <<  subvalue << " ";
		value += subvalue;

	}

	return value;
}



Linearizer::Linearizer(Constraint constraint) : constraint_(constraint)
{
		
	
	vector<int> variables = getIncludedVariables(constraint.getExpression());

	for (int j = 0; j < variables.size(); j++)
	{
		Expression derivative;

		differentiate(constraint.getExpression(), variables[j], derivative);

		gradient_.push_back( std::pair<int, Expression>( variables[j], derivative ) );

	}
};


Constraint Linearizer::linearize(const double * x)
{

	double constant = evaluateExpression(constraint_.getExpression(), x);

	Constraint constraint = constraint_;
	constraint.getExpression().clear();

	
	for (int i = 0; i < gradient_.size(); i++)
	{
		double gradientElement = evaluateExpression(gradient_[i].second.getExpression(), x);
		int index = gradient_[i].first;

		constant -= x[index]*gradientElement;

		Term term;
		term.push_back(SubTerm(gradientElement,index,1));
	
		constraint.getExpression().push_back(term);

	}

	Term constantTerm;
	constantTerm.push_back(SubTerm(constant,-1,1));
	constraint.getExpression().push_back(constantTerm);


	return constraint;
};