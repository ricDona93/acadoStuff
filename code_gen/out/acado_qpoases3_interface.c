/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include <acado_common.h>

#include "qpOASES_e/QProblemB.h"

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
#include "qpOASES_e/extras/SolutionAnalysis.h"
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

static int acado_nWSR;



#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis acado_sa;
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

int acado_solve( void )
{

	returnValue retVal;
	QProblemB qp;
	Options options;

	acado_nWSR = QPOASES_NWSRMAX;

	QProblemBCON( &qp, 10,HST_POSDEF );
	Options_setToMPC( &options );
	QProblemB_setOptions( &qp,options );
	
	retVal = QProblemB_initW( &qp, acadoWorkspace.H, acadoWorkspace.g, acadoWorkspace.lb, acadoWorkspace.ub, &acado_nWSR,0, 0,acadoWorkspace.y, 0,0 );
	retVal = qpOASES_getSimpleStatus( retVal,0 );

    /* only use solution on success, if iteration limit has been reached or if QP is infeasible! */
    if ( ( retVal == 0 ) || ( retVal == 1 ) || ( retVal == -2 ) )
    {
        QProblemB_getPrimalSolution( &qp,acadoWorkspace.x );
        QProblemB_getDualSolution( &qp,acadoWorkspace.y );
    }
	
#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = acado_sa.getHessianInverse( &qp,var );

#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int acado_getNWSR( void )
{
	return acado_nWSR;
}

const char* acado_getErrorString( int error )
{
	return MessageHandling_getErrorCodeMessage( qpOASES_getGlobalMessageHandler(),error );
}
