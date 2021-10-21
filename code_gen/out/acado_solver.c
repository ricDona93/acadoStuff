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


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
acadoWorkspace.state[0] = acadoVariables.x[0];
acadoWorkspace.state[1] = acadoVariables.x[1];
acadoWorkspace.state[8] = acadoVariables.u[0];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{

acadoWorkspace.state[8] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, lRun1 == 0);

acadoVariables.x[lRun1 * 2 + 2] = acadoWorkspace.state[0];
acadoVariables.x[lRun1 * 2 + 3] = acadoWorkspace.state[1];

acadoWorkspace.evGx[lRun1 * 4] = acadoWorkspace.state[2];
acadoWorkspace.evGx[lRun1 * 4 + 1] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 4 + 2] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 4 + 3] = acadoWorkspace.state[5];

acadoWorkspace.evGu[lRun1 * 2] = acadoWorkspace.state[6];
acadoWorkspace.evGu[lRun1 * 2 + 1] = acadoWorkspace.state[7];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 2;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[20];
acadoWorkspace.objValueIn[1] = acadoVariables.x[21];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1];
dNew[1] += + Gx1[2]*dOld[0] + Gx1[3]*dOld[1];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[2];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[3];
Gx3[2] = + Gx1[2]*Gx2[0] + Gx1[3]*Gx2[2];
Gx3[3] = + Gx1[2]*Gx2[1] + Gx1[3]*Gx2[3];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1];
Gu2[1] = + Gx1[2]*Gu1[0] + Gx1[3]*Gu1[1];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] += + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = (real_t)1.0000000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = acadoWorkspace.H[(iCol * 10) + (iRow)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)5.0000000000000000e-01*dOld[0];
dNew[1] = +dOld[1];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[2];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)5.0000000000000000e-01*Dy1[0];
QDy1[1] = +Dy1[1];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[1]*QDy1[1];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[1]*Gx1[2];
H101[1] += + E1[0]*Gx1[1] + E1[1]*Gx1[3];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 2; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0];
dNew[1] += + E1[1]*U1[0];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)5.0000000000000000e-01*Gx1[0];
Gx2[1] = + (real_t)5.0000000000000000e-01*Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e-03*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e-03*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e-03*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e-03*Gx1[3];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)5.0000000000000000e-01*Gu1[0];
Gu2[1] = +Gu1[1];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e-03*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e-03*Gu1[1];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
}

void acado_condensePrep(  )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 4 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 2 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.E[ 4 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGx[ 8 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.E[ 6 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.E[ 8 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.E[ 10 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGx[ 12 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 12 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.E[ 14 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.E[ 16 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 18 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGx[ 16 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 20 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.E[ 22 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.E[ 26 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 28 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGx[ 20 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.E[ 32 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 34 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.E[ 38 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.E[ 40 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.evGx[ 24 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 42 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 44 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.E[ 46 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.E[ 50 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 52 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 54 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.evGx[ 28 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.E[ 56 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.E[ 58 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 62 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.E[ 64 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.E[ 66 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 68 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 14 ]), &(acadoWorkspace.E[ 70 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.evGx[ 32 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.E[ 74 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 76 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.E[ 78 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.E[ 82 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.E[ 86 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 88 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.E[ 92 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.E[ 94 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 98 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.E[ 100 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 102 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.E[ 104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.E[ 106 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.E[ 108 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.QE[ 2 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QE[ 4 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.QE[ 14 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.QE[ 22 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.QE[ 26 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.QE[ 34 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.QE[ 38 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 44 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QE[ 46 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 58 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QE[ 62 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 74 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 92 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 94 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 98 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 106 ]), &(acadoWorkspace.QE[ 106 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2 ]), &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 56 ]), &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 4 ]), &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 8 ]), &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 14 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 22 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 44 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 58 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 74 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 92 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 2 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 10 ]), &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 34 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 46 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 76 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 94 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 18 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 26 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 62 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 28 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 38 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 50 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 98 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 52 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 82 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 100 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 10 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 68 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 14 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 70 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 14 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 86 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 14 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 104 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 14 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 88 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 106 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 18 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.QE[ 2 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.QE[ 4 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 14 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 22 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 44 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 58 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 74 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 92 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 34 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 46 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 94 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 26 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 62 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 96 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 38 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 98 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 100 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QE[ 4 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.QE[ 14 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.QE[ 22 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 44 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 58 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 74 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 92 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 34 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 46 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 94 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.QE[ 26 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 62 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 96 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 38 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 98 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 100 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QE[ 10 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.QE[ 34 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QE[ 46 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 94 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 26 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 62 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 96 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.QE[ 38 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 98 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 100 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.QE[ 26 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QE[ 62 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 38 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 98 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 100 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.QE[ 38 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 50 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 98 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 100 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.QE[ 82 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QE[ 70 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.QE[ 86 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 104 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 106 ]), &(acadoWorkspace.QE[ 106 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 106 ]), &(acadoWorkspace.QE[ 108 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 6 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 20 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 42 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 56 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 4 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 8 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 14 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 22 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 44 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 58 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 74 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 92 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 34 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 46 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 76 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 94 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 18 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 26 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 62 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 28 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 38 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 50 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 98 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 52 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 82 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 100 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 68 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 70 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 86 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 104 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 88 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 106 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 9 ]) );
acadoWorkspace.lb[0] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[9];
acadoWorkspace.ub[0] = (real_t)5.0000000000000000e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)5.0000000000000000e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)5.0000000000000000e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)5.0000000000000000e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)5.0000000000000000e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)5.0000000000000000e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)5.0000000000000000e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)5.0000000000000000e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)5.0000000000000000e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)5.0000000000000000e-01 - acadoVariables.u[9];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 9 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 18 ]) );

acadoWorkspace.QDy[20] = + (real_t)1.0000000000000000e-03*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[21] = + (real_t)1.0000000000000000e-03*acadoWorkspace.DyN[1];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2 ]), &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QDy[ 4 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 106 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 9 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[1] += + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[2] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[3] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[4] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[5] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[6] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[7] += + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[8] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[9] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];

acadoVariables.x[2] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1];
acadoVariables.x[3] += + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[1];
acadoVariables.x[4] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1];
acadoVariables.x[8] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1];
acadoVariables.x[9] += + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[1];
acadoVariables.x[10] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1];
acadoVariables.x[11] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1];
acadoVariables.x[12] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1];
acadoVariables.x[13] += + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[1];
acadoVariables.x[14] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1];
acadoVariables.x[15] += + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[1];
acadoVariables.x[16] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1];
acadoVariables.x[17] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1];
acadoVariables.x[18] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1];
acadoVariables.x[19] += + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[1];
acadoVariables.x[20] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1];
acadoVariables.x[21] += + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[1];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 2 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2 ]), acadoWorkspace.x, &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 6 ]), acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), acadoWorkspace.x, &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 14 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 20 ]), acadoWorkspace.x, &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 22 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 26 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 34 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 38 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 42 ]), acadoWorkspace.x, &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 46 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 56 ]), acadoWorkspace.x, &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 58 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 62 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 74 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 82 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 86 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 94 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 98 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 106 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 20 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 2];
acadoWorkspace.state[1] = acadoVariables.x[index * 2 + 1];
acadoWorkspace.state[8] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 2 + 2] = acadoWorkspace.state[0];
acadoVariables.x[index * 2 + 3] = acadoWorkspace.state[1];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 2] = acadoVariables.x[index * 2 + 2];
acadoVariables.x[index * 2 + 1] = acadoVariables.x[index * 2 + 3];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[20] = xEnd[0];
acadoVariables.x[21] = xEnd[1];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[20];
acadoWorkspace.state[1] = acadoVariables.x[21];
if (uEnd != 0)
{
acadoWorkspace.state[8] = uEnd[0];
}
else
{
acadoWorkspace.state[8] = acadoVariables.u[9];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[20] = acadoWorkspace.state[0];
acadoVariables.x[21] = acadoWorkspace.state[1];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[9] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9];
kkt = fabs( kkt );
for (index = 0; index < 10; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[20];
acadoWorkspace.objValueIn[1] = acadoVariables.x[21];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*(real_t)5.0000000000000000e-01;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3 + 2];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e-03;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e-03;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

