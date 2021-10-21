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


void acado_rhs_forw(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;

/* Compute outputs: */
out[0] = ((((real_t)(-1.7612000000000005e+01)*xd[0])-((real_t)(-2.8399999999999995e+01)*xd[1]))+((real_t)(4.8400000000000006e+01)*u[0]));
out[1] = ((((real_t)(-5.2666666666666684e-01)*xd[0])-((real_t)(1.5333333333333334e+01)*xd[1]))+((real_t)(7.3333333333333330e+00)*u[0]));
out[2] = (((real_t)(-1.7612000000000005e+01)*xd[2])-((real_t)(-2.8399999999999995e+01)*xd[4]));
out[3] = (((real_t)(-1.7612000000000005e+01)*xd[3])-((real_t)(-2.8399999999999995e+01)*xd[5]));
out[4] = (((real_t)(-5.2666666666666684e-01)*xd[2])-((real_t)(1.5333333333333334e+01)*xd[4]));
out[5] = (((real_t)(-5.2666666666666684e-01)*xd[3])-((real_t)(1.5333333333333334e+01)*xd[5]));
out[6] = ((((real_t)(-1.7612000000000005e+01)*xd[6])-((real_t)(-2.8399999999999995e+01)*xd[7]))+(real_t)(4.8400000000000006e+01));
out[7] = ((((real_t)(-5.2666666666666684e-01)*xd[6])-((real_t)(1.5333333333333334e+01)*xd[7]))+(real_t)(7.3333333333333330e+00));
}

/* Fixed step size:0.1 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[2] = 1.0000000000000000e+00;
rk_eta[3] = 0.0000000000000000e+00;
rk_eta[4] = 0.0000000000000000e+00;
rk_eta[5] = 1.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[8] = rk_eta[8];

for (run1 = 0; run1 < 1; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acado_rhs_forw( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acado_rhs_forw( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 8 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[8] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[9] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[10] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[11] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[12] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[13] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[14] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)5.0000000000000003e-02*acadoWorkspace.rk_kkk[15] + rk_eta[7];
acado_rhs_forw( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 16 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[16] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[17] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[18] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[19] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[20] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[21] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[22] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.0000000000000001e-01*acadoWorkspace.rk_kkk[23] + rk_eta[7];
acado_rhs_forw( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 24 ]) );
rk_eta[0] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[0] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[8] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[16] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[24];
rk_eta[1] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[1] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[9] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[17] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[25];
rk_eta[2] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[2] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[10] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[18] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[26];
rk_eta[3] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[3] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[11] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[19] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[27];
rk_eta[4] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[4] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[12] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[20] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[28];
rk_eta[5] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[5] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[13] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[21] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[29];
rk_eta[6] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[6] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[14] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[22] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[30];
rk_eta[7] += + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[7] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[15] + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[23] + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[31];
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
error = 0;
return error;
}

