//
// Created by Riccardo Donà on 10/28/21.
//
#include <stdio.h>
#include "lib_models.h"

int main(){
    double t1, t2, t3, t4;

    t1 = calcTimeST_delta(20., 3.5);
    t2 = calcTimeST_delta(30., 3.5);
    t3 = calcTimeST_jerk(20., 3.5);
    t4 = calcTimeST_jerk(30., 3.5);

    printf("T1  =  %.3e (s)\n", t1 );
    printf("T2  =  %.3e (s)\n", t2 );
    printf("T3  =  %.3e (s)\n", t3 );
    printf("T4  =  %.3e (s)\n", t4 );

    return 0;
}
