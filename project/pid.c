#include <stdio.h>
#include <stdint.h>
#include "initialise_functions.h"



    void PID(double* E, double*sp, double* pv, double Kp, double Ki, double Kd,int* maxstep,double* pulse){
        *E = *sp - *pv;
        double PID,P,D,dE,dt;
        static double I = 0;
        static double prevE = 0;
        static double prevTime = 0;

        uint64_t currTime = time_us_64();

        dt = (currTime-prevTime)/1000000.0;
        if (dt == 0){dt = 0.0000001;}
        dE = *E - prevE;

        P = Kp*(*E);
        I += Ki*(*E)*dt;
        D = dE/dt;  
        PID = P + I + Kd*D;
        
        if(PID>*maxstep){
            PID = *maxstep;}
        if(PID<-(*maxstep)){
            PID= -(*maxstep);}

        if(*E>0){
            *pulse += PID;
            if(*pulse>=125){
                *pulse = 125;
            }
        }
        if(*E<0){
            *pulse -=  PID;
            if(*pulse<=5){
                *pulse = 5;
            }
        }

        prevTime = currTime;
        prevE = *E;
    }
