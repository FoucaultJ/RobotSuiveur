/*
 * Master.cpp
 *
 *  Created on: Avril 16, 2018
 *      Author: julie foucault
 */

#include "Master.hpp"
#include <wiringPi.h>
#include "screen.hpp"

Master::Master(int ahi, int ali, int bhi, int bli, int dis) :Moteur(ahi,ali,bhi,bli,dis){

sens=0;

}

Master::~Master() {
	// TODO Auto-generated destructor stub
}

void Master::avance(int pwm) {
	if(pwm<0)pwm=0;
	digitalWrite(ALIpin,LOW); //
	digitalWrite(BLIpin,HIGH);
	//digitalWrite(BHIpin,LOW);
	softPwmWrite(BHIpin,0);
	//screen->dispStr(10,4,"J'avance");
	digitalWrite(DISpin,LOW); //disable
	softPwmWrite(AHIpin,pwm);

}

void Master::recule(int pwm) {


	digitalWrite(BLIpin,LOW);
	digitalWrite(ALIpin,HIGH);
	//digitalWrite(AHIpin,LOW);
	softPwmWrite(AHIpin,0);
	digitalWrite(DISpin,LOW); //disable
	softPwmWrite(BHIpin,pwm);
}

void Master::Move(int pwm) {


	if(pwm>0)
	{
		if(sens)this->avance(pwm);
		else
		{
			this->arrete();
			this->arreteS();
			usleep(150000);
			this->avance(pwm);
		}
		sens=1;
	}
	else
	{
		pwm=pwm*-1;
		if(!sens)this->recule(pwm);
		else
		{
			this->arrete();
			this->arreteS();
			usleep(150000);
			this->recule(pwm);
		}
		sens=0;
	}
}



void Master::arrete(void) {
	digitalWrite(ALIpin,LOW);
	digitalWrite(BLIpin,LOW);
	digitalWrite(BHIpin,LOW);
	digitalWrite(AHIpin,LOW);
	digitalWrite(DISpin,HIGH); //disable
}

void Master::arreteS() {
	digitalWrite(23,LOW);
	digitalWrite(18,LOW);
	digitalWrite(14,LOW);
	digitalWrite(15,LOW);
	digitalWrite(DISpin,HIGH); //disable

}
