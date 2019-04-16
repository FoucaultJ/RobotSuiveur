/*
 * Slave.cpp
 *
 *  Created on: Avril 16, 2018
 *      Author: gbergeron
 */

#include "Slave.hpp"
#include <wiringPi.h>
#include "screen.hpp"
#include <iostream>
using namespace std;

Slave::Slave(int ahi, int ali, int bhi, int bli, int dis) :Moteur(ahi,ali,bhi,bli,dis){

sens=0;
}

Slave::~Slave() {
	// TODO Auto-generated destructor stub
}


void Slave::avance(int pwm) {
	if(pwm<0)pwm=0;
	/*if(pwmSens)
	{
	    softPwmCreate(BHIpin,50,100); //1/100hz =range en us, range/2= valeur
		pwmSens=1;
		cout<<"allo"<<endl;
	}*/
	softPwmWrite(AHIpin,0);

	digitalWrite(BLIpin,LOW);
	digitalWrite(ALIpin,HIGH);
	digitalWrite(DISpin,LOW); //disable
	softPwmWrite(BHIpin,pwm);
}

void Slave::recule(int pwm) {
	/*if(!pwmSens)
	{
		softPwmCreate(AHIpin,50,100); //1/100hz =range en us, range/2= valeur
		pwmSens=0;
		cout<<"allo2"<<endl;

	}*/
	softPwmWrite(BHIpin,0);

	digitalWrite(ALIpin,LOW); //
	digitalWrite(BLIpin,HIGH);
	digitalWrite(DISpin,LOW); //disable
	softPwmWrite(AHIpin,pwm);
}

void Slave::Move(int pwm) {
	if(pwm>0)
		{
			if(sens)this->avance(pwm);
			else
			{
				this->arrete();
				this->arreteM();
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
				this->arreteM();
				usleep(150000);
				this->recule(pwm);
			}
			sens=0;
		}
}


void Slave::arrete(void) {
	digitalWrite(ALIpin,LOW);
	digitalWrite(BLIpin,LOW);
	digitalWrite(BHIpin,LOW);
	digitalWrite(AHIpin,LOW);
	digitalWrite(DISpin,HIGH); //disable
}

void Slave::arreteM() {
	digitalWrite(17,LOW);
	digitalWrite(27,LOW);
	digitalWrite(0,LOW);
	digitalWrite(22,LOW);
	digitalWrite(DISpin,HIGH); //disable
}
