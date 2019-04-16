/*
 * Moteur.cpp
 *
 *  Created on: Avril 16, 2018
 *      Author: Julie Foucault
 */

#include "Moteur.hpp"
#include <wiringPi.h>
#include "screen.hpp"


Moteur::Moteur(int ahi, int ali, int bhi, int bli, int dis) :AHIpin(ahi),ALIpin(ali),BHIpin(bhi),BLIpin(bli),
		DISpin(dis){

	AHI=0;
	ALI =0;
	BHI =0;
	BLI=0;
	DIS=1;

}

Moteur::~Moteur() {
	// TODO Auto-generated destructor stub
}


void Moteur::arrete(void) {
	digitalWrite(ALIpin,LOW);
	digitalWrite(BLIpin,LOW);
	digitalWrite(BHIpin,LOW);
	digitalWrite(AHIpin,LOW);
	digitalWrite(DISpin,HIGH); //disable
}
