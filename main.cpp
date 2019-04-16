//============================================================================
// Name        : TemplateThreadCom.cpp
// Author      : SG
// Version     :
// Copyright   : Your copyright notice
// Description : Template Thread Com
//============================================================================

#define  AHI1 17
#define  ALI1 27
#define  BHI1 0
#define  BLI1 22

#define  AHI2 23
#define  ALI2 18
#define  BHI2 14
#define  BLI2 15
#define  DIS1 4
#include "clavier.hpp"
#include "screen.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <iostream>
#include <fstream>
#include "taskM.hpp"
#include <ctime>
#include <time.h>
#include <iostream>
#include "Hsv.hpp"
#include "Sonar.hpp"
#include "DataBase.hpp"
#include <wiringPi.h>
#include "Moteur.hpp"
#include "Master.hpp"
#include "Slave.hpp"
#include "Robot.hpp"

using namespace std;
/**
 * @brief startTime :  store le temps de départ de l'encodeur master
 */
long startTime;
/**
 * @brief final : store la vitesse de rotation du moteur master en us
 */
long final;
/**
 * @brief endTime :  store le temps final de l'encodeur master
 */
long endTime;
/**
 * @brief startTime2 :  store le temps de départ de l'encodeur slave
 */
long startTime2;
/**
 * @brief final2 :  store la vitesse de rotation du moteur slave en us
 */
long final2;
/**
 * @brief endTime2 :  store le temps final de l'encodeur slave
 */
long endTime2;
/**
 * @brief buffer : utilisé par screen pour afficher des variables
 */
char buffer[10];
/**
 * @brief X : coordonné de l'axe des X de l'objet
 */
int X;
/**
 * @brief Y : coordonné de l'axe des y de l'objet
 */
int Y;

/**
 * @brief cote : détermine si les capteurs ultrasons ont détecté un mur à gauche ou à droite selon la valeur
 */
bool cote=0;
/**
 * @brief  : valeur du capteur infrarouge
 */
float infra;
/**
 * @brief value : store la valeur taper au clavier
 */
float value=0;
/**
 * @brief modeM : détermine si le robot utilisera ces moteur (1 = oui 0 = non) choisi lors du lancement du programme
 */
bool modeM;
/**
 * @brief capteur1 : valeur de capteur ultrason 1
 */
float capteur1;
/**
 * @brief capteur2 : valeur de capteur ultrason 2
 */
float capteur2;
/**
 * @brief capteur3 : valeur de capteur ultrason 3
 */
float capteur3;
/**
 * @brief capteur4 : valeur de capteur ultrason 4
 */
float capteur4;
/**
 * @brief capteur5 : valeur de capteur ultrason 5
 */
float capteur5;
/**
 * @brief capteur6 : valeur de capteur ultrason 6
 */
float capteur6;
/**
 * @brief erreur : nombre de pixel entre le centre de l'image et de la position en X de l'objet
 */
float erreur;
/**
 * @brief integral : valeur de l'integral calculé dans le pid (en mode suiveur)
 */
float integral=0;
/**
 * @brief Kp : constante P pour le calcule du pid  (en mode suiveur)
 */
float Kp=0.15;
/**
 * @brief Ki : constante I pour le calcule du pid (en mode suiveur)
 */
float Ki =0.01;
/**
 * @brief commande : valeur calculé du pid  pour les moteurs (en mode suiveur)
 */
float commande;
/**
 * @brief limiteH : valeur de X maximale pour débuter le calcul intégral lors du pid (en mode suiveur)
 */
int limiteH=350;
/**
 * @brief limiteB : valeur de X minimale pour débuter le calcul intégral lors du pid (en mode suiveur)
 */
int limiteB=290;
/**
 * @brief integrals : valeur de l'integral calculé dans le pid (en mode recharge)
 */
float integrals=0;
/**
 * @brief commandes : valeur calculé du pid pour les moteurs (en mode recharge)
 */
float commandes=0;
/**
 * @brief Kps : constante P pour le calcule du pid  (en mode recharge)
 */
float Kps=0.065;
/**
 * @brief Kis  : constante I pour le calcule du pid (en mode recharge)
 */
float Kis =0.005;
/**
 * @brief limiteHs : valeur de X maximale pour débuter le calcul intégral lors du pid (en mode recharge)
 */
int limiteHs=360;
/**
 * @brief limiteBs :  valeur de X minimale pour débuter le calcul intégral lors du pid (en mode recharge)
 */
int limiteBs=280;
/**
 * @brief sens : mémoire du dernier type de mouvement des moteurs utilisés pour implementer des délais
 */
int sens=0;
/**
 * @brief modeR : mode du robot (1 = suiveur, 0 = recharge, 2 = stop)
 */
int modeR=1;
/**
 * @brief couleur : couleur de l'objet suivi (0 = turquoise, 1 = orange, 2 = jaune, 3 = majenta)
 */
int couleur=0;
/**
 * @brief file : fichier utiliser pour mieux visualiser les valeurs des encodeurs ou du courant
 */
ofstream file;
/**
 * @brief The m_state enum
 */
enum m_state{ //enum type
    cherche,
    suivre,
    analyse,
    recule,
    avancer,
    recharge,
    suiveur
}state;//variable
/**
 * @brief encodeur : fonction qui calcule la vitesse de rotation du moteur master (appeler à tous les fronts montant de la pin des encodeurs)
 */
void encodeur(void){
    endTime=micros();
    final=endTime-startTime;
    startTime=micros();
    DataBase::getDb()->setEncM(final);
    if(file.is_open()){
        file<<final<<"= master"<<endl;
    }

}
/**
 * @brief encodeur2 : fonction qui calcule la vitesse de rotation du moteur slave (appeler à tous les fronts montant de la pin des encodeurs)
 */
void encodeur2(void){
    endTime2=micros();
    final2=endTime2-startTime2;
    startTime2=micros();
    DataBase::getDb()->setEncS(final2);
    if(file.is_open()){
            file<<final2<<"= slave"<<endl;
        }

}
/**
 * @brief pidObjet : fonction qui calcul le pid des moteur (en mode suiveur)
 * @param X position de l'objet dans la caméra en pixel (0 à 640)
 */
void pidObjet(int X)
{
    erreur=X-320;
    commande=erreur*Kp;
    if(X<limiteH and X>limiteB)
    {
        integral+=erreur*Ki;
    }
    else integral=0;
    commande+=integral;
}
/**
 * @brief pidStation : fonction qui calcul le pid des moteur (en mode recharge)
 * @param X position de l'objet dans la caméra en pixel (0 à 640)
 */
void pidStation(int X)
{
    erreur=X-320;
    commandes=erreur*Kps;
    if(X<limiteHs and X>limiteBs)
    {
        integrals+=erreur*Kis;
    }
    else integrals=0;
    commandes+=integrals;
}
int main(int argc, char **argv)
{
    //Setup Moteur**************************************************************************
     if (wiringPiSetupGpio() == -1)
            return -1;
    char car;
    //création des deux moteurs et assignations des bonnes pins
    Master *master= new Master(AHI1, ALI1, BHI1, BLI1, DIS1);
    Slave  *slave= new Slave(AHI2, ALI2, BHI2, BLI2, DIS1);
    pinMode(AHI1,PWM_OUTPUT);
    pinMode(BHI2,PWM_OUTPUT);
    pinMode(ALI1,OUTPUT);
    pinMode(BLI1,OUTPUT);
    pinMode(BHI1,PWM_OUTPUT);
    pinMode(DIS1,OUTPUT);
    pinMode(ALI2,OUTPUT);
    pinMode(BLI2,OUTPUT);
    pinMode(AHI2,PWM_OUTPUT);
    softPwmCreate(AHI1,50,100); //1/100hz =range en us, range/2= valeur
    softPwmCreate(BHI2,50,100); //1/100hz =range en us, range/2= valeur
    softPwmCreate(AHI2,50,100); //1/100hz =range en us, range/2= valeur
    softPwmCreate(BHI1,50,100); //1/100hz =range en us, range/2= valeur
    wiringPiISR (2,INT_EDGE_FALLING,&encodeur); //à chaque front montant, la fonction se déclenche
    wiringPiISR (3,INT_EDGE_FALLING,&encodeur2); //à chaque front montant, la fonction se déclenche

    file.open("encodeur.txt");
    //création des objets des vues
    TTaskM *taskM;
    Sonar* sonar;
    Robot*robot;
    Hsv * hsv;
  // Initialisation task Principal
    TThread::initTaskMain();

    // Création Clavier et console
    clavier = new TClavier();
    screen  = new TScreen();
    screen->start();
    screen->dispStr(1,2,"set up fait");
    //setup des différentes vues
    sonar =new Sonar("Sonar 1",NULL,SCHED_RR,98,TThread::DESTRUCTION_ASYNCHRONE,0);
    hsv = new Hsv ("Hsv",NULL,SCHED_RR,99,TThread::DESTRUCTION_ASYNCHRONE,3);
    taskM=new TTaskM("Mqtt",NULL,SCHED_RR,97,TThread::DESTRUCTION_ASYNCHRONE,2);
    robot=new Robot("robot",NULL,SCHED_FIFO,90,TThread::DESTRUCTION_ASYNCHRONE,1);
    //mode display(0 = off, 1 = on)
    screen->dispStr(1,0,"Select display mode : 0 OFF, 1 ON");
    while(!clavier->kbhit());
    car = clavier->getch();
    if(car=='1') hsv->setMode(1);
    else if(car=='0')  hsv->setMode(0);

    //mode moteur (0 = off, 1 = on)
    screen->dispStr(1,1,"Select Moteur mode : 0 OFF, 1 ON");
    while(!clavier->kbhit());
    car = clavier->getch();
    if(car=='1') modeM=1;
    else if(car=='0')  modeM=0;
    //lancement des différentes task
    sonar->start();
    hsv->start();
    taskM->start();
    robot->start();
    master->arrete();
    slave->arrete();
    //vérification du mode courant et application des modifications s'il y a changement.
    do
    {
        couleur=DataBase::getDb()->getCouleur();
        if(hsv->getCouleurN()!=couleur)hsv->setCouleur(couleur);
        if(modeR!=DataBase::getDb()->getModeR())
        {
            modeR=DataBase::getDb()->getModeR();
            if(modeR==0)
            {
                master->arrete();
                slave->arrete();
                usleep(200000);
                state=cherche;
            }
            else if(modeR==1)
            {
                master->arrete();
                slave->arrete();
                usleep(200000);
                state=suiveur;
            }
        }
        if(clavier->kbhit())
            //configuration des commandes par clavier pour modifier les valeurs des constantes dans le pwm
          {
              car=' ';
             car = clavier->getch();

              if(car=='i')
              {
                 cin>>value;
                 Kis=value;
                 value=0;

              }
              else if (car=='p')
              {
                  cin>>value;
                  Kps=value;
                  value=0;
              }
              else if(car=='h')
              {
                 cin>>value;
                 limiteHs=value;
                 value=0;

              }
              else if (car=='b')
              {
                  cin>>value;
                  limiteBs=value;
                  value=0;
              }
              else if(car=='m')
              {
                  cin>>value;
                  DataBase::getDb()->setModeR(value);
                  value=0;
              }
              else if(car=='c')
              {
                  cin>>value;
                  hsv->setCouleur(value);
                  value=0;
              }




          }

            infra= DataBase::getDb()->getMCapteurInfra();
            //affichage des données
        X=DataBase::getDb()->getX();
        Y=DataBase::getDb()->getY();
        sprintf(buffer,"%.2f",infra);
        screen->dispStr(30,10,buffer);
        screen->dispStr(20,10,"infra=");
        sprintf(buffer,"%d",limiteHs);
        screen->dispStr(30,11,buffer);
        screen->dispStr(20,11,"limiteH=s");
        sprintf(buffer,"%d",limiteBs);
        screen->dispStr(30,12,buffer);
        screen->dispStr(20,12,"limiteBs=");
        sprintf(buffer,"%.6f",Kis);
        screen->dispStr(30,13,buffer);
        sprintf(buffer,"%.2f",Kps);
        screen->dispStr(30,14,buffer);
        screen->dispStr(25,14,"Kps=");
        sprintf(buffer,"%.5f",integrals);
        screen->dispStr(40,15,buffer);
        screen->dispStr(25,15,"integrale=");
        screen->dispStr(25,13,"Kis=");
        sprintf(buffer,"%d",sens);
        screen->dispStr(30,16,buffer);
        screen->dispStr(25,16,"sens=");
        sprintf(buffer,"%.2ld",final);
        screen->dispStr(30,18,buffer);
        screen->dispStr(22,17,"master=");
        sprintf(buffer,"%.2ld=8",final2);
        screen->dispStr(30,19,buffer);
        screen->dispStr(22,19,"slave=");
        sprintf(buffer,"%f",DataBase::getDb()->getAmpM());
        screen->dispStr(40,21,buffer);
        screen->dispStr(30,21,"courant master=");
        //storage local de données importantes pour les mouvements du robot
        X=DataBase::getDb()->getX();
        Y=DataBase::getDb()->getY();
        capteur1= DataBase::getDb()->getMCapteur(0);
        capteur2= DataBase::getDb()->getMCapteur(1);
        capteur3= DataBase::getDb()->getMCapteur(2);
        capteur4= DataBase::getDb()->getMCapteur(3);
        capteur5= DataBase::getDb()->getMCapteur(4);
        capteur6= DataBase::getDb()->getMCapteur(5);
        infra=DataBase::getDb()->getMCapteurInfra();
            if(modeM)
            {
            	if(modeR!=2)
            	{
					switch(state){
					//recherche de l'objet
					case cherche:

						if(DataBase::getDb()->isObjet() and X>280 and X<360)
						{
							state=suivre;
						}
						else
							state=cherche;
						if(sens!=1)
							{
							master->arrete();
							slave->arrete();
							usleep(200000);
							}
						if(cote)
						{
							master->avance(10);
							slave->recule(10);

						}
						else
						{
							master->recule(10);
							slave->avance(10);
						}

							screen->dispStr(30,4,"TABARNAK ou est l'objet");
							screen->dispStr(30,7,"-----------------------");
							screen->dispStr(30,5,"-----------------------");
							screen->dispStr(30,6,"-----------------------");
						sens=1;

						break;
						//le robot se dirige vers l'objet en utilisant la caméra pour se centrer
					case suivre:
						if(sens!=2)
						{
							master->arrete();
							slave->arrete();
							usleep(150000);
						//	master->avance(10);
							//slave->avance(10);
							//usleep(700000);
						}
						pidStation(X);
					/*	if(commandes>29)commandes=29;
						if(commandes<-29)commandes=-29;
						master->Move((15-commandes));
						slave->Move((15+commandes));*/
						master->avance(20-commandes);
						slave->avance(20+commandes);
				        sprintf(buffer,"%.2f",commandes);
						screen->dispStr(40,10,buffer);
						screen->dispStr(30,10,"commande=");
						screen->dispStr(40,11,buffer);
						screen->dispStr(30,11,"      x=");
						screen->dispStr(30,4,"-----------------------");
						screen->dispStr(30,7,"-----------------------");
						screen->dispStr(30,6,"-----------------------");
						screen->dispStr(30,5,"trouver!               ");
						screen->dispStr(30,8,"-----------------------");
						if(!DataBase::getDb()->isObjet()){
							state=cherche;
						}
						else if(capteur2<10 or infra<10){
							state =analyse;
						}
						else
							state=suivre;
						sens=2;
						break;
							//le robot s'immobilise et vérifie s'il capte un mur à gauche et à droite
					case analyse:
							master->arrete();
							slave->arrete();
							sleep(1);
							screen->dispStr(30,4,"-----------------------");
							screen->dispStr(30,5,"-----------------------");
							screen->dispStr(30,7,"-----------------------");
							screen->dispStr(30,6,"analyse de la situation");
							screen->dispStr(30,8,"-----------------------");
							if(capteur3> 60 && capteur1 <50)
							{
								cote=1;
								state=recule;
							}
							else if(capteur3 <50 && capteur1>60)
							{
								cote=0;
								state=recule;
							}
							else
								state=recharge;
							sens=0;
							break;
					//le robot recule et tourne dans la direction opposé au mur capter
					case recule:
						screen->dispStr(30,7,"recule  ");
						master->recule(12);
						slave->recule(10);
						sleep(4);
						master->arrete();
						slave->arrete();
						usleep(200000);

						sprintf(buffer,"%d",cote);
						screen->dispStr(30,3,buffer);
						if(cote){
							master->recule(15);
							slave->avance(20);
							screen->dispStr(30,4,"-----------------------");
							screen->dispStr(30,5,"-----------------------");
							screen->dispStr(30,6,"-----------------------");
							screen->dispStr(30,7,"droite                ");
							screen->dispStr(30,8,"-----------------------");

							usleep(700000);
						}
						else
						{
							screen->dispStr(30,4,"-----------------------");
							screen->dispStr(30,5,"-----------------------");
							screen->dispStr(30,6,"-----------------------");
							screen->dispStr(30,7,"gauche                 ");
							screen->dispStr(30,8,"-----------------------");
							master->avance(20);
							slave->recule(15);

							usleep(700000);
						}

						state=avancer;

						master->arrete();
						slave->arrete();
						usleep(200000);
						sens=3;
						break;
					//le robot avance durant 1 seconde parralèlement au mur
					case avancer:
						master->avance(15);
						slave->avance(14);
						sleep(1);
						master->arrete();
						slave->arrete();
						usleep(125000);
						screen->dispStr(30,4,"-----------------------");
						screen->dispStr(30,5,"-----------------------");
						screen->dispStr(30,6,"-----------------------");
						screen->dispStr(30,8,"-----------------------");
						screen->dispStr(30,7,"reajuste               ");
						state=cherche;
						sens=4;
						break;
					//le robot s'avance et vérifie s'il est centrer avec l'objet puis recule s'il ne l'est pas.
					case recharge:
						screen->dispStr(30,8,"recharge time!!!");
						if(sens!=5)
						{
							slave->avance(10);

							master->avance(12);
							sleep(2);
							master->arrete();
							slave->arrete();
							usleep(200000);
						}
						if(X<305 or X>335)
						{
							master->recule(16);
							slave->recule(15);
							sleep(5);
							state=cherche;
						}
						master->arrete();
						slave->arrete();
						sens=5;
						break;
						//le robot suit l'objet constamment en utilisant les capteurs ultrasons pour éviter de rentrer
						//dans les obstacles

					case suiveur:
						screen->dispStr(30,4,"-----------------------");
						screen->dispStr(30,5,"-----------------------");
						screen->dispStr(30,7,"-----------------------");
						screen->dispStr(30,6,"-----------------------");
						screen->dispStr(30,8,"Suiveur");
						if(DataBase::getDb()->isObjet()&&DataBase::getDb()->getArea()<4000000 and infra>40)
						{
							if(sens!=1 and   sens!=0)
							{
								master->arrete();
								slave->arrete();
								usleep(125000);
								pidObjet(X);
								if(capteur1<20)
								{
									commande=-15;
								}
								if(capteur3<20)
								{
									commande=15;
								}
								master->Move((50-commande));//gauche
								slave->Move((50+commande));//droite
							}
							else
							{
								pidObjet(X);
								if(capteur1<20)
								{
									commande=-15;
								}
								if(capteur3<20)
								{
									commande=15;
								}
								master->Move((50-commande));//gauche
								slave->Move((50+commande));//droite
							}
							sens=1;
						}
						else if(infra<20 and DataBase::getDb()->isObjet() and capteur4>25 and capteur5>20 and capteur6>25)
						{
							if(sens!=2 and sens!=0)
							{
								master->arrete();
								slave->arrete();
								usleep(125000);
								master->recule(10);
								slave->recule(10);
							}
							else
							{
								master->recule(10);
								slave->recule(10);
							}

							sens=2;
						}
						else
						{
							master->arrete();
							slave->arrete();
						}
						if(!DataBase::getDb()->isObjet())
						{
							sprintf(buffer,"%d",sens);
							screen->dispStr(30,17,buffer);
							screen->dispStr(24,17,"sens=");
							if(X>320)
							{
								if(sens!=3 )
								{
									master->arrete();
									slave->arrete();
									usleep(125000);
									master->recule(15);
									slave->avance(15);
								}
								else
								{
									master->recule(15);
									slave->avance(15);
								}

								sens=3;

							}
							else if(X<321)
							{
								if(sens!=4 )
								{
									master->arrete();
									slave->arrete();
									usleep(125000);
									master->avance(15);
									slave->recule(15);
								}
								else
								{
									master->avance(15);
									slave->recule(15);
								}
								sens=4;
							}
						}
						break;
				}
            }
            	else
            	{
            		master->arrete();
            		slave->arrete();
            	}
        }

        usleep(25000);

      }
    while( (car != 'q') && (car != 'Q') );
    // Destruction tâches
    master->arrete();
     slave->arrete();
    if(clavier)
    delete clavier;
    if(screen)
    delete screen;
    if(sonar)
    delete sonar;
    if(hsv)
    delete hsv;
    if(taskM)
    delete taskM;
    if(robot)
    delete robot;
    return 0;
    }

