/*
 * Moteur.hpp
 *
 *  Created on: Avril 16, 2018
 *      Author: Julie Foucault
 */

#ifndef MOTEUR_HPP_
#define MOTEUR_HPP_

#include <iostream>
#include <softPwm.h>
class Moteur
{
public:
    /**
    * @brief Master : constructeur de moteur
    * @param ahi : numero de pin voulu pour ahi
    * @param ali : numero de pin voulu pour ali
    * @param bhi : numero de pin voulu pour bhi
    * @param bli : numero de pin voulu pour bli
    * @param dis : numero de pin voulu pour dis
    */
    Moteur(int ahi, int ali, int bhi, int bli, int dis);
    /**
     * @brief ~Moteur : destructeur de moteur
     */
    virtual ~Moteur();
    /**
     * @brief avance : méthode qui fait avancer moteur
     * @param pwm : vitesse du pwm
     */
    virtual void avance(int pwm)=0;
    /**
     * @brief recule : méthode qui fait reculer le moteur
     * @param pwm : vitesse du pwm
     */
    virtual void recule(int pwm)=0;
    /**
     * @brief Move : méthode qui fait avancer ou recule le moteur ( utilisé dans le pid)
     * @param pwm : vitesse du moteur ( -100 à 0 recule, 0 à 100 avancer)
     */
    virtual void Move(int pwm)=0;
    /**
     * @brief arrete : méthode qui arrète le moteur
     */
    virtual void arrete(void);



protected:
    /**
     * @brief AHI, ALI,BHI,BLI,DIS : variables utilisées par les autres classes pour assigner un numero de pin lors de la création d'un moteur
     */
    int AHI, ALI,BHI,BLI,DIS;//constructeur
    /**
     * @brief AHIpin, ALIpin, BHIpin,BLIpin,DISpin : variables utilisées par les autres classes
     *  pour garder le numero de pin assigné lors de la création d'Un moteur
     */
    int AHIpin, ALIpin, BHIpin,BLIpin,DISpin;//constructeur avec valeur en surcharge



};

#endif /* MOTEUR_HPP_ */
