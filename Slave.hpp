/*
 * Slave.hpp
 *
 *  Created on: Avril 16, 2018
 *      Author: Julie Foucault
 */

#ifndef SLAVE_HPP_
#define SLAVE_HPP_
#include "Moteur.hpp"
#include <iostream>
#include <softPwm.h>

class Slave : public Moteur
{
public:
/**
     * @brief Slave : constructeur de slave
     * @param ahi : numero de pin voulu pour ahi
     * @param ali : numero de pin voulu pour ali
     * @param bhi : numero de pin voulu pour bhi
     * @param bli : numero de pin voulu pour bli
     * @param dis : numero de pin voulu pour dis
     */
    Slave(int ahi, int ali, int bhi, int bli, int dis);
    /**
     * @brief ~Slave : destructeur de slave
     */
    virtual ~Slave();
    /**
     * @brief avance : méthode qui fait avancer le moteur slave
     * @param pwm : vitesse du pwm (0 à 100)
     */
    virtual void avance(int pwm);
    /**
     * @brief recule : méthode qui fait reculer le moteur slave
     * @param pwm : vitesse du pwm (0 à 100)
     */
    virtual void recule(int pwm);
    /**
     * @brief Move : méthode qui fait avancer ou recule le moteur slave ( utilisé dans le pid)
     * @param pwm : vitesse du pwm et direction du moteur (-100 à 0 reculer, 0 à 100 avancer)
     */
    virtual void Move(int pwm);
    /**
     * @brief arrete : arrète le moteur master
     */
    void arreteM();
    /**
     * @brief arreteS : arrète le moteur slave
     */
    virtual void arrete();

private:
    /**
     * @brief sens : variable utilisé pour garder en mémoire le dernier sens de rotation
     */
    bool sens;


};

#endif /* MOTEUR_HPP_ */
