 /* MASTEUR_HPP_ */
/*
 * Master.hpp
 *
 *  Created on: Avril 16, 2018
 *      Author: julie Foucault
 */

#ifndef MASTER_HPP_
#define MASTER_HPP_
#include "Moteur.hpp"
#include <iostream>
#include <softPwm.h>
class Master :public Moteur
{
public:
    /**
         * @brief Master : constructeur de master
         * @param ahi : numero de pin voulu pour ahi
         * @param ali : numero de pin voulu pour ali
         * @param bhi : numero de pin voulu pour bhi
         * @param bli : numero de pin voulu pour bli
         * @param dis : numero de pin voulu pour dis
         */
        Master(int ahi, int ali, int bhi, int bli, int dis);
        /**
         * @brief ~Master : destructeur de master
         */
        virtual ~Master();
        /**
         * @brief avance : méthode qui fait avancer le moteur master
         * @param pwm : vitesse du pwm (0 à 100)
         */
        virtual void avance(int pwm);
        /**
         * @brief recule : méthode qui fait reculer le moteur master
         * @param pwm : vitesse du pwm (0 à 100)
         */
        virtual void recule(int pwm);
        /**
         * @brief Move : méthode qui fait avancer ou recule le moteur master ( utilisé dans le pid)
         * @param pwm : vitesse du pwm et direction du moteur (-100 à 0 reculer, 0 à 100 avancer)
         */
        virtual void Move(int pwm);
        /**
         * @brief arrete : arrète le moteur master
         */
        virtual void arrete();
        /**
         * @brief arreteS : arrète le moteur slave
         */
        virtual void arreteS();

private:
        /**
         * @brief sens : variable utilisé pour garder en mémoire le dernié sens de rotation
         */
        bool sens;





};

#endif /* MASTEUR_HPP_ */
