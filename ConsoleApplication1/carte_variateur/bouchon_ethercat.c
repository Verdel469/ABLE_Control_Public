/*
 * carte_variateur_V3_ethercat.c - 
 *
 * Copyright (C) 2012, HAPTION S.A.
 * Auteur : Pascal Louveau
 * Date de création : 04/01/2012
 *
 ********************************************************************/ 

#include "carte_variateur_V3_ethercat.h"


/*-------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Datas - lecture de la carte variateur
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Datas(ServoComEthercat * eth)
|
------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Datas(ServoComEthercat * eth)
{

}



/*-------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Modes - sélection du mode de commande
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Modes(ServoComEthercat * eth, int Mode_Variateur)
|
| Description --
|   Cette fonction permet de sélectionner le type de mode de commande.
|
------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Modes(ServoComEthercat * eth, int Mode_Variateur)
{

}


/*----------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Inhibitions - inhibition ou désinhibition
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Inhibitions(ServoComEthercat * eth, int * Inhibition_Moteur)
|
| Description --
|   Cette fonction permet d'inhiber ou de désinhiber de tous les moteurs.
|		0 => désinhibition
|		1 => inhibition
------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Inhibitions(ServoComEthercat * eth, int * Inhibition_Moteur)
{

}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Stor - positionnement des sorties TOR
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Stor(ServoComEthercat * eth, int Stor)
|
------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Stor(ServoComEthercat * eth, int Stor)
{

}


/*---------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Consignes_Vitesse - positionnement des consignes variateur
|
| Syntaxe --
|	void ETHERCAT_carte_variateur_V3_Consignes_Vitesse(ServoComEthercat* eth, 
|												  float* consignes_vitesse)
|
| Entrées --
|	consignes_vitesse -> consigne de vitesse en tours/s
|
-----------------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Consignes_Vitesse(ServoComEthercat * eth, float* consignes_vitesse)
{

}


/*---------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Consignes_Courant - positionnement des consignes variateur
|
| Syntaxe --
|	void ETHERCAT_carte_variateur_V3_Consignes_Courant(ServoComEthercat * eth, 
|													   float* consignes_courant)
|
-----------------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Consignes_Courant(ServoComEthercat * eth, float* consignes_courant)
{

}


/*---------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Consignes_Position - consignes position codeur et vitesse codeur
|
| Syntaxe --
|	void ETHERCAT_carte_variateur_V3_Consignes_Position(ServoComEthercat * eth, 
|												   float* consignes_position, 
|												   float* consignes_vitesse)
| Entrées --
|	consignes_position -> consigne de position moteur en tours
|	consignes_vitesse -> consigne de vitesse moteur en tours/s
|
-----------------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Consignes_Position(ServoComEthercat * eth, float* consignes_position, float* consignes_vitesse)
{

}


/*---------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Set_Motor_Parameters - positionnement des parametres moteurs
|
| Syntaxe --
|	void ETHERCAT_carte_variateur_V3_Set_Motor_Parameters(ServoComEthercat * eth, float * nb_points_codeur, double * Kconv_I)
|
-----------------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Set_Motor_Parameters(ServoComEthercat * eth, float * nb_points_codeur, double * Kconv_I)
{

}


/*---------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Set_K_B - positionnement des gains moteurs
|
| Syntaxe --
|	void ETHERCAT_carte_variateur_V3_Set_K_B(ServoComEthercat * eth)
|
-----------------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Set_K_B(ServoComEthercat * eth, double * Kp_P, double * Kd_P)
{

}


/*----------------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Set_Asserv_Parameters - positionnement des gains d'asservissement
|
| Syntaxe --
|	void ETHERCAT_carte_variateur_V3_Set_Asserv_Parameters(ServoComEthercat * eth)
|
------------------------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Set_Asserv_Parameters(ServoComEthercat * eth, double * Kp_V, double * Kp_I, double * Ki_I)
{

}


/*--------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Initialise_Master - ouverture du canal
|											
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Initialise_Master()
|
----------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Initialise_Master()
{
	return 0;
}


/*--------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Reset_Channel - reset de la carte maitre
|											
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Reset_Channel(ServoComEthercat * eth)
|
----------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Reset_Channel()
{
	return 0;
}


/*--------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Initialise - initialisation des structures
|											
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Initialise(ServoComEthercat * eth)
|
----------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Initialise(ServoComEthercat * eth)
{
	return 0;
}



/*--------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Reset_Toggle - remise à 1 du toggle, à 0 sur la carte
|											
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Reset_Toggle()
|
----------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Reset_Toggle(ServoComEthercat * eth)
{
	return 0;
}



/*--------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Close - fermeture du cannal
|											
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Close()
|
----------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Close()
{
	return 0;
}


/*-----------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Connexion - envoie d'une trame de connexion
|
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Connexion(ServoComEthercat * eth)
|
--------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Connexion(ServoComEthercat * eth)
{

	return (0);
}


/*-----------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Send_Consignes - envoie des consignes à la carte
|
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Send_Consignes(ServoComEthercat * eth)
|
--------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Send_Consignes(ServoComEthercat * eth)
{
	return (0);
}


/*----------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Send_Asserv_Parameters - envoie des gains d'asservissements
|
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Send_Asserv_Parameters(ServoComEthercat * eth)
|
------------------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Send_Asserv_Parameters(ServoComEthercat * eth)
{

	return (0);
}


/*----------------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Send_Motors_Parameters - envoie des parametres des moteurs
|
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Send_Motors_Parameters(ServoComEthercat * eth)
|
------------------------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Send_Motors_Parameters(ServoComEthercat * eth)
{

	return (0);
}


/*----------------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Send_Calibration - envoie de la structure calibration
|
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Send_Calibration(ServoComEthercat * eth)
|
------------------------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Send_Calibration(ServoComEthercat * eth)
{
	return (0);
}


/*----------------------------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Send_Request_ADC - envoie de la structure requete ADC courant
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Send_Request_ADC(ServoComEthercat * eth)
|
------------------------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Send_Request_ADC(ServoComEthercat * eth)
{
	return (0);
}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Receive_Identification - réception de l'identification
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Receive_Identification(ServoComEthercat * eth)
|
------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Receive_Identification(ServoComEthercat * eth)
{
	return 0;
}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Receive_ADC_Offset - réception de l'offset courant
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Receive_ADC_Offset(ServoComEthercat * eth)
|
------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Receive_ADC_Offset(ServoComEthercat * eth, float * adc_offset)
{
	return 0;
}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Receive - réception de données
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Receive(ServoComEthercat * eth)
|
------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Receive(ServoComEthercat * eth)
{
	return (0);
}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Set_Calibration - 
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Set_Calibration(ServoComEthercat * eth, long * position_codeur_init)
|
------------------------------------------------------------------------*/
void ETHERCAT_carte_variateur_V3_Set_Calibration(ServoComEthercat * eth, long * position_codeur_init)
{
}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Receive_State - réception d'une trame d'Etat
|
| Syntaxe --
|   int ETHERCAT_carte_variateur_V3_Receive_State(ServoComEthercat * eth)
|
------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Receive_State(ServoComEthercat * eth)
{
	return (0);
}


/*---------------------------------------------------------------------
| ETHERCAT_carte_variateur_V3_Get_Status - lecture des status
|
| Syntaxe --
|   void ETHERCAT_carte_variateur_V3_Get_Status(ServoComEthercat * eth)
|
------------------------------------------------------------------------*/
int ETHERCAT_carte_variateur_V3_Get_Status(ServoComEthercat * eth)
{
	return (0); 
}


