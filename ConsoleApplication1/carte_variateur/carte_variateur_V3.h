/***********************************************************************************************************************
* carte_variateur_V3.h -
*
* Copyright (C) 2010, HAPTION
* Auteur : Pascal Louveau
* Date de création : 08/01/2010
*
***********************************************************************************************************************/

#ifndef _CARTE_VARIATEUR_V3_H_
#define _CARTE_VARIATEUR_V3_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "eth.h"


/* Ensemble des donnees utilisées pour la communication avec la carte variateur */
typedef struct {
	
	Struct_Asservissement_Message asservissement_message;
	Struct_Id_Message id_message;
	Struct_Etat_Message etat_message;
	Struct_Consigne_Message consigne_message;
	Struct_Calibration_Message calibration_message;
	Struct_Connexion_Message connexion_message;
	Struct_Parametrage_Message parametrage_message;
	Struct_Requete_adc_courant_Message requete_adc_courant_message;
	Struct_Offset_Courant_Message offset_courant_message;
	Struct_Parametrage_Reseau parametrage_reseau_message;
	Struct_Requete_Lecture_Parametrage_Reseau requete_lecture_reseau_message;
	SOCKET sock;
	struct sockaddr_in service;
	struct sockaddr_in recvAddr;
	BYTE recvBuffer[1024];
	struct timeval timeout;
	fd_set rfds;
	int nb_moteurs;
	Rec_Moteur moteur[NB_MOTEURS_V3B];
	int entree_tor;
	int etat_variateur;
	int presence_48v;
	int arret_urgence;
	int watchdog;
	int panne_codeur[NB_MOTEURS_V3B];
	int panne_courant[NB_MOTEURS_V3B];
	int erreur_pwm[NB_MOTEURS_V3B];
	int erreur_checksum_piccolo[NB_MOTEURS_V3B];
	int erreur_checksum_controleur[NB_MOTEURS_V3B];
	int etat_inhibition[NB_MOTEURS_V3B];
	int i2t[NB_MOTEURS_V3B];
	int pwm_surchauffe[NB_MOTEURS_V3B];
	int type_device;
	int numero_serie;
	int etat_calibration;
	int status;
	int panne_relais;
	int panne_homme_mort;
	int activation_checksum;
	unsigned int checksum_recu;
	unsigned int checksum_calcule;
	int compteur_checksum[NB_MOTEURS_V3B];
	int erreur_fatale_checksum;
	int demande_inhibition[NB_MOTEURS_V3B];
	int activation_complement_a_un;

} ServoComEth;



extern int ETH_carte_variateur_V3_Initialise(ServoComEth * eth, const char* dest_ip, int local_port);
extern int ETH_carte_variateur_V3_Close(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Connexion(ServoComEth * eth);
extern void ETH_carte_variateur_V3_Datas(ServoComEth * eth);
extern void ETH_carte_variateur_V3_Modes(ServoComEth * eth, int Mode_Variateur);
extern void ETH_carte_variateur_V3_Set_K_B(ServoComEth * eth, double * Kp_P, double * Kd_P);
extern void ETH_carte_variateur_V3_Set_Asserv_Parameters(ServoComEth * eth, double * Kp_V, double * Kp_I, double * Ki_I);
extern void ETH_carte_variateur_V3_Set_Calibration(ServoComEth * eth, long * position_codeur_init);
extern void ETH_carte_variateur_V3_Set_Motor_Parameters(ServoComEth * eth, float * nb_points_codeur, double * Kconv_I);
extern void ETH_carte_variateur_V3_Inhibitions(ServoComEth * eth, int Inhibition_Moteur);
extern void ETH_carte_variateur_V3_Inhibitions2(ServoComEth * eth, int* Inhibition_Moteur);
extern void ETH_carte_variateur_V3_Stor(ServoComEth * eth, int Stor);
extern void ETH_carte_variateur_V3_Consignes_Vitesse(ServoComEth * eth, float* consignes_vitesse);
extern void ETH_carte_variateur_V3_Consignes_Courant(ServoComEth * eth, float* consignes_courant);
extern void ETH_carte_variateur_V3_Consignes_Position(ServoComEth * eth, float* consignes_position, float* consignes_vitesse);
extern int ETH_carte_variateur_V3_Send_Consignes(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Send_Asserv_Parameters(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Send_Calibration(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Send_Motors_Parameters(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Send_Request_ADC(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Receive_Identification(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Receive_ADC_Offset(ServoComEth * eth, float * adc_offset);
extern int ETH_carte_variateur_V3_Receive_State(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Receive(ServoComEth * eth);
extern int ETH_carte_variateur_V3_Write_Network_Parameters_in_EEPROM(ServoComEth * eth, char * ip,
																	 char * masque, char * passerelle,
																	 char * dns);
extern int ETH_carte_variateur_V3_Read_Network_Parameters_from_EEPROM(ServoComEth * eth, char * ip,
																	  char * masque, char * passerelle,
																	  char * dns);
#ifdef __cplusplus 
}
#endif

#endif		/* _CARTE_VARIATEUR_V3_H_ */

