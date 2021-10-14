/***********************************************************************************************************************
 * carte_variateur_V3.c - 
 *
 * Copyright (C) 2010, HAPTION S.A.
 * Auteur : Pascal Louveau
 * Date de création : 11/01/2010
 *
 ***********************************************************************************************************************/ 
#if defined(WIN32)
#include <windows.h>
#elif defined(LINUX)
#include <string.h>
#include <netinet/in.h>
#include <sys/select.h>
typedef int SOCKET;
typedef struct sockaddr SOCKADDR;
#define INVALID_SOCKET -1
#define closesocket(s) close(s)
#define sprintf_s(a,b,c,...) sprintf(a,c,##__VA_ARGS__)
#define strtok_s(a,b,c) strtok(a,b)
#endif
#include <stdio.h>
#include "eth.h"
#include "carte_variateur_V3.h"

static WORD swapw(WORD data);
static int swapl(int data);



/*--------------------------------------------------------------------------
| swapw
|									       	
| Syntaxe --
|   WORD swapw(WORD data)
|
----------------------------------------------------------------------------*/
static WORD swapw(WORD data)
{
	BYTE data2[2];
	WORD* result = (WORD*) &data2;

	data2[1] = (BYTE) (data & 0x00FF);
	data2[0] = (BYTE) ((data & 0xFF00) >> 8);

	return (*result);
}


/*--------------------------------------------------------------------------
| swapl
|											
| Syntaxe --
|   int swapl(int data)
|
----------------------------------------------------------------------------*/
static int swapl(int data)
{
	BYTE data2[4];
	int* result = (int*) &data2;

	data2[3] = (BYTE) (data & 0x00FF);
	data2[2] = (BYTE) ((data & 0xFF00) >> 8);
	data2[1] = (BYTE) ((data & 0x00FF0000) >> 16);
	data2[0] = (BYTE) ((data & 0xFF000000) >> 24);

	return (*result);
}


/*-------------------------------------------------------------------
| ETH_carte_variateur_V3_Datas - lecture de la carte variateur
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Datas(ServoComEth * eth)
|
------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Datas(ServoComEth * eth)
{
	int etat_moteur = 0;
	int i = 0;

	/* Entrée TOR */
	eth->entree_tor = swapl(eth->etat_message.Entree_Tor);

	/* Etat général */
	eth->etat_variateur = eth->etat_message.Etat_General;
	eth->presence_48v = (eth->etat_variateur & 0x0001);
	eth->arret_urgence = ((eth->etat_variateur >> 1) & 0x0001);
	eth->watchdog = ((eth->etat_variateur >> 2) & 0x0001);
	eth->panne_relais = ((eth->etat_variateur >> 3) & 0x0001);
	eth->panne_homme_mort = ((eth->etat_variateur >> 4) & 0x0001);

	/* Etat des moteurs */
	for (i = 0; i < eth->nb_moteurs; i++)
	{
		if (eth->erreur_checksum_controleur[i] == 0)
		{
			etat_moteur = swapw(eth->etat_message.Etat_Moteur[i].Etat);

			eth->panne_codeur[i] = (etat_moteur & 0x01);
			eth->panne_courant[i] = ((etat_moteur >> 1) & 0x01);
			eth->erreur_pwm[i] = ((etat_moteur >> 2) & 0x01);
			eth->erreur_checksum_piccolo[i] = ((etat_moteur >> 4) & 0x01);
			eth->i2t[i] = ((etat_moteur >> 9) & 0x01);
			eth->etat_inhibition[i] = ((etat_moteur >> 8) & 0x01);
			eth->pwm_surchauffe[i] = ((etat_moteur >> 3) & 0x01);
			eth->moteur[i].Position_Codeur = 
				swapl(eth->etat_message.Etat_Moteur[i].Position_Codeur);
			eth->moteur[i].Vitesse_Filtree = 
				Q15_to_Float(swapl(eth->etat_message.Etat_Moteur[i].Vitesse_Moteur));
			eth->moteur[i].ADC_Courant = 
				(float) (swapw(eth->etat_message.Etat_Moteur[i].ADC_Courant));
			eth->moteur[i].ADC_Potentiometre =
				((float) (swapw(eth->etat_message.Etat_Moteur[i].ADC_Potentiometre))) / 4096.0f ;
		}
	}
}



/*-------------------------------------------------------------------
| ETH_carte_variateur_V3_Modes - sélection du mode de commande
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Modes(ServoComEth * eth, int Mode_Variateur)
|
| Description --
|   Cette fonction permet de sélectionner le type de mode de commande.
|
------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Modes(ServoComEth * eth, int Mode_Variateur)
{
	eth->asservissement_message.Mode = Mode_Variateur;
}


/*----------------------------------------------------------------------
| ETH_carte_variateur_V3_Inhibitions - inhibition ou désinhibition
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Inhibitions(ServoComEth * eth, int Inhibition_Moteur)
|
| Description --
|   Cette fonction permet d'inhiber ou de désinhiber de tous les moteurs.
|		0 => désinhibition
|		1 => inhibition
------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Inhibitions(ServoComEth * eth, int Inhibition_Moteur)
{
	int i = 0;

	for(i = 0; i < eth->nb_moteurs; i++)
		eth->demande_inhibition[i] = Inhibition_Moteur;
}

/*----------------------------------------------------------------------
| ETH_carte_variateur_V3_Inhibitions2 - inhibition ou désinhibition specifique aux moteurs
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Inhibitions2(ServoComEth * eth, int Inhibition_Moteur)
|
| Description --
|   Cette fonction permet d'inhiber ou de désinhiber spécifiquement chaques moteurs.
|		0 => désinhibition
|		1 => inhibition
------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Inhibitions2(ServoComEth * eth, int* Inhibition_Moteur)
{
	int i = 0;

	for(i = 0; i < eth->nb_moteurs; i++)
		eth->demande_inhibition[i] = Inhibition_Moteur[i];
}

/*---------------------------------------------------------------------
| ETH_carte_variateur_V3_Stor - positionnement des sorties TOR
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Stor(ServoComEth * eth, int Stor)
|
------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Stor(ServoComEth * eth, int Stor)
{
	eth->consigne_message.Sortie_Tor = swapl(Stor);
}


/*---------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Consignes_Vitesse - positionnement des consignes variateur
|
| Syntaxe --
|	void ETH_carte_variateur_V3_Consignes_Vitesse(ServoDonnees* don, 
|												  float* consignes_vitesse)
|
| Entrées --
|	consignes_vitesse -> consigne de vitesse en tours/s
|
-----------------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Consignes_Vitesse(ServoComEth * eth, float* consignes_vitesse)
{
	int i = 0;

	for(i = 0; i < eth->nb_moteurs; i++)
	{
		eth->consigne_message.Consigne_Moteur[i].Position_Moteur = 0;
		eth->consigne_message.Consigne_Moteur[i].Vitesse_Moteur = 
			swapl(Float_to_Q15(consignes_vitesse[i]));
		eth->consigne_message.Consigne_Moteur[i].Courant = 0;
	}
}


/*---------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Consignes_Courant - positionnement des consignes variateur
|
| Syntaxe --
|	void ETH_carte_variateur_V3_Consignes_Courant(ServoDonnees* don, 
|												  float* consignes_courant)
|
-----------------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Consignes_Courant(ServoComEth * eth, float* consignes_courant)
{
	int i = 0;

	for(i = 0; i < eth->nb_moteurs; i++)
	{
		eth->consigne_message.Consigne_Moteur[i].Position_Moteur = 0;
		eth->consigne_message.Consigne_Moteur[i].Vitesse_Moteur = 0;
		eth->consigne_message.Consigne_Moteur[i].Courant = swapl(Float_to_Q15(consignes_courant[i]));
	}
}


/*---------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Consignes_Position - consignes position codeur et vitesse codeur
|
| Syntaxe --
|	void ETH_carte_variateur_V3_Consignes_Position(ServoComEth * eth, 
|												   float* consignes_position, 
|												   float* consignes_vitesse)
| Entrées --
|	consignes_position -> consigne de position moteur en tours
|	consignes_vitesse -> consigne de vitesse moteur en tours/s
|
-----------------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Consignes_Position(ServoComEth * eth, float* consignes_position, float* consignes_vitesse)
{
	int i = 0;

	for (i = 0; i < eth->nb_moteurs; i++)
	{
		eth->consigne_message.Consigne_Moteur[i].Position_Moteur = swapl(Float_to_Q15(consignes_position[i]));
		eth->consigne_message.Consigne_Moteur[i].Vitesse_Moteur = swapl(Float_to_Q15(consignes_vitesse[i]));
		eth->consigne_message.Consigne_Moteur[i].Courant = 0;
	}
}


/*---------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Set_Motor_Parameters - positionnement des parametres moteurs
|
| Syntaxe --
|	void ETH_carte_variateur_V3_Set_Motor_Parameters(ServoComEth * eth, float * nb_points_codeur,
|																		double * Kconv_I)
|
-----------------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Set_Motor_Parameters(ServoComEth * eth, float * nb_points_codeur, double * Kconv_I)
{
	int i = 0;

	for (i = 0; i < eth->nb_moteurs; i++)
	{
		eth->parametrage_message.Parametrage_Moteur[i].Kconv_I = 
			swapl(Float_to_Q20(Kconv_I[i]));
		eth->parametrage_message.Parametrage_Moteur[i].Kconv_V = 
			swapl(Float_to_Q20(1000000.0 / (4.0 * nb_points_codeur[i] * 5.0 * 40.0)));
		eth->parametrage_message.Parametrage_Moteur[i].Kconv_P = 
			swapl(Float_to_Q20(1.0 / (4.0 * nb_points_codeur[i])));
	}
}


/*---------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Set_K_B - positionnement des gains moteurs
|
| Syntaxe --
|	void ETH_carte_variateur_V3_Set_K_B(ServoComEth * eth, double * Kp_P, double * Kd_P)
|
-----------------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Set_K_B(ServoComEth * eth, double * Kp_P, double * Kd_P)
{
	int i = 0;

	for (i = 0; i < eth->nb_moteurs; i++)
	{
		eth->asservissement_message.Asservissement_Moteur[i].Kp_P = 
			swapl(Float_to_Q15(Kp_P[i]));
		eth->asservissement_message.Asservissement_Moteur[i].Kd_P = 
			swapl(Float_to_Q15(Kd_P[i]));
	}
}


/*----------------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Set_Asserv_Parameters - positionnement des gains d'asservissement
|
| Syntaxe --
|	void eth_carte_variateur_V3_Set_Asserv_Parameters(ServoComEth * eth)
|
------------------------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Set_Asserv_Parameters(ServoComEth * eth, double * Kp_V, 
												  double * Kp_I, double * Ki_I)
{
	int i = 0;

	for (i = 0; i < eth->nb_moteurs; i++)
	{
		eth->asservissement_message.Asservissement_Moteur[i].Kp_V = 
			swapl(Float_to_Q15(Kp_V[i]));
		eth->asservissement_message.Asservissement_Moteur[i].Kp_I = 
			swapl(Float_to_Q15(Kp_I[i]));
		eth->asservissement_message.Asservissement_Moteur[i].Ki_I = 
			swapl(Float_to_Q15(Ki_I[i]));
	}
}


/*--------------------------------------------------------------------------
| ETH_carte_variateur_V3_Initialise - initialisation de la communication UDP
|											
| Syntaxe --
|   int ETH_carte_variateur_V3_Initialise(ServoComEth * eth, char* dest_ip, int local_port)
|
----------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Initialise(ServoComEth * eth, const char* dest_ip, int local_port)
{
#if defined(WIN32)
	WSADATA wsaData = {0};
#endif
	int i = 0;


	memset(&eth->connexion_message, 0, sizeof(Struct_Connexion_Message));
	memset(&eth->asservissement_message, 0, sizeof(Struct_Asservissement_Message));
	memset(&eth->id_message, 0, sizeof(Struct_Id_Message));
	memset(&eth->calibration_message, 0, sizeof(Struct_Calibration_Message));
	memset(&eth->etat_message, 0, sizeof(Struct_Etat_Message));
	memset(&eth->consigne_message, 0, sizeof(Struct_Consigne_Message));
	memset(&eth->parametrage_message, 0, sizeof(Struct_Parametrage_Message));
	memset(&eth->requete_adc_courant_message, 0, sizeof(Struct_Requete_adc_courant_Message));
	memset(&eth->offset_courant_message, 0, sizeof(Struct_Offset_Courant_Message));
	memset(&eth->parametrage_reseau_message, 0, sizeof(Struct_Parametrage_Reseau));
	memset(&eth->requete_lecture_reseau_message, 0, sizeof(Struct_Requete_Lecture_Parametrage_Reseau));

	eth->connexion_message.Message_ID = OPEN_CONNEXION_MESSAGE_ID;
	eth->asservissement_message.Message_ID = ASSERVISSEMENT_MESSAGE_ID;
	eth->id_message.Message_ID = IDENTIFICATION_MESSAGE_ID;
	eth->etat_message.Message_ID = ETAT_MOTEUR_MESSAGE_ID;
	eth->consigne_message.Message_ID = CONSIGNE_MOTEUR_MESSAGE_ID;
	eth->calibration_message.Message_ID = CALIBRATION_MESSAGE_ID;
	eth->parametrage_message.Message_ID = PARAMETRAGE_MESSAGE_ID;
	eth->offset_courant_message.Message_ID = OFFSET_COURANT_MESSAGE_ID;
	eth->requete_adc_courant_message.Message_ID = REQUETE_ADC_COURANT_MESSAGE_ID;
	eth->parametrage_reseau_message.Message_ID = ECRITURE_PARAMETRAGE_RESEAU_MESSAGE_ID;
	eth->requete_lecture_reseau_message.Message_ID = LECTURE_PARAMETRAGE_RESEAU_MESSAGE_ID;

	eth->erreur_fatale_checksum = 0;
	eth->checksum_recu = 0;
	eth->checksum_calcule = 0;
	eth->status = 0;
	eth->panne_relais = 0;
	eth->sock = 0;

	for (i = 0; i < NB_MOTEURS_V3B; i++)
	{
		eth->panne_codeur[i] = 0;
		eth->panne_courant[i] = 0;
		eth->erreur_pwm[i] = 0;
		eth->erreur_checksum_piccolo[i] = 0;
		eth->erreur_checksum_controleur[i] = 0;
		eth->pwm_surchauffe[i] = 0;
		eth->etat_inhibition[i] = 1;
		eth->i2t[i] = 0;
		eth->compteur_checksum[i] = 0;
		eth->demande_inhibition[i] = 1;
	}

#if defined(WIN32)
	WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

	/* creation de la socket */
	eth->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (eth->sock == INVALID_SOCKET) {
		printf("Error at socket()\n");
		return -1;
	}

	eth->recvAddr.sin_family = AF_INET;
	eth->recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	eth->recvAddr.sin_port = htons(local_port);
	if(bind(eth->sock, (SOCKADDR *) &eth->recvAddr, sizeof(eth->recvAddr)))
	{
		printf("Error at bind() port %d\n", local_port);
		return -1;
	}

	eth->service.sin_family = AF_INET;
	eth->service.sin_addr.s_addr = inet_addr(dest_ip);
	eth->service.sin_port = htons(5000);

	return 0;
}


/*--------------------------------------------------------------------------
| ETH_carte_variateur_V3_Close - fermeture de la communication UDP
|											
| Syntaxe --
|   int ETH_carte_variateur_V3_Close(ServoComEth * eth)
|
----------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Close(ServoComEth * eth)
{
	if (eth->sock != 0) closesocket(eth->sock);
	eth->sock = 0;
	return 0;
}


/*-----------------------------------------------------------------------
| ETH_carte_variateur_V3_Connexion - envoie d'une trame de connexion
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Connexion(ServoComEth * eth)
|
--------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Connexion(ServoComEth * eth)
{
	int iResult = 0;

	iResult = sendto(eth->sock, (const char*) &eth->connexion_message, 
					 sizeof(Struct_Connexion_Message), 0, 
	 				 (SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult > 0)
	{
		eth->status = 0;
		return 0;
	}
	else
	{
		eth->status = iResult;
		return -1;
	}
}


/*-----------------------------------------------------------------------
| ETH_carte_variateur_V3_Send_Consignes - envoie des consignes à la carte
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Send_Consignes(ServoComEth * eth)
|
--------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Send_Consignes(ServoComEth * eth)
{
	Struct_Consigne_Moteur* pConsigne = 0;
	unsigned long checksum = 0;
	WORD etat = 0;
	BYTE* pData = 0;
	int nBytes = 0;
	int iResult = 0;
	int i = 0;
	int j = 0;

	nBytes = sizeof(BYTE) + sizeof(Q0) + eth->nb_moteurs * (3 * sizeof(Q15) + sizeof(WORD));

	/* calcul checksum */
	if (eth->activation_checksum == 1)
	{
		for (i = 0; i < eth->nb_moteurs; i++)
		{
			checksum = 0;
			pConsigne = &(eth->consigne_message.Consigne_Moteur[i]);
			pData = (BYTE*) pConsigne;
			pConsigne->Etat = 0;
			for (j = 0; j < (3 * sizeof(Q15) + sizeof(WORD)); j++)
				checksum += pData[j];

			// HOTFIX n°00 : complément à 1
			if (eth->activation_complement_a_un == 1)
				checksum = ~checksum;

			etat = (WORD) eth->demande_inhibition[i];
			etat |= ((checksum & 0x3FFF) << 2);
			pConsigne->Etat = swapw(etat);
		}
	}
	else
	{
		for (i = 0; i < eth->nb_moteurs; i++)
		{
			pConsigne = &(eth->consigne_message.Consigne_Moteur[i]);
			etat = (WORD) eth->demande_inhibition[i];
			pConsigne->Etat = swapw(etat);
		}
	}

	iResult = sendto(eth->sock, (const char*) &eth->consigne_message, nBytes, 0, 
	 				(SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult > 0)
	{
		eth->status = 0;
		return 0;
	}
	else
	{
		eth->status = iResult;
		return -1;
	}
}


/*----------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Send_Asserv_Parameters - envoie des gains d'asservissements
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Send_Asserv_Parameters(ServoComEth * eth)
|
------------------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Send_Asserv_Parameters(ServoComEth * eth)
{
	int nBytes = 0;
	int iResult = 0;

	nBytes = sizeof(BYTE) + sizeof(BYTE) + eth->nb_moteurs * (5 * sizeof(Q15));
	iResult = sendto(eth->sock, (const char*) &eth->asservissement_message, nBytes, 0, 
	 	   (SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult > 0)
	{
		eth->status = 0;
		return 0;
	}
	else
	{
		eth->status = iResult;
		return -1;
	}
}


/*----------------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Send_Motors_Parameters - envoie des parametres des moteurs
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Send_Motors_Parameters(ServoComEth * eth)
|
------------------------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Send_Motors_Parameters(ServoComEth * eth)
{
	int nBytes = 0;
	int iResult = 0;

	nBytes = sizeof(BYTE) + eth->nb_moteurs * 3 * sizeof(Q20);
	iResult = sendto(eth->sock, (const char*) &eth->parametrage_message, nBytes, 0, 
	 	   (SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult > 0)
	{
		eth->status = 0;
		return 0;
	}
	else
	{
		eth->status = iResult;
		return -1;
	}
}


/*----------------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Send_Calibration - envoie de la structure calibration
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Send_Calibration(ServoComEth * eth)
|
------------------------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Send_Calibration(ServoComEth * eth)
{
	int nBytes = 0;
	int iResult = 0;

	nBytes = sizeof(BYTE) + eth->nb_moteurs * sizeof(Q0);
	iResult = sendto(eth->sock, (const char*) &eth->calibration_message, nBytes, 0, 
	 	   (SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult > 0)
	{
		eth->status = 0;
		return 0;
	}
	else
	{
		eth->status = iResult;
		return -1;
	}
}


/*----------------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Send_Request_ADC - envoie de la structure requete ADC courant
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Send_Request_ADC(ServoComEth * eth)
|
------------------------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Send_Request_ADC(ServoComEth * eth)
{
	int iResult = 0;

	iResult = sendto(eth->sock, (const char*) &eth->requete_adc_courant_message, 1, 0, 
	 	   (SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult > 0)
	{
		eth->status = 0;
		return 0;
	}
	else
	{
		eth->status = iResult;
		return -1;
	}
}


/*---------------------------------------------------------------------
| ETH_carte_variateur_V3_Receive_Identification - réception de l'identification
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Receive_Identification(ServoComEth * eth)
|
------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Receive_Identification(ServoComEth * eth)
{
	int nBytes = 0;
	int fromLen = sizeof(eth->service);
	int cr;

	/* reception avec un timeout de 100 ms */
	FD_ZERO(&eth->rfds);
	FD_SET(eth->sock, &eth->rfds);
	eth->timeout.tv_sec = 0;
	eth->timeout.tv_usec = 100000;
	
	cr = select((int) eth->sock+1, &eth->rfds, NULL, NULL, &eth->timeout);
	if (cr == 0)
	{
		/* timeout */
//		printf("Timeout Receive_Identification\n");
		return -1;
	}

	nBytes = recvfrom(eth->sock, (char*) eth->recvBuffer, 1024, 0, (SOCKADDR *) &eth->service, &fromLen);
	if (nBytes > 0)
	{
		if (eth->recvBuffer[0] == IDENTIFICATION_MESSAGE_ID)
		{
			memcpy(&eth->id_message, (const void*) eth->recvBuffer, sizeof(Struct_Id_Message));
			eth->type_device = eth->id_message.Type_Device;
			eth->numero_serie = eth->id_message.Numero_Serie;
			eth->etat_calibration = eth->id_message.Etat_Calibration;
			return 0;
		}
	}
	return -1;
}


/*---------------------------------------------------------------------
| ETH_carte_variateur_V3_Receive_ADC_Offset - réception de l'offset courant
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Receive_ADC_Offset(ServoComEth * eth, float * adc_offset)
|
------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Receive_ADC_Offset(ServoComEth * eth, float * adc_offset)
{
	int fromLen = sizeof(eth->service);
	int nBytes = 0;
	int i = 0;
	int cr;

	/* reception avec un timeout de 100 ms */
	FD_ZERO(&eth->rfds);
	FD_SET(eth->sock, &eth->rfds);
	eth->timeout.tv_sec = 0;
	eth->timeout.tv_usec = 100000;
	
	cr = select((int) eth->sock+1, &eth->rfds, NULL, NULL, &eth->timeout);
	if (cr == 0)
	{
		/* timeout */
//		printf("Timeout Receive_Offset_Adc\n");
		return -1;
	}

	nBytes = recvfrom(eth->sock, (char*) eth->recvBuffer, 1024, 0, 
					  (SOCKADDR *) &eth->service, &fromLen);
	if (nBytes > 0)
	{
		if (eth->recvBuffer[0] == OFFSET_COURANT_MESSAGE_ID)
		{
			memcpy(&eth->offset_courant_message, (const void*) eth->recvBuffer, nBytes);
			for (i = 0; i < eth->nb_moteurs; i++)
				adc_offset[i] = 
					(float) (swapl(eth->offset_courant_message.Offset_Courant_Moteur[i].ADC_Offset));
			return 0;
		}
	}
	return -1;
}


/*---------------------------------------------------------------------
| ETH_carte_variateur_V3_Receive - réception de données
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Receive(ServoComEth * eth)
|
------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Receive(ServoComEth * eth)
{
	int fromLen = sizeof(eth->service);
	int nBytes = 0;
	int i = 0;
	int cr;

	/* reception avec un timeout de 100 ms */
	FD_ZERO(&eth->rfds);
	FD_SET(eth->sock, &eth->rfds);
	eth->timeout.tv_sec = 0;
	eth->timeout.tv_usec = 100000;
	
	cr = select((int) eth->sock+1, &eth->rfds, NULL, NULL, &eth->timeout);
	if (cr == 0)
	{
		/* timeout */
//		printf("Timeout Receive\n");
		return -1;
	}

	nBytes = recvfrom(eth->sock, (char*) eth->recvBuffer, 1024, 0, 
					  (SOCKADDR *) &eth->service, &fromLen);
	if (nBytes > 0)
		return 0;

	return -1;
}


/*---------------------------------------------------------------------
| ETH_carte_variateur_V3_Set_Calibration - 
|
| Syntaxe --
|   void ETH_carte_variateur_V3_Set_Calibration(ServoComEth * eth, long * position_codeur_init)
|
------------------------------------------------------------------------*/
void ETH_carte_variateur_V3_Set_Calibration(ServoComEth * eth, long * position_codeur_init)
{
	int i = 0;

	for (i = 0; i < eth->nb_moteurs; i++)
	{
		eth->calibration_message.Calibration_Moteur[i].Position_Codeur_Initial = 
			swapl(position_codeur_init[i]);
	}
}


/*---------------------------------------------------------------------
| ETH_carte_variateur_V3_Receive_State - réception d'une trame d'Etat
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Receive_State(ServoComEth * eth)
|
------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Receive_State(ServoComEth * eth)
{
	unsigned long checksum = 0;
	int indice_moteur = 0;
	BYTE* pData = 0;
	WORD tmp = 0;
	int fromLen = sizeof(eth->service);
	int nBytes_received = 0;
	int nBytes = 0;
	int i = 0;
	int j = 0;
	int cr = 0;
	int no_frame = 1;

	/* reception avec un timeout de 0 ms */
	FD_ZERO(&eth->rfds);
	FD_SET(eth->sock, &eth->rfds);
	eth->timeout.tv_sec = 0;
	eth->timeout.tv_usec = 0;
	
	nBytes = 
		sizeof(BYTE) + sizeof(BYTE) + sizeof(Q0) +
		eth->nb_moteurs * (sizeof(Q0) + sizeof(Q15) + 4 * sizeof(WORD));

	while(1)
	{
		cr = select((int) eth->sock+1, &eth->rfds, NULL, NULL, &eth->timeout);
		if (cr == 0)
		{
			if (no_frame == 0) return 0;
			else return -1;
		}

		nBytes_received = recvfrom(eth->sock, (char*) eth->recvBuffer, nBytes, 0, 
						  (SOCKADDR *) &eth->service, &fromLen);

		if (nBytes_received > 0)
		{
			if (eth->recvBuffer[0] == ETAT_MOTEUR_MESSAGE_ID)
			{
				no_frame = 0;

				if (eth->activation_checksum == 0)
				{
					memcpy(&eth->etat_message, (const void*) eth->recvBuffer, nBytes);
					for (i = 0; i < eth->nb_moteurs; i++)
					{
						if ((eth->etat_message.Etat_Moteur[i].Checksum != 0xBAAB) &&
							(eth->etat_message.Etat_Moteur[i].Checksum != 0x0))
						{
							eth->compteur_checksum[i]++;
							eth->erreur_checksum_controleur[i] = 1;
							if (eth->compteur_checksum[i] >= 10)
							{
								eth->compteur_checksum[i] = 0;
								eth->erreur_fatale_checksum = 1;
							}
						}
					}
				}
				else
				{
					// recopie Message_ID + Etat_general + Entree_Tor
					memcpy(&eth->etat_message, (const void*) eth->recvBuffer, 6);

					for (i = 0; i < eth->nb_moteurs; i++)
					{
						indice_moteur = sizeof(BYTE) + sizeof(BYTE) + sizeof(Q0) + 
										i * (sizeof(Q0) + sizeof(Q15) + 4 * sizeof(WORD));

						/* lecture checksum */
 						tmp = (eth->recvBuffer[indice_moteur + 15] << 8) |
							(eth->recvBuffer[indice_moteur + 14]);
						eth->checksum_recu = swapw(tmp);


						/* calcul checksum */
						pData = (BYTE*) &(eth->recvBuffer[indice_moteur]);
						eth->checksum_calcule = 0;
						for (j = 0; j < 14; j++) eth->checksum_calcule += pData[j];
						eth->checksum_calcule = (eth->checksum_calcule & 0xFFFF);

						// HOTFIX n°00 : complément à 1
						if (eth->activation_complement_a_un == 1)
							eth->checksum_calcule = ~eth->checksum_calcule;

						if (eth->checksum_recu == eth->checksum_calcule)
						{
							eth->erreur_checksum_controleur[i] = 0;
							eth->compteur_checksum[i] = 0;
							memcpy(&(eth->etat_message.Etat_Moteur[i]), (const void*) &(eth->recvBuffer[indice_moteur]), 16);
						}
						else
						{
							eth->compteur_checksum[i]++;
							eth->erreur_checksum_controleur[i] = 1;
							if (eth->compteur_checksum[i] >= 10)
							{
								eth->compteur_checksum[i] = 0;
								eth->erreur_fatale_checksum = 1;
							}
						} // if
					} // for
				}	// else
			} // if	
		}
	}

	return 0;
}


/*----------------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Write_Network_Parameters_in_EEPROM - envoie des parametres réseaux
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Write_Network_Parameters_in_EEPROM(ServoComEth * eth, 
|																 char * ip,
|																 char * masque,
|																 char * passerelle,
|																 char * dns)
|
------------------------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Write_Network_Parameters_in_EEPROM(ServoComEth * eth,
															 char * ip,
															 char * masque,
															 char * passerelle,
															 char * dns)
{
	int fromLen = sizeof(eth->service);
	int iResult = 0;
	char* token = NULL;
	char* next_token = NULL;
	int nBytes = 0;
	int cr = 0;
	int i = 0;

	/* adresse ip */
	token = strtok_s(ip, ".", &next_token);
	eth->parametrage_reseau_message.Adresse_IP[0] = atoi(token);
	for (i=1; i<4; i++)
	{
		token = strtok_s(NULL, ".", &next_token);
		eth->parametrage_reseau_message.Adresse_IP[i] = atoi(token);
	}

	/* masque */
	token = strtok_s(masque, ".", &next_token);
	eth->parametrage_reseau_message.Masque[0] = atoi(token);
	for (i=1; i<4; i++)
	{
		token = strtok_s(NULL, ".", &next_token);
		eth->parametrage_reseau_message.Masque[i] = atoi(token);
	}

	/* passerelle */
	token = strtok_s(passerelle, ".", &next_token);
	eth->parametrage_reseau_message.Passerelle[0] = atoi(token);
	for (i=1; i<4; i++)
	{
		token = strtok_s(NULL, ".", &next_token);
		eth->parametrage_reseau_message.Passerelle[i] = atoi(token);
	}

	/* dns */
	token = strtok_s(dns, ".", &next_token);
	eth->parametrage_reseau_message.DNS[0] = atoi(token);
	for (i=1; i<4; i++)
	{
		token = strtok_s(NULL, ".", &next_token);
		eth->parametrage_reseau_message.DNS[i] = atoi(token);
	}

	eth->parametrage_reseau_message.Message_ID = ECRITURE_PARAMETRAGE_RESEAU_MESSAGE_ID;

	/* envoie des parametres */
	iResult = sendto(eth->sock, (const char*) &eth->parametrage_reseau_message, sizeof(Struct_Parametrage_Reseau), 0, 
	 	   (SOCKADDR *) &eth->service, sizeof(eth->service));
	if (iResult <= 0) return -1;

	/* reception de l'acquittement avec un timeout de 100 ms */
	FD_ZERO(&eth->rfds);
	FD_SET(eth->sock, &eth->rfds);
	eth->timeout.tv_sec = 0;
	eth->timeout.tv_usec = 100000;
	
	cr = select((int) eth->sock+1, &eth->rfds, NULL, NULL, &eth->timeout);
	if (cr == 0) return -1;

	nBytes = recvfrom(eth->sock, (char*) eth->recvBuffer, 1024, 0, (SOCKADDR *) &eth->service, &fromLen);
	if (nBytes > 0)
	{
		if (eth->recvBuffer[0] == RETOUR_ECRITURE_PARAMETRAGE_RESEAU_MESSAGE_ID)
			return 0;
	}

	return -1;
}


/*----------------------------------------------------------------------------------------
| ETH_carte_variateur_V3_Read_Network_Parameters_from_EEPROM - lecture des parametres réseaux
|
| Syntaxe --
|   int ETH_carte_variateur_V3_Read_Network_Parameters_from_EEPROM(ServoComEth * eth, 
|																 char * ip,
|																 char * masque,
|																 char * passerelle,
|																 char * dns)
|
------------------------------------------------------------------------------------------*/
int ETH_carte_variateur_V3_Read_Network_Parameters_from_EEPROM(ServoComEth * eth, char * ip,
 															   char * masque, char * passerelle,
															   char * dns)
{
	int fromLen = sizeof(eth->service);
	int nBytes = 0;
	int cr;

	sendto(eth->sock, (const char*) &eth->requete_lecture_reseau_message, 
		   sizeof(Struct_Requete_Lecture_Parametrage_Reseau), 0, 
	 	   (SOCKADDR *) &eth->service, sizeof(eth->service));


	/* reception avec un timeout de 100 ms */
	FD_ZERO(&eth->rfds);
	FD_SET(eth->sock, &eth->rfds);
	eth->timeout.tv_sec = 0;
	eth->timeout.tv_usec = 100000;
	
	cr = select((int) eth->sock+1, &eth->rfds, NULL, NULL, &eth->timeout);
	if (cr == 0) return -1;

	nBytes = recvfrom(eth->sock, (char*) eth->recvBuffer, 1024, 0, 
					  (SOCKADDR *) &eth->service, &fromLen);
	if (nBytes > 0)
	{
		if (eth->recvBuffer[0] == RETOUR_LECTURE_PARAMETRAGE_RESEAU_MESSAGE_ID)
		{
			memcpy(&eth->parametrage_reseau_message, (const void*) eth->recvBuffer, nBytes);
			sprintf_s(ip, 32*sizeof(char), "%d.%d.%d.%d", eth->parametrage_reseau_message.Adresse_IP[0],
									   eth->parametrage_reseau_message.Adresse_IP[1],
									   eth->parametrage_reseau_message.Adresse_IP[2],
									   eth->parametrage_reseau_message.Adresse_IP[3]);

			sprintf_s(masque, 32*sizeof(char),"%d.%d.%d.%d", eth->parametrage_reseau_message.Masque[0],
										   eth->parametrage_reseau_message.Masque[1],
										   eth->parametrage_reseau_message.Masque[2],
										   eth->parametrage_reseau_message.Masque[3]);

			sprintf_s(passerelle, 32*sizeof(char),"%d.%d.%d.%d", eth->parametrage_reseau_message.Passerelle[0],
									   eth->parametrage_reseau_message.Passerelle[1],
									   eth->parametrage_reseau_message.Passerelle[2],
									   eth->parametrage_reseau_message.Passerelle[3]);

			sprintf_s(dns, 32*sizeof(char),"%d.%d.%d.%d", eth->parametrage_reseau_message.DNS[0],
									   eth->parametrage_reseau_message.DNS[1],
									   eth->parametrage_reseau_message.DNS[2],
									   eth->parametrage_reseau_message.DNS[3]);
			return 0;
		}
	}
	return -1;
}