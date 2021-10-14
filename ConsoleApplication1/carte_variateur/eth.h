/******************************************************************
* eth.h -
*
* Copyright (C) 2010, HAPTION
* Auteur : Pascal Louveau
* Date de création : 09/06/2010
*
********************************************************************/

#ifndef _ETH_H_
#define _ETH_H_

#ifdef WIN32
#include <Windows.h>
#endif
#pragma pack(push)
#pragma pack(1)

/* nombre max de moteurs sur carte variateur V3b */
#define	NB_MOTEURS_V3B		7


#define Float_to_Q15(A) (int) ((A) * 32768.0L)
#define	Q15_to_Float(A) (float) ((float) (A) / 32768.0)
#define Float_to_Q20(A) (int) ((A) * 1048576.0L)
#define	Q20_to_Float(A) (float) ((float) (A) / 1048576.0)


#define OPEN_CONNEXION_MESSAGE_ID		1
#define	ASSERVISSEMENT_MESSAGE_ID		2
#define IDENTIFICATION_MESSAGE_ID		4
#define CALIBRATION_MESSAGE_ID			5
#define ETAT_MOTEUR_MESSAGE_ID			6
#define CONSIGNE_MOTEUR_MESSAGE_ID		7
#define PARAMETRAGE_MESSAGE_ID			8
#define REQUETE_ADC_COURANT_MESSAGE_ID	9
#define OFFSET_COURANT_MESSAGE_ID		10
#define RESET_TOGGLE					11
#define ECRITURE_PARAMETRAGE_RESEAU_MESSAGE_ID	0xC1
#define LECTURE_PARAMETRAGE_RESEAU_MESSAGE_ID	0xC2
#define RETOUR_ECRITURE_PARAMETRAGE_RESEAU_MESSAGE_ID	0xC3
#define RETOUR_LECTURE_PARAMETRAGE_RESEAU_MESSAGE_ID	0xC4


/* Type d'asservissement */
#define MODE_STOP						0
#define MODE_ASSERVISSEMENT_VITESSE		1
#define MODE_ASSERVISSEMENT_COURANT		2
#define MODE_ASSERVISSEMENT_POSITION	3


typedef int Q0;					// 32 bits
typedef int Q15;				// 32 bits
typedef int Q20;				// 32 bits
#ifndef WIN32
typedef unsigned char BYTE;		//	8 bits
typedef unsigned short WORD;	//  16 bits
#endif



typedef struct {
	int     Position_Codeur;
    float   Vitesse_Filtree;
    float   ADC_Courant;
	float	ADC_Potentiometre;

} Rec_Moteur;


/* Connexion */
typedef struct {   
	BYTE Message_ID;
} Struct_Connexion_Message;

/* Reset Toggle */
typedef struct {   
	BYTE Message_ID;
} Struct_Toggle_Message;

/* Identification */
typedef struct {
	BYTE Message_ID;
	BYTE Type_Device;
	BYTE Numero_Serie;
	BYTE Etat_Calibration;
} Struct_Id_Message;


/* Calibration */
typedef struct {   
	Q0 Position_Codeur_Initial;
} Struct_Calibration_Moteur;


typedef struct {   
	BYTE Message_ID;
	Struct_Calibration_Moteur Calibration_Moteur[NB_MOTEURS_V3B];
} Struct_Calibration_Message;


/* Asservissement */
typedef struct {   
	Q15 Kp_P;
	Q15 Kd_P;
	Q15 Kp_V;
	Q15 Kp_I;
	Q15 Ki_I;
} Struct_Asservissement_Moteur;


typedef struct {   
	BYTE Message_ID;
	BYTE Mode;
	Struct_Asservissement_Moteur Asservissement_Moteur[NB_MOTEURS_V3B];
} Struct_Asservissement_Message;


/* paramétrage moteurs */
typedef struct {
	Q20 Kconv_I;
	Q20 Kconv_V;
	Q20 Kconv_P;
} Struct_Parametrage_Moteur;


typedef struct {   
	BYTE Message_ID;
	Struct_Parametrage_Moteur Parametrage_Moteur[NB_MOTEURS_V3B];
} Struct_Parametrage_Message;


/* Requete adc courant */
typedef struct {   
	BYTE Message_ID;
} Struct_Requete_adc_courant_Message;


/* Offset courant */
typedef struct {
	Q0 ADC_Offset;
} Struct_Offset_Courant_Moteur;

typedef struct {
	BYTE Message_ID;
	Struct_Offset_Courant_Moteur Offset_Courant_Moteur[NB_MOTEURS_V3B];
} Struct_Offset_Courant_Message;


/* Etat */
typedef struct {
	Q0 Position_Codeur;
	Q15 Vitesse_Moteur;
	WORD ADC_Courant;
	WORD ADC_Potentiometre;
	WORD Etat;
	WORD Checksum;
} Struct_Etat_Moteur;


typedef struct {
	BYTE Message_ID;
	BYTE Etat_General;
	Q0 Entree_Tor;
	Struct_Etat_Moteur Etat_Moteur[NB_MOTEURS_V3B];
} Struct_Etat_Message;


/* Consigne */
typedef struct {
	Q15 Position_Moteur;
	Q15 Vitesse_Moteur;
	Q15 Courant;
	WORD Etat;	// contient le checksum
} Struct_Consigne_Moteur;


typedef struct {
	BYTE Message_ID;
	Q0 Sortie_Tor;
	Struct_Consigne_Moteur Consigne_Moteur[NB_MOTEURS_V3B];
} Struct_Consigne_Message;


typedef struct {
	BYTE Message_ID;
	BYTE Adresse_IP[4];
	BYTE Masque[4];
	BYTE Passerelle[4];
	BYTE DNS[4];
} Struct_Parametrage_Reseau;


typedef struct {
	BYTE Message_ID;
} Struct_Requete_Lecture_Parametrage_Reseau;


#pragma pack(pop)

#endif		/* _ETH_H_ */

