/***********************************************************************************************************************
* shared_FT_struct.h -
*
* Author : Dorian Verdel
* Creation  date : 02/2019
*
* Description :
* Defines the shared FT struct
***********************************************************************************************************************/

#pragma once

#ifndef SHARED_FT_STRUCT_H
#define SHARED_FT_STRUCT_H

// FT measures global variable struct --> for Critical section use
struct FT_meas_Global
{
	BOOL streaming;
	float fx;
	float fy;
	float fz;
	float tx;
	float ty;
	float tz;
};

#endif // !SHARED_FT_STRUCT_H