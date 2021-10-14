////////////////////////////////////////////////////////////////////////////////
//
// File:
//
//    NiSerial.h
//
// Purpose:
//
//    To declare extended functions for National Instruments Serial Products.
//
// Note:
//
//    To determine which of these extended functions are supported with your
//    NI-Serial hardware please refer to the National Instruments "Serial
//    Hardware and Software for Windows Help".
//    
//    These extended serial functions should only be used with NI-Serial
//    hardware. Using these functions with unsupported hardware may result in
//    unexpected behavior.
//
//    All extended functions are accessed via the Win32 function DeviceIoControl
//    which is declared in <winioctl.h>. To specify which extended function to
//    access, pass a corresponding value (hereafter referred to as
//    "Function Code") into DeviceIoControl () for its second parameter.
//
// Copyright:
//
//    © Copyright 2005
//    National Instruments Corporation.
//    All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

// Ensure that this header file is included only once per compilation unit
#if !defined (___NiSerial_h___)
#define       ___NiSerial_h___


////////////////////////////////////////////////////////////////////////////////
//
// Function Code Definitions
//
////////////////////////////////////////////////////////////////////////////////

// Returns the current serial transceiver type
#define NISERIAL_GET_INTERFACE_TYPE                                            \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4050,METHOD_BUFFERED,FILE_ANY_ACCESS)


////////////////////////////////////////
//
// Transceiver-specific
//
////////////////////////////////////////

//
// RS-485
//

// Sets the transceiver mode
#define NISERIAL_SET_RS485_MODE                                                \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4091,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Returns the current transceiver mode
#define NISERIAL_GET_RS485_MODE                                                \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4090,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Sets the bias resistor mode
#define NISERIAL_SET_RS485_BIAS                                                \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4031,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Returns the current bias resistor mode
#define NISERIAL_GET_RS485_BIAS                                                \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4030,METHOD_BUFFERED,FILE_ANY_ACCESS)


//
// RS-232
//

// Sets the transceiver mode
#define NISERIAL_SET_RS232_MODE                                                \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4081,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Returns the current transceiver mode
#define NISERIAL_GET_RS232_MODE                                                \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4080,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Returns the state (DTE or DCE) that the transceivers are in currently, and
// whether the physical connection is valid or invalid.
//    Note: If the connection is invalid and the transceivers are in Auto mode
//    their state is undefined.  
#define NISERIAL_GET_RS232_STATE                                               \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4082,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Sets the DCD line when the port is in DCE state. The behavior is undefined
// when the port is in DTE state.
#define NISERIAL_SET_DCD                                                       \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4072,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Clears the DCD line when the port is in DCE state. The behavior is undefined
// when the port is in DTE state.
#define NISERIAL_CLR_DCD                                                       \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4071,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Returns the state of the DCD line when the port is in DCE state. The behavior
// is undefined when the port is in DTE state.
//    Note: To get the DCD state when the port is in DTE state use the Win32
//    function GetCommModemStatus.
#define NISERIAL_GET_DCD                                                       \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4070,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Sets the RI line when the port is in DCE state. The behavior is undefined
// when the port is in DTE state.
#define NISERIAL_SET_RI                                                        \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4062,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Clears the RI line when the port is in DCE state. The behavior is undefined
// when the port is in DTE state.
#define NISERIAL_CLR_RI                                                        \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4061,METHOD_BUFFERED,FILE_ANY_ACCESS)

// Returns the state of the RI line when the port is in DCE state. The behavior
// is undefined when the port is in DTE state.
//    Note: To get the RI state when the port is in DTE state use the Win32
//    function GetCommModemStatus.
#define NISERIAL_GET_RI                                                        \
   CTL_CODE(FILE_DEVICE_SERIAL_PORT,4060,METHOD_BUFFERED,FILE_ANY_ACCESS)


////////////////////////////////////////////////////////////////////////////////
//
// Type Definitions
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////
//
// Type: SERIAL_INTERFACE
//
////////////////////////////////////////

typedef unsigned long SERIAL_INTERFACE;

//
// Possible values for type
//

// RS-485
#define RS485_INTERFACE    0x00

// RS-232
#define RS232_INTERFACE    0x01

// Default
#define DEFAULT_INTERFACE  RS232_INTERFACE

// Invalid - for bounds checking only
#define INTERFACE_END_ENUM 0x02


////////////////////////////////////////
//
// Type: TRANSCEIVER_MODE
//
////////////////////////////////////////

typedef unsigned long TRANSCEIVER_MODE;

//
// RS-485: Possible values for type
//

// Four-wire mode
#define RS485_MODE_4WIRE    0x0

// Two-wire DTR Control with echo
#define RS485_MODE_2W_ECHO  0x1

// Two-wire DTR Control without echo
#define RS485_MODE_2W_DTR   0x2

// Two-wire Auto Control
#define RS485_MODE_2W_AUTO  0x3

// Default
#define RS485_MODE_DEFAULT  RS485_MODE_4WIRE

// Invalid - for bounds checking only
#define RS485_MODE_END_ENUM 0x4


//
// RS-232: Possible values for type
//

// DTE mode
#define RS232_MODE_DTE      0x00

// DCE mode
#define RS232_MODE_DCE      0x01

// Automatic Detect mode
#define RS232_MODE_AUTO     0x02

// Default
#define RS232_MODE_DEFAULT  RS232_MODE_DTE

// Invalid - for bounds checking only
#define RS232_MODE_END_ENUM 0x03


////////////////////////////////////////
//
// Type (RS-485): SERIAL_RS485_BIAS
//
////////////////////////////////////////

typedef unsigned long SERIAL_RS485_BIAS;

//
// Possible values for type
//

// Bias Off
#define RS485_BIAS_OFF      0

// Bias On
#define RS485_BIAS_ON       1

// Default
#define RS485_BIAS_DEFAULT  RS485_BIAS_ON

// Invalid - for bounds checking only
#define RS485_BIAS_END_ENUM 2


////////////////////////////////////////
//
// Type (RS-232): SERIAL_CONNECTION
//
////////////////////////////////////////

typedef unsigned long SERIAL_CONNECTION;

//
// Possible values for type
//

// Connection Invalid
#define SERIAL_CONNECTION_INVALID  0x00

// Connection Valid
#define SERIAL_CONNECTION_VALID    0x01

// Invalid - for bounds checking only
#define SERIAL_CONNECTION_END_ENUM 0x02


////////////////////////////////////////
//
// Type (RS-232): SERIAL_RS232_STATE,
//                PSERIAL_RS232_STATE
//
////////////////////////////////////////

typedef struct _SERIAL_RS232_STATE
{
   TRANSCEIVER_MODE   PortMode;
   SERIAL_CONNECTION  Connection;
}
  SERIAL_RS232_STATE,
* PSERIAL_RS232_STATE;


////////////////////////////////////////
//
// Type (RS-232): SERIAL_DCD_OUT
//
////////////////////////////////////////

typedef unsigned char SERIAL_DCD_OUT;

//
// Possible values for type
//

// DCD On
#define SERIAL_DCD_ON  1

// DCD Off
#define SERIAL_DCD_OFF 0


////////////////////////////////////////
//
// Type (RS-232): SERIAL_RI_OUT
//
////////////////////////////////////////

typedef unsigned char SERIAL_RI_OUT;

//
// Possible values for type
//

// RI On
#define SERIAL_RI_ON  1

// RI Off
#define SERIAL_RI_OFF 0

#endif // #if !defined (___NiSerial_h___)
