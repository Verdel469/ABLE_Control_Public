This C++ project contains the source files necessary to command the ABLE robot.
This project was developped with Visual Studio Community 2019 and might not be supported by other versions.
This project is only available on Windows.

Source files are supposed to be the following:

	- Solution file:
		- Low_level_command_1DoF.sln

	- Headers:
		- able_Control_FTData.h
		- able_Control_QTMData.h
		- able_OrdersManagement.h
		- communication_struct.h
		- communication_struct_ABLE.h
		- compute_orders.h
		- control_struct.h
		- data_recording_functions.h
		- get_FT_measures_WinAPI.h
		- get_FT_sensor_measures.h
		- get_qtm_measures.h
		- handle_communication.h
		- limb_identification.h
		- low_level_command_1DoF_main.h
		- motors_type_params.h
		- NiSerial.h
		- position_control.h
		- set_ABLEParameters.h
		- shared_FT_struct.h
		- torque_control.h
		- utils_for_ABLE_Com.h

	- Source code:
		- able_Control_FTData.cpp
		- able_Control_QTMData.cpp
		- able_OrdersManagement.cpp
		- compute_orders.cpp
		- data_recording_functions.cpp
		- get_FT_measures_WinAPI.cpp
		- get_FT_sensor_measures.cpp
		- get_qtm_measures.cpp
		- handle_communication.cpp
		- limb_identification.cpp
		- low_level_command_1DoF_main.cpp
		- motors_type_params.cpp
		- position_control.cpp
		- set_ABLEParameters.cpp
		- torque_control.cpp
		- utils_for_ABLE_Com.cpp

A ".gitignore" file is available to avoid overload of the github repository.