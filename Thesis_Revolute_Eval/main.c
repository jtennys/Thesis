// Author: Jason Tennyson
// Date: 11-8-10
// File: main.c
//
// This is the design for the revolute modules for Jason Tennyson's Thesis.
// This design is made for a PSoC CY8C29466-24PXI.  It is to be used for evaluation
// of functionality on a common PSoC evaluation board.
//
// Packet Structure
// ----------------
// START BYTE/START BYTE/SOURCE ID BYTE/DESTINATION ID BYTE/COMMAND TYPE/PARAM 1/PARAM 2/.../END TRANSMIT

#include <m8c.h>        	// part specific constants and macros
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules
#include "psocdynamic.h"

#pragma interrupt_handler WAIT_NC_TIMEOUT_ISR
#pragma interrupt_handler TX_01234_TIMEOUT_ISR
#pragma interrupt_handler CHILD_1_TIMEOUT_ISR
#pragma interrupt_handler CHILD_2_TIMEOUT_ISR
#pragma interrupt_handler CHILD_3_TIMEOUT_ISR
#pragma interrupt_handler CHILD_4_TIMEOUT_ISR
#pragma interrupt_handler HELLO_TIMEOUT_ISR
#pragma interrupt_handler INIT_TIMEOUT_ISR

// These defines are used as parameters of the configToggle function.
#define		WAIT						(1)
#define		MY_RESPONSE					(2)
#define 	RESPONSE_1					(3)
#define 	RESPONSE_2					(4)
#define 	RESPONSE_3					(5)
#define 	RESPONSE_4					(6)
#define		HELLO_MODE					(7)
#define		INITIALIZE					(8)

// These defines are used as comparisons to find what port the next module connected to.
#define		PORT_A						('A')
#define		PORT_B						('B')
#define		PORT_C						('C')
#define		PORT_D						('D')

// These defines are used as transmission indicators.
#define		START_TRANSMIT				(248)	// Indicates the beginning of a transmission.
#define		END_TRANSMIT				(85)	// Indicates the end of a transmission.
#define		HELLO_BYTE					(200)	// Indicates master is ready to talk.
#define		ID_ASSIGNMENT				(201)	// Indicates an ID assignment from the master.
#define		ID_ASSIGN_OK				(202)	// Indicates an ID assignment is acknowledged.
#define		PING						(203)	// Indicates that someone is pinging someone else.
#define		CLEAR_CONFIG				(204)	// Indicates that the master is asking for a config clear.
#define		CONFIG_CLEARED				(205)	// Indicates that a module has cleared its own config.
#define		MASTER_ID					(0)		// The master node's ID.
#define		BROADCAST					(254)	// The broadcast ID for talking to all nodes.
#define		DEFAULT_ID					(251)	// The ID that all modules start with

// SERVO DEFINES
// These numbers can all be found in the AX-12+ datasheet.
// These defines cover the range of IDs these servos are capable of.
#define		SERVO_ID_MIN				(0)
#define		SERVO_ID_MAX				(253)
// These defines are servo transmission indicators.
#define		SERVO_START					(255)	// This is the start byte for a servo transmission.
// These defines are used to fill in the length parameter for a given command type.
#define		READ_LENGTH					(4)		// This is the length value for all reads.
#define		WRITE_LENGTH				(4)		// This is the length value for all writes.
#define		PING_LENGTH					(2)		// This is the length value for a ping.
#define		RESET_LENGTH				(2)		// This is the length value for a reset.
// These defines are used to fill in the servo's EEPROM address parameter for a given command type.
#define		ID_ADDRESS					(3)		// This is the address where servo ID is stored.
#define		STATUS_RET_ADDRESS			(16)	// This is where the status return level is stored.
// These defines are used to fill in the instruction we are using on the servo.
#define		PING_SERVO					(1)		// This is the instruction number for ping.
#define		READ_SERVO					(2)		// This is the instruction number for a read.
#define		WRITE_SERVO					(3)		// This is the instruction number for a write.
#define		RESET_SERVO					(6)		// This is the instruction to reset the servo EEPROM.
// These defines are used to fill in the blanks on common write values.
#define		STATUS_RET_NEVER			(0)		// Only respond to ping commands.
#define		STATUS_RET_READ				(1)		// Only respond to read data commands (recommended).
#define		STATUS_RET_ALL				(2)		// Respond to every command.

// This is the number of attempts we make to contact the servo before writing to its EEPROM.
#define		SERVO_COMM_ATTEMPTS			(10)
// This is the status return level, which is set to one of the possible status return values above.
#define		STATUS_RET_LEVEL			(STATUS_RET_READ)

// This function receives a mode identifier as a parameter and toggles the system configuration.
void configToggle(int mode);
// This function checks the current mode and unloads the configuration for that mode.
void unloadAllConfigs(void);
// This function unloads the configuration corresponding to the number passed to it.
void unloadConfig(int config_num);
// This function blocks and waits for a formal hello from the master node.
void waitForHello(void);
// This function is a response to the master sending out a hello message.
void sayHello(void);
// This function looks for commands and returns 1 if a command has been read, 0 if not.
int commandReady(void);
// This function interprets the command that has just been read and performs an action accordingly.
void takeAction(void);
// This function responds to a ping.
void pingResponse(void);
// This function tells the master node that an ID assignment was completed on this module.
void assignedID(void);
// This function sends out an acknowledgement of a configuration reset.
void configCleared(void);
// This function listens for children and registers the port that they talk to.
int childListen(void);
// This function waits for a child response.
int childResponse(void);
// This function does everything it can to find a servo.
void servoFinder(void);
// This function carries out the passed servo instruction.
void servoInstruction(char id, char length, char instruction, char address, char value);

char CHILD;		// Keeps track of where the child is connected.
char ID;		// Stores the ID that the master gives this module.

int CONFIGURED;	// Keeps track of whether or not this module has been configured by the master.
int TIMEOUT;	// This flag is set if a timeout occurs.
int STATE;		// This stores the ID of the currently-loaded configuration.

char COMMAND_SOURCE;		// Stores who the current command is from.
char COMMAND_DESTINATION;	// Stores who the current command is for.
char COMMAND_TYPE;			// Stores the type of command that was just read.
char COMMAND_PARAM;			// Stores a parameter that accompanies the command (if any).
char COMMAND_LENGTH;		// Stores the length parameter of a servo command.
char COMMAND_ERROR;			// Stores the error code of a servo command.

char SERVO_ID;				// Stores the ID of the servo connected inside of this module.

void main()
{	
	CHILD = 0;					// There is no child yet.
	CONFIGURED = 0;				// This module is not configured yet.
	TIMEOUT = 0;				// Set the timeout flag low to start.
	COMMAND_PARAM = 0;					// There is no parameter yet.
	STATE = 0;					// There is no state yet.
	SERVO_ID = 255;				// Start with a servo ID of 255 (out of valid range).
	ID = DEFAULT_ID;			// Set the ID of this module to the default to start with.

	M8C_EnableGInt;				// Turn on global interrupts for the transmission timeout timer.
	
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); // Activate GPIO ISR
	
	// Block and try to talk to your servo and don't do anything until you do.
	servoFinder();
	
	// Switch to normal operation.
	//configToggle(WAIT);
	
	// Loop and wait for commands.
	while(1)
	{	
		if(commandReady())
		{
			// If the command is ready, take action.
			takeAction();
		}
	}
}

void sayHello(void)
{	
	configToggle(MY_RESPONSE);		// Switch to response mode.
	
	// Transmit a hello response to the master node.
	TX_014_PutChar(START_TRANSMIT);	// Start byte one
	TX_014_PutChar(START_TRANSMIT);	// Start byte two
	TX_014_PutChar(ID);				// My ID
	TX_014_PutChar(MASTER_ID);		// Destination ID (master)
	TX_014_PutChar(HELLO_BYTE);		// This is a hello command
	TX_014_PutChar(CHILD);			// Sends child port number, default 0.
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission.
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission.
	
	// Wait for the transmission to finish.
	while(!(TX_014_bReadTxStatus() & TX_014_TX_COMPLETE));

	configToggle(WAIT);				// Switch back to wait mode.
}

// This function receives a mode flag and switches the microcontroller to the
// desired hardware configuration.
void configToggle(int mode)
{	
	// Set the pins high and disconnect from the global bus.
	// This keeps false start bits from happening while we swap configs.
	PRT0DR |= 0b00011111;
	PRT0GS &= 0b11100000;
	
	// Unload the configuration of the current state.
	// If there is no state, blindly wipe all configurations.
	if(STATE)
	{
		unloadConfig(STATE);
	}
	else
	{
		unloadAllConfigs();
	}
	
	if(mode == WAIT)
	{
		// Load the wait receiver configuration.
		LoadConfig_waiting();
		
		// Start the receivers.
		WAIT_RECV_Start(WAIT_RECV_PARITY_NONE);
		RX8_2_Start(RX8_2_PARITY_NONE);
		
		// Set the current state.
		STATE = WAIT;
	}
	else if(mode == MY_RESPONSE)
	{
		// Load the transmitter configuration.
		LoadConfig_my_response();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start the transmitters.
		TX_014_Start(TX_014_PARITY_NONE);
		TX_23_Start(TX_014_PARITY_NONE);
		
		TX_01234_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		TX_01234_TIMEOUT_Start();		// Start the timer.
		
		while(!TIMEOUT)
		{
			// Do nothing while we wait for one timeout period.
			// This is to allow everyone to get in the right configuration.
		}
		
		TX_01234_TIMEOUT_Stop();		// Stop the timer.
		TIMEOUT = 0;					// Reset the timeout flag.
	
		// Set the current state.
		STATE = MY_RESPONSE;
	}
	else if(mode == RESPONSE_1)
	{
		// Load the response wait on port 1.
		LoadConfig_response1();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start listening for a response through child port 1.
		CHILD_1_Start(CHILD_1_PARITY_NONE);
		
		CHILD_1_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		CHILD_1_TIMEOUT_Start();		// Start the timer.
		
		// Set the current state.
		STATE = RESPONSE_1;
	}
	else if(mode == RESPONSE_2)
	{
		// Load the response wait on port 2.
		LoadConfig_response2();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start listening for a response through child port 2.
		CHILD_2_Start(CHILD_2_PARITY_NONE);
		
		CHILD_2_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		CHILD_2_TIMEOUT_Start();		// Start the timer.
		
		// Set the current state.
		STATE = RESPONSE_2;
	}
	else if(mode == RESPONSE_3)
	{
		// Load the response wait on port 3.
		LoadConfig_response3();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start listening for a response through child port 3.
		CHILD_3_Start(CHILD_3_PARITY_NONE);
		
		CHILD_3_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		CHILD_3_TIMEOUT_Start();		// Start the timer.
		
		// Set the current state.
		STATE = RESPONSE_3;
	}
	else if(mode == RESPONSE_4)
	{
		// Load the response wait on port 4.
		LoadConfig_response4();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start listening for a response through child port 4.
		CHILD_4_Start(CHILD_4_PARITY_NONE);
		
		CHILD_4_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		CHILD_4_TIMEOUT_Start();		// Start the timer.
		
		// Set the current state.
		STATE = RESPONSE_4;
	}
	else if(mode == HELLO_MODE)
	{
		// Load the hello wait mode.
		LoadConfig_hello();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		{
		// Start listening for a response through child port 1.
		HELLO_1_Start(HELLO_1_PARITY_NONE);
		}
		
		{
		// Start listening for a response through child port 2.
		HELLO_2_Start(HELLO_2_PARITY_NONE);
		}
		
		{
		// Start listening for a response through child port 3.
		HELLO_3_Start(HELLO_3_PARITY_NONE);
		}
		
		{
		// Start listening for a response through child port 4.
		HELLO_4_Start(HELLO_4_PARITY_NONE);
		}
		
		HELLO_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		HELLO_TIMEOUT_Start();		// Start the timer.
		
		// Set the current state.
		STATE = HELLO_MODE;
	}
	else if(mode == INITIALIZE)
	{
		// Load the configuration for initialization.
		LoadConfig_initial();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start the receiver.
		INIT_RX_Start(INIT_RX_PARITY_NONE);
		
		INIT_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		INIT_TIMEOUT_Start();		// Start the timer.
		
		// Set the current state.
		STATE = INITIALIZE;
	}
	
	// If this module is configured, talk on all pins for potential children.
	if(CONFIGURED)
	{
		PRT0GS |= 0b00011111;	// Connect all pins to the global bus.
		PRT2DR &= 0b11111110;	// Turn on the LED (active low).
		
		// Print out the servo ID.
		PRT1DR |= 0b11111111;
		
		if(SERVO_ID == 1)
		{
			PRT1DR &= 0b11110111;
		}
		else if(SERVO_ID == 2)
		{
			PRT1DR &= 0b11011111;
		}
		else if(SERVO_ID == 3)
		{
			PRT1DR &= 0b11010111;
		}
		else if(SERVO_ID == 4)
		{
			PRT1DR &= 0b01111111;
		}
		else if(SERVO_ID == 5)
		{
			PRT1DR &= 0b01110111;
		}
		else if(SERVO_ID == 6)
		{
			PRT1DR &= 0b01011111;
		}
	}
	else
	{
		PRT0GS |= 0b00000001;	// Just connect pin 0;
		PRT2DR |= 0b00000001;	// Turn off the LED (active low).
	}
}

// This function tries to peek and see if a start byte has been written to the bus.
// If there is no start byte, the function exits.  If a start byte is detected, the function
// blocks and waits for the transmission to finish.
int commandReady(void)
{
	// This conditional checks which configuration is loaded and uses the proper devices to
	// read a transmission and store the important information from that transmission.
	if(STATE == WAIT)
	{	
		if(WAIT_RECV_cReadChar() == START_TRANSMIT)
		{
			// If we definitely have a transmission starting, grab it from the rx buffer
			// and store it in the proper variables for actions to be taken later.
			if(WAIT_RECV_cGetChar() == START_TRANSMIT)
			{
				COMMAND_SOURCE = WAIT_RECV_cGetChar();
				COMMAND_DESTINATION = WAIT_RECV_cGetChar();
				COMMAND_TYPE = WAIT_RECV_cGetChar();
				COMMAND_PARAM = WAIT_RECV_cGetChar();
				
				return 1;
			}
		}
	}
	else if(STATE == HELLO_MODE)
	{
		// Check all of the ports for a start byte.
		if(HELLO_1_cReadChar() == START_TRANSMIT)
		{		
			CHILD = PORT_A;
			
			return 1;
		}
		else if(HELLO_2_cReadChar() == START_TRANSMIT)
		{		
			CHILD = PORT_B;
			
			return 1;
		}
		else if(HELLO_3_cReadChar() == START_TRANSMIT)
		{
			CHILD = PORT_C;
			
			return 1;
		}
		else if(HELLO_4_cReadChar() == START_TRANSMIT)
		{
			CHILD = PORT_D;
			
			return 1;
		}
	}
	else if(STATE == RESPONSE_1)
	{
		// If the transmission is over, we are done.
		if(CHILD_1_cReadChar() == END_TRANSMIT)
		{
			return 1;
		}
	}
	else if(STATE == RESPONSE_2)
	{
		// If the transmission is over, we are done.
		if(CHILD_2_cReadChar() == END_TRANSMIT)
		{
			return 1;
		}
	}
	else if(STATE == RESPONSE_3)
	{
		// If the transmission is over, we are done.
		if(CHILD_3_cReadChar() == END_TRANSMIT)
		{
			return 1;
		}
	}
	else if(STATE == RESPONSE_4)
	{
		// If the transmission is over, we are done.
		if(CHILD_4_cReadChar() == END_TRANSMIT)
		{
			return 1;
		}
	}
	else if(STATE == INITIALIZE)
	{
		if(INIT_RX_cReadChar() == SERVO_START)
		{
			// If we definitely have a transmission starting, grab it from the rx buffer
			// and store it in the proper variables for actions to be taken later.
			if(INIT_RX_cGetChar() == SERVO_START)
			{
				COMMAND_SOURCE = INIT_RX_cGetChar();
				COMMAND_LENGTH = INIT_RX_cGetChar();
				COMMAND_ERROR = INIT_RX_cGetChar();
				COMMAND_PARAM = INIT_RX_cGetChar();
				
				return 1;
			}
		}
	}
	
	return 0;
}

// This function interprets what has been read by the command ready function
// and performs the appropriate action.
void takeAction(void)
{
	if(COMMAND_TYPE == HELLO_BYTE)				// The master is probing for new modules.
	{
		// If this module has not been acknowledged by the master node...
		if(!CONFIGURED)
		{
			// Announce this module's presence.
			sayHello();
		}
		else if(!CHILD)
		{
			// Listen for children.
			if(childListen())
			{
				// If a child was heard saying hello, forward the command with the port parameter filled in.
				sayHello();
			}
		}
		else if(CHILD)
		{
			// If you have a child established, listen to that child.
			childResponse();
		}
	}
	else if(COMMAND_TYPE == PING)			// The master is trying to find a module that is configured.
	{
		// If this is to me, act accordingly.
		if(COMMAND_DESTINATION == ID)
		{
			// Ping back to the master.
			pingResponse();
		}
		else if(COMMAND_DESTINATION > ID)
		{
			// If you have a child established, listen to that child.
			childResponse();
		}
	}
	else if(COMMAND_TYPE == ID_ASSIGNMENT)	// The master is assigning an ID to someone.
	{
		// If this is meant for me, change my ID.
		if(COMMAND_DESTINATION == ID)
		{
			if((COMMAND_PARAM > MASTER_ID) && (COMMAND_PARAM < DEFAULT_ID))
			{
				// Assign this module the ID that has been passed by the master.
				ID = COMMAND_PARAM;
				
				// This module is now configured.
				CONFIGURED = 1;
				
				// Let the master node know that you got the ID assignment.
				assignedID();
				
				// If the servo ID doesn't match what we want, change it to match.
				if(ID != SERVO_ID)
				{
					// This is our index variable for communication attempt timeouts.
					int i;
					
					while(ID != SERVO_ID)
					{	
						// Send a request to change the servo ID to match the controller ID.
						servoInstruction(SERVO_ID, WRITE_LENGTH, WRITE_SERVO, ID_ADDRESS, ID);
					
						// Try to read the servo's ID several times.
						for(i = 0; i < SERVO_COMM_ATTEMPTS; i++)
						{
							// Send a request for the servo ID, which is presumably now equal to ID.
							servoInstruction(BROADCAST, PING_LENGTH, PING_SERVO, 0, 0);
							
							// Wait for either a timeout or an indication that we want to exit the loop.
							while(!TIMEOUT)
							{
								// If we have a command to interpret, read it.
								if(commandReady())
								{
									if(!COMMAND_ERROR)
									{
										// If we have a valid servo ID, exit the loop.
										if(COMMAND_SOURCE == ID)
										{
											// Set the timeout flag to exit the while loop.
											TIMEOUT = 1;
											// Set i such that the for loop is exited.
											i = SERVO_COMM_ATTEMPTS;
											// Store the ID value.
											SERVO_ID = ID;
											// Toggle back to normal wait mode.
											configToggle(WAIT);
										}
									}
								}
							}
						}
					}
				}
			}
		}
		else if(COMMAND_DESTINATION > ID)
		{
			// Switch to listen to your child.
			childResponse();
		}
	}
	else if(COMMAND_TYPE == CLEAR_CONFIG)	// The master wants to clear one or all configurations.
	{
		// Only respond if this is directly to me and not a mass config clear.
		if(COMMAND_DESTINATION == ID)
		{
			configCleared();	// Notify the master that I am clearing my config.
		}
		
		// If this is meant for me, deconfigure.  Also, if a module ahead of you is
		// getting deconfigured, you have no choice but to deconfigure as well to
		// avoid errors on reconfiguration.
		if((COMMAND_DESTINATION <= ID) || (COMMAND_DESTINATION == BROADCAST))
		{
			ID = DEFAULT_ID;	// Reset my ID to the default.
			CONFIGURED = 0;		// I am no longer configured.
			CHILD = 0;			// No one can depend on you anymore.
		}
//		else if(COMMAND_DESTINATION > ID)
//		{
//			// Switch to listen to your child.
//			childResponse();
//			// Switch back to wait for a master response.
//			configToggle(WAIT);
//		}
//		Going to also have to take into account of this is my child.
		
		// Turn off the LED.
		PRT2DR |= 0b00000001;
	}
}

// This function sends out an acknowledgement of a configuration reset.
void configCleared(void)
{
	configToggle(MY_RESPONSE);		// Switch to response mode.
	
	// Transmit a ping to everyone.
	TX_014_PutChar(START_TRANSMIT);	// Start byte one
	TX_23_PutChar(START_TRANSMIT);	// Start byte one
	TX_014_PutChar(START_TRANSMIT);	// Start byte two
	TX_23_PutChar(START_TRANSMIT);	// Start byte two
	TX_014_PutChar(ID);				// My ID
	TX_23_PutChar(ID);				// My ID
	TX_014_PutChar(MASTER_ID);		// Destination ID (master)
	TX_23_PutChar(MASTER_ID);		// Destination ID (master)
	TX_014_PutChar(CONFIG_CLEARED);	// This is a ping response
	TX_23_PutChar(CONFIG_CLEARED);	// This is a ping response
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_23_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_23_PutChar(END_TRANSMIT);	// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_014_bReadTxStatus() & TX_014_TX_COMPLETE));
	while(!(TX_23_bReadTxStatus() & TX_23_TX_COMPLETE));
	
	configToggle(WAIT);				// Switch back to wait for a master response.
}

// This function sends out a ping response for everyone to hear.
void pingResponse(void)
{
	configToggle(MY_RESPONSE);		// Switch to response mode.
	
	// Transmit a ping to everyone.
	TX_014_PutChar(START_TRANSMIT);	// Start byte one
	TX_23_PutChar(START_TRANSMIT);	// Start byte one
	TX_014_PutChar(START_TRANSMIT);	// Start byte two
	TX_23_PutChar(START_TRANSMIT);	// Start byte two
	TX_014_PutChar(ID);				// My ID
	TX_23_PutChar(ID);				// My ID
	TX_014_PutChar(MASTER_ID);		// Destination ID (master)
	TX_23_PutChar(MASTER_ID);		// Destination ID (master)
	TX_014_PutChar(PING);			// This is a ping response
	TX_23_PutChar(PING);			// This is a ping response
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_23_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_23_PutChar(END_TRANSMIT);	// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_014_bReadTxStatus() & TX_014_TX_COMPLETE));
	while(!(TX_23_bReadTxStatus() & TX_23_TX_COMPLETE));
	
	configToggle(WAIT);				// Switch back to wait for a master response.
}

// This function blindly unloads all user configurations. This will be called once,
// when the system initially has no known state.
void unloadAllConfigs(void)
{
	UnloadConfig_waiting();
	UnloadConfig_hello();
	UnloadConfig_my_response();
	UnloadConfig_response1();
	UnloadConfig_response2();
	UnloadConfig_response3();
	UnloadConfig_response4();
	UnloadConfig_initial();
}

// This function unloads the configuration corresponding to the config number passed to it.
// We do this instead of unloadAllConfigs to cut down on set up time.
void unloadConfig(int config_num)
{
	if(config_num == WAIT)
	{
		UnloadConfig_waiting();
	}
	else if(config_num == HELLO_MODE)
	{
		UnloadConfig_hello();
	}
	else if(config_num == MY_RESPONSE)
	{
		UnloadConfig_my_response();
	}
	else if(config_num == RESPONSE_1)
	{
		UnloadConfig_response1();
	}
	else if(config_num == RESPONSE_2)
	{
		UnloadConfig_response2();
	}
	else if(config_num == RESPONSE_3)
	{
		UnloadConfig_response3();
	}
	else if(config_num == RESPONSE_4)
	{
		UnloadConfig_response4();
	}
	else if(config_num == INITIALIZE)
	{
		UnloadConfig_initial();
	}
}

// This function responds that an ID has been assigned to it.
void assignedID(void)
{
	configToggle(MY_RESPONSE);		// Switch to response mode.
	
	// Transmit a ping to everyone.
	TX_014_PutChar(START_TRANSMIT);	// Start byte one
	TX_23_PutChar(START_TRANSMIT);	// Start byte one
	TX_014_PutChar(START_TRANSMIT);	// Start byte two
	TX_23_PutChar(START_TRANSMIT);	// Start byte two
	TX_014_PutChar(ID);				// My ID
	TX_23_PutChar(ID);				// My ID
	TX_014_PutChar(MASTER_ID);		// Destination ID (master)
	TX_23_PutChar(MASTER_ID);		// Destination ID (master)
	TX_014_PutChar(ID_ASSIGN_OK);	// This is an assignment ack response
	TX_23_PutChar(ID_ASSIGN_OK);	// This is an assignment ack response
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_23_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission
	TX_23_PutChar(END_TRANSMIT);	// This is the end of this transmission
	
	// Wait for the transmission to finish.
	while(!(TX_014_bReadTxStatus() & TX_014_TX_COMPLETE));
	while(!(TX_23_bReadTxStatus() & TX_23_TX_COMPLETE));
	
	configToggle(WAIT);				// Switch back to wait for a master response.
}

// This function listens for children and registers the port that they talk to.
int childListen(void)
{
	configToggle(HELLO_MODE);	// Switch to listen for hellos on every port.
	
	// Wait to either hear a child or time out.
	while(!TIMEOUT)
	{		
		if(commandReady())
		{
			return 1;
		}
	}
	
	HELLO_TIMEOUT_Stop();		// Stop the timer.
	TIMEOUT = 0;				// Clear the timeout flag.
	
	configToggle(WAIT);			// Switch back to wait for a master response.
	
	return 0;					// Return the result of our listening session.
}

// This function waits for a child response.
int childResponse(void)
{
	int child_responded = 0;
	
	// Switch to the right port.
	if(CHILD == PORT_A)
	{
		configToggle(RESPONSE_1);
	}
	else if(CHILD == PORT_B)
	{
		configToggle(RESPONSE_2);
	}
	else if(CHILD == PORT_C)
	{
		configToggle(RESPONSE_3);
	}
	else if(CHILD == PORT_D)
	{
		configToggle(RESPONSE_4);
	}
	
	// Wait for a response or a timeout.
	while((!child_responded) && (!TIMEOUT))
	{
		if(commandReady())
		{
			child_responded = 1;
		}
	}
	
	// Stop the right timer.
	if(CHILD == PORT_A)
	{
		CHILD_1_TIMEOUT_Stop();
	}
	else if(CHILD == PORT_B)
	{
		CHILD_2_TIMEOUT_Stop();
	}
	else if(CHILD == PORT_C)
	{
		CHILD_3_TIMEOUT_Stop();
	}
	else if(CHILD == PORT_D)
	{
		CHILD_4_TIMEOUT_Stop();
	}
	
	TIMEOUT = 0;					// Reset the timeout flag.
	
	configToggle(WAIT);				// Switch back to wait for a master response.
	
	return child_responded;
}

// This function is used to find the servo that is directly connected to this module's controller.
// After the servo ID is found, the status return level is changed so that packets are only
// returned for the desired status return level.
void servoFinder(void)
{	
	// Index variable for incrementing and checking against the maximum servo comm attempts.
	int i = 0;
	
	// Integer used as a flag so that EEPROM writes aren't done more than once.
	int flashWrite = 0;
	
	// Create a status return level variable and set it to an out of range value initially.
	char status_return_level = 3;
	
	// Sit here and wait until we get a valid servo ID.
	while(SERVO_ID == SERVO_START)
	{	
		for(i = 0; i < SERVO_COMM_ATTEMPTS; i++)
		{
			// Send a ping out for any servo connected to me (should only be one).
			servoInstruction(BROADCAST, PING_LENGTH, PING_SERVO, 0, 0);
			
			// Wait for either a timeout or a valid servo ID.
			while(!TIMEOUT)
			{
				if(commandReady())
				{
					if(!COMMAND_ERROR)
					{
						// If we read a source ID within the range, exit the loop.
						if((COMMAND_SOURCE >= SERVO_ID_MIN) && (COMMAND_SOURCE <= SERVO_ID_MAX))
						{
							// Exit this while loop by setting the timeout flag.
							TIMEOUT = 1;
							// Set the servo ID variable to where the ping came from.
							SERVO_ID = COMMAND_SOURCE;
							// Set the index variable such that the for loop exits.
							i = SERVO_COMM_ATTEMPTS;
						}
						else
						{
							// Exit this while loop and try to ping again.
							TIMEOUT = 1;
						}
					}
				}
			}
		}
		
		// If we didn't get a response and haven't written to the flash of the servo.
		if((SERVO_ID == SERVO_START) && (!flashWrite))
		{
			// Set the flash write flag so that we only do this once per power cycle.
			flashWrite = 1;
			
			// Send out a broadcast reset so that we know that the response time interval
			// is large enough (default delay time for a servo is 500 microseconds).
			//servoInstruction(BROADCAST, RESET_LENGTH, RESET_SERVO, 0, 0);
		}
	}
	
//	// Print out the servo ID.
//	PRT1DR |= 0b11111111;
//	
//	if(SERVO_ID == 1)
//	{
//		PRT1DR &= 0b11110111;
//	}
//	else if(SERVO_ID == 2)
//	{
//		PRT1DR &= 0b11011111;
//	}
//	else if(SERVO_ID == 3)
//	{
//		PRT1DR &= 0b11010111;
//	}
//	else if(SERVO_ID == 4)
//	{
//		PRT1DR &= 0b01111111;
//	}
//	else if(SERVO_ID == 251)
//	{
//		PRT1DR &= 0b01110111;
//	}
//	else if(SERVO_ID == 6)
//	{
//		PRT1DR &= 0b01011111;
//	}
//	
//	while(1) { }
	
	// Check to see if the status return level is equal to the level previously defined.
	while(status_return_level != STATUS_RET_LEVEL)
	{
		// Attempt to read the status return level for the defined number of attempts.
		for(i = 0; i < SERVO_COMM_ATTEMPTS; i++)
		{
			// Send a request for the servo's status return level.
			servoInstruction(SERVO_ID, READ_LENGTH, READ_SERVO, STATUS_RET_ADDRESS, 1);
			
			// Wait for either a timeout or an indication that we want to exit the loop.
			while(!TIMEOUT)
			{
				// If a valid command is ready, interpret it.
				if(commandReady())
				{
					if(!COMMAND_ERROR)
					{
						// If the return level is equal to what is desired, store it.
						if(COMMAND_PARAM == STATUS_RET_LEVEL)
						{
							// Set the timeout flag to exit the loop.
							TIMEOUT = 1;
							// Store the status return level.
							status_return_level = COMMAND_PARAM;
							// Set i so that we exit the for loop.
							i = SERVO_COMM_ATTEMPTS;
						}
						else
						{
							// Set the timeout flag to exit the loop.
							TIMEOUT = 1;
						}
					}
				}
			}
		}
	
		// If we didn't get a response and haven't written to the flash of the servo.
		if(status_return_level != STATUS_RET_LEVEL)
		{
			// Try to force the return status to what we want.
			//servoInstruction(SERVO_ID, WRITE_LENGTH, WRITE_SERVO, STATUS_RET_ADDRESS, STATUS_RET_LEVEL);
		}
	}
	
//	PRT1DR |= 0b11111111;
//	
//	if(status_return_level == 1)
//	{
//		PRT1DR &= 0b11110111;
//	}
//	else if(status_return_level == 2)
//	{
//		PRT1DR &= 0b11011111;
//	}
//	
//	while(1) { }

	// Switch to wait for the master node to speak to you.
	configToggle(WAIT);
}

// This function receives a destination, command length, instruction type, address, and value.
// With these parameters, the function sends a packet to the communication bus.
void servoInstruction(char id, char length, char instruction, char address, char value)
{
	int total;
	char checksum;
	
	// Toggle into transmit mode.
	configToggle(MY_RESPONSE);
	
	// Disconnect your children from the global bus, just in case.
	PRT0GS &= 0b11100001;
	
	// Calculate the checksum value for our servo communication.
	total = 255-((id + length + instruction + address + value)%256);
	checksum = total;
	
	// Talk to the servo.
	if(instruction == PING_SERVO)
	{
		TX_014_PutChar(SERVO_START);	// Start byte one
		TX_014_PutChar(SERVO_START);	// Start byte two
		TX_014_PutChar(id);				// Servo ID
		TX_014_PutChar(length);			// The instruction length.
		TX_014_PutChar(instruction);	// The instruction to carry out.
		TX_014_PutChar(checksum);		// This is the checksum.
	}
	else
	{
		TX_014_PutChar(SERVO_START);	// Start byte one
		TX_014_PutChar(SERVO_START);	// Start byte two
		TX_014_PutChar(id);				// Servo ID
		TX_014_PutChar(length);			// The instruction length.
		TX_014_PutChar(instruction);	// The instruction to carry out.
		TX_014_PutChar(address);		// The address to read/write from/to.
		TX_014_PutChar(value);			// The value to write or number of bytes to read.
		TX_014_PutChar(checksum);		// This is the checksum.
	}
	
	// Wait for the transmission to finish.
	while(!(TX_014_bReadTxStatus() & TX_014_TX_COMPLETE));
	
	// Switch back to wait for a servo response.
	configToggle(INITIALIZE);
}

// This timeout ISR is for waiting before a transmission is made from this module.
// This is to give all the other modules a chance to set up and clear their buffers.
// It is currently set so that there is 1 ms of down time between the last transmission
// and this module's transmission.
void TX_01234_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,TX_01234_TIMEOUT_INT_MASK);
}

// This is the ISR for a hello response timeout.
void HELLO_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,HELLO_TIMEOUT_INT_MASK);
}

// These remaining ISRs are for all the child timeout scenarios.
void CHILD_1_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,CHILD_1_TIMEOUT_INT_MASK);
}

void CHILD_2_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,CHILD_2_TIMEOUT_INT_MASK);
}

void CHILD_3_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,CHILD_3_TIMEOUT_INT_MASK);
}

void CHILD_4_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,CHILD_4_TIMEOUT_INT_MASK);
}

void INIT_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,INIT_TIMEOUT_INT_MASK);
}