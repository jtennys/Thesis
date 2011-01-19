// Author: Jason Tennyson
// Date: 1-18-11
// File: main.c
//
// This is the design for the revolute modules for Jason Tennyson's Thesis.
// This design is made for a PSoC CY8C29466-24PXI.  It is to be used for evaluation
// of functionality on a common PSoC evaluation board.
//
// Packet Structure
// ----------------
// START BYTE/START BYTE/SOURCE ID BYTE/DESTINATION ID BYTE/COMMAND TYPE/PARAM 1/.../PARAM N/END TRANSMIT

#include <m8c.h>        	// Part-specific constants and macros.
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules.
#include "psocdynamic.h"	// Required for dynamically swapping configurations at run time.

// These are declarations of all of the timer interrupts that are used for all configurations.
#pragma interrupt_handler WAIT_NC_TIMEOUT_ISR
#pragma interrupt_handler TX_01234_TIMEOUT_ISR
#pragma interrupt_handler CHILD_1_TIMEOUT_ISR
#pragma interrupt_handler CHILD_2_TIMEOUT_ISR
#pragma interrupt_handler CHILD_3_TIMEOUT_ISR
#pragma interrupt_handler CHILD_4_TIMEOUT_ISR
#pragma interrupt_handler HELLO_TIMEOUT_ISR
#pragma interrupt_handler INIT_TIMEOUT_ISR
#pragma interrupt_handler SERVO_TX_TIMEOUT_ISR

// These defines are used as parameters of the configToggle function.  Passing one of
// these identifiers to configToggle will put the chip in that device configuration.
#define		WAIT						(1)
#define		MY_RESPONSE					(2)
#define 	RESPONSE_1					(3)
#define 	RESPONSE_2					(4)
#define 	RESPONSE_3					(5)
#define 	RESPONSE_4					(6)
#define		HELLO_MODE					(7)
#define		INITIALIZE					(8)
#define		SERVO_COMM					(9)

// These defines are used as comparisons to find what port the next module connected to.
#define		PORT_A						('A')
#define		PORT_B						('B')
#define		PORT_C						('C')
#define		PORT_D						('D')

// These defines are used as transmission indicators for transmissions between PSoC controllers.
#define		START_TRANSMIT				(252)	// Indicates the beginning of a transmission.
#define		END_TRANSMIT				(253)	// Indicates the end of a transmission.
#define		HELLO_BYTE					(200)	// Indicates master is ready to talk.
#define		ID_ASSIGNMENT				(201)	// Indicates an ID assignment from the master.
#define		ID_ASSIGN_OK				(202)	// Indicates an ID assignment is complete.
#define		PING						(203)	// Indicates a ping message to or from the master.
#define		CLEAR_CONFIG				(204)	// Indicates that the master is asking for a config clear.
#define		CONFIG_CLEARED				(205)	// Indicates that a module has cleared its own config.
#define		MASTER_ID					(0)		// The master node's ID.
#define		DEFAULT_ID					(251)	// The ID that all modules start with.
#define		BROADCAST					(254)	// The broadcast ID for all controllers and servos.

// SERVO DEFINES
// These numbers can all be found in the AX-12+ datasheet.
// These defines cover the range of IDs these servos are capable of.
#define		SERVO_ID_MIN				(0)		// This is the lowest servo ID possible.
#define		SERVO_ID_MAX				(253)	// This is the highest servo ID possible.
// These defines are servo transmission indicators.
#define		SERVO_START					(255)	// This is the start byte for a servo transmission.
// These defines are used to fill in the length parameter for a given command type.  These are the only
// lengths used by this controller for servo configuration purposes.  It is worth noting that any type
// and length of command can be issued from the master after configuration is complete.
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
// These defines cover all of the status return level possibilities.
#define		STATUS_RET_NEVER			(0)		// Only respond to ping commands.
#define		STATUS_RET_READ				(1)		// Only respond to read data commands (recommended).
#define		STATUS_RET_ALL				(2)		// Respond to every command.

// This is the number of attempts we make to contact the servo per sweep of attempts before
// writing to its EEPROM in an attempt to alter settings that keep us from communicating.
#define		SERVO_COMM_ATTEMPTS			(10)
// This is the number of times we do a loop of SERVO_COMM_ATTEMPTS.  We would like this to be at least 2.
// This is because we do an EEPROM write after the first unsuccessful loop of SERVO_COMM_ATTEMPTS.
// If we don't then do at least one more loop, the EEPROM write was done for no reason.
#define		SERVO_COMM_LOOPS			(5)

// This is the status return level, which is set to one of the possible status return values above.
// We want the status return level to be return on read commands only so that we don't have garbage
// return packets flying around every time we tell the servo to move.
#define		STATUS_RET_LEVEL			(STATUS_RET_READ)

// This function receives a mode identifier as a parameter and toggles the system configuration.
void configToggle(int mode);
// This function unloads all configurations.  This should only be needed at startup.
void unloadAllConfigs(void);
// This function unloads the configuration corresponding to the number passed to it.
void unloadConfig(int config_num);
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
// This function waits for a known child's response to a command to that child from the master.
int childResponse(void);
// This function does everything it can to find the servo attached to this controller.
void servoFinder(void);
// This function carries out the passed servo instruction.
void servoInstruction(char id, char length, char instruction, char address, char value);
// This function does a simple for loop to stall and make doubly sure that the transmission finished.
// It is meant to be used as a definite amount of wait time after the transmission complete flag is set.
void xmitWait(void);

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

char SERVO_ID;				// Stores the ID of the servo inside of this module.

void main()
{	
	// Initial value assignment for variables of importance.
	CHILD = 0;				// There is no child yet.
	CONFIGURED = 0;			// This module is not configured yet.
	TIMEOUT = 0;			// Set the timeout flag low to start.
	COMMAND_PARAM = 0;		// There is no parameter yet.
	STATE = 0;				// There is no state yet.
	ID = DEFAULT_ID;		// Set the ID of this controller to the default to start with.

	M8C_EnableGInt;			// Turn on global interrupts for the transmission timeout timer.
	
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); // Activate GPIO ISR
	
	// Find the servo that is inside of this module.
	servoFinder();
	
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

// This function transmits a response to a hello command from the master.
void sayHello(void)
{	
	configToggle(MY_RESPONSE);		// Switch to response mode.
	
	// Transmit a hello response to the master node.
	TX_014_PutChar(START_TRANSMIT);	// Start byte one
	TX_014_PutChar(START_TRANSMIT);	// Start byte two
	TX_014_PutChar(ID);				// My ID (source)
	TX_014_PutChar(MASTER_ID);		// Master ID (destination)
	TX_014_PutChar(HELLO_BYTE);		// This is a hello command.
	TX_014_PutChar(CHILD);			// Sends child port value, default 0.
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission.
	TX_014_PutChar(END_TRANSMIT);	// This is the end of this transmission.
	
	// Wait for the transmission to finish.
	while(!(TX_014_bReadTxStatus() & TX_014_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();

	configToggle(WAIT);				// Switch back to wait mode.
}

// This function receives a mode flag and switches the microcontroller to the
// desired hardware configuration.
void configToggle(int mode)
{	
	// Set the pins high and disconnect from the global bus.
	// This keeps false start bits from happening while we swap configs.
	PRT0DR |= 0b00011111;	// Set pins P00 through P04 high.
	PRT0GS &= 0b11100000;	// Disconnect pins P00 through P04 from the global bus.
	
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
	
	// Go through the list of possible modes until we find the one that was passed in to us.
	// Then, load that configuration and initialize whatever needs to be initialized.
	if(mode == WAIT)
	{
		// Load the wait receiver configuration.  This is the receiver configuration used after
		// initialization is complete.  It listens and forwards everything it hears.
		LoadConfig_waiting();
		
		// Start the receivers.
		WAIT_RECV_Start(WAIT_RECV_PARITY_NONE);
		RX8_2_Start(RX8_2_PARITY_NONE);
		
		// Set the current state.
		STATE = WAIT;
	}
	else if(mode == MY_RESPONSE)
	{
		// Load the transmitter configuration.  This is for transmitting messages on all ports.
		LoadConfig_my_response();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start the transmitters.
		TX_014_Start(TX_014_PARITY_NONE);	// Transmits on P00, P01, and P04.
		TX_23_Start(TX_23_PARITY_NONE);		// Transmits on P02 and P03.
		
		TX_01234_TIMEOUT_EnableInt();		// Make sure interrupts are enabled.
		TX_01234_TIMEOUT_Start();			// Start the timer.
		
		// Do nothing while we wait for one timeout period (1 ms).
		// This is to allow everyone to get in the right configuration before talking.
		while(!TIMEOUT) { }
		
		TX_01234_TIMEOUT_Stop();			// Stop the timer.
		TIMEOUT = 0;						// Reset the timeout flag.
	
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
		
		CHILD_1_TIMEOUT_EnableInt();		// Make sure interrupts are enabled.
		CHILD_1_TIMEOUT_Start();			// Start the timer.
		
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
		
		CHILD_2_TIMEOUT_EnableInt();		// Make sure interrupts are enabled.
		CHILD_2_TIMEOUT_Start();			// Start the timer.
		
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
		
		CHILD_3_TIMEOUT_EnableInt();		// Make sure interrupts are enabled.
		CHILD_3_TIMEOUT_Start();			// Start the timer.
		
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
		
		CHILD_4_TIMEOUT_EnableInt();		// Make sure interrupts are enabled.
		CHILD_4_TIMEOUT_Start();			// Start the timer.
		
		// Set the current state.
		STATE = RESPONSE_4;
	}
	else if(mode == HELLO_MODE)
	{
		// Load the hello wait mode.  This is for listening on all ports for a hello response.
		LoadConfig_hello();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// The seemingly unnecessary brackets around each line are unfortunately needed.
	
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
		// Load the configuration for initialization.  This config listens but does not forward.
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
	else if(mode == SERVO_COMM)
	{
		// Load the configuration for servo communication.  This config only transmits on P00.
		LoadConfig_servo_transmit();
		
		// Clear the timeout flag.
		TIMEOUT = 0;
		
		// Start the transmitter.
		SERVO_TX_Start(SERVO_TX_PARITY_NONE);
		
//		SERVO_TX_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
//		SERVO_TX_TIMEOUT_Start();		// Start the timer.
//		
//		while(!TIMEOUT)
//		{
//			// Do nothing while we wait for one timeout period.
//			// This is to allow everyone to get in the right configuration before talking.
//		}
//		
//		SERVO_TX_TIMEOUT_Stop();		// Stop the timer.
//		TIMEOUT = 0;					// Reset the timeout flag.
	
		// Set the current state.
		STATE = SERVO_COMM;
	}
	
	// If this module is configured, talk on all pins for potential children.
	if(CONFIGURED)
	{
		PRT0GS |= 0b00011111;	// Connect all pins to the global bus.
		PRT2DR &= 0b11111110;	// Turn on the LED (active low).
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
	int i = 0;			// This integer is used for looping through the remaining bytes of commands.
	char tempByte = 0;	// This byte is used to store each byte for comparison as it comes in.
	
	// This conditional checks which configuration is loaded and uses the proper devices to
	// read a transmission and store the important information from that transmission.
	if(STATE == WAIT)
	{	
		// In wait mode, the only thing that progresses things forward is a master node transmission.
		// With this being the case, we use a blocking operation to sit and wait for a byte.
		tempByte = WAIT_RECV_cGetChar();
		
		// If a transmission has started for either a controller or a servo...
		if(tempByte == START_TRANSMIT)
		{
			// While we keep reading start bytes, sit and spin.
			while(tempByte == START_TRANSMIT)
			{
				tempByte = WAIT_RECV_cGetChar();
			}
			
			// The tempByte variable contains the source ID.  If the source is good, store all bytes.
			if(tempByte == MASTER_ID)
			{
				COMMAND_SOURCE = tempByte;
				COMMAND_DESTINATION = WAIT_RECV_cGetChar();
				COMMAND_TYPE = WAIT_RECV_cGetChar();
				COMMAND_PARAM = WAIT_RECV_cGetChar();
				
				return 1;
			}
		}
		else if(tempByte == SERVO_START)
		{
			// While we keep reading start bytes, sit and spin.
			while(tempByte == SERVO_START)
			{
				tempByte = WAIT_RECV_cGetChar();
			}
			
			// We assume (and hopefully rightly so) that this is a command from master.
			COMMAND_SOURCE = MASTER_ID;
			// The first parameter after the servo start is the destination.
			COMMAND_DESTINATION = tempByte;
			// The second parameter after the servo start is the command length.  We need to
			// know this length so that we know how long to wait to let the whole string through.
			tempByte = WAIT_RECV_cGetChar();
			// Now we store the command type.  If it is a read, we have special duties.
			COMMAND_TYPE = WAIT_RECV_cGetChar();
			
			// This basically waits for the rest of the command to pass through.
			for(i = 0; i < (tempByte - 1); i++)
			{
				WAIT_RECV_cGetChar();
			}
				
			return 1;
		}
	}
	else if(STATE == HELLO_MODE)
	{
		// Check all of the ports for a start byte.  Only one port will produce one.
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
		// Let the transmission through.
		if(CHILD_1_cReadChar())
		{
			tempByte = CHILD_1_cGetChar();
			
			if(tempByte == START_TRANSMIT)
			{
				// Wait for the end transmit byte to go through.
				while(CHILD_1_cReadChar() != END_TRANSMIT) { }
			}
			else if(tempByte == SERVO_START)
			{
				// While we keep reading start bytes, sit and spin.
				while(tempByte == SERVO_START)
				{
					tempByte = CHILD_1_cGetChar();
				}
				
				// The second parameter after the servo start is the command length.  We need to
				// know this length so that we know how long to wait to let the whole string through.
				tempByte = CHILD_1_cGetChar();
				
				// This basically waits for the rest of the command to pass through.
				for(i = 0; i < tempByte; i++)
				{
					CHILD_1_cGetChar();
				}
			}
			
			return 1;
		}
	}
	else if(STATE == RESPONSE_2)
	{
		// Let the transmission through.
		if(CHILD_2_cReadChar())
		{
			tempByte = CHILD_2_cGetChar();
			
			if(tempByte == START_TRANSMIT)
			{
				// Wait for the end transmit byte to go through.
				while(CHILD_2_cReadChar() != END_TRANSMIT) { }
			}
			else if(tempByte == SERVO_START)
			{
				// While we keep reading start bytes, sit and spin.
				while(tempByte == SERVO_START)
				{
					tempByte = CHILD_2_cGetChar();
				}
				
				// The second parameter after the servo start is the command length.  We need to
				// know this length so that we know how long to wait to let the whole string through.
				tempByte = CHILD_2_cGetChar();
				
				// This basically waits for the rest of the command to pass through.
				for(i = 0; i < tempByte; i++)
				{
					CHILD_2_cGetChar();
				}
			}
			
			return 1;
		}
	}
	else if(STATE == RESPONSE_3)
	{
		// Let the transmission through.
		if(CHILD_3_cReadChar())
		{
			tempByte = CHILD_3_cGetChar();
			
			if(tempByte == START_TRANSMIT)
			{
				// Wait for the end transmit byte to go through.
				while(CHILD_3_cReadChar() != END_TRANSMIT) { }
			}
			else if(tempByte == SERVO_START)
			{
				// While we keep reading start bytes, sit and spin.
				while(tempByte == SERVO_START)
				{
					tempByte = CHILD_3_cGetChar();
				}
				
				// The second parameter after the servo start is the command length.  We need to
				// know this length so that we know how long to wait to let the whole string through.
				tempByte = CHILD_3_cGetChar();
				
				// This basically waits for the rest of the command to pass through.
				for(i = 0; i < tempByte; i++)
				{
					CHILD_3_cGetChar();
				}
			}
			
			return 1;
		}
	}
	else if(STATE == RESPONSE_4)
	{
		// Let the transmission through.
		if(CHILD_4_cReadChar())
		{
			tempByte = CHILD_4_cGetChar();
			
			if(tempByte == START_TRANSMIT)
			{
				// Wait for the end transmit byte to go through.
				while(CHILD_4_cReadChar() != END_TRANSMIT) { }
			}
			else if(tempByte == SERVO_START)
			{
				// While we keep reading start bytes, sit and spin.
				while(tempByte == SERVO_START)
				{
					tempByte = CHILD_4_cGetChar();
				}
				
				// The second parameter after the servo start is the command length.  We need to
				// know this length so that we know how long to wait to let the whole string through.
				tempByte = CHILD_4_cGetChar();
				
				// This basically waits for the rest of the command to pass through.
				for(i = 0; i < tempByte; i++)
				{
					CHILD_4_cGetChar();
				}
			}
			
			return 1;
		}
	}
	else if(STATE == INITIALIZE)
	{
		if(INIT_RX_cReadChar() == SERVO_START)
		{
			// If we definitely have a transmission starting, grab all bytes from the rx buffer
			// and store them in the proper variables for actions to be taken later.
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
	if(COMMAND_TYPE == HELLO_BYTE)		// The master is probing for new modules.
	{
		if(!CONFIGURED)
		{
			// Announce this module's presence if not configured.
			sayHello();
		}
		else if(!CHILD)
		{
			// Listen for children if we have none.
			if(childListen())
			{
				// If a child was heard saying hello, forward the command with the port number added.
				sayHello();
			}
		}
		else if(CHILD)
		{
			// If you have a child established, listen to that child.
			childResponse();
		}
	}
	else if(COMMAND_TYPE == PING)		// The master is trying to find a module that is configured.
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
				
				// If the servo ID doesn't match what we want, change it to match.
				if(ID != SERVO_ID)
				{
					// These are our index variables for communication attempt timeouts.
					int i;
					int j;
					
					//while(ID != SERVO_ID)
					
					for(j = 0; j < SERVO_COMM_LOOPS; j++)
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
											// Set j such that we exit the outer loop as well.
											j = SERVO_COMM_LOOPS;
											// Store the ID value.
											SERVO_ID = ID;
										}
									}
								}
							}
						}
					}	
				}
				
				if(ID != SERVO_ID)
				{
					// Toggle back to normal wait mode.
					configToggle(WAIT);
				}
				else
				{
					// Let the master node know that you got the ID assignment.
					assignedID();
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
	else if((COMMAND_TYPE == PING_SERVO) || (COMMAND_TYPE == READ_SERVO))
	{
		if(COMMAND_DESTINATION > ID)
		{
			// Allow the child response through.
			childResponse();
		}
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
	
	// Make completely sure we're done.
	xmitWait();
	
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
	
	// Make completely sure we're done.
	xmitWait();
	
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
	UnloadConfig_servo_transmit();
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
	else if(config_num == SERVO_COMM)
	{
		UnloadConfig_servo_transmit();
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
	
	// Make completely sure we're done.
	xmitWait();
	
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
// returned for the desired status return level defined at the top of this file.
void servoFinder(void)
{				
	// Index variables for incrementing and checking against the maximum servo comm attempts.
	int i = 0;
	int j = 0;
	
	// Integer used as a flag so that EEPROM writes aren't done more than once.
	int flashWrite = 0;
	
	// Create a status return level variable and set it to an out of range value initially.
	char status_return_level = 3;
	
	// Start with a servo ID of 255 (out of valid range).
	SERVO_ID = SERVO_START;
	
	// Sit here and wait until we get a valid servo ID.
//	while(SERVO_ID == SERVO_START)

	// This for loop will loop SERVO_COMM_LOOPS number of times and ping the servo SERVO_COMM_ATTEMPTS
	// number of times in each loop (unless stopped short due to early success).  If this fails for the
	// first round of pings, a broadcast reset will be performed to reset the servo.  This is done
	// because we assume that the baud rate is matching up, but the servo's return delay time is too
	// fast for the controller to switch into receive mode to read the response.  The default return
	// delay time is 500 microseconds. If we loop for SERVO_COMM_LOOPS number of times and still don't
	// see anything, we assume that there is something is too wrong for us to fix.
	for(j = 0; j < SERVO_COMM_LOOPS; j++)
	{	
		// Ping SERVO_COMM_ATTEMPTS times to try and extract the servo ID.
		for(i = 0; i < SERVO_COMM_ATTEMPTS; i++)
		{
			// Send a ping out for any servo connected to me (will only be one).
			servoInstruction(BROADCAST, PING_LENGTH, PING_SERVO, 0, 0);
			
			// Wait for either a timeout or a valid servo ID (which will trigger a timeout).
			while(!TIMEOUT)
			{	
				if(commandReady())
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
						// Set the outer index variable to 2 to not attempt again for no reason.
						j = SERVO_COMM_LOOPS;
					}
					else
					{
						// Exit this while loop and try to ping again.
						TIMEOUT = 1;
					}
				}
			}
		}
		
		// If we didn't get a response and haven't written to the flash of the
		// servo (first time through), send out a broadcast reset.
		if((SERVO_ID == SERVO_START) && (!flashWrite))
		{
			// Set the flash write flag so that we only do this once per power cycle.
			flashWrite = 1;
			
			// Send out a broadcast reset so that we know that the response time interval
			// is large enough (default delay time for a servo is 500 microseconds).
			servoInstruction(BROADCAST, RESET_LENGTH, RESET_SERVO, 0, 0);
		}
	}
	
	// Reset flash write flag.
	flashWrite = 0;
	
	// If we have a valid servo ID, set the status return level.  If we don't, just skip this
	// because all hope is lost.
	if(SERVO_ID < BROADCAST)
	{
	
		// Check to see if the status return level is equal to the level previously defined.
		//while(status_return_level != STATUS_RET_LEVEL)
	
	
		// This for loop will loop SERVO_COMM_LOOPS number of times and poll for the servo's status
		// return level SERVO_COMM_ATTEMPTS number of times in each loop (unless stopped short due
		// to early success).  If this fails for the first iteration, or we read a status return level
		// other than what we want, we will attempt to write the desired status return level onto the servo.
		for(j = 0; j < SERVO_COMM_LOOPS; j++)
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
								// Set the outer loop's variable so that we don't loop again.
								j = SERVO_COMM_LOOPS;
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
		
			// If we didn't get a good response and haven't written to the flash of the servo,
			// force a change in the status return level with an EEPROM write.
			if((status_return_level != STATUS_RET_LEVEL) && (!flashWrite))
			{	
				flashWrite = 1;
				
				// Try to force the return status to what we want.
				servoInstruction(SERVO_ID, WRITE_LENGTH, WRITE_SERVO, STATUS_RET_ADDRESS, STATUS_RET_LEVEL);
			}
		}
		
		if(status_return_level != STATUS_RET_LEVEL)
		{
			// Break on purpose to show that the status return is not correct.
			while(1)
			{
			
			}
		}
	}
	else
	{
		// Purposely break the module to show that we did not resolve the communication with our servo.
		while(1)
		{
		
		}
	}

// In between these comments is a function!

	configToggle(INITIALIZE);	// Switch back to initialize mode to do this timeout routine.
	
	// For SERVO_COMM_ATTEMPTS*SERVO_COMM_LOOPS cycles, let the other controllers find their servos.
	// The reason we loop this many times is to allow for a possible worst case scenario of setup
	// time to complete.
	for(i = 0; i < (SERVO_COMM_ATTEMPTS*SERVO_COMM_LOOPS); i++)
	{
		while(!TIMEOUT)
		{
			// Do nothing while we wait for one timeout period.
			// This is to allow everyone to get in the right configuration before talking.
		}
		
		TIMEOUT = 0;			// Reset the timeout flag.
	}
	
	INIT_TIMEOUT_Stop();		// Stop the timer.
	TIMEOUT = 0;				// Reset the timeout flag.

// In between these comments is a function!

	// Switch to wait for the master node to speak to you.
	configToggle(WAIT);
}

// This function receives a destination, command length, instruction type, address, and value.
// With these parameters, the function sends a packet to the communication bus.
void servoInstruction(char id, char length, char instruction, char address, char value)
{
	char checksum;
	
	// Toggle into transmit mode.
	configToggle(SERVO_COMM);
	
	// Calculate the checksum value for our servo communication.
	checksum = 255-((id + length + instruction + address + value)%256);
	
	// Talk to the servo.
	if(instruction == PING_SERVO)
	{
		SERVO_TX_PutChar(SERVO_START);	// Start byte one
		SERVO_TX_PutChar(SERVO_START);	// Start byte two
		SERVO_TX_PutChar(id);			// Servo ID
		SERVO_TX_PutChar(length);		// The instruction length.
		SERVO_TX_PutChar(instruction);	// The instruction to carry out.
		SERVO_TX_PutChar(checksum);		// This is the checksum.
	}
	else
	{
		SERVO_TX_PutChar(SERVO_START);	// Start byte one
		SERVO_TX_PutChar(SERVO_START);	// Start byte two
		SERVO_TX_PutChar(id);			// Servo ID
		SERVO_TX_PutChar(length);		// The instruction length.
		SERVO_TX_PutChar(instruction);	// The instruction to carry out.
		SERVO_TX_PutChar(address);		// The address to read/write from/to.
		SERVO_TX_PutChar(value);		// The value to write or number of bytes to read.
		SERVO_TX_PutChar(checksum);		// This is the checksum.
	}
	
	// Wait for the transmission to finish.
	while(!(SERVO_TX_bReadTxStatus() & SERVO_TX_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
	
	// Switch back to wait for a servo response.
	configToggle(INITIALIZE);
}

void xmitWait(void)
{
	int i;
	
	for(i = 0; i < 25; i++)
	{
		// Does nothing and wastes approximately 50 microseconds.
	}
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

void SERVO_TX_TIMEOUT_ISR(void)
{
	TIMEOUT = 1;	// Set the timeout flag.
	M8C_ClearIntFlag(INT_CLR0,SERVO_TX_TIMEOUT_INT_MASK);
}