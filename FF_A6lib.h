/*!
	\file
	\brief	Implements a fully asynchronous SMS send/receive in PDU mode for A6/GA6 modem class
	\author	Flying Domotic
	\date	August 16th, 2023

	Have a look at FF_A6lib.cpp for details

*/

#ifndef FF_A6lib_h
#define FF_A6lib_h

#include <Arduino.h>

// Constants
#define A6_CMD_TIMEOUT 4000									//!< Standard AT command timeout (ms)
#define MAX_SMS_NUMBER_LEN 20								//!< SMS number max length
#define MAX_SMS_DATE_LEN 25									//!< SMS date max length
#define MAX_SMS_MESSAGE_LEN 180								//!< SMS message max length
#define MAX_ANSWER 250										//!< AT command answer max length
#define DEFAULT_ANSWER "\r\nOK\r\n"							//!< AT command default answer
#define SMS_READY_MSG "SMS Ready"							//!< SMS ready signal
#define SMS_INDICATOR "+CMT: "								//!< SMS received indicator
#define CSCA_INDICATOR "+CSCA:"								//!< SCA value indicator

// Enums
#define A6_OK 0
#define A6_RUNNING 1
#define A6_TIMEOUT 1
#define A6_TOO_LONG 2
#define A6_BAD_ANSWER 3
#define A6_CM_ERROR 4
#define A6_NEED_INIT 5

#define A6_IDLE 0
#define A6_SEND 1
#define A6_RECV 2
#define A6_STARTING 3

// Class definition
class FF_A6lib {
public:
	// public class
	/*!	\class FF_A6lib
		\brief Implements a fully asynchronous SMS send/receive in PDU mode for A6/GA6 modem class

		This class allows asynchronously sending/receiving SMS using an A6 or GA6 (and probably others) modem using PDU mode.

		Messages are in UTF-8 format and automatically converted into GSM7 (160 characters) or UCS2 (70 characters).

		A callback routine in your program will be called each time a SMS is received.

		You also may send SMS directly.

		By default, logging/debugging is done through FF_TRACE macros, allowing to easily change code.

		It may also be used with FF_WebServer class, as containing routines to map with it.

		You may have a look at https://github.com/FlyingDomotic/FF_SmsServer which shows how to use it

	*/
	FF_A6lib();

	// Public routines (documented in FF_A6lib.cpp)
	void begin(long baudRate);
	void doLoop(void);
	void debugState(void);
	void sendSMS(const char* number, const char* text);
	void registerSmsCb(void (*readSmsCallback)(int __index, const char* __number, const char* __date, const char* __message));
	void registerLineCb(void (*recvLineCallback)(const char* __answer));
	void deleteSMS(int index, int flag);
	void sendAT(const char* command);
	void sendEOF(void);
	bool needRestart(void);
	void setRestart(bool restartFlag);
	bool isIdle(void);
	bool isSending(void);
	bool isReceiving(void);

	// Public variables
	bool debugFlag;											//!< Show debug messages flag
	bool traceFlag;											//!< Show trace messages flag
	bool ignoreErrors;										//!< Ignore errors flag

private:
	// Private routines (documented in FF_A6lib.cpp)
	void sendCommand(const char *command, void (FF_A6lib::*nextStep)(void)=NULL, const char *resp=DEFAULT_ANSWER, unsigned long cdeTimeout = A6_CMD_TIMEOUT);
	void sendCommand(const uint8_t command, void (FF_A6lib::*nextStep)(void)=NULL, const char *resp=DEFAULT_ANSWER, unsigned long cdeTimeout = A6_CMD_TIMEOUT);
	void waitMillis(unsigned long waitMs, void (FF_A6lib::*nextStep)(void)=NULL);
	void waitSmsReady(unsigned long waitMs, void (FF_A6lib::*nextStep)(void)=NULL);
	int readBuffer(char* reply, int bufferSize);
	void startReset(void);
	void setReset(void);
	void echoOff(void);
	void detailedErrors(void);
	void setCallerId(void);
	void setTextMode(void);
	void getSca(void);
	void gotSca(void);
	void setHeaderDetails(void);
	void waitSmsReady(void);
	void setIndicOff(void);
	void detailedRegister(void);
	void deleteReadSent(void);
	void initComplete(void);
	void sendSMStext(void);
	void setIdle(void);
	void executeSmsCb(void);
	void enterRoutine(const char* routineName);
	void readSmsHeader(const char* msg);
	void readSmsMessage(const char* msg);

	// Private variables
	unsigned long startTime;								//!< Last command start time
	unsigned int commandCount;								//!< Count of commands sent
	unsigned int resetCount;								//!< Count of GSM reset
	unsigned int restartCount;								//!< Count of successful GSM restart
	unsigned int smsReadCount;								//!< Count of SMS read
	unsigned int smsForwardedCount;							//!< Count of SMS analyzed
	unsigned int smsSentCount;								//!< Count of SMS sent
	bool smsReady;											//!< True if "SMS ready" seen
	void (FF_A6lib::*nextStepCb)(void);						//!< Callback for next step in command execution
	void (*readSmsCb)(int __index, const char* __number, const char* __date, const char* __message); //!< Callback for readSMS
	void (*recvLineCb)(const char* __answer);				//!< Callback for received line
	int index;												//!< Index of last read SMS
	int restartReason;										//!< Last restart reason
	unsigned long gsmTimeout;								//!< Timeout value (ms)
	int gsmStatus;											//!< Last command status
	int gsmIdle;											//!< GSM is idle flag
	bool inReceive;											//!< Are we receiving answer?
	bool inWait;											//!< Are we waiting for some time?
	bool inWaitSmsReady;									//!< Are we waiting for SMS Ready?
	bool restartNeeded;										//!< Restart needed flag
	bool nextLineIsSmsMessage;								//!< True if next line will be an SMS message (just after SMS header)
	char lastAnswer[MAX_ANSWER];							//!< Contains the last GSM command anwser
	char expectedAnswer[10];								//!< Expected answer to consider command ended
	char number[MAX_SMS_NUMBER_LEN];						//!< Number of last read SMS
	char date[MAX_SMS_DATE_LEN];							//!< Sent date of last read SMS
	char message[MAX_SMS_MESSAGE_LEN];						//!< Message of last read SMS
	char lastCommand[60];									//!< Last command sent
	char lastCommandInError[60];							//!< Last command generating an error
	char lastErrorMessage[60];								//!< Last error seen
};
#endif