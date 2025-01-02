/*!
	\file
	\brief	Implements a fully asynchronous SMS send/receive in PDU mode for A6/GA6 modem class
	\author	Flying Domotic
	\date	September 9th, 2023

	Have a look at FF_A6lib.cpp for details

*/

#ifndef FF_A6lib_h
#define FF_A6lib_h

#include <Arduino.h>

// Constants
#define A6_CMD_TIMEOUT 4000									//!< Standard AT command timeout (ms)
#define MAX_SMS_NUMBER_LEN 20								//!< SMS number max length
#define MAX_ANSWER 500										//!< AT command answer max length
#define DEFAULT_ANSWER "OK"									//!< AT command default answer
#define SMS_READY_MSG "SMS Ready"							//!< SMS ready signal
#define SMS_INDICATOR "+CMT: "								//!< SMS received indicator
#define CSCA_INDICATOR "+CSCA:"								//!< SCA value indicator
//#define A6LIB_KEEP_CR_LF									//!< Keep CR & LF in displayed messages (by default, thry're replaced by ".")

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

		Messages are in UTF-8 format and automatically converted into GSM7 (160 characters) or UCS-2 (70 characters).

		A callback routine in your program will be called each time a SMS is received.

		You also may send SMS directly.

		By default, logging/debugging is done through FF_TRACE macros, allowing to easily change code.

		It may also be used with FF_WebServer class, as containing routines to map with it.

		You may have a look at https://github.com/FlyingDomotic/FF_SmsServer which shows how to use it

		By default, A6lib uses (G)A6 modem connected on D8(TX) and D7(RX).
		
		You can use #define USE_DIRECT_CONNECTIONS_FOR_A6LIB if A6is connected directly to TX/RX. Don't forget to set Serial.setDebugOutput to avoid garbage.
		
		You may also use #define USE_SOFTSERIAL_FOR_A6LIB to use SoftwareSerial instead of Serila.swap(), but 
			be aware that program will crash if you're using any asynchronous libraries (like espAsyncxxx).

	*/
	FF_A6lib();

	// Public routines (documented in FF_A6lib.cpp)
	void begin(long baudRate, int8_t rxPin, int8_t txPin);
	void doLoop(void);
	void debugState(void);
	void sendSMS(const char* number, const char* text);
	void sendOneSmsChunk(const char* number, const char* text, const unsigned short msgId = 0, const unsigned char msgCount = 0, const unsigned char msgIndex = 0);
	void registerSmsCb(void (*readSmsCallback)(int __index, const char* __number, const char* __date, const char* __message));
	void registerLineCb(void (*recvLineCallback)(const char* __answer));
	void deleteSMS(int index, int flag);
	void sendAT(const char* command);
	void sendEOF(void);
	bool needRestart(void);
    int getRestartReason(void);
	void setRestart(bool restartFlag);
	bool isIdle(void);
	bool isSending(void);
	bool isReceiving(void);
	const char* getLastReceivedNumber(void);
	const char* getLastReceivedDate(void);
	const char* getLastReceivedMessage(void);
	const char* getLastSentNumber(void);
	const char* getLastSentDate(void);
	const char* getLastSentMessage(void);
	uint8_t getGsm7EquivalentLen(const uint8_t c1, const uint8_t c2, const uint8_t c3);
	uint16_t ucs2MessageLength(const char* text);

	// Public variables
	bool debugFlag;											//!< Show debug messages flag
	bool traceFlag;											//!< Show trace messages flag
	bool traceEnterFlag;									//!< Show each routine entering flag
	bool ignoreErrors;										//!< Ignore errors flag

private:
	// Private routines (documented in FF_A6lib.cpp)
	void sendCommand(const char *command, void (FF_A6lib::*nextStep)(void)=NULL, const char *resp=DEFAULT_ANSWER, unsigned long cdeTimeout = A6_CMD_TIMEOUT);
	void sendCommand(const uint8_t command, void (FF_A6lib::*nextStep)(void)=NULL, const char *resp=DEFAULT_ANSWER, unsigned long cdeTimeout = A6_CMD_TIMEOUT);
	void waitMillis(unsigned long waitMs, void (FF_A6lib::*nextStep)(void)=NULL);
	void waitSmsReady(unsigned long waitMs, void (FF_A6lib::*nextStep)(void)=NULL);
	void sendNextSmsChunk(void);
	void openModem(long baudRate);
	void setReset(void);
	void echoOff(void);
	void detailedErrors(void);
	void setCallerId(void);
	void setTextMode(void);
	void getSca(void);
	void gotSca(void);
	void setHeaderDetails(void);
	void waitUntilSmsReady(void);
	void setIndicOff(void);
	void detailedRegister(void);
	void deleteReadSent(void);
	void initComplete(void);
	void sendSMStext(void);
	void setIdle(void);
	void enterRoutine(const char* routineName);
	void readSmsHeader(const char* msg);
	void readSmsMessage(const char* msg);
	void resetLastAnswer(void);

	// Private variables
	unsigned long startTime;								//!< Last command start time
	unsigned int commandCount;								//!< Count of commands sent
	unsigned int resetCount;								//!< Count of GSM reset
	unsigned int restartCount;								//!< Count of successful GSM restart
	unsigned int smsReadCount;								//!< Count of SMS read
	unsigned int smsForwardedCount;							//!< Count of SMS analyzed
	unsigned int smsSentCount;								//!< Count of SMS sent
	int8_t modemRxPin;										//!< Modem RX pin
	int8_t modemTxPin;										//!< Modem TX pin
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
	char lastCommand[30];									//!< Last command sent
	uint16_t gsm7Length;									//!< Size of GSM-7 message (0 for UCS-2 messages)
	unsigned short smsMsgId;								//!< Multi-part message ID (to be incremented for each multi-part message sent)
	uint8_t smsMsgIndex;									//!< Chunk index of current multi-part message
	uint8_t smsMsgCount;									//!< Chunk total count of current multi-part message
	uint8_t smsChunkSize;									//!< Chunk size for this message
	String lastReceivedNumber;								//!< Phone number of last received SMS
	String lastReceivedDate;								//!< Date of last received SMS
	String lastReceivedMessage;								//!< Message of last received SMS
	String lastSentNumber;									//!< Phone number of last SMS sent
	String lastSentDate;									//!< Date of last SMS sent
	String lastSentMessage;									//!< Message of last SMS sent
};
#endif