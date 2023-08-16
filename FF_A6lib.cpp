/*!
	\file
	\brief	Implements a fully asynchronous SMS send/receive in PDU mode for A6/GA6 modem class
	\author	Flying Domotic
	\date	August 16th, 2023
*/

#include <FF_A6lib.h>
#include <stdio.h>
#include <FF_Trace.h>
#include <SoftwareSerial.h>
#define PM													// Define PDU tables in flash
#include <PDUlib.h>											// https://github.com/mgaman/PDUlib

#define PDU_BUFFER_LENGTH 200								// Max workspace length
PDU smsPdu = PDU(PDU_BUFFER_LENGTH);						// Instantiate PDU class

SoftwareSerial a6Serial(D6, D5);							// We use software serial to keep Serial usable

// Class constructor : init some variables
FF_A6lib::FF_A6lib() {
	restartNeeded = true;
	gsmStatus = A6_NEED_INIT;
	restartReason = gsmStatus;
	inReceive = false;
	gsmIdle = A6_STARTING;
	debugFlag = false;
	traceFlag = false;
	nextLineIsSmsMessage = false;
	commandCount = 0;
	resetCount = 0;
	smsReadCount = 0;
	smsForwardedCount = 0;
	smsSentCount = 0;
}

/*!

	\brief	Initialize the GSM connection

	Open the modem connection at initial baud rate

	\param[in]	baudRate: initial modem speed (in bauds)
	\return	none

*/
void FF_A6lib::begin(long baudRate) {
	if (traceFlag) enterRoutine(__func__);
	strncpy_P(lastCommandInError, PSTR("[None]"), sizeof(lastCommandInError));
	strncpy_P(lastErrorMessage, PSTR("[None]"), sizeof(lastErrorMessage));
	restartNeeded = false;
	inReceive = false;
	inWait = false;
	gsmIdle = A6_STARTING;
	a6Serial.end();
	a6Serial.begin(baudRate, SWSERIAL_8N1, D6, D5, false, 255); // Connect ESP:D6(12) on A6:U_TX and ESP:D5(14) on A6:U_RX
	a6Serial.enableIntTx(false);
	startReset();
}

/*!

	\brief	Modem loop (should be called in main loop)

	This routine should be calledd at regular interval in order for the modem code to run (as we're asynchronous)

	\param	none
	\return	none

*/
void FF_A6lib::doLoop(void) {
	char toAdd[250];
	int addLength;
	char *eolPos;

	if (traceFlag) enterRoutine(__func__);

	// Get additional text to add to previously received answer
	addLength = readBuffer(toAdd, sizeof(toAdd));
	if (addLength) {										// Something received.
		if ((strlen(lastAnswer) + addLength) > sizeof(lastAnswer)) {
			if (debugFlag) trace_debug_P("Answer too long: >%s< and >%s<", lastAnswer, toAdd);
			// Answer is too long
			gsmStatus = A6_TOO_LONG;
			memset(lastAnswer, 0, sizeof(lastAnswer));		// Reset last answer
			setIdle();
			return;
		}
		strcat(lastAnswer, toAdd);		// Add to answer
	}

	// Do we have an "SMS Ready" message?
	if (!smsReady && strstr(lastAnswer, SMS_READY_MSG)) {
		if (debugFlag) trace_debug_P("Got SMS Ready");
		smsReady = true;
	}

	if (inReceive) {										// We're waiting for a command answer
		// Is this the expected answer?
		if (strstr(lastAnswer, expectedAnswer)) {
			if (debugFlag) trace_debug_P("Reply in %d ms: >%s<", millis() - startTime, lastAnswer);
			gsmStatus = A6_OK;
			if (nextStepCb) {								// Do we have another callback to execute?
				(this->*nextStepCb)();						// Yes, do it
			} else {
				setIdle();									// No, we just finished.
			}
			return;
		}
		if ((millis() - startTime) >= gsmTimeout) {
			if (ignoreErrors) {
				if (nextStepCb) {								// Do we have another callback to execute?
					(this->*nextStepCb)();						// Yes, do it
				} else {
					setIdle();									// No, we just finished.
				}
				return;
			}
			// Here, we've got a time-out on answer
			strncpy(lastCommandInError, lastCommand, sizeof(lastCommandInError));	// Save last command in error
			if (lastAnswer[0]) {
				strncpy(lastErrorMessage, lastAnswer, sizeof(lastErrorMessage));	// Save last error message
				if (strstr(lastAnswer,"+CMS ERROR") || strstr(lastAnswer,"+CME ERROR")) {
					// This is a CMS or CME answer
					if (debugFlag) trace_debug_P("Error answer: >%s< after %d ms", lastAnswer, millis() - startTime);
					gsmStatus = A6_CM_ERROR;
					restartNeeded = true;
					restartReason = gsmStatus;
					setIdle();
					return;
				} else {									// Unknown answer
					if (debugFlag) trace_debug_P("Bad answer: >%s< after %d ms", lastAnswer, millis() - startTime);
					gsmStatus = A6_BAD_ANSWER;
					restartNeeded = true;
					restartReason = gsmStatus;
					setIdle();
					return;
				}
			} else {										// Time-out without any anwser
				if (debugFlag) trace_debug_P("Timed out after %d ms, received >%s<", millis() - startTime, lastAnswer);
				strncpy(lastErrorMessage, "[TimeOut]", sizeof(lastErrorMessage));	 // Save last error message
				gsmStatus = A6_TIMEOUT;
				restartNeeded = true;
				restartReason = gsmStatus;
				setIdle();
				return;
			}
		}
	} else {												// Not in receive
		eolPos = strstr(lastAnswer, "\r\n");				// Find end of line
		if (eolPos) {										// Found
			eolPos[0] = 0;									// Force zero at EOL (first) position
			if (strlen(lastAnswer)) {						// Answer is not null
				if (debugFlag) trace_debug_P("Answer is >%s<", lastAnswer);
				if (recvLineCb) recvLineCb(lastAnswer);		// Activate callback with answer
				if (nextLineIsSmsMessage) {					// Are we receiving a SMS message?
					readSmsMessage(lastAnswer);				// Yes, read it
					nextLineIsSmsMessage = false;			// Clear flag
				} else {									// Not in SMS reception
					if (strstr(lastAnswer, SMS_INDICATOR)) {// Is this indicating an SMS reception?
						readSmsHeader(lastAnswer);
					} else {								// Can't understand received data
						if (debugFlag) trace_debug_P("Ignoring >%s<", lastAnswer);
					}
				}
			}
			// Remove first part of message
			strncpy(lastAnswer, eolPos + 2, sizeof(lastAnswer));
		}
	}
	if (inWaitSmsReady && smsReady) {
		if (debugFlag) trace_debug_P("End of %d ms SMS ready wait, received >%s<", millis() - startTime, lastAnswer);
		inWait = false;
		inWaitSmsReady = false;
		gsmStatus = A6_OK;
		if (nextStepCb) {								// Do we have another callback to execute?
			(this->*nextStepCb)();						// Yes, do it
		} else {
			setIdle();									// No, we just finished.
		}
		return;
	}

	if (inWait) {
		if ((millis() - startTime) >= gsmTimeout) {
			if (debugFlag) trace_debug_P("End of %d ms wait, received >%s<", millis() - startTime, lastAnswer);
			inWait = false;
			gsmStatus = A6_OK;
			if (nextStepCb) {								// Do we have another callback to execute?
				(this->*nextStepCb)();						// Yes, do it
			} else {
				setIdle();									// No, we just finished.
			}
		}
	}
}

/*!

	\brief	Trace some internal variables values (user for debug)

	This routine dumps (almaost all) variables using trace_info_P macro (usually defined in Ff_TRACE)

	\param	none
	\return	none

*/
void FF_A6lib::debugState(void) {
	if (traceFlag) enterRoutine(__func__);
	trace_info_P("lastCommand=%s", lastCommand);
	trace_info_P("expectedAnswer=%s", expectedAnswer);
	trace_info_P("lastAnswer=%s", lastAnswer);
	trace_info_P("lastCommandInError=%s", lastCommandInError);
	trace_info_P("lastErrorMessage=%s", lastErrorMessage);
	trace_info_P("restartNeeded=%d", restartNeeded);
	trace_info_P("restartReason=%d", restartReason);
	trace_info_P("smsReady=%d", smsReady);
	trace_info_P("gsmIdle=%d", gsmIdle);
	trace_info_P("inReceive=%d", inReceive);
	trace_info_P("gsmTimeout=%d", gsmTimeout);
	trace_info_P("gsmStatus=%d", gsmStatus);
	trace_info_P("index=%d", index);
	trace_info_P("startTime=%d", millis()-startTime);
	trace_info_P("resetCount=%d", resetCount);
	trace_info_P("restartCount=%d", restartCount);
	trace_info_P("commandCount=%d", commandCount);
	trace_info_P("smsReadCount=%d", smsReadCount);
	trace_info_P("smsForwardedCount=%d", smsForwardedCount);
	trace_info_P("smsSentCount=%d", smsSentCount);
	trace_info_P("number=%s", number);
	trace_info_P("date=%s", date);
	trace_info_P("message=%s", message);
	trace_info_P("a6-debugFlag=%d", debugFlag);
	trace_info_P("a6-traceFlag=%d", traceFlag);
}

/*!

	\brief	Queue a SMS for sending

	This routine adds a couple message/number to send message to in queue

	\param[in]	number: phone number to send message to
	\param[in]	text: message to send
	\return	none

*/
void FF_A6lib::sendSMS(const char* number, const char* text) {
	if (traceFlag) enterRoutine(__func__);
	char tempBuffer[50];
	int len = smsPdu.encodePDU(number, text);
	if (len < 0)  {
		trace_error_P("Encode error %d sending SMS to %s >%s<", len, number, text);
		return;
	}

	if (debugFlag) trace_debug_P("Sending SMS to %s >%s<", number, text);
	gsmIdle = A6_SEND;
	smsSentCount++;
	snprintf_P(tempBuffer, sizeof(tempBuffer),PSTR("AT+CMGS=%d"), len);
	sendCommand(tempBuffer, &FF_A6lib::sendSMStext, ">");
}

/*!

	\brief	Register a SMS received callback routine

	This routine register a callback routine to call when a SMS message is received

	Callback routine will be called with 4 parameters:
		(int) index: not used yet
		(char*) number: phone number of SMS sender
		(char*) date: date of received SMS (as delivered by the network)
		(char*) message: received message in UTF-8 encoding

	\param[in]	routine to call when a SMS is received
	\return	none

*/
void FF_A6lib::registerSmsCb(void (*readSmsCallback)(int __index, const char* __number, const char* __date, const char* __message)) {
	if (traceFlag) enterRoutine(__func__);
	readSmsCb = readSmsCallback;
}

/*!

	\brief	Register an answer received callback routine

	This routine register a callback routine to call when an answer is received from modem

	Callback routine will be called with 1 parameter:
		(char*) answer: received answer

	\param[in]	routine to call when a SMS is received
	\return	none

*/
void FF_A6lib::registerLineCb(void (*recvLineCallback)(const char* __answer)) {
	if (traceFlag) enterRoutine(__func__);
	recvLineCb = recvLineCallback;
}

/*!

	\brief	Delete SMS from the storage area

	Tis routine delete (some) SMS from storage using AT+CMGD command
	\param[in]	index as used by AT+CMGD
	\param[in]	flag as used by AT+CMGD
	\return	none

*/
void FF_A6lib::deleteSMS(int index, int flag) {
	if (traceFlag) enterRoutine(__func__);
	char tempBuffer[50];

	snprintf_P(tempBuffer, sizeof(tempBuffer), PSTR("AT+CMGD=%d,%d"), index, flag);
	sendCommand(tempBuffer, &FF_A6lib::setIdle, DEFAULT_ANSWER, 10000);
}

/*!

	\brief	Issue an AT command (to be used as debug as no command tracking is done)

	This routine will send an out of band AT command to modem.

	It could be used to debug/test modem.

	Modem answer is ignored and discarded.

	\param[in]	AT command to be send
	\return	none

*/
void FF_A6lib::sendAT(const char *command) {
	if (traceFlag) enterRoutine(__func__);
	sendCommand(command);
	inReceive = false;
}

/*!

	\brief	Issue an EOF command (to be used as debug as no command tracking is done)

	This routine will send an EOF command to modem.

	It could be used to debug/test modem.

	Modem answer is ignored and discarded.

	\param	none
	\return	none

*/
void FF_A6lib::sendEOF(void) {
	if (traceFlag) enterRoutine(__func__);
	sendCommand(0x1a);
	inReceive = false;
}

/*!

	\brief	Check if modem shoudl be restarted

	This routine check if modem restart has been asked by code.

	\param	none
	\return	true if modem should be restarted, false else

*/
bool FF_A6lib::needRestart(void){
	if (traceFlag) enterRoutine(__func__);
	return restartNeeded;
}

/*!

	\brief	Set the restart flag

	This routine sets the restart required flag to a given value

	\param[in]	value to be set (true or false)
	\return	none

*/
void FF_A6lib::setRestart(bool restartFlag){
	if (traceFlag) enterRoutine(__func__);
	restartNeeded = restartFlag;
}

/*!

	\brief	Checks if modem is idle

	This routine checks if modem is idle (not initializing, sending nor receiving)

	\param	none
	\return	true if modem is idle, false else

*/
bool FF_A6lib::isIdle(void){
	if (traceFlag) enterRoutine(__func__);
	return (gsmIdle == A6_IDLE);
}

/*!

	\brief	Checks if modem is sending something

	This routine checks if modem is sending something (either commands or messages)

	\param	none
	\return	true is modem is sending, false else

*/
bool FF_A6lib::isSending(void){
	if (traceFlag) enterRoutine(__func__);
	return (gsmIdle == A6_SEND);
}

/*!

	\brief	Checks if modem is receiving

	This routine checks if modem is receiving a SMS

	\param	none
	\return	true if a SMS is receiving, false else

*/
bool FF_A6lib::isReceiving(void){
	if (traceFlag) enterRoutine(__func__);
	return (gsmIdle == A6_RECV);
}

/*!

	\brief	[Private] Modem initialization: start sequence

	\param	none
	\return	none

*/
void FF_A6lib::startReset(void) {
	if (traceFlag) enterRoutine(__func__);
	resetCount++;
	smsReady = false;
	// Send attention message
	sendCommand("AT", &FF_A6lib::setReset);
}

/*!

	\brief	[Private] Modem initialization: reset modem to factory defaults

	\param	none
	\return	none

*/
void FF_A6lib::setReset(void) {
	if (traceFlag) enterRoutine(__func__);
	// Reset to factry defaults
	sendCommand("AT&F", &FF_A6lib::echoOff);
}

/*!

	\brief	[Private] Modem initialization: turn echo off

	\param	none
	\return	none

*/
void FF_A6lib::echoOff(void) {
	if (traceFlag) enterRoutine(__func__);
	// Echo off
	sendCommand("ATE0", &FF_A6lib::detailedErrors);
}

/*!

	\brief	[Private] Modem initialization: ask for detailled errors

	\param	none
	\return	none

*/
void FF_A6lib::detailedErrors(void) {
	if (traceFlag) enterRoutine(__func__);
	// Show detailed errors (instead of number)
	sendCommand("AT+CMEE=2", &FF_A6lib::detailedRegister);
}

/*!

	\brief	[Private] Modem initialization: ask for detailled registration

	\param	none
	\return	none

*/
void FF_A6lib::detailedRegister(void) {
	if (traceFlag) enterRoutine(__func__);
	// Set detailed registration
	sendCommand("AT+CREG=2", &FF_A6lib::waitSmsReady);
}

/*!

	\brief	[Private] Modem initialization: wait to receive SMS ready for 30 seconds

	\param	none
	\return	none

*/
void FF_A6lib::waitSmsReady(void) {
	if (traceFlag) enterRoutine(__func__);
	waitSmsReady(30000, &FF_A6lib::setCallerId);
}

/*!

	\brief	[Private] Modem initialization: set caller id on

	\param	none
	\return	none

*/
void FF_A6lib::setCallerId(void) {
	if (traceFlag) enterRoutine(__func__);
	// Set caller ID on
	sendCommand("AT+CLIP=1", &FF_A6lib::setTextMode);
}

/*!

	\brief	[Private] Modem initialization: send SMS in PDU mode (instead of text)

	\param	none
	\return	none

*/
void FF_A6lib::setTextMode(void) {
	if (traceFlag) enterRoutine(__func__);
	// Set SMS to PDU mode
	sendCommand("AT+CMGF=0", &FF_A6lib::setIndicOff);
}

/*!

	\brief	[Private] Modem initialization: set modem indicators

	\param	none
	\return	none

*/
void FF_A6lib::setIndicOff(void) {
	if (traceFlag) enterRoutine(__func__);
	// Turn SMS indicators on
	sendCommand("AT+CNMI=0,2,0,1,1", &FF_A6lib::setHeaderDetails);
}

/*!

	\brief	[Private] Modem initialization: show results details

	\param	none
	\return	none

*/
void FF_A6lib::setHeaderDetails(void) {
	if (traceFlag) enterRoutine(__func__);
	// Show result details
	sendCommand("AT+CSDH=1", &FF_A6lib::getSca);
}

/*!

	\brief	[Private] Modem initialization: ask modem for SCA (sms server) number

	\param	none
	\return	none

*/
void FF_A6lib::getSca(void) {
	if (traceFlag) enterRoutine(__func__);
	// Set encoding to 8 bits
	sendCommand("AT+CSCA?", &FF_A6lib::gotSca, CSCA_INDICATOR);
}

/*!

	\brief	[Private] Modem initialization: extract SCA number and give it to PDU class

	\param	none
	\return	none

*/
void FF_A6lib::gotSca(void) {
	if (traceFlag) enterRoutine(__func__);
	// Extract SCA from message
	char* ptrStart;
	char scaNumber[MAX_SMS_NUMBER_LEN];

	// Parse the response if it contains a valid CSCA_INDICATOR
	ptrStart = strstr(lastAnswer, CSCA_INDICATOR);

	if (ptrStart == NULL) {
		if (debugFlag) trace_debug_P("Can't find %s",CSCA_INDICATOR);
		restartReason = A6_BAD_ANSWER;
		restartNeeded = true;
		return;
	}
	//	First token is +CMT
	char* token = strtok (ptrStart, "\"");
	if (token == NULL) {
		if (debugFlag) trace_debug_P("Can't find first token");
		restartReason = A6_BAD_ANSWER;
		restartNeeded = true;
		return;
	}

	// Second token is is SCA number
	token = strtok (NULL, "\"");
	if (token == NULL) {
		if (debugFlag) trace_debug_P("Can't find second token");
		restartReason = A6_BAD_ANSWER;
		restartNeeded = true;
		return;
	}
	strncpy(scaNumber, token, sizeof(scaNumber));
	trace_debug_P("setting SCA to %s", scaNumber);
	smsPdu.setSCAnumber(scaNumber);
	deleteReadSent();
}

/*!

	\brief	[Private] Modem initialization: delete all received SMS

	\param	none
	\return	none

*/
void FF_A6lib::deleteReadSent(void) {
	if (traceFlag) enterRoutine(__func__);
	// Delete read or sent SMS
	sendCommand("AT+CMGD=1,4", &FF_A6lib::initComplete, DEFAULT_ANSWER, 10000);
}

/*!

	\brief	[Private] Modem initialization: end of initialization

	\param	none
	\return	none

*/
void FF_A6lib::initComplete(void) {
	if (traceFlag) enterRoutine(__func__);
	if (gsmStatus) {
		restartNeeded = true;
		restartReason = gsmStatus;
	} else {
		setIdle();
		trace_debug_P("SMS gateway started, restart count = %d", restartCount);
		restartCount++;
	}
}

/*!

	\brief	[Private] Send a PDU containing a SMS

	\param	none
	\return	none

*/
void FF_A6lib::sendSMStext(void) {
	if (traceFlag) enterRoutine(__func__);

	if (debugFlag) trace_debug_P("Message: %s", smsPdu.getSMS());
	a6Serial.write(smsPdu.getSMS());
	sendCommand('\n', &FF_A6lib::setIdle, "+CMGS:", 10000);
}

/*!

	\brief	[Private] Call SMS received callback routine

	\param	none
	\return	none

*/
void FF_A6lib::executeSmsCb(void) {
	if (traceFlag) enterRoutine(__func__);
	if (readSmsCb) {
		(*readSmsCb)(index, number, date, message);
	}
}

/*!

	\brief	[Private] Wait for SMS ready message a given time

	\param[in]	waitMs: Time (ms) to wait for SMS ready message
	\param[in]	nextStep: Routine to call as next step in sequence
	\return	none

*/
void FF_A6lib::waitSmsReady(unsigned long waitMs, void (FF_A6lib::*nextStep)(void)) {
	if (traceFlag) enterRoutine(__func__);
	if (debugFlag) trace_debug_P("Waiting SMS Ready for %d ms", waitMs);
	gsmTimeout = waitMs;
	gsmStatus = A6_RUNNING;
	nextStepCb = nextStep;
	startTime = millis();
	inReceive = false;
	inWait = true;
	inWaitSmsReady = true;
}

/*!

	\brief	[Private] Wait a given time

	\param[in]	waitMs: Time (ms) to wait
	\param[in]	nextStep: Routine to call as next step in sequence
	\return	none

*/
void FF_A6lib::waitMillis(unsigned long waitMs, void (FF_A6lib::*nextStep)(void)) {
	if (traceFlag) enterRoutine(__func__);
	if (debugFlag) trace_debug_P("Waiting for %d ms", waitMs);
	gsmTimeout = waitMs;
	gsmStatus = A6_RUNNING;
	nextStepCb = nextStep;
	startTime = millis();
	inReceive = false;
	inWait = true;
	inWaitSmsReady = false;
}

/*!

	\brief	[Private] Sends a (char*) command

	\param[in]	command: Command to send (char*)
	\param[in]	nextStep: Routine to call as next step in sequence
	\param[in]	resp: Expected command answer
	\param[in]	cdeTimeout: Maximum time (ms) to wait for correct answer
	\return	none

*/
void FF_A6lib::sendCommand(const char *command, void (FF_A6lib::*nextStep)(void), const char *resp, unsigned long cdeTimeout) {
	if (traceFlag) enterRoutine(__func__);
	commandCount++;
	delay(100);
	gsmTimeout = cdeTimeout;
	gsmStatus = A6_RUNNING;
	nextStepCb = nextStep;
	strncpy(expectedAnswer, resp, sizeof(expectedAnswer));
	strncpy(lastCommand, command, sizeof(lastCommand));			// Save last command
	memset(lastAnswer, 0, sizeof(lastAnswer));
	if (debugFlag) trace_debug_P("Issuing command: %s", command);
	a6Serial.write(command);
	a6Serial.write('\r');
	startTime = millis();
	inReceive = true;
	inWait = false;
	inWaitSmsReady = false;
}

/*!

	\brief	[Private] Sends a (uint8_t) command

	\param[in]	command: Command to send (uint8_t)
	\param[in]	nextStep: Routine to call as next step in sequence
	\param[in]	resp: Expected command answer
	\param[in]	cdeTimeout: Maximum time (ms) to wait for correct answer
	\return	none

*/
void FF_A6lib::sendCommand(const uint8_t command, void (FF_A6lib::*nextStep)(void), const char *resp, unsigned long cdeTimeout) {
	if (traceFlag) enterRoutine(__func__);
	commandCount++;
	gsmTimeout = cdeTimeout;
	gsmStatus = A6_RUNNING;
	nextStepCb = nextStep;
	strncpy(expectedAnswer, resp, sizeof(expectedAnswer));
	memset(lastAnswer, 0, sizeof(lastAnswer));
	if (debugFlag) trace_debug_P("Issuing command: 0x%x", command);
	a6Serial.write(command);
	startTime = millis();
	inReceive = true;
	inWaitSmsReady = false;
}


/*!

	\brief	[Private] Non blocking partial modem answer read

	\param[in]	reply: buffer to store partial answer into
	\param[in]	bufferSize: maxximum answer characters to read
	\return	count of returned characters

*/
int FF_A6lib::readBuffer(char* reply, int bufferSize) {
	if (traceFlag) enterRoutine(__func__);
	int retSize = 0;

	// Clear reply buffer
	//memset(reply, 0 , bufferSize);
	// Read all available characters
	while (a6Serial.available()) {
		char c = a6Serial.read();
		// Skip NULL characters
		if (c) {
			reply[retSize++] = c;
			reply[retSize] = 0;
		}
		if (retSize >= bufferSize) {						// Buffer is full, return it
			return retSize;
		}
	}
	return retSize;
}

/*!

	\brief	[Private] Set modem idle

	\param	none
	\return	none

*/
void FF_A6lib::setIdle(void) {
	if (traceFlag) enterRoutine(__func__);
	gsmIdle = A6_IDLE;
	inReceive = false;
	memset(lastAnswer, 0, sizeof(lastAnswer));
}

/*!

	\brief	[Private] Debug: trace each entered routine

	By default, do nothing as very verbose. Enable it only when really needed.

	\param[in]	routine name to display
	\return	none

*/
void FF_A6lib::enterRoutine(const char* routineName) {
	//trace_debug_P("Entering %s", routineName);
}

/*!

	\brief	[Private] Read a SMS header

	\param[in]	msg: modem answer
	\return	none

*/
void FF_A6lib::readSmsHeader(const char* msg) {
	char* ptrStart;

	number[0] = 0;
	date[0] = 0;
	message[0] = 0;
	index = 0;

	// Answer format is:
	// +CMT ,33
	// 07913396050066F0040B913306672146F00000328041102270800FCDF27C1E3E9741E432885E9ED301

	// Parse the response if it contains a valid SMS_INDICATOR
	ptrStart = strstr(msg, SMS_INDICATOR);
		if (debugFlag) trace_debug_P("Read SMS");

	if (ptrStart == NULL) {
		if (debugFlag) trace_debug_P("Can't find %s", SMS_INDICATOR);
		return;
	}
	nextLineIsSmsMessage = true;
}

/*!

	\brief	[Private] Read a PDU containing a received SMS

	\param[in]	msg: received PDU
	\return	none

*/
void FF_A6lib::readSmsMessage(const char* msg) {
	if (smsPdu.decodePDU(msg)) {
		if (smsPdu.getOverflow()) {
			trace_info_P("Overflow, partial message only");
		}
	strncpy(message, msg, sizeof(message));
		//Serial.print("SCA ");Serial.println(smsPdu.getSCAnumber());
		strncpy(number, smsPdu.getSender(), sizeof(number));
		strncpy(date, smsPdu.getTimeStamp(), sizeof(date));
		strncpy(message, smsPdu.getText(),sizeof(message));
		smsForwardedCount++;
		if (debugFlag) trace_debug_P("Got SMS from %s, sent at %s, >%s<", number, date, message);
		executeSmsCb();
	} else {
		trace_error_P("Decode failed");
	}
	deleteSMS(1,2);
}