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

SoftwareSerial a6Serial(-1, -1);							// We use software serial to keep Serial usable

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

	Open the modem connection at given baud rate

	\param[in]	baudRate: modem speed (in bauds). Modem will be properly switched to this speed if needed
	\param[in]	rxPin: ESP pin used to receive data from modem (connected to A6/GA6 UTX)
	\param[in]	txPin: ESP pin used to send data to modem (connected to A6/GA6 URX)
	\return	none

*/
void FF_A6lib::begin(long baudRate, int8_t rxPin, int8_t txPin) {
	if (traceFlag) enterRoutine(__func__);
	restartNeeded = false;
	inReceive = false;
	inWait = false;
	gsmIdle = A6_STARTING;
	// Save RX pin, TX pin and requested speed
	modemRxPin = rxPin;
	modemTxPin = txPin;
	modemRequestedSpeed = baudRate;
	// Open modem at requested speed initially
	openModem(baudRate);
	// find modem speed
	findSpeed(&FF_A6lib::setReset);												// Find modem speed, then setReset
}

/*!

	\brief	Modem loop (should be called in main loop)

	This routine should be calledd at regular interval in order for the modem code to run (as we're asynchronous)

	\param	none
	\return	none

*/
void FF_A6lib::doLoop(void) {
	if (traceFlag) enterRoutine(__func__);

	// Read modem until \n (LF) character found, removing \r (CR)
	if (a6Serial.available()) {
		size_t answerLen = strlen(lastAnswer);				// Get answer length
		while (a6Serial.available()) {
			char c = a6Serial.read();
			// Skip NULL and CR characters
			if (c != 0 && c != 13) {
				if (answerLen >= sizeof(lastAnswer)-2) {
					trace_error_P("Answer too long: >%s<", lastAnswer);
					// Answer is too long
					gsmStatus = A6_TOO_LONG;
					memset(lastAnswer, 0, sizeof(lastAnswer));	// Reset last answer
					return;
				}
				// Do we have an answer?
				if (c == 10) {
					// Do we have an "SMS Ready" message?
					if (!smsReady && strstr(lastAnswer, SMS_READY_MSG)) {
						if (debugFlag) trace_debug_P("Got SMS Ready");
						smsReady = true;
						return;
					}
					if (inReceive) {						// Are we waiting for a command answer?
						// Is this the expected answer?
						bool isDefaultAnwer = !(strcmp(expectedAnswer, DEFAULT_ANSWER));
						if ((isDefaultAnwer && !strcmp(lastAnswer, expectedAnswer)) || (!isDefaultAnwer && strstr(lastAnswer, expectedAnswer))) {
							if (debugFlag) trace_debug_P("Reply in %d ms: >%s<", millis() - startTime, lastAnswer);
							gsmStatus = A6_OK;
							if (nextStepCb) {				// Do we have another callback to execute?
								(this->*nextStepCb)();		// Yes, do it
							} else {
								setIdle();					// No, we just finished.
							}
							return;
						}
						if (!ignoreErrors) {				// Should we ignore errors?
							// No, check for CMS/CME error
							if (strstr(lastAnswer,"+CMS ERROR") || strstr(lastAnswer,"+CME ERROR")) {
								// This is a CMS or CME answer
								trace_error_P("Error answer: >%s< after %d ms, command was %s", lastAnswer, millis() - startTime, lastCommand);
								gsmStatus = A6_CM_ERROR;
								restartNeeded = true;
								restartReason = gsmStatus;
								setIdle();
								return;
							}
						}
					}
					if (strlen(lastAnswer)) {						// Answer is not null
						if (nextLineIsSmsMessage) {					// Are we receiving a SMS message?
							if (debugFlag) trace_debug_P("Message is >%s<", lastAnswer);	// Display cleaned message
							readSmsMessage(lastAnswer);				// Yes, read it
							memset(lastAnswer, 0, sizeof(lastAnswer));	// Reset last answer
							nextLineIsSmsMessage = false;			// Clear flag
							return;
						} else {									// Not in SMS reception
							if (strstr(lastAnswer, SMS_INDICATOR)) {// Is this indicating an SMS reception?
								if (debugFlag) trace_debug_P("Indicator is >%s<", lastAnswer);	// Display cleaned message
								readSmsHeader(lastAnswer);
								memset(lastAnswer, 0, sizeof(lastAnswer));	// Reset last answer
								inReceive = true;
								startTime = millis();
								return;
							} else {								// Can't understand received data
								if (debugFlag) trace_debug_P("Ignoring >%s<", lastAnswer);	// Display cleaned message
								if (recvLineCb) recvLineCb(lastAnswer);		// Activate callback with answer
								memset(lastAnswer, 0, sizeof(lastAnswer));	// Reset last answer
								return;
							}
						}
					}
				} else {
					// Add received character to lastAnswer
					lastAnswer[answerLen++] = c;			// Copy character
					lastAnswer[answerLen] = 0;				// Just in case we forgot cleaning buffer
				}
			}
		}
	}

	if (inReceive) {										// We're waiting for a command answer
		if ((millis() - startTime) >= gsmTimeout) {
			if (ignoreErrors) {								// If errors should be ignored, call next step, if any
				if (nextStepCb) {							// Do we have another callback to execute?
					(this->*nextStepCb)();					// Yes, do it
				} else {
					setIdle();								// No, we just finished.
				}
				return;
			}
			// Here, we've got a time-out on answer
			if (lastAnswer[0]) {
				trace_error_P("Partial answer: >%s< after %d ms, command was %s", lastAnswer, millis() - startTime, lastCommand);
				gsmStatus = A6_BAD_ANSWER;
				restartNeeded = true;
				restartReason = gsmStatus;
				setIdle();
				return;
			} else {										// Time-out without any anwser
				trace_error_P("Timed out after %d ms, received >%s<, command was %s", millis() - startTime, lastAnswer, lastCommand);
				gsmStatus = A6_TIMEOUT;
				restartNeeded = true;
				restartReason = gsmStatus;
				setIdle();
				return;
			}
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

	This routine delete (some) SMS from storage using AT+CMGD command
	\param[in]	index as used by AT+CMGD
	\param[in]	flag as used by AT+CMGD
	\return	none

*/
void FF_A6lib::deleteSMS(int index, int flag) {
	if (traceFlag) enterRoutine(__func__);
	char tempBuffer[50];

	snprintf_P(tempBuffer, sizeof(tempBuffer), PSTR("AT+CMGD=%d,%d"), index, flag);
	sendCommand(tempBuffer, &FF_A6lib::setIdle, DEFAULT_ANSWER, 10000);	// Wait up to 10 seconds for OK
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

	\brief	[Private] Modem initialization: open modem at a given speed

	\param[in]	baudRate: speed (in bds) to use to open modem
	\return	none

*/
void FF_A6lib::openModem(long baudRate) {
	// Don't reopen modem if speed is the good one
	if (baudRate != modemLastSpeed) {
		if (debugFlag) trace_debug_P("Opening modem at %d bds", baudRate);
		// Close modem (as it'll probably already be in use)
		a6Serial.end();
		// Open modem at given speed
		a6Serial.begin(baudRate, SWSERIAL_8N1, modemTxPin, modemRxPin, false);	// Connect to Serial Software
		// Enable TX interruption for speeds up to 19600 bds
		a6Serial.enableIntTx((baudRate <= 19600));
		modemLastSpeed = baudRate;
	}
}

/*!

	\brief	[Private] Modem initialization: find current modem speed

	\param	none
	\return	none

*/
void FF_A6lib::findSpeed(void (FF_A6lib::*nextStep)(void)) {
	if (traceFlag) enterRoutine(__func__);
	findSpeedCb = nextStep;									// Save nextStep to execute wfter speed determination
	ignoreErrors = true;									// Ignore errors
	speedsToTestIndex = -1;									// Init speed index
	// Send attention command at current speed first
	sendCommand("AT", &FF_A6lib::findSpeedAnswer, DEFAULT_ANSWER, 1500);
}

/*!

	\brief	[Private] Modem initialization: find current modem speed answer

	\param	none
	\return	none

*/
void FF_A6lib::findSpeedAnswer(void) {
	if (traceFlag) enterRoutine(__func__);
	// Was last answer the expected one?
	if (!strstr(lastAnswer, expectedAnswer)) {
		speedsToTestIndex++;
		long modemSpeed = speedsToTest[speedsToTestIndex];	// Load next speed to test
		// Is speed defined?
		if (modemSpeed) {
			// Open modem at test speed
			openModem(modemSpeed);
			// Send attention command
			sendCommand("AT", &FF_A6lib::findSpeedAnswer, DEFAULT_ANSWER, 1500);
			return;
		} else {
			// No, open modem at requested speed
			if (debugFlag) trace_info_P("Forcing modem at %d bds", modemRequestedSpeed);
			openModem(modemRequestedSpeed);
			sendCommand("AT", findSpeedCb);
			return;
		}
	}
	if (debugFlag) trace_info_P("Modem found at %d bds", modemLastSpeed);
	// We're here with right or default speed, continue with next step
	if (findSpeedCb) {
		(this->*findSpeedCb)();						// Execute next step if defined
	} else {
		setIdle();									// Else, we just finished.
	}
}

/*!

	\brief	[Private] Modem initialization: reset modem to factory defaults

	\param	none
	\return	none

*/
void FF_A6lib::setReset(void) {
	if (traceFlag) enterRoutine(__func__);
	resetCount++;
	smsReady = false;
	// Reset to factory defaults
	sendCommand("AT&F", &FF_A6lib::setModemSpeed);
}

/*!

	\brief	[Private] Modem initialization: set modem speed

	\param	none
	\return	none

*/
void FF_A6lib::setModemSpeed(void) {
	if (traceFlag) enterRoutine(__func__);
	// Is modem already at right speed?
	if (modemLastSpeed == modemRequestedSpeed) {
		// Skip +IPR if modem already at right speed
		setSpeedComplete();
	} else {
		char tempBuffer[50];
		snprintf_P(tempBuffer, sizeof(tempBuffer), PSTR("AT+IPR=%d"), modemRequestedSpeed);
		// Set speed
		sendCommand(tempBuffer, &FF_A6lib::setSpeedComplete);
	}
}

/*!

	\brief	[Private] Modem initialization: reopen modem after set speed

	\param	none
	\return	none

*/
void FF_A6lib::setSpeedComplete(void) {
	if (traceFlag) enterRoutine(__func__);
	ignoreErrors = false;
	// Has modem speed been changed?
	if (modemLastSpeed != modemRequestedSpeed) {
		// Reopen modem at right speed after setModemSpeed
		openModem(modemRequestedSpeed);
		// Send attention to activate speed
		sendCommand("AT", &FF_A6lib::echoOff);
	} else {
		echoOff();
	}
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
	sendCommand("AT+CMEE=2", &FF_A6lib::setTextMode);
}

/*!

	\brief	[Private] Modem initialization: send SMS in PDU mode (instead of text)

	\param	none
	\return	none

*/
void FF_A6lib::setTextMode(void) {
	if (traceFlag) enterRoutine(__func__);
	// Set SMS to PDU mode
	sendCommand("AT+CMGF=0", &FF_A6lib::detailedRegister);
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
	if (!smsReady) {
		waitSmsReady(30000, &FF_A6lib::setCallerId);
	} else {
		if (debugFlag) trace_debug_P("SMS ready already received");
		setCallerId();
	}
}

/*!

	\brief	[Private] Modem initialization: set caller id on

	\param	none
	\return	none

*/
void FF_A6lib::setCallerId(void) {
	if (traceFlag) enterRoutine(__func__);
	// Set caller ID on
	sendCommand("AT+CLIP=1", &FF_A6lib::setIndicOff);
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
		if (debugFlag) trace_debug_P("Can't find %s in %s",CSCA_INDICATOR, lastAnswer);
		restartReason = A6_BAD_ANSWER;
		restartNeeded = true;
		return;
	}
	//	First token is +CSCA
	char* token = strtok (ptrStart, "\"");
	if (token == NULL) {
		if (debugFlag) trace_debug_P("Can't find first token in %s", lastAnswer);
		restartReason = A6_BAD_ANSWER;
		restartNeeded = true;
		return;
	}

	// Second token is is SCA number
	token = strtok (NULL, "\"");
	if (token == NULL) {
		if (debugFlag) trace_debug_P("Can't find second token in %s", lastAnswer);
		restartReason = A6_BAD_ANSWER;
		restartNeeded = true;
		return;
	}
	strncpy(scaNumber, token, sizeof(scaNumber));
	if (debugFlag) trace_debug_P("setting SCA to %s", scaNumber);
	smsPdu.setSCAnumber(scaNumber);
	sendCommand("", &FF_A6lib::deleteReadSent);
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
		trace_info_P("SMS gateway started, restart count = %d", restartCount);
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

	\param[in]	command: Command to send (char*). If empty, will wait for answer of a previously sent command
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
	if (debugFlag) trace_debug_P("Issuing command: %s", command);
	// Send command if defined (else, we'll just wait for answer of a previously sent command)
	if (command[0]) {
		strncpy(lastCommand, command, sizeof(lastCommand));			// Save last command
		memset(lastAnswer, 0, sizeof(lastAnswer));
		a6Serial.write(command);
		a6Serial.write('\r');
	}
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
	if (ptrStart == NULL) {
		trace_error_P("Can't find %s in %s", SMS_INDICATOR, msg);
		return;
	}
	if (debugFlag) trace_debug_P("Waiting for SMS");
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
			trace_warn_P("SMS decode overflow, partial message only");
		}
		strncpy(message, msg, sizeof(message));
		strncpy(number, smsPdu.getSender(), sizeof(number));
		strncpy(date, smsPdu.getTimeStamp(), sizeof(date));
		strncpy(message, smsPdu.getText(),sizeof(message));
		smsForwardedCount++;
		if (debugFlag) trace_debug_P("Got SMS from %s, sent at %s, >%s<", number, date, message);
		executeSmsCb();
	} else {
		trace_error_P("SMS PDU decode failed");
	}
	deleteSMS(1,2);
}
