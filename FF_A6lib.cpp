/*!
	\file
	\brief	Implements a fully asynchronous SMS send/receive in PDU mode for A6/GA6 modem class
	\author	Flying Domotic
	\date	December 23rd, 2024
*/

/*
Todo:
	- move last send/received variables into a6lib
	- base sendSms and sendSmsNext on this
	- implements routines to read them
*/

#include <FF_A6lib.h>
#include <stdio.h>
#include <FF_Trace.h>
#include <SoftwareSerial.h>
#define PM												    // Define PDU tables in flash
#include <NtpClientLib.h>									// https://github.com/gmag11/NtpClient
#include <pdulib.h>											// https://github.com/mgaman/PDUlib

#define PDU_BUFFER_LENGTH 400								// Max workspace length
PDU smsPdu = PDU(PDU_BUFFER_LENGTH);						// Instantiate PDU class

#ifdef USE_SOFTSERIAL_FOR_A6LIB                             // Define USE_SOFTSERIAL_FOR_A6LIB to use SofwareSerial instead of Serial
                                                            // ************************** WARNING **************************
                                                            //  This will cause interrupt problems (understand crashes)
                                                            //      if used with asynchronous libraries (like async web server)
                                                            // ************************** WARNING **************************
    SoftwareSerial a6Serial;               					// We use software serial to keep Serial usable
    #warning Using SoftwareSerial will crash when using other asynchronous librairies
#else
                                                            // ************************** WARNING **************************
															//	Due to a bug, Serial.swap needed to swap TX/RX with D8/D7
															//		doesn't work when setDebugOutput is set to true.
															//		Be sure your revert it to false before doing the swap!!!
                                                            // ************************** WARNING **************************
    #define a6Serial Serial                                 // Use Serial for A6lib
#endif

// Class constructor : init some variables
FF_A6lib::FF_A6lib() {
	restartNeeded = true;
	gsmStatus = A6_NEED_INIT;
	restartReason = gsmStatus;
	inReceive = false;
	gsmIdle = A6_STARTING;
	debugFlag = false;
	traceFlag = false;
	traceEnterFlag = false;
	nextLineIsSmsMessage = false;
	commandCount = 0;
	resetCount = 0;
	smsReadCount = 0;
	smsForwardedCount = 0;
	smsSentCount = 0;
	lastReceivedNumber = PSTR("[none]");
	lastReceivedDate = PSTR("[never]");
	lastReceivedMessage = PSTR("[no message]");
	lastSentNumber = PSTR("[none]");
	lastSentDate = PSTR("[never]");
	lastSentMessage = PSTR("[no message]");
}

/*!

	\brief	Initialize the GSM connection

	Open the modem connection at given baud rate

	\param[in]	baudRate: modem speed (in bauds). Modem will be properly switched to this speed if needed
	\param[in]	rxPin: ESP pin used to receive data from modem (connected to A6/GA6 UTX, SoftwareSerial only)
	\param[in]	txPin: ESP pin used to send data to modem (connected to A6/GA6 URX, SoftwareSerial only)
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
					resetLastAnswer();
					return;
				}
				// Do we have an answer?
				if (c == 10) {
					#ifdef FF_A6LIB_DUMP_MESSAGE_ON_SERIAL
						Serial.print("<LF>");
					#endif
					// Do we have an "SMS Ready" message?
					if (!smsReady && strstr(lastAnswer, SMS_READY_MSG)) {
						if (debugFlag) trace_debug_P("Got SMS Ready", NULL);
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
							resetLastAnswer();
							nextLineIsSmsMessage = false;			// Clear flag
							return;
						} else {									// Not in SMS reception
							if (strstr(lastAnswer, SMS_INDICATOR)) {// Is this indicating an SMS reception?
								if (debugFlag) trace_debug_P("Indicator is >%s<", lastAnswer);	// Display cleaned message
								// Load last command with indicator
								strncpy(lastCommand, lastAnswer, sizeof(lastCommand));
								readSmsHeader(lastAnswer);
								resetLastAnswer();
								inReceive = true;
								startTime = millis();
								return;
							} else {								// Can't understand received data
								if (debugFlag) trace_debug_P("Ignoring >%s<", lastAnswer);	// Display cleaned message
								if (recvLineCb) recvLineCb(lastAnswer);		// Activate callback with answer
									resetLastAnswer();
								return;
							}
						}
					}
				} else {
					// Add received character to lastAnswer
					#ifdef FF_A6LIB_DUMP_MESSAGE_ON_SERIAL
						Serial.print(c);
					#endif
					lastAnswer[answerLen++] = c;			// Copy character
					lastAnswer[answerLen] = 0;				// Just in case we forgot cleaning buffer
					// Check for one character answer (like '>' when sending SMS) which have no <CR><LF>
					if (strlen(expectedAnswer) == 1 && c == expectedAnswer[0]) {
						if (debugFlag) trace_debug_P("Reply in %d ms: >%s<", millis() - startTime, lastAnswer);
						gsmStatus = A6_OK;
						if (nextStepCb) {				// Do we have another callback to execute?
							(this->*nextStepCb)();		// Yes, do it
						} else {
							setIdle();					// No, we just finished.
						}
						return;
					}
				}
			#ifdef FF_A6LIB_DUMP_MESSAGE_ON_SERIAL
			} else {
				if (c == 0) {
					Serial.print("<NULL>");
				} else if (c == 13 ) {
					Serial.print("<CR>");
				}
			#endif
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

	This routine dumps (almost all) variables using trace_info_P macro (usually defined in Ff_TRACE)

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
	trace_info_P("a6-debugFlag=%d", debugFlag);
	trace_info_P("a6-traceFlag=%d", traceFlag);
	trace_info_P("a6-traceEnterFlag=%d", traceEnterFlag);
}

/*!

	\brief	Sends an SMS to modem

	This routine pushes an SMS to modem.
		It determines if message is a GMS7 only message or not (in this case, this will be UCS-2)
		If message is GSM7, max length of non chunked SMS is 160. For UCS-2, this is 70.
		When message is longer than these limits, it'll be split in chunks of 153 chars for GSM7, or 67 chars for UCS-2.
		There's a theoretical limit of 255 chunks, but most of operators are limiting in lower size.
		It seems that 7 to 8 messages are accepted by almost everyone, meaning 1200 GSM7 chars, or 550 UCS-2 chars.

	\param[in]	number: phone number to send message to
	\param[in]	text: message to send
	\return	none

*/
void FF_A6lib::sendSMS(const char* number, const char* text) {
	uint16_t utf8Length = strlen(text);						// Size of UTF-8 message
	uint8_t lengthToAdd;									// Length of one UTF-8 char in GSM-7 (or zero if UTF-8 input character outside GSM7 table)
	uint8_t c1;												// First char of UTF-8 message
	uint8_t c2;												// Second char of UTF-8 message
	uint8_t c3;												// Third char of UTF-8 message
	gsm7Length = 0;											// Size of GSM-7 message

	for (uint16_t i = 0; i < utf8Length; i++) {				// Scan the full message
		c1 = text[i];										// Extract first to third chars
		if (i+1 < utf8Length) {c2 = text[i+1];} else {c2 = 0;}
		if (i+2 < utf8Length) {c3 = text[i+2];} else {c3 = 0;}
		lengthToAdd = getGsm7EquivalentLen(c1, c2, c3);		// Get equivalent GSM-7 length
		if (lengthToAdd) {									// If char is GSM-7
			gsm7Length += lengthToAdd;						// Add length
		} else {
			if (debugFlag) trace_info_P("Switched to UTF-8 on char %d (0x%02x) at pos %d", c1, c1, i);
			gsm7Length = 0;									// Set length to zero
			break;											// Exit loop
		}
	}
	
	// Should we split message in chunks?
	if (gsm7Length) {										// Is this a GSM-7 message ?
		if (gsm7Length > 160) {								// This is a multi-part message
			smsMsgCount = (gsm7Length + 151) / 152;			// Compute total chunks
			smsMsgId++;
			smsChunkSize = 152;
		} else {
			smsMsgCount = 0;
		}
		if (debugFlag) trace_info_P("gsm7, lenght=%d, msgs=%d", gsm7Length, smsMsgCount);
	} else {												// This is an UCS-2 message
		uint16_t ucs2Length = ucs2MessageLength(text);		// Get UCS-2 message length
		if (ucs2Length > 70) {								// This is a multi-part message
			smsMsgCount = (ucs2Length + 66) / 67;			// Compute total chunks
			smsMsgId++;
			smsChunkSize = 67;
		} else {
			smsMsgCount = 0;
		}
		if (debugFlag) trace_info_P("ucs2, length=%d, msgs=%d", ucs2Length, smsMsgCount);
	}
	// Save last used number and message
	lastSentNumber = String(number);
	lastSentMessage = String(text);
	lastSentDate = NTP.getDateStr() + " " + NTP.getTimeStr();
	// Send first (or only) SMS part
	if (smsMsgCount == 0) {
		sendOneSmsChunk(number, text);
	} else {
		smsMsgIndex = 0;
		uint16_t startPos = smsMsgIndex++ * smsChunkSize;
		sendOneSmsChunk(number, lastSentMessage.substring(startPos, startPos+smsChunkSize).c_str(), smsMsgId, smsMsgCount, smsMsgIndex);	// Send first chunk
	}
}

/*!

	\brief	Sends next SMS chunk to modem

	This routine is called when an SMS chunk has been pushed to modem, to send next part, if it exists

	\param	none
	\return	none

*/
void FF_A6lib::sendNextSmsChunk(void){
	if (smsMsgCount) {										// Are we in multi-part message ?
		if (smsMsgIndex < smsMsgCount) {				// Do we have more chunks to send ?
			uint16_t startPos = smsMsgIndex++ * smsChunkSize;
			sendOneSmsChunk(lastSentNumber.c_str(), lastSentMessage.substring(startPos, startPos+smsChunkSize).c_str(), smsMsgId, smsMsgCount, smsMsgIndex);	// Send first chunk
			return;
		}
	}
	setIdle();												// Message has fully be sent
}

/*!

	\brief	Sends an SMS chunk to modem

	This routine pushes an SMS chunk to modem

	\param[in]	number: phone number to send message to
	\param[in]	text: message to send
	\param[in]	msgId: SMS message identifier (should be incremented for each multi-part message, zero if not multi-part message)
	\param[in]	msgCount: total number of SMS chunks (zero if not multi-part message)
	\param[in]	msgIndex: index of this message chunk (zero if not multi-part message)
	\return	none

*/
void FF_A6lib::sendOneSmsChunk(const char* number, const char* text, const unsigned short msgId, const unsigned char msgCount, const unsigned char msgIndex) {
	if (traceFlag) enterRoutine(__func__);
	char tempBuffer[50];
	int len = smsPdu.encodePDU(number, text, msgId, msgCount, msgIndex);
	if (len < 0)  {
			// -1: OBSOLETE_ERROR
			// -2: UCS2_TOO_LONG
			// -3 GSM7_TOO_LONG
			// -4 MULTIPART_NUMBERS
			// -5 ADDRESS_FORMAT
			// -6 WORK_BUFFER_TOO_SMALL
			// -7 ALPHABET_8BIT_NOT_SUPPORTED
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
	if (traceFlag) enterRoutine(__func__);
	// Don't reopen modem if speed is the good one
	if (baudRate != modemLastSpeed) {
		if (debugFlag) trace_debug_P("Opening modem at %d bds", baudRate);
        #ifdef USE_SOFTSERIAL_FOR_A6LIB
            // Open modem at given speed
            a6Serial.begin(baudRate, SWSERIAL_8N1, modemTxPin, modemRxPin, false, 128);	// Connect to Serial Software
            // Enable TX interruption for speeds up to 19200 bds
            a6Serial.enableIntTx((baudRate <= 19200));
        #else
            a6Serial.begin(baudRate, SERIAL_8N1);
        #endif
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
	sendCommand("AT", &FF_A6lib::findSpeedAnswer, DEFAULT_ANSWER, 500);
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
			sendCommand("AT", &FF_A6lib::findSpeedAnswer, DEFAULT_ANSWER, 500);
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
	sendCommand("AT+CREG=2", &FF_A6lib::waitUntilSmsReady);
}

/*!

	\brief	[Private] Modem initialization: wait to receive SMS ready for 30 seconds

	\param	none
	\return	none

*/
void FF_A6lib::waitUntilSmsReady(void) {
	if (traceFlag) enterRoutine(__func__);
	if (!smsReady) {
		waitSmsReady(30000, &FF_A6lib::setCallerId);
	} else {
		if (debugFlag) trace_debug_P("SMS ready already received", NULL);
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
	// Check SCA number (first char can be "+", all other should be digit)
	for (int i = 0; scaNumber[i]; i++) {
		// Is char not a number?
		if (scaNumber[i] < '0' || scaNumber[i] > '9') {
			// First char could be '+'
			if (scaNumber[0] != '+' || i != 0) {
				if (debugFlag) trace_debug_P("Bad SCA number %s", scaNumber);
				restartReason = A6_BAD_ANSWER;
				restartNeeded = true;
				return;
			}
		}
	}
	if (debugFlag) trace_debug_P("setting SCA to %s", scaNumber);
	smsPdu.setSCAnumber(scaNumber);
	resetLastAnswer();
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
	sendCommand(0x1a, &FF_A6lib::sendNextSmsChunk, "+CMGS:", 10000);
}

/*!

	\brief	[Private] Call SMS received callback routine

	\param	none
	\return	none

*/
void FF_A6lib::executeSmsCb(void) {
	if (traceFlag) enterRoutine(__func__);
	if (readSmsCb) {
		(*readSmsCb)(index, lastReceivedNumber.c_str(), lastReceivedDate.c_str(), lastReceivedMessage.c_str());
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
		resetLastAnswer();
		a6Serial.write(command);
		a6Serial.write('\r');
	}
	startTime = millis();
	inReceive = true;
	inWait = false;
	inWaitSmsReady = false;
	nextLineIsSmsMessage = false;
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
	resetLastAnswer();
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
	resetLastAnswer();
}

/*!

	\brief	[Private] Debug: trace each entered routine

	By default, do nothing as very verbose. Enable it only when really needed.

	\param[in]	routine name to display
	\return	none

*/
void FF_A6lib::enterRoutine(const char* routineName) {
	if (traceEnterFlag) trace_debug_P("Entering %s", routineName);
}

/*!

	\brief	[Private] Read a SMS header

	\param[in]	msg: modem answer
	\return	none

*/
void FF_A6lib::readSmsHeader(const char* msg) {
	if (traceFlag) enterRoutine(__func__);
	index = 0;

	// Answer format is:
	// +CMT ,33
	// 07913396050066F0040B913306672146F00000328041102270800FCDF27C1E3E9741E432885E9ED301

	// Parse the response if it contains a valid SMS_INDICATOR
	char* ptrStart = strstr(msg, SMS_INDICATOR);
	if (ptrStart == NULL) {
		trace_error_P("Can't find %s in %s", SMS_INDICATOR, msg);
		return;
	}
	if (debugFlag) trace_debug_P("Waiting for SMS", NULL);
	nextLineIsSmsMessage = true;
}

/*!

	\brief	[Private] Read a PDU containing a received SMS

	\param[in]	msg: received PDU
	\return	none

*/
void FF_A6lib::readSmsMessage(const char* msg) {
	if (traceFlag) enterRoutine(__func__);
	if (smsPdu.decodePDU(msg)) {
		if (smsPdu.getOverflow()) {
			trace_warn_P("SMS decode overflow, partial message only", NULL);
		}
		lastReceivedNumber = String(smsPdu.getSender());
		lastReceivedDate = String(smsPdu.getTimeStamp());
		lastReceivedMessage = String(smsPdu.getText());
		smsForwardedCount++;
		if (debugFlag) trace_debug_P("Got SMS from %s, sent at %s, >%s<", lastReceivedNumber.c_str(), lastReceivedDate.c_str(), lastReceivedMessage.c_str());
		executeSmsCb();
	} else {
		trace_error_P("SMS PDU decode failed", NULL);
	}
	deleteSMS(1,2);
}

/*!

	\brief	[Private] Clean ast answer

	\param	none
	\return	none

*/
void FF_A6lib::resetLastAnswer(void) {
	if (traceFlag) enterRoutine(__func__);
	memset(lastAnswer, 0, sizeof(lastAnswer));
}

/*!

	\brief	Return GSM7 equivalent length of one UTF-8 character

	This routine takes one UTF-8 character coded on up-to 3 bytes to return it's length when coded in GSM7

	\param[in]	c1: first byte of UTF-8 character to analyze
	\param[in]	c2: second byte of UTF-8 character to analyze (or zero if end of message)
	\param[in]	c3: third byte of UTF-8 character to analyze (or zero if end of message)
	\return	Length of character when coded in GSM7 (or zero if UTF-8 input character outside GSM7 table)

*/

uint8_t FF_A6lib::getGsm7EquivalentLen(const uint8_t c1, const uint8_t c2, const uint8_t c3) {
	if (
			// These are one byte UTF8 char coded on one byte GSM7 char
			(c1 == 0x0a)	/* Linefeed */ 																						||
			(c1 == 0x0d)	/* Carriage return */ 																				||
			(c1 >= 0x20		/* Space */ 								&& c1 <= 0x5a) /* Capital letter Z */					|| 
			(c1 == 0x5f)	/* Underscore */ 																					||
			(c1 >= 0x61		/* Small letter a */ 						&& c1 <= 0x7a) /* Small letter z */ 
		) {
			return 1;
	}
	if ((c1 == 0xc2) && (
			// These are two bytes UTF8 char coded on one byte GSM7 char
			(c2 == 0xa1)	/* Inverted exclamation mark */																		||
			(c2 >= 0xa3		/* Pound sign */							&& c2 <= 0xa5) /* Yuan/Yen sign */						|| 
			(c2 == 0xa7)	/* Section sign */ 																					||
			(c2 == 0xbf)	/* Inverted question mark */
		)) {
			return 1;
	}
	if ((c1 == 0xc3) && (
			// These are two bytes UTF8 char coded on one byte GSM7 char
			(c2 >= 0x84		/* Capital letter A with diaeresis */		&& c2 <= 0x87) /* Capital letter C with cedilla */		|| 
			(c2 == 0x89)	/* Capital letter E with acute accent */															|| 
			(c2 == 0x91)	/* Capital letter N with tilde */																	||
			(c2 == 0x96)	/* Capital letter O with diaeresis */																||
			(c2 == 0x98)	/* Capital letter O with stroke */																	||
			(c2 == 0x9c)	/* Capital letter U with diaeresis */																||
			(c2 >= 0x9f		/* Small letter German Eszett */			&& c2 <= 0xa0) /* Small letter a with grave accent */	|| 
			(c2 >= 0xa4		/* Small letter a with diaeresis */			&& c2 <= 0xa6) /* Small letter ae */					|| 
			(c2 >= 0xa8		/* Small letter e with grave accent */		&& c2 <= 0xa9) /* Small letter e with acute accent */	|| 
			(c2 == 0xac)	/* Small letter i with grave accent */																||
			(c2 >= 0xb1		/* Small letter n with tilde */				&& c2 <= 0xb2) /* Small letter o with grave accent */	|| 
			(c2 == 0xb6)	/* Small letter o with diaeresis */																	||
			(c2 >= 0xb8		/* Small letter o with stroke */			&& c2 <= 0xb9) /* Small letter u with grave accent */	|| 
			(c2 == 0xbc)	/* Small letter u with diaeresis */ 
		)) {
			return 1;
	}
	if (
			// These are one byte UTF8 char coded on two bytes GSM7 char
			(c1 == 0x0c)	/* Form feed */ ||
			(c1 >= 0x5b		/* Left square bracket */					&& c1 <= 0x5e) /* Caret / Circumflex */					|| 
			(c1 >= 0x7b		/* Left curly bracket */					&& c1 <= 0x7e) /* Tilde */
		) {
			return 2;
	}
	if (c1 == 0xe2 && 
			// This is three bytes UTF8 char coded on two bytes GSM7 char
			c2 == 0x82 &&
			c3 == 0xac 		/* Euro sign */
		) {
		return 2;
	}
	return 0;
}

/*!

	\brief	Return UCS-2 equivalent length of one UTF-8 character

	This routine takes one UTF-8 message to return it's length when coded in UCS-2

	\param[in]	text: message to be scanned
	\return	Length of character when coded in UCS-2

*/

uint16_t FF_A6lib::ucs2MessageLength(const char* text) {
	uint16_t utf8CharCount = 0;

	// Get UTF-8 message length
	while (*text) {
		// Cout only first byte of each UTF-8 sequence
		utf8CharCount += (*text++ & 0xc0) != 0x80;
	}
	// UCS-2 if 2 chars for each UTF-8 character
	return utf8CharCount * 2;
}

/*!

	\brief	Return phone number of last received SMS

	This routine returns the phone number of last received SMS

	\param	None
	\return	Phone number of last received SMS

*/

const char* FF_A6lib::getLastReceivedNumber(void) {
	return lastReceivedNumber.c_str();
}

/*!

	\brief	Return date of last received SMS

	This routine returns the phone number of last received SMS

	\param	None
	\return	Date of last received SMS

*/
const char* FF_A6lib::getLastReceivedDate(void) {
	return lastReceivedDate.c_str();
}

/*!

	\brief	Return message of last received SMS

	This routine returns the message of last received SMS

	\param	None
	\return	Message of last received SMS

*/
const char* FF_A6lib::getLastReceivedMessage(void) {
	return lastReceivedMessage.c_str();
}

/*!

	\brief	Return phone number of last sent SMS

	This routine returns the phone number of last sent SMS

	\param	None
	\return	Phone number of last sent SMS

*/
const char* FF_A6lib::getLastSentNumber(void) {
	return lastSentNumber.c_str();
}

/*!

	\brief	Return date of last sent SMS

	This routine returns the date of last sent SMS

	\param	None
	\return	Date of last sent SMS

*/
const char* FF_A6lib::getLastSentDate(void) {
	return lastSentDate.c_str();
}

/*!

	\brief	Return message of last sent SMS

	This routine returns the message of last sent SMS

	\param	None
	\return	Message of last sent SMS

*/
const char* FF_A6lib::getLastSentMessage(void) {
	return lastSentMessage.c_str();
}
