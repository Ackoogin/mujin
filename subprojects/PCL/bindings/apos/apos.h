#ifndef __APOS_BINDING
#define __APOS_BINDING

#ifdef __cplusplus
extern "C" {
#endif

//Structure to represent a timeout period.
typedef struct CTimeout
{
	//Number of seconds
	unsigned int uiSeconds;

	//Number of nanoseconds
	unsigned int uiNanoSeconds;
} CTimeout;

//ASAAC Time Status Type
typedef enum TM_Status
{
	TM_SUCCESS = 1,
	TM_TIMEOUT = 2,
	TM_ERROR = 0

} TM_Status;
//ASAAC Resource Status Type
typedef enum RS_Status
{
	RS_SUCCESS = 1,
	RS_RESOURCE = 2,
	RS_ERROR = 0

} RS_Status;

typedef void ( * DelayCallback)(int uS);

/// Attach Process to APOS
void setupAPOS (unsigned int uiProcessNo);
/// Attach Process to APOS
void setupApos (unsigned int uiProcessNo);
/// Set delay function used by APOS (instead of sleep)
void setDelayFunction(DelayCallback pDelay);

/// Send data to Local Virtual Channel
void sendMessage(
				int iLVC,
				unsigned char * pucData,
				int iDataSize,
				CTimeout * pTimeoutLength,
				TM_Status * pStatus);

/// Send data to Local Virtual Channel
void sendMessageNonBlocking(
							int iLVC,
							unsigned char * pucData,
							int iDataSize,
							RS_Status * pStatus);

/// Read data from Local Virtual Channel							
void receiveMessage(
					int iLVC,
					unsigned char * pucData,
					int iMaxSize,
					int * piDataSize,
					CTimeout * pTimeoutLength,
					TM_Status * pStatus);

/// Read data from Local Virtual Channel												
void receiveMessageNonBlocking(
								int iLVC,
								unsigned char * pucData,
								int iMaxSize,
								int * piDataSize,
								RS_Status * pStatus);

/// Query set of LVC to see if any have data to read
void waitOnMultiChannel(
						int * piVCSetIn,
						int iMinVcNo,
						int * piVCSetOut,
						CTimeout * pTimeoutLength,
						TM_Status * pStatus);


/// Write log entry
void logEvent(
	char* pcLogEntry
);

#ifdef __cplusplus
}
#endif

#endif
