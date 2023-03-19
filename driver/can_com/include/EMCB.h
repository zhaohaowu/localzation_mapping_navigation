#ifndef _EMCB_H_
#define _EMCB_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once

#ifndef EMCB_API
	#define EMCB_API __stdcall
#endif
#else
	#define EMCB_API
#endif

#ifdef __cplusplus
extern "C" {
#endif


//-----------------------------------------------------------------------------
//
//	Status codes
//
//-----------------------------------------------------------------------------
/* Description
 *   The library is not yet or unsuccessfully initialized. 
 *   SusiLibInitialize needs to be called prior to the first access of any 
 *   other function.
 * Actions
 *   Call EMCBLibInitialize..
 */
#define EMCB_STATUS_NOT_INITIALIZED				0xFFFFFFFF

/* Description
 *   Library is initialized.
 * Actions
 *   none.
 */
#define EMCB_STATUS_INITIALIZED					0xFFFFFFFE

/* Description 
 *  One or more of the function call parameters are out of the defined range.
 * Actions
 *   Verify Function Parameters.
 */
#define EMCB_STATUS_INVALID_PARAMETER			0xFFFFFEFF

/* Description
 *   This function or channel is not supported at the actual hardware environment.
 * Actions
 *   none.
 */
#define EMCB_STATUS_UNSUPPORTED					0xFFFFFCFF

/* Description
 *   Selected device was not found
 * Actions
 *   none.
 */
#define EMCB_STATUS_NOT_FOUND					0xFFFFFBFF

/* Description
 *    Device has no response.
 * Actions
 *   none.
 */
#define EMCB_STATUS_TIMEOUT						0xFFFFFBFE

/* Description
 *   An error was detected during a read operation.
 * Actions
 *   Retry.
 */
#define EMCB_STATUS_READ_ERROR					0xFFFFFAFF

/* Description
 *   An error was detected during a write operation.
 * Actions
 *   Retry.
 */
#define EMCB_STATUS_WRITE_ERROR					0xFFFFFAFE

#define EMCB_STATUS_WRITE_BUSY					0xFFFFFAFD

/* Description
 *   The amount of available data exceeds the buffer size. Storage buffer 
 *   overflow was prevented. Read count was larger than the defined buffer
 *   length.
 * Actions
 *   Either increase the buffer size or reduce the block length.
 */
#define EMCB_STATUS_MORE_DATA					0xFFFFF9FF

/* Description
 *   The operation has no data.
 * Actions
 *   Wait a while and try again.
 */
#define EMCB_STATUS_NO_DATA						0xFFFFF9FE

/* Description
 *   Generic error message. No further error details are available.
 * Actions
 *   none.
 */
#define EMCB_STATUS_ERROR						0xFFFFF0FF

/* Description
 *   The operation was successful.
 * Actions
 *   none.
 */
#define EMCB_STATUS_SUCCESS						0

//-----------------------------------------------------------------------------
//
//	APIs
//
//-----------------------------------------------------------------------------

// Should be called before calling any other function is called.
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Already initialized				| EMCB_STATUS_INITIALIZED
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBLibInitialize(void);

// Should be called before program exit 
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBLibUninitialize(void);

#define EMCB_EID			1
#define EMCB_SID			0
// Send CAN Message
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// pData == NULL							| EMCB_STATUS_INVALID_PARAMETER
// Len > 8 | EID > 1 | RTR > 1				| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Tx failed								| EMCB_STATUS_WRITE_ERROR
// Tx timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBMsgTx(BYTE ch, UINT32 ID, BYTE EID, BYTE RTR, BYTE *pData,BYTE Len);

// Receive CAN Message
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// pID==NULL || pEID==NULL || pRTR==NULL	| EMCB_STATUS_INVALID_PARAMETER
// pData==NULL || pLen==NULL				| EMCB_STATUS_INVALID_PARAMETER
// *pLen < Receive length					| EMCB_STATUS_MORE_DATA
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Rx failed								| EMCB_STATUS_READ_ERROR
// Rx timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBMsgRx(BYTE ch, UINT32 *pID, BYTE *pEID, BYTE *pRTR, BYTE *pData, BYTE *pLen);

// Reset CAN Bus
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Reset failed								| EMCB_STATUS_WRITE_ERROR
// Reset timeout							| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBReset(BYTE ch);


#define EMCB_IID_TERMINAL_RES					0x00000000		// (RW)
	#define EMCB_TR_ENABLE						1
	#define EMCB_TR_DISABLE						0

#define EMCB_IID_BUS_STATUS						0x00010000		// (RO)
	#define EMCB_BUS_STATUS_NORMAL				0x00
	#define EMCB_BUS_STATUS_RX1OVR				(1 << 7)		// Receive Buffer 1 Overflow
	#define EMCB_BUS_STATUS_RX0OVR				(1 << 6)		// Receive Buffer 0 Overflow
	#define EMCB_BUS_STATUS_TXBO				(1 << 5)		// Bus-Off error
	#define EMCB_BUS_STATUS_TXEP				(1 << 4)		// Transmit Error-Passive
	#define EMCB_BUS_STATUS_RXEP				(1 << 3)		// Receive Error-Passive
	#define EMCB_BUS_STATUS_TXWAR				(1 << 2)		// Transmit Error-Warning
	#define EMCB_BUS_STATUS_RXWAR				(1 << 1)		// Receive Error-Warning
	#define EMCB_BUS_STATUS_EWARN				(1 << 0)		// Error-Warning
	
#define EMCB_IID_BUS_SPEED						0x00010001		// (WO) Unit: kbps

#define EMCB_IID_BUS_MODE						0x00010002		// (WO) 
	#define EMCB_MODE_NORMAL					0x00
	#define EMCB_MODE_LISTENONLY				0x03

#define EMCB_IID_RXB0_MODE						0x00020000		// (WO) 
#define EMCB_IID_RXB1_MODE						0x00020001		// (WO) 
	#define EMCB_BUF_MODE_FILTER_MASK_OFF		0x03
	#define EMCB_BUF_MODE_EX_ONLY				0x02			// Redundant on MCP2515
	#define EMCB_BUF_MODE_STD_ONLY				0x01			// Redundant on MCP2515
	#define EMCB_BUF_MODE_BOTH					0x00

// Set CAN Bus value
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Wrong value								| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Unknown ItemID							| EMCB_STATUS_UNSUPPORTED
// Set failed								| EMCB_STATUS_WRITE_ERROR
// Set timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBSetChannelValue(BYTE ch, UINT32 ItemID, UINT32 Val);

// Get CAN Bus value
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// pValue==NULL								| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Unknown ItemID							| EMCB_STATUS_UNSUPPORTED
// Get failed								| EMCB_STATUS_READ_ERROR
// Get timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBGetChannelValue(BYTE ch, UINT32 ItemID, UINT32 *pVal);

#define EMCB_MASK0		0
#define EMCB_MASK1		1
// Set CAN Mask
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Wrong value of EID/ID/Mask				| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Set failed								| EMCB_STATUS_WRITE_ERROR
// Set timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBSetMask(BYTE ch, BYTE Mask, UINT32 ID, BYTE EID);

#define EMCB_FILTER0		0
#define EMCB_FILTER1		1
#define EMCB_FILTER2		2
#define EMCB_FILTER3		3
#define EMCB_FILTER4		4
#define EMCB_FILTER5		5
// Set CAN Filter
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Wrong value of EID/ID/Filter				| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Set failed								| EMCB_STATUS_WRITE_ERROR
// Set timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBSetFilter(BYTE ch, BYTE Filter, UINT32 ID, BYTE EID);

// Item ID
// -----------------------------------------+------------------------------
//	Version Format:
//		31		23		15				0 bit
//		+-------+-------+---------------+
//		| Major | Minor |     Build     |
//		+-------+-------+---------------+
//
#define EMCB_IID_FW_VER					0x00000000
#define EMCB_IID_LIB_VER				0x00000001
#define EMCB_IID_CHANNEL_NUM			0x00000010
// Get device value by Item ID
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Unknown Item ID							| EMCB_STATUS_UNSUPPORTED
// pVal == NULL								| EMCB_STATUS_INVALID_PARAMETER
// Get failed								| EMCB_STATUS_READ_ERROR
// Get timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBDeviceGetValue(UINT32 ItemID, UINT32 *pVal);

// CMD
#define CAN_READ		0x03
#define CAN_WRITE		0x02
// Write CAN SPI CMD/Data
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Unknown CMD								| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Write failed								| EMCB_STATUS_WRITE_ERROR
// Write timeout							| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBSPIWrite(BYTE ch, BYTE CMD, BYTE Reg, BYTE Data);

// Read CAN SPI/Vendor Data
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// pData == NULL							| EMCB_STATUS_INVALID_PARAMETER
// Unknown CMD								| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Read failed								| EMCB_STATUS_READ_ERROR
// Read timeout								| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBSPIRead(BYTE ch, BYTE CMD, BYTE Reg, BYTE *pData);

// Bit Modify CAN SPI Data
//
// Condition								| Return Values 
// -----------------------------------------+------------------------------
// Success									| EMCB_STATUS_SUCCESS
// Library Uninitialized					| EMCB_STATUS_NOT_INITIALIZED
// Unknown Reg								| EMCB_STATUS_INVALID_PARAMETER
// Unknown ch								| EMCB_STATUS_UNSUPPORTED
// Write failed								| EMCB_STATUS_WRITE_ERROR
// Write timeout							| EMCB_STATUS_TIMEOUT
// Else										| EMCB_STATUS_ERROR
UINT32 EMCB_API EMCBSPIBitModify(BYTE ch, BYTE Reg, BYTE Maks, BYTE Data);

#ifdef __cplusplus
}
#endif

#endif /* _EMCB_H_ */
