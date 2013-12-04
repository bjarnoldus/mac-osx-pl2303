/*
 * osx_pl2303.h Prolific PL2303 USB to serial adaptor driver for OS X
 *
 * Copyright (c) 2013 NoZAP B.V., Jeroen Arnoldus (opensource@nozap.me , http://www.nozap.me http://www.nozap.nl )
 * Copyright (c) 2006-2012 BJA Electronics, Jeroen Arnoldus (opensource@bja-electronics.nl)
 *
 * Additional contributors:
 *     Michael Haller
 *     Maarten Pepping
 *     Bryan Berg
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Source of inspiration:
 * - Linux kernel PL2303.c Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *                         Copyright (C) 2003 IBM Corp.
 * - Apple16x50Serial Copyright (c) 1997-2003 Apple Computer, Inc. All rights reserved.
 *                    Copyright (c) 1994-1996 NeXT Software, Inc.  All rights reserved.
 * - AppleRS232Serial Copyright (c) 2002 Apple Computer, Inc. All rights reserved.
 * - AppleUSBIrda Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
 *
 */

#include <IOKit/IOService.h>
#include <IOKit/serial/IOSerialDriverSync.h>
#include <IOKit/serial/IORS232SerialStreamSync.h>
#include <IOKit/usb/IOUSBDevice.h>

#define PROLIFIC_REV_H			0x0202
#define PROLIFIC_REV_X			0x0300
#define PROLIFIC_REV_HX_CHIP_D	0x0400
#define PROLIFIC_REV_1			0x0001

#define baseName        "NoZAP-PL2303-"

#define defaultName     "PL2303 Device"
#define productNameLength   32                  // Arbitrary length
#define propertyTag     "Product Name"

#define MAX_BLOCK_SIZE			PAGE_SIZE
// was PAGE_SIZE, but due to errors lowered to maximum of 10 characters

#define kXOnChar  '\x11'
#define kXOffChar '\x13'



// New Prolific 2303HX supported speeds form Manual ds_pl2303HXD_v1.1.doc
// Revision Data Apr, 16 2007, Note from prolific (manual page 9):
// By taking advantage of USB bulk transfer mode, large data buffers,
// and automatic flow control, PL-2303HX is capable of achieving higher
// throughput compared to traditional UART (Universal Asynchronous Receiver
// Transmitter) ports. When real RS232 signaling is not required, baud rate
// higher than 115200 bps could be used for even higher performance. The
// flexible baud rate generator of PL-2303HX could be programmed to generate
// any rate between 75 bps to 12M bps.

// My note, however not all the baudrated may be supported by the driver.
// The following ones are given for sure (on page 19) other rates maybe
// available depending on the model.

#define kLinkSpeedIgnored	0
#define kLinkSpeed75		75
#define kLinkSpeed150		150
#define kLinkSpeed300		300
#define kLinkSpeed600		600
#define kLinkSpeed1200		1200
#define kLinkSpeed1800		1800
#define kLinkSpeed2400		2400
#define kLinkSpeed3600		3600
#define kLinkSpeed4800		4800
#define kLinkSpeed7200		7200
#define kLinkSpeed9600		9600
#define kLinkSpeed19200		19200
#define kLinkSpeed38400		38400
#define kLinkSpeed57600		57600
#define kLinkSpeed115200    115200
#define kLinkSpeed230400	230400
#define kLinkSpeed460800	460800
#define kLinkSpeed614400	614400
#define kLinkSpeed921600	921600
#define kLinkSpeed1228800	1228800
#define kLinkSpeed1843200	1843200
#define kLinkSpeed2457600	2457600
#define kLinkSpeed3000000	3000000
#define kLinkSpeed6000000	6000000

#define kDefaultBaudRate    9600
#define kMaxBaudRate        6000000
#define kMinBaudRate        75


// If not working at very high rate one can reconsider also
// increasing the size of the circular buffer to store at
// lease 0.1 sec: before was about 460800/10bits/10= 4608
// now should be  6000000/10bits/10 = 60Kbytes in the worst case
// In my code set it to 16K to balance among a convenient
// speed and memory use.

#define kMaxCirBufferSize   16384


#define LAST_BYTE_COOLDOWN  100000
#define BYTE_WAIT_PENALTY   2

#define SPECIAL_SHIFT       (5)
#define SPECIAL_MASK        ((1<<SPECIAL_SHIFT) - 1)
#define STATE_ALL           ( PD_RS232_S_MASK | PD_S_MASK )
#define FLOW_RX_AUTO        ( PD_RS232_A_RFR | PD_RS232_A_DTR | PD_RS232_A_RXO )
#define FLOW_TX_AUTO        ( PD_RS232_A_CTS | PD_RS232_A_DSR | PD_RS232_A_TXO | PD_RS232_A_DCD )
#define CAN_BE_AUTO         ( FLOW_RX_AUTO | FLOW_TX_AUTO )
#define CAN_NOTIFY          ( PD_RS232_N_MASK )
#define EXTERNAL_MASK       ( PD_S_MASK | (PD_RS232_S_MASK & ~PD_RS232_S_LOOP) )
#define INTERNAL_DELAY      ( PD_RS232_S_LOOP )
#define DEFAULT_AUTO        ( PD_RS232_A_DTR | PD_RS232_A_RFR | PD_RS232_A_CTS | PD_RS232_A_DSR )
#define DEFAULT_NOTIFY      0x00
#define DEFAULT_STATE       ( PD_S_TX_ENABLE | PD_S_RX_ENABLE | PD_RS232_A_TXO | PD_RS232_A_RXO )

#define	CONTINUE_SEND		1
#define	PAUSE_SEND		2

#define kRxAutoFlow	((UInt32)( PD_RS232_A_RFR | PD_RS232_A_DTR | PD_RS232_A_RXO ))
#define kTxAutoFlow	((UInt32)( PD_RS232_A_CTS | PD_RS232_A_DSR | PD_RS232_A_TXO | PD_RS232_A_DCD ))
#define kControl_StateMask	((UInt32)( PD_RS232_S_CTS | PD_RS232_S_DSR | PD_RS232_S_CAR | PD_RS232_S_RI  ))
#define kRxQueueState ((UInt32)( PD_S_RXQ_EMPTY | PD_S_RXQ_LOW_WATER | PD_S_RXQ_HIGH_WATER | PD_S_RXQ_FULL ))
#define kTxQueueState ((UInt32)( PD_S_TXQ_EMPTY | PD_S_TXQ_LOW_WATER | PD_S_TXQ_HIGH_WATER | PD_S_TXQ_FULL ))

#define kCONTROL_DTR		0x01
#define kCONTROL_RTS		0x02


enum tXO_State {
    kXOnSent = -2,
    kXOffSent = -1,
    kXO_Idle = 0,
    kXOffNeeded = 1,
    kXOnNeeded = 2
} ;

//#define IDLE_XO          0
//#define NEEDS_XOFF       1
//#define SENT_XOFF       -1
//#define NEEDS_XON        2
//#define SENT_XON        -2

#define kStateTransientMask	0x74
#define kBreakError			0x04
#define kFrameError			0x10
#define kParityError		0x20
#define kOverrunError		0x40

#define kCTS				0x80
#define kDSR				0x02
#define kRI					0x08
#define kDCD				0x01
#define kHandshakeInMask	((UInt32)( PD_RS232_S_CTS | PD_RS232_S_DSR | PD_RS232_S_CAR | PD_RS232_S_RI  ))


#define INTERRUPT_BUFF_SIZE 10
#define USBLapPayLoad       2048

#define kUART_STATE			0x08

#define SET_LINE_REQUEST_TYPE		0x21
#define SET_LINE_REQUEST			0x20

#define SET_CONTROL_REQUEST_TYPE	0x21
#define SET_CONTROL_REQUEST			0x22
#define CONTROL_DTR					0x01
#define CONTROL_RTS					0x02

#define BREAK_REQUEST_TYPE			0x21
#define BREAK_REQUEST				0x23
#define BREAK_ON					0xffff
#define BREAK_OFF					0x0000

#define GET_LINE_REQUEST_TYPE		0xa1
#define GET_LINE_REQUEST			0x21

#define VENDOR_WRITE_REQUEST_TYPE	0x40
#define VENDOR_WRITE_REQUEST		0x01

#define VENDOR_READ_REQUEST_TYPE	0xc0
#define VENDOR_READ_REQUEST			0x01

#define SIEMENS_VENDOR_ID			0x11f5
#define SIEMENS_PRODUCT_ID_X65		0x0003

/*
 * Device Configuration Registers (DCR0, DCR1, DCR2)
 */

#define SET_DCR0                                0x00
#define GET_DCR0                                0x80
#define DCR0_INIT                               0x01
#define DCR0_INIT_H                             0x41
#define DCR0_INIT_X                             0x61

#define SET_DCR1                                0x01
#define GET_DCR1                                0x81
#define DCR1_INIT_H                             0x80
#define DCR1_INIT_X                             0x00

#define SET_DCR2                                0x02
#define GET_DCR2                                0x82
#define DCR2_INIT_H                             0x24
#define DCR2_INIT_X                             0x44

/*
 * On-chip Date Buffers:
 */
#define RESET_DOWNSTREAM_DATA_PIPE              0x08
#define RESET_UPSTREAM_DATA_PIPE                0x09

enum pl2303_type {
	unknown,
	type_1,		/* don't know the difference between type 0 and */
	rev_X,		/* type 1, until someone from prolific tells us... */
	rev_HX,		/* HX version of the pl2303 chip */
	rev_H
};


typedef struct BufferMarks
{
    unsigned long   BufferSize;
    unsigned long   HighWater;
    unsigned long   LowWater;
    bool            OverRun;
} BufferMarks;

typedef struct CirQueue
{
    UInt8   *Start;
    UInt8   *End;
    UInt8   *NextChar;
    UInt8   *LastChar;
    size_t  Size;
    size_t  InQueue;
} CirQueue;

typedef enum QueueStatus
{
    kQueueNoError = 0,
    kQueueFull,
    kQueueEmpty,
    kQueueMaxStatus
} QueueStatus;


// selects between bits of a and b.  if a bit in m is set, then take the corresponding
// bit of b; otherwise, take the corresponding bit of a.
UInt32 static inline maskMux(UInt32 a, UInt32 b, UInt32 m) { return (a&(~m)) | (b&m); }

// if b is true, then return a with all m's bits set; otherwise return a with all m's
// bits cleared.
UInt32 static inline boolBit(UInt32 a, bool b, UInt32 m) { return b ? (a|m) : (a&(~m)); }


/* Inline time conversions */

static inline unsigned long tval2long( mach_timespec val )
{
    return (val.tv_sec * NSEC_PER_SEC) + val.tv_nsec;
}

static inline mach_timespec long2tval( unsigned long val )
{
    mach_timespec   tval;
    
    tval.tv_sec  = val / NSEC_PER_SEC;
    tval.tv_nsec = val % NSEC_PER_SEC;
    return tval;
}


typedef struct
{
	enum pl2303_type type;
    UInt32          State;
	UInt8          lineState;
    
    UInt32          WatchStateMask;
    IOLock          *serialRequestLock;
    
	// queue control structures:
    
    CirQueue        RX;
    CirQueue        TX;
    
    BufferMarks     RXStats;
    BufferMarks     TXStats;
    
	// UART configuration info:
    
    UInt32          CharLength;
    UInt32          StopBits;
    UInt32          TX_Parity;
    UInt32          RX_Parity;
    UInt32          BaudRate;
    UInt8           FCRimage;
    UInt8           IERmask;
    bool            MinLatency;
    
	// flow control state & configuration:
    
    UInt8           XONchar;
    UInt8           XOFFchar;
    UInt32          SWspecial[ 0x100 >> SPECIAL_SHIFT ];
    UInt32          FlowControl;    // notify-on-delta & auto_control
	
    tXO_State       RXOstate;    /* Indicates our receive state.    */
    tXO_State       TXOstate;    /* Indicates our transmit state, if we have received any Flow Control. */
    
    UInt32			FlowControlState;			// tx flow control state, one of PAUSE_SEND if paused or CONTINUE_SEND if not blocked
    bool			DCDState;
    bool			CTSState;
    bool			xOffSent;				// init false, set true if sw flow control and we've sent an xoff
    bool			DTRAsserted;				// init true, set false if DTR flow control and DTR is cleared to hold back rx
    bool			RTSAsserted;				// init true, set false if RTS flow control and RTS is cleared to hold back rx
    bool			aboveRxHighWater;
    bool			BreakState;
    
    IOThread        FrameTOEntry;
    
    mach_timespec   DataLatInterval;
    mach_timespec   CharLatInterval;
    
    bool            AreTransmitting;
    
	/* extensions to handle the Driver */
    
    bool            isDriver;
    void            *DriverPowerRegister;
    UInt32          DriverPowerMask;
	
	
} PortInfo_t;

class me_nozap_driver_PL2303 : public IOSerialDriverSync
{
	OSDeclareDefaultStructors(me_nozap_driver_PL2303)
private:
    UInt32          fCount;         // usb write length
    UInt8           fSessions;      // Active sessions (count of opens on /dev/tty entries)
    bool            fUSBStarted;        // usb family has started (stopped) us
    bool            fTerminate;     // Are we being terminated (ie the device was unplugged)
    UInt8           fProductName[productNameLength];    // Actually the product String from the Device
    PortInfo_t      *fPort;         // The Port
    bool            fReadActive;    // usb read is active
#if FIX_PARITY_PROCESSING
    clock_sec_t			_fReadTimestampSecs;
    clock_nsec_t        _fReadTimestampNanosecs;
#endif
    bool            fWriteActive;   // usb write is active
    UInt8           fPowerState;    // off,on ordinal for power management
	IORS232SerialStreamSync		*fNub;              // glue back to IOSerialStream side
    
    IOWorkLoop			*fWorkLoop;		// holds the workloop for this driver
	IOCommandGate		*fCommandGate;		// and the command gate
    
    UInt32          fBaudCode;          //  encoded baud code for change speed byte
    UInt32          fCurrentBaud;       //  current speed in bps
    
    
    
	IOBufferMemoryDescriptor    *fpinterruptPipeMDP;
    IOBufferMemoryDescriptor    *fpPipeInMDP;
    IOBufferMemoryDescriptor    *fpPipeOutMDP;
    
    UInt8               *fpinterruptPipeBuffer;
    UInt8               *fPipeInBuffer;
    UInt8               *fPipeOutBuffer;
    
    UInt8               fpInterfaceNumber;
    
    IOUSBCompletion     finterruptCompletionInfo;
    IOUSBCompletion     fReadCompletionInfo;
    IOUSBCompletion     fWriteCompletionInfo;
    
    static void         interruptReadComplete(  void *obj, void *param, IOReturn ior, UInt32 remaining );
    static void         dataReadComplete(  void *obj, void *param, IOReturn ior, UInt32 remaining );
    static void         dataWriteComplete( void *obj, void *param, IOReturn ior, UInt32 remaining );
    
    bool                initForPM(IOService *provider);
	
public:
	IOUSBDevice         *fpDevice;
    IOUSBInterface      *fpInterface;
    IOUSBPipe           *fpInPipe;
    IOUSBPipe           *fpOutPipe;
    IOUSBPipe           *fpInterruptPipe;
	// IOKit methods
    
	virtual bool init(OSDictionary *dictionary = 0);
	virtual IOService*  probe( IOService *provider, SInt32 *score );
    
	virtual void free(void);
	virtual bool start(IOService *provider);
	virtual void stop(IOService *provider);
	virtual IOReturn    message( UInt32 type, IOService *provider,  void *argument = 0 );
    
	
	// IORS232SerialStreamSync Abstract Method Implementation
    
	virtual	IOReturn	acquirePort(bool sleep, void *refCon);
    virtual	IOReturn	releasePort(void *refCon);
    virtual	UInt32		getState(void *refCon);
    virtual	IOReturn	setState(UInt32 state, UInt32 mask, void *refCon);
    virtual	IOReturn	watchState(UInt32 *state, UInt32 mask, void *refCon);
    virtual UInt32		nextEvent(void *refCon);
    virtual	IOReturn	executeEvent(UInt32 event, UInt32 data, void *refCon);
    virtual	IOReturn	requestEvent(UInt32 event, UInt32 *data, void *refCon);
    virtual	IOReturn	enqueueEvent(UInt32 event, UInt32 data, bool sleep, void *refCon);
    virtual	IOReturn	dequeueEvent(UInt32 *event, UInt32 *data, bool sleep, void *refCon);
    virtual	IOReturn	enqueueData(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *refCon);
    virtual	IOReturn	dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon);
    
    virtual	IOWorkLoop	*getWorkLoop() const;
    
	// Static stubs for IOCommandGate::runAction
    
    static	IOReturn	acquirePortAction(OSObject *owner, void *arg0, void *arg1, void *, void *);
	static	IOReturn	releasePortAction(OSObject *owner, void *arg1, void *, void *, void *);
    static	IOReturn	setStateAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *);
    static	IOReturn	watchStateAction(OSObject *owner, void *arg0, void *arg1, void *, void *);
	
    static	IOReturn	executeEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *);
    static	IOReturn	requestEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *);
    static	IOReturn	enqueueEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *);
    static	IOReturn	enqueueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3);
    static	IOReturn	dequeueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3);
    
	// Gated methods called by the Static stubs
	virtual	IOReturn	acquirePortGated(bool sleep, void *refCon);
    virtual	IOReturn	releasePortGated(void *refCon);
    virtual	IOReturn	setStateGated(UInt32 state, UInt32 mask, void *refCon);
    virtual	IOReturn	watchStateGated(UInt32 *state, UInt32 mask);
    virtual	IOReturn	executeEventGated(UInt32 event, UInt32 data, void *refCon);
    virtual	IOReturn	requestEventGated(UInt32 event, UInt32 *data, void *refCon);
    virtual	IOReturn	enqueueDataGated(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep);
    virtual	IOReturn	dequeueDataGated(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min);
    
	bool				setUpTransmit( void );
	IOReturn			setSerialConfiguration( void );
    IOReturn			startTransmit( UInt32 control_length, UInt8 *control_buffer, UInt32 data_length, UInt8 *data_buffer );
	
	
private:
    
	/**** Queue primatives ****/
    
    QueueStatus     addBytetoQueue( CirQueue *Queue, char Value );
    QueueStatus     getBytetoQueue( CirQueue *Queue, UInt8 *Value );
    QueueStatus     peekBytefromQueue( CirQueue *Queue, UInt8 *Value, size_t offset);
    QueueStatus     initQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size );
    QueueStatus     closeQueue( CirQueue *Queue );
	QueueStatus     flush( CirQueue *Queue );
	QueueStatus     getQueueStatus( CirQueue *Queue );
    size_t          addtoQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size );
    size_t          removefromQueue( CirQueue *Queue, UInt8 *Buffer, size_t MaxSize );
    size_t          freeSpaceinQueue( CirQueue *Queue );
    size_t          usedSpaceinQueue( CirQueue *Queue );
    size_t          getQueueSize( CirQueue *Queue );
    void            checkQueues( PortInfo_t *port );
    
	/**** State manipulations ****/
    
    IOReturn        privateWatchState( PortInfo_t *port, UInt32 *state, UInt32 mask );
    UInt32          readPortState( PortInfo_t *port );
    void            changeState( PortInfo_t *port, UInt32 state, UInt32 mask );
    IOReturn        CheckSerialState();       // combines fSessions, fStartStopUserClient, fStartStopUSB to new state
	/**** USB Specific ****/
    
    virtual bool    configureDevice( UInt8 numConfigs );
    void            Workaround();                               // reset confused silicon
    
    bool            allocateResources( void );                  // allocate pipes
    void            releaseResources( void );                   // free pipes
    bool            startPipes();                               // start the usb reads going
    void            stopPipes();
    bool            createSerialStream();                       // create bsd stream
    void            destroySerialStream();                      // delete bsd stream
    bool            createSuffix( unsigned char *sufKey );
    bool            startSerial();                               // start serial up
    void            stopSerial( bool resetDevice );             // shut down serial
    bool            createNub();                                // create nub (and port)
    void            destroyNub();
    void            SetStructureDefaults( PortInfo_t *port, bool Init );
    bool            allocateRingBuffer( CirQueue *Queue, size_t BufferSize );
    void            freeRingBuffer( CirQueue *Queue );
    
    /**** FlowControl ****/
	IOReturn        setControlLines( PortInfo_t *port );
    UInt32			generateRxQState( PortInfo_t *port );
	IOReturn		setBreak( bool data);
	
	IOReturn        vendor_write0( UInt16 value, UInt16 index);
    
};

