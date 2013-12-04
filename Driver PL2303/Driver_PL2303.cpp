/*
 *
 * osx_pl2303.cpp Prolific PL2303 USB to serial adaptor driver for OS X
 *
 * Copyright (c) 2013 NoZAP B.V., Jeroen Arnoldus (opensource@nozap.me, http://www.nozap.me http://www.nozap.nl )
 * Copyright (c) 2006 BJA Electronics, Jeroen Arnoldus (opensource@bja-electronics.nl)
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
 *
 *
 */


#define FIX_PARITY_PROCESSING 1

#include <IOKit/IOLib.h>
#include <IOKit/IOTypes.h>
#include <IOKit/IOMessage.h>


#include "Driver_pl2303.h"
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/usb/IOUSBInterface.h>
#include <IOKit/usb/IOUSBLog.h>
#include <kern/clock.h>


extern "C" {
#include <pexpert/pexpert.h>
}

//To enable logging remove comments from #define DEBUG and #define DATALOG
//Use USB Prober to monitor the logs.
//#define DEBUG
//#define DATALOG

#ifdef DEBUG
#define DEBUG_IOLog(args...) USBLog (args)
#else
#define DEBUG_IOLog(args...)
#endif


#ifdef DATALOG
#define DATA_IOLog(args...)	USBLog (args)
#else
#define DATA_IOLog(args...)
#endif


#define super IOSerialDriverSync

OSDefineMetaClassAndStructors(me_nozap_driver_PL2303, IOSerialDriverSync)


/****************************************************************************************************/
//
//      Function:   Asciify
//
//      Inputs:     i - the nibble
//
//      Outputs:    return byte - ascii byte
//
//      Desc:       Converts to ascii.
//
/****************************************************************************************************/

static UInt8 Asciify(UInt8 i)
{
    i &= 0xF;
    if ( i < 10 )
		return( '0' + i );
    else return( 55  + i );
    
}/* end Asciify */

bool me_nozap_driver_PL2303::init(OSDictionary *dict)
{
	bool res = super::init(dict);
	DEBUG_IOLog(4,"%s(%p)::Initializing\n", getName(), this);
	return res;
}

void me_nozap_driver_PL2303::free(void)
{
	DEBUG_IOLog(4,"%s(%p)::Freeing\n", getName(), this);
	super::free();
}

IOService *me_nozap_driver_PL2303::probe(IOService *provider, SInt32 *score)
{
	IOUSBDevice		*Provider;
	DEBUG_IOLog(4,",%s(%p)::Probe\n", getName(), this);
	Provider = OSDynamicCast(IOUSBDevice, provider);
    if (!Provider) {
        IOLog("%s(%p)::Probe Attached to non-IOUSBDevice provider!  Failing probe()\n", getName(), this);
        return NULL;
    }
	IOService *res = super::probe(provider, score);
	DEBUG_IOLog(5,"%s(%p)::Probe successful\n", getName(), this);
	return res;
}


bool me_nozap_driver_PL2303::start(IOService *provider)
{
    enum pl2303_type type = type_1;
    
    OSNumber *release;
    
    fTerminate = false;     // Make sure we don't think we're being terminated
    fPort = NULL;
    fNub = NULL;
    fpInterface = NULL;
    
    fpinterruptPipeBuffer = NULL;
    fPipeInBuffer = NULL;
    fPipeOutBuffer = NULL;
    
    fpDevice = NULL;
    fpInPipe = NULL;
    fpOutPipe = NULL;
    fpInterruptPipe = NULL;
    
    fUSBStarted = false;            // set to true when start finishes up ok
    fSessions = 0;
    
    fReadActive = false;
	fWriteActive = false;
	
    DEBUG_IOLog(4,"%s(%p)::start PL2303 Driver\n", getName(), this);
	
    if( !super::start( provider ) )
	{
		IOLog("%s(%p)::start - super failed\n", getName(), this);
        goto Fail;
    }
	
    fpDevice = OSDynamicCast(IOUSBDevice, provider);
	
    if(!fpDevice)
    {
        IOLog("%s(%p)::start - Provider isn't a USB device!!!\n", getName(), this);
        goto Fail;
    }
    
	
    if (fpDevice->GetNumConfigurations() < 1)
    {
        IOLog("%s(%p)::start - no composite configurations\n", getName(), this);
        goto Fail;
    }
    
    // make our nub (and fPort) now
    if( !createNub() ) goto Fail;
	
    // Now configure it (leaves device suspended)
    if( !configureDevice( fpDevice->GetNumConfigurations() ) ) goto Fail;
	
    // Finally create the bsd tty (serial stream) and leave it there until usb stop
	
    if( !createSerialStream() ) goto Fail;
    
    fWorkLoop = getWorkLoop();
    if (!fWorkLoop)
    {
        IOLog("%s(%p)::start - getWorkLoop failed\n", getName(), this);
        goto Fail;
    }
    
    fWorkLoop->retain();
    
    fCommandGate = IOCommandGate::commandGate(this);
    if (!fCommandGate)
    {
        IOLog("%s(%p)::start - create commandGate failed\n", getName(), this);
        goto Fail;
    }
    
    if (fWorkLoop->addEventSource(fCommandGate) != kIOReturnSuccess)
    {
        IOLog("%s(%p)::start - addEventSource fCommandGate to WorkLoop failed\n", getName(), this);
        goto Fail;
    }
	
    fCommandGate->enable();
    
    release = (OSNumber *) fpDevice->getProperty(kUSBDeviceReleaseNumber);
    
	DEBUG_IOLog(1,"%s(%p)::start - Get device version: %p \n", getName(), this, release->unsigned16BitValue() );
	
	if (release->unsigned16BitValue()==PROLIFIC_REV_H) {
		DEBUG_IOLog(1,"%s(%p)::start - Chip type: H \n" );
		type = type_1; // was rev_H
	} else if ( release->unsigned16BitValue()==PROLIFIC_REV_X ) {
		DEBUG_IOLog(1,"%s(%p)::start - Chip type: X \n" );
		type = rev_HX; // was rev_X
	} else if ( release->unsigned16BitValue()==PROLIFIC_REV_HX_CHIP_D ) {
		DEBUG_IOLog(1,"%s(%p)::start - Chip type: HX \n" );
		type = rev_HX;
	} else if ( release->unsigned16BitValue()==PROLIFIC_REV_1 ) {
		DEBUG_IOLog(1,"%s(%p)::start - Chip type: 1 \n" );
		type = type_1;
	} else {
		DEBUG_IOLog(1,"%s(%p)::start - Chip type: unkwown \n" );
		type = unknown;
	}
    
	
    fPort->type = type;
    
	fUSBStarted = true;
	
	DEBUG_IOLog(3,"%s(%p)::start - Allocate resources \n", getName(), this);
    
    
	return true;
    
    
Fail:
	if (fNub)
	{
		destroyNub();
	}
    if (fCommandGate)
    {
        fCommandGate->release();
        fCommandGate = NULL;
    }
    if (fWorkLoop)
    {
        fWorkLoop->release();
        fWorkLoop = NULL;
    }
    DEBUG_IOLog(1,"%s(%p)::start - failed\n", getName(), this);
    stop( provider );
    return false;
    
}


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::stop
//
//      Inputs:     provider - my provider
//
//      Outputs:    None
//
//      Desc:       Stops
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::stop( IOService *provider )
{
    
    
    
    fUSBStarted = false;        // reset usb start/stop flag for CheckSerialState
    CheckSerialState();         // turn serial off, release resources
	DEBUG_IOLog(5,"%s(%p)::stop  CheckSerialState succeed\n", getName(), this);
    
    if (fCommandGate)
    {
        fCommandGate->release();
        fCommandGate = NULL;
		DEBUG_IOLog(5,"%s(%p)::stop Command gate destroyed\n", getName(), this);
    }
    if (fWorkLoop)
    {
        fWorkLoop->release();
        fWorkLoop = NULL;
		DEBUG_IOLog(5,"%s(%p)::stop workloop destroyed\n", getName(), this);
    }
	
    destroySerialStream();      // release the bsd tty
	
    destroyNub();               // delete the nubs and fPort
    
    if (fpInterface)
	{
		fpInterface->release();     // retain done in ConfigureDevice
		fpInterface = NULL;
		DEBUG_IOLog(5,"%s(%p)::stop fpInterface destroyed\n", getName(), this);
	}
	
	// release our power manager state - NOT IMPLEMENTED
	//   PMstop();
    
    super::stop( provider );
    return;
    
}/* end stop */


/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::getWorkLoop
//
//		Inputs:
//
//		Outputs:
//
//		Desc:		create our own workloop if we don't have one already.
//
/****************************************************************************************************/
IOWorkLoop* me_nozap_driver_PL2303::getWorkLoop() const
{
    IOWorkLoop *w;
    DEBUG_IOLog(4,"%s(%p)::getWorkLoop\n", getName(), this);
    
    if (fWorkLoop) w = fWorkLoop;
    else  w = IOWorkLoop::workLoop();
    
    return w;
    
}/* end getWorkLoop */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::privateWatchState
//
//      Inputs:     port - the specified port, state - state watching for, mask - state mask (the specific bits)
//
//      Outputs:    IOReturn - kIOReturnSuccess, kIOReturnIOError or kIOReturnIPCError
//
//      Desc:       Wait for the at least one of the state bits defined in mask to be equal
//                  to the value defined in state. Check on entry then sleep until necessary.
//                  A return value of kIOReturnSuccess means that at least one of the port state
//                  bits specified by mask is equal to the value passed in by state.  A return
//                  value of kIOReturnIOError indicates that the port went inactive.  A return
//                  value of kIOReturnIPCError indicates sleep was interrupted by a signal.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::privateWatchState( PortInfo_t *port, UInt32 *state, UInt32 mask )
{
    unsigned            watchState, foundStates;
    bool                autoActiveBit   = false;
    IOReturn            rtn             = kIOReturnSuccess;
	
    DEBUG_IOLog(4,"%s(%p)::privateWatchState\n", getName(), this);
	
    watchState              = *state;
    
	// hack to get around problem with carrier detection - Do we need this hack ??
	
    
	
    if ( !(mask & (PD_S_ACQUIRED | PD_S_ACTIVE)) )
	{
		watchState &= ~PD_S_ACTIVE; // Check for low PD_S_ACTIVE
		mask       |=  PD_S_ACTIVE; // Register interest in PD_S_ACTIVE bit
		autoActiveBit = true;
	}
	
    for (;;)
    {
	    // Check port state for any interesting bits with watchState value
	    // NB. the '^ ~' is a XNOR and tests for equality of bits.
		DEBUG_IOLog(4,"%s(%p)::privateWatchState :watchState %p port->State %p mask %p\n", getName(), this, watchState,port->State,mask );
        
		foundStates = (watchState ^ ~port->State) & mask;
		DEBUG_IOLog(4,"%s(%p)::privateWatchState :foundStates %p \n", getName(), this, foundStates );
        
		if ( foundStates )
		{
			*state = port->State;
			if ( autoActiveBit && (foundStates & PD_S_ACTIVE) )
			{
				rtn = kIOReturnIOError;
			} else {
				rtn = kIOReturnSuccess;
			}
			break;
		}
		port->WatchStateMask |= mask;
        
		retain();							// Just to make sure all threads are awake
		fCommandGate->retain();					// before we're released
        
		rtn = fCommandGate->commandSleep((void *)&port->State);
        
		fCommandGate->retain();
		
		if (rtn == THREAD_TIMED_OUT)
		{
			rtn = kIOReturnTimeout;
			break;
		} else {
			if (rtn == THREAD_INTERRUPTED)
			{
				rtn = kIOReturnAborted;
				break;
			}
		}
		release();
		
    }/* end for */
    
    // As it is impossible to undo the masking used by this
    // thread, we clear down the watch state mask and wakeup
    // every sleeping thread to reinitialize the mask before exiting.
	
    port->WatchStateMask = 0;
	fCommandGate->commandWakeup((void *)&port->State);
	DEBUG_IOLog(4,"%s(%p)::privateWatchState end\n", getName(), this);
    
    return rtn;
    
}/* end privateWatchState */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::allocateResources
//
//      Inputs:
//
//      Outputs:    return code - true (allocate was successful), false (it failed)
//
//      Desc:       Finishes up the rest of the configuration and gets all the endpoints open
//
/****************************************************************************************************/

bool me_nozap_driver_PL2303::allocateResources( void )
{
    IOUSBFindEndpointRequest    epReq;      // endPoint request struct on stack
    bool                        goodCall;   // return flag fm Interface call
	vm_size_t					aBuffSize;
	
	DEBUG_IOLog(4,"%s(%p)::allocateResources\n", getName(), this);
	
    // Open all the end points
    if (!fpInterface) {
	    IOLog("%s(%p)::allocateResources failed - no fpInterface.\n", getName(), this);
		goto Fail;
	}
	
    goodCall = fpInterface->open( this );       // close done in releaseResources
    if ( !goodCall ){
		IOLog("%s(%p)::allocateResources - open data interface failed.\n", getName(), this);
		fpInterface->release();
		fpInterface = NULL;
		return false;
    }
	
    fpInterfaceNumber = fpInterface->GetInterfaceNumber();
    
    epReq.type          = kUSBBulk;
    epReq.direction     = kUSBIn;
    epReq.maxPacketSize = 0;
    epReq.interval      = 0;
    fpInPipe = fpInterface->FindNextPipe( 0, &epReq );
	if (!fpInPipe) {
	    IOLog("%s(%p)::allocateResources failed - no fpInPipe.\n", getName(), this);
		goto Fail;
	}
	
    epReq.direction = kUSBOut;
    fpOutPipe = fpInterface->FindNextPipe( 0, &epReq );
	if (!fpOutPipe) {
	    IOLog("%s(%p)::allocateResources failed - no fpOutPipe.\n", getName(), this);
		goto Fail;
	}
	
    epReq.type          = kUSBInterrupt;
    epReq.direction     = kUSBIn;
    fpInterruptPipe = fpInterface->FindNextPipe( 0, &epReq );
	if (!fpInterruptPipe) {
	    IOLog("%s(%p)::allocateResources failed - no fpInterruptPipe.\n", getName(), this);
		goto Fail;
	}
	
    // Allocate Memory Descriptor Pointer with memory for the interrupt-in pipe:
	aBuffSize = INTERRUPT_BUFF_SIZE;
	if ( (fpDevice->GetVendorID() == SIEMENS_VENDOR_ID ) && (fpDevice->GetProductID() == SIEMENS_PRODUCT_ID_X65) ) {
        aBuffSize = 1;
        DEBUG_IOLog( 3, "%s(%p)::allocateResources interrupt Buff size = 1\n", getName(), this);
    }
    fpinterruptPipeMDP = IOBufferMemoryDescriptor::withCapacity( aBuffSize, kIODirectionIn );
	if (!fpinterruptPipeMDP) {
	    IOLog("%s(%p)::allocateResources failed - no fpinterruptPipeMDP.\n", getName(), this);
		goto Fail;
	}
    fpinterruptPipeMDP->setLength( aBuffSize );
    fpinterruptPipeBuffer = (UInt8*)fpinterruptPipeMDP->getBytesNoCopy();
    // Allocate Memory Descriptor Pointer with memory for the data-in bulk pipe:
	
    fpPipeInMDP = IOBufferMemoryDescriptor::withCapacity( USBLapPayLoad, kIODirectionIn );
	if (!fpPipeInMDP) {
	    IOLog("%s(%p)::allocateResources failed - no fpPipeInMDP.\n", getName(), this);
		goto Fail;
	}
    fpPipeInMDP->setLength( USBLapPayLoad );
    fPipeInBuffer = (UInt8*)fpPipeInMDP->getBytesNoCopy();
	
    // Allocate Memory Descriptor Pointer with memory for the data-out bulk pipe:
	
    fpPipeOutMDP = IOBufferMemoryDescriptor::withCapacity( MAX_BLOCK_SIZE, kIODirectionOut );
	if (!fpPipeOutMDP) {
	    IOLog("%s(%p)::allocateResources failed - no fpPipeOutMDP.\n", getName(), this);
		goto Fail;
	}
    fpPipeOutMDP->setLength( MAX_BLOCK_SIZE );
    fPipeOutBuffer = (UInt8*)fpPipeOutMDP->getBytesNoCopy();
    
    // set up the completion info for all three pipes
    
	if (!fPort) {
	    IOLog("%s(%p)::allocateResources failed - no fPort.\n", getName(), this);
		goto Fail;
	}
    finterruptCompletionInfo.target = this;
    finterruptCompletionInfo.action = interruptReadComplete;
    finterruptCompletionInfo.parameter  = fPort;
    
    fReadCompletionInfo.target  = this;
    fReadCompletionInfo.action  = dataReadComplete;
    fReadCompletionInfo.parameter   = fPort;
	
    fWriteCompletionInfo.target = this;
    fWriteCompletionInfo.action = dataWriteComplete;
    fWriteCompletionInfo.parameter  = fPort;
	
	if( setSerialConfiguration() ){
		IOLog("%s(%p)::allocateResources setSerialConfiguration failed\n", getName(), this);
		goto Fail;
	}
    
    
    DEBUG_IOLog(5,"%s(%p)::allocateResources successful\n", getName(), this);
    return true;
	
Fail:
    return false;
    
} // allocateResources


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::releaseResources
//
//      Inputs:     port - the Port
//
//      Outputs:    None
//
//      Desc:       Frees up the pipe resources allocated in allocateResources
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::releaseResources( void )
{
    DEBUG_IOLog(4,"me_nozap_driver_PL2303::releaseResources\n");
    
    if ( fpInterface ) {
		fpInterface->close( this );
    }
    
    if ( fpPipeOutMDP  ) {
		fpPipeOutMDP->release();
		fpPipeOutMDP    = 0;
    }
    
    if ( fpPipeInMDP   ) {
		fpPipeInMDP->release();
		fpPipeInMDP     = 0;
    }
    
    if ( fpinterruptPipeMDP ) {
		fpinterruptPipeMDP->release();
		fpinterruptPipeMDP  = 0;
    }
	
    return;
    
}/* end releaseResources */



//
// startSerial
//
// assumes createSerialStream is called once at usb start time
// calls allocateResources to open endpoints
//
bool me_nozap_driver_PL2303::startSerial()
{
	IOUSBDevRequest request;
	char * buf;
	IOReturn rtn;
	DEBUG_IOLog(1,"%s(%p)::startSerial \n", getName(), this);
	
    
	
	/* Ugly hack to make device clean */
	DEBUG_IOLog(5,"%s(%p)::startSerial RESET DEVICE \n", getName(), this);
	fUSBStarted = false;
	DEBUG_IOLog(5,"%s(%p)::startSerial close device-1\n", getName(), this);
	if(fpDevice) { fpDevice->close( fpDevice ); }
	DEBUG_IOLog(5,"%s(%p)::startSerial reset device-1 \n", getName(), this);
	if(fpDevice) { fpDevice->ResetDevice(); }
	int i;
	i = 0;
	while (!fUSBStarted & (i < 10)) {	IOSleep(10); i++; }
	DEBUG_IOLog(5,"%s(%p)::startSerial close device-2 timout: %d \n", getName(), this, i);
	if(fpDevice) { fpDevice->close( fpDevice ); }
	DEBUG_IOLog(5,"%s(%p)::startSerial reset device-2 \n", getName(), this);
	if(fpDevice) { fpDevice->ResetDevice(); }
	/*    ****************************     */
    
    
    if (!fNub) {
		IOLog("%s(%p)::startSerial fNub not available\n", getName(), this);
		goto	Fail;
	}
	
	buf = (char *) IOMalloc(10);
    if (!buf) {
		IOLog("%s(%p)::startSerial could not alloc memory for buf\n", getName(), this);
		goto	Fail;
	}
    
    
    
    // make chip as sane as can be
#define FISH(a,b,c,d)								\
request.bmRequestType = a; \
request.bRequest = b; \
request.wValue =  c; \
request.wIndex = d; \
request.wLength = 1; \
request.pData = buf; \
rtn =  fpDevice->DeviceRequest(&request); \
DEBUG_IOLog(5,"%s(%p)::startSerial FISH 0x%x:0x%x:0x%x:0x%x  %d - %x\n", getName(), this,a,b,c,d,rtn,buf[0]);
    
#define SOUP(a,b,c,d)								\
request.bmRequestType = a; \
request.bRequest = b; \
request.wValue =  c; \
request.wIndex = d; \
request.wLength = 0; \
request.pData = NULL; \
rtn =  fpDevice->DeviceRequest(&request); \
DEBUG_IOLog(5,"%s(%p)::startSerial SOUP 0x%x:0x%x:0x%x:0x%x  %d\n", getName(), this,a,b,c,d,rtn);
    
    
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0x0404, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8383, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0x0404, 1);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8383, 0);
    //	FISH (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0x81, 1);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0, 1);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 1, 0);
    
	if (fPort->type == rev_HX) {
		/* HX chip */
		SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 2, 0x44);
		/* reset upstream data pipes */
        SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 8, 0);
        SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 9, 0);
	} else {
		SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 2, 0x24);
	}
	
    IOFree(buf, 10);
	
	// open the pipe endpoints
	if (!allocateResources() ) {
		IOLog("%s(%p)::start Allocate resources failed\n", getName(), this);
		goto	Fail;
	}
	
    
    startPipes();                           // start reading on the usb pipes
    
    return true;
	
Fail:
    return false;
}

void me_nozap_driver_PL2303::stopSerial( bool resetDevice )
{
    
	DEBUG_IOLog(1,"%s(%p)::stopSerial\n", getName(), this);
    stopPipes();                            // stop reading on the usb pipes
    
    if (fpPipeOutMDP != NULL)               // better test for releaseResources?
    {
		releaseResources( );
    }
    
	
    
	DEBUG_IOLog(1,"%s(%p)::stopSerial stopSerial succeed\n", getName(), this);
    
Fail:
    return;
}

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::CheckSerialState
//
//      Inputs:     open session count (fSessions)
//                  usb start/stop (fStartStopUSB) -- replace with fTerminate?
//
//      Outputs:
//
//      Desc:       Turns Serial on or off if appropriate
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::CheckSerialState( void )
{
    Boolean     newState = fUSBStarted &    // usb must have started, and
    //			(fPowerState == kIrDAPowerOnState) &   // powered on by the power manager, and
	(fSessions > 0); // one of the clients too
	
    DEBUG_IOLog(4,"%s(%p)::CheckSerialState\n", getName(), this);
    if ( newState ){
		fTerminate = false;
		if ( !startSerial() )
		{
			fTerminate = true;
			IOLog("%s(%p)::CheckSerialState - StartSerial failed\n", getName(), this);
		} else {
			DEBUG_IOLog(5,"%s(%p)::CheckSerialState - StartSerial successful\n", getName(), this);
		}
		
	} else if (!newState && !fTerminate)      // Turn Serial off if needed
    {
		DEBUG_IOLog(5,"%s(%p)::CheckSerialState - StopSerial\n", getName(), this);
		fTerminate = true;              // Make it look like we've been terminated
		stopSerial(true);                     // stop irda and stop pipe i/o
		DEBUG_IOLog(5,"%s(%p)::CheckSerialState - StopSerial successful\n", getName(), this);
        
    }
	return kIOReturnSuccess;
	//    return ior;
}/* end CheckSerialState */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::configureDevice
//
//      Inputs:     numconfigs - number of configurations present
//
//      Outputs:    return Code - true (device configured), false (device not configured)
//
//      Desc:       Finds the configurations and then the appropriate interfaces etc.
//
/****************************************************************************************************/

bool me_nozap_driver_PL2303::configureDevice( UInt8 numConfigs )
{
    IOUSBFindInterfaceRequest           req;            // device request Class on stack
    const IOUSBConfigurationDescriptor  *cd = NULL;     // configuration descriptor
    IOUSBInterfaceDescriptor            *intf = NULL;   // interface descriptor
    IOReturn                            ior;
    UInt8                               cval;
    UInt8                               config = 0;
	
    DEBUG_IOLog(4,"%s(%p)::configureDevice\n", getName(), this);
	
    for (cval=0; cval<numConfigs; cval++)
    {
		cd = fpDevice->GetFullConfigurationDescriptor(cval);
		if ( !cd )
		{
			IOLog("%s(%p)::configureDevice - Error getting the full configuration descriptor\n", getName(), this);
		}
		
		else {
			
			// Find the first one - there may be more to go on in the future
			
			req.bInterfaceClass = kIOUSBFindInterfaceDontCare;
			req.bInterfaceSubClass  = kIOUSBFindInterfaceDontCare;
			req.bInterfaceProtocol  = kIOUSBFindInterfaceDontCare;
			req.bAlternateSetting   = kIOUSBFindInterfaceDontCare;
			
			ior = fpDevice->FindNextInterfaceDescriptor(cd, intf, &req, &intf);
			if ( ior == kIOReturnSuccess )
			{
				if ( intf ){
					config = cd->bConfigurationValue;
					DEBUG_IOLog(5,"%s(%p)::configureDevice - Interface descriptor found\n", getName(), this);
					break;
				} else {
					DEBUG_IOLog(5,"%s(%p)::configureDevice - That's weird the interface was null\n", getName(), this);
					cd = NULL;
				}
			} else {
				IOLog("%s(%p)::configureDevice - No CDC interface found this configuration\n", getName(), this);
				cd = NULL;
			}
		}
    }
	
    if ( !cd )
    {
		goto Fail;
    }
	
	// Now lets do it for real
	
    req.bInterfaceClass = kIOUSBFindInterfaceDontCare;
    req.bInterfaceSubClass  = kIOUSBFindInterfaceDontCare;
    req.bInterfaceProtocol  = kIOUSBFindInterfaceDontCare;
    req.bAlternateSetting   = kIOUSBFindInterfaceDontCare;
    
    fpInterface = fpDevice->FindNextInterface( NULL, &req );
    if ( !fpInterface )
	{
		DEBUG_IOLog(4,"%s(%p)::configureDevice - Find next interface failed open device and reallocate objects\n", getName(), this);
		if (!fpDevice->open(fpDevice))
        {
			IOLog("%s(%p)::configureDevice - unable to open device for configuration \n", getName(), this);
			goto Fail;
        }
		IOReturn rtn =  fpDevice->SetConfiguration(fpDevice, fpDevice->GetFullConfigurationDescriptor(0)->bConfigurationValue, true);
		if (rtn)
		{
			IOLog("%s(%p)::configureDevice - unable to set the configuration\n", getName(), this);
			goto Fail;
		}
		fpInterface = fpDevice->FindNextInterface( NULL, &req );
		if ( !fpInterface )
		{
			IOLog("%s(%p)::configureDevice - Find interface failed\n", getName(), this);
			goto Fail;
		} else {
			DEBUG_IOLog(5,"%s(%p)::configureDevice Interface found\n", getName(), this);
		}
	} else {
		DEBUG_IOLog(5,"%s(%p)::configureDevice Interface found\n", getName(), this);
	}
	
    fpInterface->retain();      // release done in stop()
    
    return true;
    
Fail:
    return false;
	
}/* end configureDevice */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::createNub
//
//      Inputs:
//
//      Outputs:    fNub  and fPort
//
//      Desc:       allocates and inits, but doesn't publish the BSD info on the nub yet
//              create serial stream finishes the job later.
//
/****************************************************************************************************/
bool me_nozap_driver_PL2303::createNub(void)
{
    DEBUG_IOLog(4,"%s(%p)::createNub\n", getName(), this);
    
	if (fNub == NULL) {
		fNub = new IORS232SerialStreamSync;
    }
	
    if( !fNub ) goto Fail;
	
    if (fPort == NULL) {
		fPort = (PortInfo_t*)IOMalloc( sizeof(PortInfo_t) );
    }
	
    if( !fPort ) goto Fail;
	
    bzero(fPort, sizeof(PortInfo_t));
	
    if( !fNub->init(0, fPort ) ) goto Fail;
	
    if( !fNub->attach( this ) ) goto Fail;
	
    return true;
	
Fail:
	IOLog("%s(%p)::Createnub failed\n", getName(), this);
    // could try and clean up here, but let's start by just not crashing.
    return false;
}

void me_nozap_driver_PL2303::destroyNub()
{
	DEBUG_IOLog(4,"%s(%p)::destroyNub Try to destroy nub\n", getName(), this);
    if (fPort != NULL) {
		IOFree( fPort, sizeof(PortInfo_t) );
		fPort = NULL;
		DEBUG_IOLog(5,"%s(%p)::destroyNub fPort reset \n", getName(), this);
		
    }
    
    if (fNub) {
		fNub->detach(this);
		fNub->release();    // crash boom?
		fNub = NULL;
		DEBUG_IOLog(5,"%s(%p)::destroyNub Nub destroyed \n", getName(), this);
    }
}

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::createSuffix
//
//      Inputs:     None
//
//      Outputs:    return Code - true (suffix created), false (suffix not create), sufKey - the key
//
//      Desc:       Creates the suffix key. It attempts to use the serial number string from the device
//                  if it's reasonable i.e. less than 8 bytes ascii. Remember it's stored in unicode
//                  format. If it's not present or not reasonable it will generate the suffix based
//                  on the location property tag. At least this remains the same across boots if the
//                  device is plugged into the same physical location. In the latter case trailing
//                  zeros are removed.
//
/****************************************************************************************************/

bool me_nozap_driver_PL2303::createSuffix( unsigned char *sufKey )
{
    
    IOReturn                rc;
    UInt8                   serBuf[10];     // arbitrary size > 8
    OSNumber                *location;
    UInt32                  locVal;
    UInt8                   *rlocVal;
    UInt16                  offs, i, sig = 0;
    UInt8                   indx;
    bool                    keyOK = false;
    DEBUG_IOLog(4,"%s(%p)::createSuffix\n", getName(), this);
	
    indx = fpDevice->GetSerialNumberStringIndex();
	DEBUG_IOLog(5,"%s(%p)::createSuffix the index of string descriptor describing the device's serial number: %p\n", getName(), this, indx );
	
    if (indx != 0 )
    {
		// Generate suffix key based on the serial number string (if reasonable <= 8 and > 0)
		
		rc = fpDevice->GetStringDescriptor(indx, (char *)&serBuf, sizeof(serBuf));
		if ( !rc )
		{
			DEBUG_IOLog(5,"%s(%p)::createSuffix serial number: %s\n", getName(), this, serBuf );
			
			size_t serBufLength = strlen((char *)&serBuf);
            
			if ( (serBufLength > 0) && (serBufLength < 9) )
			{
				strncpy( (char *)sufKey, (const char *)&serBuf, serBufLength);
				keyOK = true;
			}
            
		} else {
			IOLog("%s(%p)::createSuffix error reading serial number string\n", getName(), this );
		}
    }
    
    if ( !keyOK )
	{
		// Generate suffix key based on the location property tag
		
		location = (OSNumber *)fpDevice->getProperty(kUSBDevicePropertyLocationID);
		DEBUG_IOLog(5,"%s(%p)::createSuffix location number: %d\n", getName(), this, location );
		
		if ( location )
		{
			locVal = location->unsigned32BitValue();
			offs = 0;
			rlocVal = (UInt8*)&locVal;
			for (i=0; i<4; i++)
			{
				sufKey[offs] = Asciify(rlocVal[i] >> 4);
				if ( sufKey[offs++] != '0')
					sig = offs;
				sufKey[offs] = Asciify(rlocVal[i]);
				if ( sufKey[offs++] != '0')
					sig = offs;
			}
			sufKey[sig] = 0x00;
			keyOK = true;
		}
    }
    
	DEBUG_IOLog(4,"%s(%p)::createSuffix the suffix: %s\n", getName(), this, sufKey );
	
    return keyOK;
	
}/* end createSuffix */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::SetStructureDefaults
//
//      Inputs:     port - the port to set the defaults, Init - Probe time or not
//
//      Outputs:    None
//
//      Desc:       Sets the defaults for the specified port structure
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::SetStructureDefaults( PortInfo_t *port, bool Init )
{
    UInt32  tmp;
    
    DEBUG_IOLog(1,"%s(%p)::SetStructureDefaults\n", getName(), this);
	
	/* These are initialized when the port is created and shouldn't be reinitialized. */
    if ( Init )
	{
		DEBUG_IOLog(1,"%s(%p)::SetStructureDefaults INIT\n", getName(), this);
        
		port->FCRimage          = 0x00;
		port->IERmask           = 0x00;
		
		port->State             = ( PD_S_TXQ_EMPTY | PD_S_TXQ_LOW_WATER | PD_S_RXQ_EMPTY | PD_S_RXQ_LOW_WATER );
		port->WatchStateMask    = 0x00000000;
		port->lineState			= 0x00;
        //		port->serialRequestLock = 0;
    }
	
    port->BaudRate          = kDefaultBaudRate;         // 9600 bps
    port->CharLength        = 8;                        // 8 Data bits
    port->StopBits          = 2;                        // 1 Stop bit
    port->TX_Parity         = 1;                        // No Parity
    port->RX_Parity         = 1;                        // --ditto--
    port->MinLatency        = false;
    port->XONchar           = kXOnChar;
    port->XOFFchar          = kXOffChar;
    port->RXOstate          = kXO_Idle;
    port->TXOstate          = kXO_Idle;
    port->FrameTOEntry      = NULL;
	
    port->RXStats.BufferSize    = kMaxCirBufferSize;
    port->RXStats.HighWater     = (port->RXStats.BufferSize << 1) / 3;
    port->RXStats.LowWater      = port->RXStats.HighWater >> 1;
	
    port->TXStats.BufferSize    = kMaxCirBufferSize;
    port->TXStats.HighWater     = (port->RXStats.BufferSize << 1) / 3;
    port->TXStats.LowWater      = port->RXStats.HighWater >> 1;
    
    port->FlowControl           = (DEFAULT_AUTO | DEFAULT_NOTIFY);
    
    port->FlowControlState		= CONTINUE_SEND;
    port->DCDState				= false;
    port->BreakState			= false;
    
    port->xOffSent				= false;
    port->RTSAsserted			= true;
    port->DTRAsserted			= true;
    
    port->AreTransmitting		= FALSE;
	
    for ( tmp=0; tmp < (256 >> SPECIAL_SHIFT); tmp++ )
		port->SWspecial[ tmp ] = 0;
    
    DEBUG_IOLog(5,"%s(%p)::SetStructureDefaults finished\n", getName(), this);
    
    return;
    
}/* end SetStructureDefaults */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::createSerialStream
//
//      Inputs:     None
//
//      Outputs:    return Code - true (created and initialilzed ok), false (it failed)
//
//      Desc:       Creates and initializes the nub and port structure
//
/****************************************************************************************************/

bool me_nozap_driver_PL2303::createSerialStream()
{
    UInt8           indx;
    IOReturn            rc;
    unsigned char       rname[10];
    const char          *suffix = (const char *)&rname;
    DEBUG_IOLog(4,"%s(%p)::createSerialStream\n", getName(), this);
    
    if (!fNub || !fPort) return false;
	
    SetStructureDefaults( fPort, true );            // init the Port structure
    
    // Allocate the request lock
    fPort->serialRequestLock = IOLockAlloc();   // init lock used to protect code on MP
    if ( !fPort->serialRequestLock )
	{
		return false;
    }
    
    // now the ring buffers
    if (!allocateRingBuffer(&(fPort->TX), fPort->TXStats.BufferSize) ||
		!allocateRingBuffer(&(fPort->RX), fPort->RXStats.BufferSize))
	{
		DEBUG_IOLog(4,"%s(%p)::createSerialStream init ringbuffers  failed\n", getName(), this);
		return false;
	}
	
    if ( !fTerminate )
    {
		// Report the base name to be used for generating device nodes
		
		fNub->setProperty( kIOTTYBaseNameKey, baseName );
		
		// Create suffix key and set it
		
		if ( createSuffix( (unsigned char *)suffix ) )
		{
			fNub->setProperty( kIOTTYSuffixKey, suffix );
		}
		
		
		// Save the Product String  (at least the first productNameLength's worth).
		
		indx = fpDevice->GetProductStringIndex();
		if ( indx != 0 )
		{
			rc = fpDevice->GetStringDescriptor( indx, (char *)&fProductName, sizeof(fProductName) );
			if ( !rc )
			{
				DEBUG_IOLog(4,"%s(%p)::createSerialStream product name: %s\n", getName(), this, fProductName);
				if ( strlen((char *)fProductName) == 0 )        // believe it or not this sometimes happens (null string with an index defined???)
				{
					strncpy( (char *)fProductName, defaultName, (size_t) productNameLength);
				}
				fNub->setProperty( (const char *)propertyTag, (const char *)fProductName );
			}
		}
	    
		fNub->registerService();
    }
    
    return true;
    
}/* end createSerialStream */

//
// release things created in createSerialStream
//
void
me_nozap_driver_PL2303::destroySerialStream(void)
{
    DEBUG_IOLog(4,"%s(%p)::destroySerialStream\n", getName(), this);
	if( !fPort ) goto Fail;
    
	
    if ( fPort->serialRequestLock )
	{
		IOLockFree( fPort->serialRequestLock ); // free the Serial Request Lock
		fPort->serialRequestLock = NULL;
	}
	
    // Remove all the buffers.
	
    freeRingBuffer( &fPort->TX );
    freeRingBuffer( &fPort->RX );
	
    removeProperty( (const char *)propertyTag );    // unhook from BSD
	DEBUG_IOLog(5,"%s(%p)::destroySerialStream serial stream destroyed \n", getName(), this);
	
Fail:
    return;
}




//
// start reading on the pipes
//
bool me_nozap_driver_PL2303::startPipes( void )
{
    IOReturn                    rtn;
    DEBUG_IOLog(4,"%s(%p)::startPipes\n", getName(), this);
    
    if(!fPort) goto Fail;
    if(!fpPipeInMDP) goto Fail;
    if(!fpPipeOutMDP) goto Fail;
    
	// Read the data-in bulk pipe
	rtn = fpInPipe->Read(fpPipeInMDP, &fReadCompletionInfo, NULL );
    
    if( !(rtn == kIOReturnSuccess) ) goto Fail;
    
	// Read the data-in interrupt pipe
    if(!fPort) goto Fail;
    
	if(!fpinterruptPipeMDP) goto Fail;
	rtn = fpInterruptPipe->Read(fpinterruptPipeMDP, &finterruptCompletionInfo, NULL );
    if( !(rtn == kIOReturnSuccess) ) goto Fail;
	
    
    // is this really referenced by anyone??
    fReadActive = true;     // remember if we did a read
    DEBUG_IOLog(5,"%s(%p)::startPipes pipes started\n", getName(), this);
    return true;
    
Fail:
    IOLog("%s(%p)::startPipes Failed\n", getName(), this);
    
	return false;
}/* end startPipes */

//
// stop i/o on the pipes
//
void me_nozap_driver_PL2303::stopPipes()
{
	DEBUG_IOLog(4,"%s(%p)::Stopping\n", getName(), this);
    if (fpInterruptPipe){
		fpInterruptPipe->Abort();}
    DEBUG_IOLog(5,"%s(%p)::stopPipes fpInterruptPipe succeed\n", getName(), this);
	
    DEBUG_IOLog(5,"%s(%p)::stopPipes fpInPipe %p\n", getName(), this, fpInPipe);
    if (fpInPipe){
	    fpInPipe->Abort();}
	DEBUG_IOLog(5,"%s(%p)::stopPipes fpOutPipe %p\n", getName(), this, fpOutPipe);
	
    if (fpOutPipe){
		fpOutPipe->Abort();}
	DEBUG_IOLog(5,"%s(%p)::stopPipes succeed\n", getName(), this);
    
    
    
    
    
}


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::message
//
//      Inputs:     type - message type, provider - my provider, argument - additional parameters
//
//      Outputs:    return Code - kIOReturnSuccess
//
//      Desc:       Handles IOKit messages.
//
/****************************************************************************************************/
enum {                                  // messageType for the callback routines
    kIrDACallBack_Status    = 0x1000,   // Status Information is coming
    kIrDACallBack_Unplug    = 0x1001    // USB Device is unplugged
};

IOReturn me_nozap_driver_PL2303::message( UInt32 type, IOService *provider,  void *argument)
{
    IOReturn err = kIOReturnSuccess;
    DEBUG_IOLog(4,"%s(%p)::message %p\n", getName(), this, type);
    
	switch ( type )
    {
		case kIOMessageServiceIsTerminated:
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsTerminated sessions: %p\n", getName(), this,fSessions);
			
			if ( fSessions ){
				stopSerial( false );         // stop serial now
                
				DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsTerminated fSessions\n", getName(), this);
                
				if ( (fPort != NULL) && (fPort->serialRequestLock != 0) ){
				    DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsTerminated changeState\n", getName(), this);
					changeState( fPort, 0, (UInt32)PD_S_ACTIVE );
				}
				DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsTerminated send KUNCUserNotificationDisplayNotice\n", getName(), this);
                
			} else {
				stopSerial( false);         // stop serial now
                
				if ( fpInterface ) {
					fpInterface->close( this );
					fpInterface->release();
					fpInterface = NULL;
				}
			}
			
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsTerminated terminated\n", getName(), this);
            
			fTerminate = true;      // we're being terminated (unplugged)
			/* We need to disconnect the user client interface */
			break;
			
		case kIOMessageServiceIsSuspended:
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsSuspended\n", getName(), this);
			break;
			
		case kIOMessageServiceIsResumed:
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsResumed\n", getName(), this);
			break;
			
		case kIOMessageServiceIsRequestingClose:
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceIsRequestingClose\n", getName(), this);
			break;
			
		case kIOMessageServiceWasClosed:
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceWasClosed\n", getName(), this);
			break;
			
		case kIOMessageServiceBusyStateChange:
			DEBUG_IOLog(4,"%s(%p)::message - kIOMessageServiceBusyStateChange\n", getName(), this);
			break;
			
		case kIOMessageServiceIsAttemptingOpen:
			DEBUG_IOLog(4,"%s(%p)::received kIOMessageServiceIsAttemptingOpen with argument: %p \n", getName(), this, (int) argument );
			
			break;
			
		case kIOUSBMessagePortHasBeenResumed:
			DEBUG_IOLog(4,"%s(%p)::message - kIOUSBMessagePortHasBeenResumed\n", getName(), this);
			
			if ( !fTerminate )
			{
				DEBUG_IOLog(4,"4,%s(%p)::message - port already started \n", getName(), this);
            }
            else {                  // we're trying to resume, so start serial
				if ( !startSerial() )
				{
					fTerminate = true;
					DEBUG_IOLog(4,"%s(%p)::message - startSerial failed\n", getName(), this);
				}
				else {
					DEBUG_IOLog(4,"%s(%p)::message - startSerial successful\n", getName(), this);
				}
			}
			break;
			
		case kIOUSBMessageHubResumePort:
			DEBUG_IOLog(4,"%s(%p)::message - kIOUSBMessageHubResumePort\n", getName(), this);
			if ( !fTerminate )
			{
				DEBUG_IOLog(4,"%s(%p)::message - port already started \n", getName(), this);
            }
            else {                  // we're trying to resume, so start serial
				if ( !startSerial() )
				{
					fTerminate = true;
					DEBUG_IOLog(4,"%s(%p)::message - startSerial failed\n", getName(), this);
				}
				else {
					DEBUG_IOLog(4,"%s(%p)::message - startSerial successful\n", getName(), this);
				}
			}
			break;
            
		case kIOUSBMessagePortHasBeenReset:
			DEBUG_IOLog(1,"%s(%p)::message - kIOUSBMessagePortHasBeenReset\n", getName(), this);
            
            
			if (fpDevice->GetNumConfigurations() < 1)
            {
				DEBUG_IOLog(1,"%s(%p)::message - no composite configurations\n", getName(), this);
				err = kIOUSBConfigNotFound;
				goto Fail;
            }
            
			// Now configure it (leaves device suspended)
			if( !configureDevice( fpDevice->GetNumConfigurations() ) )
            {
				err = kIOUSBConfigNotFound;
				goto Fail;
            }
            
			fUSBStarted = true;
            
			DEBUG_IOLog(1,"%s(%p)::message - Port reconfigurated\n", getName(), this);
            
        Fail:
			return err;
			break;
            
		default:
			DEBUG_IOLog(4,"%s(%p)::message - unknown message %p \n", getName(), this, type );
			break;
    }
    
    return err;
}


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::readPortState
//
//      Inputs:     port - the specified port
//
//      Outputs:    returnState - current state of the port
//
//      Desc:       Reads the current Port->State.
//
/****************************************************************************************************/

UInt32 me_nozap_driver_PL2303::readPortState( PortInfo_t *port )
{
    UInt32              returnState;
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::readPortState IOLockLock( port->serialRequestLock );\n" );
    
    IOLockLock( port->serialRequestLock );
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::readPortState port->State\n", returnState );
    
	returnState = port->State;
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::readPortState IOLockUnLock( port->serialRequestLock );\n" );
    
	IOLockUnlock( port->serialRequestLock);
	
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::readPortState returnstate: %p \n", returnState );
	
    return returnState;
    
}/* end readPortState */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::changeState
//
//      Inputs:     port - the specified port, state - new state, mask - state mask (the specific bits)
//
//      Outputs:    None
//
//      Desc:       Change the current Port->State to state using the mask bits.
//                  if mask = 0 nothing is changed.
//                  delta contains the difference between the new and old state taking the
//                  mask into account and it's used to wake any waiting threads as appropriate.
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::changeState( PortInfo_t *port, UInt32 state, UInt32 mask )
{
    UInt32              delta;
    DEBUG_IOLog(6,"%s(%p)::changeState\n", getName(), this);
	
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::changeState IOLockLock( port->serialRequestLock );\n" );
    
	IOLockLock( port->serialRequestLock );
	
    
	DEBUG_IOLog(6,"state before: %p mask %p \n",state,mask);
    
    state = (port->State & ~mask) | (state & mask); // compute the new state
	DEBUG_IOLog(6,"state after: %p \n",state);
    
    delta = state ^ port->State;                    // keep a copy of the diffs
	DEBUG_IOLog(6,"state port: %p delta %p \n",port->State, delta);
    
    port->State = state;
    
    
	// Wake up all threads asleep on WatchStateMask
	
    if ( delta & port->WatchStateMask )
	{
		fCommandGate->commandWakeup((void *)&fPort->State);
	}
    
    
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::changeState IOLockUnLock( port->serialRequestLock );\n" );
    
    IOLockUnlock( port->serialRequestLock );
    
	// if any modem control signals changed, we need to do an setControlLines()
	
	if ((mask & PD_RS232_S_DTR) && ((port->FlowControl & PD_RS232_A_DTR) != PD_RS232_A_DTR))
    {
        if ((state & PD_RS232_S_DTR) != (fPort->State & PD_RS232_S_DTR))
        {
            if (state & PD_RS232_S_DTR)
            {
                port->State |= PD_RS232_S_DTR;
                setControlLines( port );
                
            } else {
                port->State &= ~PD_RS232_S_DTR;
                setControlLines( port );
                
            }
        }
    }
    
	if (delta & ( PD_RS232_S_DTR | PD_RS232_S_RFR )){
		DEBUG_IOLog(5,"setControlLines aanroepen\n");
        setControlLines( port );
    }
    DEBUG_IOLog(6,"%s(%p)::changeState delta: %p Port->State: %p\n", getName(), this, delta, port->State);
	
    return;
    
}/* end changeState */


/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::acquirePort
//
//		Inputs:		sleep - true (wait for it), false (don't)
//				refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		Set up for gated acquirePort call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::acquirePort(bool sleep, void *refCon)
{
    IOReturn	ret;
    DEBUG_IOLog(4,"%s(%p)::acquirePort\n", getName(), this);
    
    retain();
    ret = fCommandGate->runAction(acquirePortAction, (void *)sleep, (void *)refCon);
    release();
    
    return ret;
	
}/* end acquirePort */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::acquirePortAction
//
//		Desc:		Dummy pass through for acquirePortGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::acquirePortAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{
    DEBUG_IOLog(4,"me_nozap_driver_PL2303::acquirePortAction\n");
    
    return ((me_nozap_driver_PL2303 *)owner)->acquirePortGated((bool)arg0, (void *)arg1);
    
}/* end acquirePortAction */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::acquirePortGated
//
//		Inputs:		sleep - true (wait for it), false (don't), refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		acquirePort tests and sets the state of the port object.  If the port was
//					available, then the state is set to busy, and kIOReturnSuccess is returned.
//					If the port was already busy and sleep is YES, then the thread will sleep
//					until the port is freed, then re-attempts the acquire.  If the port was
//					already busy and sleep is NO, then kIOReturnExclusiveAccess is returned.
//
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::acquirePortGated( bool sleep, void *refCon )
{
    PortInfo_t          *port = (PortInfo_t *) refCon;
    UInt32              busyState = 0;
    IOReturn            rtn = kIOReturnSuccess;
	
    DEBUG_IOLog(4,"%s(%p)::acquirePortGated\n", getName(), this);
    
    if ( fTerminate ) {
		DEBUG_IOLog(4,"%s(%p)::acquirePortGated Port is offline\n", getName(), this);
		
		//	    return kIOReturnOffline;
	}
    SetStructureDefaults( port, FALSE );    /* Initialize all the structures */
    
    for (;;)
	{
        DEBUG_IOLog(5,"%s(%p)::acquirePortGated readportstate\n", getName(), this);
        
		busyState = readPortState( port ) & PD_S_ACQUIRED;
		if ( !busyState )
		{
			// Set busy bit, and clear everything else
			changeState( port, (UInt32)PD_S_ACQUIRED | DEFAULT_STATE, (UInt32)STATE_ALL);
			break;
		} else {
			if ( !sleep )
			{
				IOLog("%s(%p)::acquirePortGated - Busy exclusive access\n", getName(), this);
				return kIOReturnExclusiveAccess;
			} else {
				busyState = 0;
				rtn = watchState( &busyState, PD_S_ACQUIRED, refCon );
				if ( (rtn == kIOReturnIOError) || (rtn == kIOReturnSuccess) )
				{
					continue;
				} else {
					IOLog("%s(%p)::acquirePortGated - Interrupted!\n", getName(), this);
					return rtn;
				}
			}
		}
	} /* end for */
    
    fSessions++;    //bump number of active sessions and turn on clear to send
    //   DEBUG_IOLog(5,"%s(%p)::acquirePortGated change state\n", getName(), this);
    
    //    changeState( port, PD_RS232_S_CTS, PD_RS232_S_CTS);
    
    DEBUG_IOLog(5,"%s(%p)::acquirePortGated check serial state\n", getName(), this);
    
	CheckSerialState();       // turn serial on/off if appropriate
    
    return rtn;
    
}/* end acquirePort */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::releasePort
//
//		Inputs:		refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		Set up for gated acquirePort call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::releasePort(void *refCon)
{
    IOReturn	ret;
    DEBUG_IOLog(4,"%s(%p)::releasePort\n", getName(), this);
    
    retain();
    ret = fCommandGate->runAction(releasePortAction, (void *)refCon);
    release();
    
    return ret;
    
}/* end releasePort */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::releasePortAction
//
//		Desc:		Dummy pass through for releasePortGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::releasePortAction(OSObject *owner, void *arg0, void *, void *, void *)
{
    DEBUG_IOLog(4,"me_nozap_driver_PL2303::releasePortAction\n");
    
    return ((me_nozap_driver_PL2303 *)owner)->releasePortGated((void *) arg0);
}/* end releasePortAction */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::releasePortGated
//
//		Inputs:		refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		releasePort returns all the resources and does clean up.
//
//
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::releasePortGated( void *refCon )
{
    PortInfo_t          *port = (PortInfo_t *) refCon;
    UInt32              busyState;
    DEBUG_IOLog(4,"%s(%p)::releasePortGated\n", getName(), this);
	
    
    busyState = (readPortState( port ) & PD_S_ACQUIRED);
    if ( !busyState )
	{
		IOLog("%s(%p)::releasePortGated - port not open\n", getName(), this);
		return kIOReturnNotOpen;
	}
    
    changeState( port, 0, (UInt32)STATE_ALL );  // Clear the entire state word which also deactivates the port
	
    fSessions--;        // reduce number of active sessions
    CheckSerialState();   // turn serial off if appropriate
	
    if ((fTerminate) && (fSessions == 0))       // if it's the result of a terminate and session count is zero we also need to close things
	{
		if (0 && fpInterface )      // jdg - this was bogus
		{
			fpInterface->close( this );
			fpInterface->release();
			fpInterface = NULL;
		}
        else DEBUG_IOLog(5,"%s(%p)::releasePortGated - would have released fpInteface here\n", getName(), this);
    }
    
    return kIOReturnSuccess;
    
}/* end releasePort */


/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::setState
//
//		Inputs:		state - state to set
//					mask - state mask
//					refCon - the Port (not used)
//
//		Outputs:	Return Code - See setStateGated
//
//		Desc:		Set up for gated setState call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::setState(UInt32 state, UInt32 mask, void *refCon)
{
    PortInfo_t *port = (PortInfo_t *) refCon;
    IOReturn	ret;
    DEBUG_IOLog(4,"%s(%p)::setState state %p mask %p\n", getName(), this, mask, state);
    
	// Cannot acquire or activate via setState
    
    if (mask & (PD_S_ACQUIRED | PD_S_ACTIVE | (~EXTERNAL_MASK)))
    {
        return kIOReturnBadArgument;
    }
	
	// ignore any bits that are read-only
	
	mask &= (~port->FlowControl & PD_RS232_A_MASK) | PD_S_MASK ;
	
	// always store handshakeline state
	mask |=  kHandshakeInMask;
	if (port->lineState & kCTS) state |= PD_RS232_S_CTS; else state &= ~( PD_RS232_S_CTS );
	if (port->lineState & kDSR) state |= PD_RS232_S_DSR; else state &= ~( PD_RS232_S_DSR );
	if (port->lineState & kRI)  state |= PD_RS232_S_RI; else state &= ~( PD_RS232_S_RI );
	if (port->lineState & kDCD) state |= PD_RS232_S_CAR; else state &= ~( PD_RS232_S_CAR );
    DEBUG_IOLog(5,"%s(%p)::setState linestatestate %p mask %p state %p\n", getName(), this, port->lineState, mask, state);
    
	if (mask)
	{
		retain();
		ret = fCommandGate->runAction(setStateAction, (void *)state, (void *) mask, (void *)refCon);
		release();
		return ret;
	}
	
	
    return kIOReturnSuccess;
    
}/* end setState */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::setStateAction
//
//		Desc:		Dummy pass through for setStateGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::setStateAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *)
{
    DEBUG_IOLog(4,"me_nozap_driver_PL2303::setStateAction\n");
    
#if defined(__x86_64__)
    return ((me_nozap_driver_PL2303 *)owner)->setStateGated((UInt64)arg0, (UInt64)arg1, (void *)arg2);
#else
    return ((me_nozap_driver_PL2303 *)owner)->setStateGated((UInt32)arg0, (UInt32)arg1, (void *)arg2);
#endif
}/* end setStateAction */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::setState
//
//      Inputs:     state - state to set, mask - state mask, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess or kIOReturnBadArgument
//
//      Desc:       Set the state for the port device.  The lower 16 bits are used to set the
//                  state of various flow control bits (this can also be done by enqueueing a
//                  PD_E_FLOW_CONTROL event).  If any of the flow control bits have been set
//                  for automatic control, then they can't be changed by setState.  For flow
//                  control bits set to manual (that are implemented in hardware), the lines
//                  will be changed before this method returns.  The one weird case is if RXO
//                  is set for manual, then an XON or XOFF character may be placed at the end
//                  of the TXQ and transmitted later.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::setStateGated( UInt32 state, UInt32 mask, void *refCon )
{
    PortInfo_t *port = (PortInfo_t *) refCon;
    DEBUG_IOLog(4,"%s(%p)::setStateGated\n", getName(), this);
    
    if ( mask & (PD_S_ACQUIRED | PD_S_ACTIVE | (~EXTERNAL_MASK)) )
		return kIOReturnBadArgument;
	
    if ( readPortState( port ) & PD_S_ACQUIRED )
	{
	    // ignore any bits that are read-only
		mask &= (~port->FlowControl & PD_RS232_A_MASK) | PD_S_MASK;
		DEBUG_IOLog(5,"%s(%p)::setStateGated mask: %p state %p ", getName(), this,mask, state);
		
		if ( mask)
			changeState( port, state, mask );
		
		return kIOReturnSuccess;
	}
	
	DEBUG_IOLog(4,"%s(%p)::setStateGated port not open \n", getName(), this);
    return kIOReturnNotOpen;
    
}/* end setState */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::watchState
//
//		Inputs:		state - state to watch for
//				mask - state mask bits
//				refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from ::watchState
//
//		Desc:		Set up for gated watchState call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::watchState(UInt32 *state, UInt32 mask, void *refCon)
{
    IOReturn 	ret;
    DEBUG_IOLog(4,"%s(%p)::watchState state %p mask  %p\n", getName(), this, *state, mask);
    
    if (!state)
        return kIOReturnBadArgument;
	
    if (!mask)
        return kIOReturnSuccess;
	
    retain();
    ret = fCommandGate->runAction(watchStateAction, (void *)state, (void *)mask);
    release();
    return ret;
	
}/* end watchState */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::watchStateAction
//
//		Desc:		Dummy pass through for watchStateGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::watchStateAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{
    DEBUG_IOLog(4,"me_nozap_driver_PL2303::watchStateAction\n");
    
#if defined(__x86_64__)
    return ((me_nozap_driver_PL2303 *)owner)->watchStateGated((UInt32 *)arg0, (UInt64)arg1);
#else
    return ((me_nozap_driver_PL2303 *)owner)->watchStateGated((UInt32 *)arg0, (UInt32)arg1);
#endif
}/* end watchStateAction */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::watchState
//
//      Inputs:     state - state to watch for, mask - state mask bits, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess or value returned from ::watchState
//
//      Desc:       Wait for the at least one of the state bits defined in mask to be equal
//                  to the value defined in state. Check on entry then sleep until necessary,
//                  see watchState for more details.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::watchStateGated( UInt32 *state, UInt32 mask)
{
    IOReturn    ret = kIOReturnNotOpen;
    DEBUG_IOLog(4,"%s(%p)::watchStateGated state: %p mask: %p\n", getName(), this, *state, mask);
	
	
    if ( readPortState( fPort ) & PD_S_ACQUIRED )
	{
		ret = kIOReturnSuccess;
		mask &= EXTERNAL_MASK;
		ret = privateWatchState( fPort, state, mask );
		*state &= EXTERNAL_MASK;
	}
    
    return ret;
    
}/* end watchState */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::nextEvent
//
//      Inputs:     refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess
//
//      Desc:       Returns either EOQ (no events), or queue state, depending on its contents.
//
/****************************************************************************************************/

UInt32 me_nozap_driver_PL2303::nextEvent( void *refCon )
{
    DEBUG_IOLog(4,"%s(%p)::nextEvent\n", getName(), this);
    
#if FIX_PARITY_PROCESSING
    UInt8 t = 0;
    UInt32 qret = peekBytefromQueue( &fPort->RX, &t, 1);
    if(qret != kQueueEmpty) {
        if(t == 0xff) {
            qret = peekBytefromQueue( &fPort->RX, &t, 2);
            if(qret != kQueueEmpty && t == 0x00) {
                DEBUG_IOLog(5,"%s(%p)::nextEvent PD_E_INTEGRITY_ERROR\n", getName(), this);
                return PD_E_INTEGRITY_ERROR;
            }
        }
    }
    
    if(getQueueStatus(&fPort->RX) != kQueueEmpty) {
        DEBUG_IOLog(5,"%s(%p)::nextEvent PD_E_VALID_DATA\n", getName(), this);
        return PD_E_VALID_DATA;
    }
#endif
    
    DEBUG_IOLog(5,"%s(%p)::nextEvent PD_E_EOQ\n", getName(), this);
    return PD_E_EOQ;
    
}/* end nextEvent */


/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::executeEvent
//
//		Inputs:		event - The event
//				data - any data associated with the event
//				refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//		Desc:		Set up for gated executeEvent call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::executeEvent(UInt32 event, UInt32 data, void *refCon)
{
    IOReturn 	ret;
	DEBUG_IOLog(4,"%s(%p)::executeEventAction\n", getName(), this);
    
    retain();
    ret = fCommandGate->runAction(executeEventAction, (void *)event, (void *)data, (void *)refCon);
    release();
	
    return ret;
    
}/* end executeEvent */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::executeEventAction
//
//		Desc:		Dummy pass through for executeEventGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::executeEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *)
{
	DEBUG_IOLog(4,"me_nozap_driver_PL2303::executeEventAction\n");
    
#if defined(__x86_64__)
	return ((me_nozap_driver_PL2303 *)owner)->executeEventGated((UInt64)arg0, (UInt64)arg1, (void *)arg2);
#else
    return ((me_nozap_driver_PL2303 *)owner)->executeEventGated((UInt32)arg0, (UInt32)arg1, (void *)arg2);
#endif
}/* end executeEventAction */


/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::executeEventGated
//
//
//      Inputs:     event - The event, data - any data associated with the event, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//      Desc:       executeEvent causes the specified event to be processed immediately.
//                  This is primarily used for channel control commands like START & STOP
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::executeEventGated( UInt32 event, UInt32 data, void *refCon )
{
    PortInfo_t  *port = (PortInfo_t *) refCon;
    IOReturn    ret = kIOReturnSuccess;
    UInt32      state, delta, old;
    int rtn;
	DEBUG_IOLog(4,"%s(%p)::executeEventGated\n", getName(), this);
    
    delta = 0;
    state = readPortState( port );
    
    
    if ( (state & PD_S_ACQUIRED) == 0 )
		return kIOReturnNotOpen;
	
    switch ( event )
	{
		case PD_RS232_E_XON_BYTE:
			port->XONchar = data;
			break;
		case PD_RS232_E_XOFF_BYTE:
			port->XOFFchar = data;
			break;
		case PD_E_SPECIAL_BYTE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_SPECIAL_BYTE\n", getName(), this );
			port->SWspecial[ data >> SPECIAL_SHIFT ] |= (1 << (data & SPECIAL_MASK));
			break;
			
		case PD_E_VALID_DATA_BYTE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_VALID_DATA_BYTE\n", getName(), this );
			port->SWspecial[ data >> SPECIAL_SHIFT ] &= ~(1 << (data & SPECIAL_MASK));
			break;
			
		case PD_E_FLOW_CONTROL:
			
			old = port->FlowControl;				    // save old modes for unblock checks
            port->FlowControl = data & (CAN_BE_AUTO | CAN_NOTIFY);  // new values, trimmed to legal values
			DEBUG_IOLog(1,"%s(%p)::executeEvent - PD_E_FLOW_CONTROL port->FlowControl %p\n", getName(), this, port->FlowControl );
            
			// now cleanup if we've blocked RX or TX with the previous style flow control and we're switching to a different kind
			// we have 5 different flow control modes to check and unblock; 3 on rx, 2 on tx
            
			
			if ( !(old & PD_RS232_S_CTS) && (PD_RS232_S_CTS & port->FlowControl) )
            {
				DEBUG_IOLog(1,"%s(%p)::executeEvent - Automatic CTS flowcontrol On\n", getName(), this);
                IOUSBDevRequest request;
                
                if (fPort->type == rev_HX ) {
                    request.wIndex = 0x61;
                } else {
                    request.wIndex = 0x41;
                    
                }
                request.bmRequestType = VENDOR_WRITE_REQUEST_TYPE;
                request.bRequest = VENDOR_WRITE_REQUEST;
                request.wValue =  0;
                request.wLength = 0;
                request.pData = NULL;
                rtn = fpDevice->DeviceRequest(&request);
                DEBUG_IOLog(1,"%s(%p)::executeEvent - executeEvent - device request: %p \n", getName(), this,  rtn);
				
                port->FlowControlState = CONTINUE_SEND;
            }
            
			if ( (old & PD_RS232_S_CTS) && !(PD_RS232_S_CTS & port->FlowControl) )
            {
                DEBUG_IOLog(1,"%s(%p)::executeEvent - Automatic CTS flowcontrol Off\n", getName(), this);
                IOUSBDevRequest request;
                
                request.wIndex = 0x00;
                request.bmRequestType = VENDOR_WRITE_REQUEST_TYPE;
                request.bRequest = VENDOR_WRITE_REQUEST;
                request.wValue =  0;
                request.wLength = 0;
                request.pData = NULL;
                rtn = fpDevice->DeviceRequest(&request);
                DEBUG_IOLog(1,"%s(%p)::executeEvent - device request: %p \n", getName(), this,  rtn);
				
                port->FlowControlState = CONTINUE_SEND;
            }
            
			if (!fTerminate && old && (old ^ port->FlowControl))		// if had some modes, and some modes are different
			{
                DEBUG_IOLog(1,"%s(%p)::executeEvent - We zijn in de IF  %p \n", getName(), this , PD_RS232_S_CTS);
                
                
                
#define SwitchingAwayFrom(flag) ((old & flag) && !(port->FlowControl & flag))
#define SwitchingTo(flag) (!(old & flag) && (port->FlowControl & flag))
				
				// if switching away from rx xon/xoff and we've sent an xoff, unblock
				if (SwitchingAwayFrom(PD_RS232_A_RXO) && port->xOffSent)
				{
					DEBUG_IOLog(1,"%s(%p)::executeEvent - PD_E_FLOW_CONTROL send xoff\n", getName(), this, port->FlowControl );
					addBytetoQueue(&(port->TX), port->XONchar);
					port->xOffSent = false;
					setUpTransmit( );
				}
				
				// if switching away from RTS flow control and we've lowered RTS, need to raise it to unblock
				if (SwitchingAwayFrom(PD_RS232_A_RTS) && !port->RTSAsserted)
				{
					DEBUG_IOLog(1,"%s(%p)::executeEvent - PD_E_FLOW_CONTROL set RTS\n", getName(), this, port->FlowControl );
					port->RTSAsserted = true;
					port->State |= PD_RS232_S_RFR;		    // raise RTS again
				}
				
				// if switching away from DTR flow control and we've lowered DTR, need to raise it to unblock
				if (SwitchingAwayFrom(PD_RS232_A_DTR) && !port->DTRAsserted)
				{
					DEBUG_IOLog(1,"%s(%p)::executeEvent - PD_E_FLOW_CONTROL set DTR\n", getName(), this, port->FlowControl );
					port->DTRAsserted = true;
					port->State |= PD_RS232_S_DTR;		    // raise DTR again
				}
                
                
				
				// If switching away from TX xon/xoff and we've paused tx, continue it
				if (SwitchingAwayFrom(PD_RS232_S_TXO) && port->RXOstate == kXOnNeeded)
				{
					port->RXOstate = kXOffNeeded;
					port->FlowControlState = CONTINUE_SEND;
				}
				changeState( port, (UInt32)PD_S_ACTIVE, (UInt32)PD_S_ACTIVE );
                
                DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_FLOW_CONTROL end port->FlowControl %p\n", getName(), this, port->FlowControl );
                
			}
            
			break;
			
		case PD_E_ACTIVE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_ACTIVE\n", getName(), this );
			if ( (bool)data )
			{
				if ( !(state & PD_S_ACTIVE) )
				{
					SetStructureDefaults( port, FALSE );
					changeState( port, (UInt32)PD_S_ACTIVE, (UInt32)PD_S_ACTIVE ); // activate port
                    //					changeState( port, generateRxQState( port ), PD_S_TXQ_MASK | PD_S_RXQ_MASK | kRxAutoFlow);
				}
			} else {
				if ( (state & PD_S_ACTIVE) )
				{
					changeState( port, 0, (UInt32)PD_S_ACTIVE );
				}
			}
			if( setSerialConfiguration() ){
				DEBUG_IOLog(4,"%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
			}
			
            /*		if ( (bool)data )
             {
             if ( !(state & PD_S_ACTIVE) )
             {
             SetStructureDefaults( port, FALSE );
             changeState( port, (UInt32)PD_S_ACTIVE, (UInt32)PD_S_ACTIVE ); // activate port
             
             USBSetControlLineState(true, true);			// set RTS and set DTR
             }
             } else {
             if ( (state & PD_S_ACTIVE) )
             {
             changeState( port, 0, (UInt32)PD_S_ACTIVE );
             
             USBSetControlLineState(false, false);			// clear RTS and clear DTR
             }
             }*/
			break;
			
		case PD_E_DATA_LATENCY:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_DATA_LATENCY\n", getName(), this );
			port->DataLatInterval = long2tval( data * 1000 );
			break;
			
		case PD_RS232_E_MIN_LATENCY:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_RS232_E_MIN_LATENCY \n", getName(), this );
			port->MinLatency = bool( data );
			break;
			
		case PD_E_DATA_INTEGRITY:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_DATA_INTEGRITY\n", getName(), this );
			if ( (data < PD_RS232_PARITY_NONE) || (data > PD_RS232_PARITY_SPACE))
			{
				ret = kIOReturnBadArgument;
			}
			else
			{
				port->TX_Parity = data;
				port->RX_Parity = PD_RS232_PARITY_DEFAULT;
			}
			if( setSerialConfiguration() ){
				DEBUG_IOLog(4,"%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
			}
			break;
			
		case PD_E_DATA_RATE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_DATA_RATE \n", getName(), this );
			/* For API compatiblilty with Intel.    */
			data >>= 1;
			DEBUG_IOLog(4,"%s(%p)::executeEvent - actual data rate baudrate: %d \n", getName(), this, data );
			if ( (data < kMinBaudRate) || (data > kMaxBaudRate) )       // Do we really care
				ret = kIOReturnBadArgument;
			else
			{
				port->BaudRate = data;
			}
            if( setSerialConfiguration() ){
                DEBUG_IOLog(4,"%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
            }
            break;
			
		case PD_E_DATA_SIZE:
			/* For API compatiblilty with Intel.    */
			data >>= 1;
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_DATA_SIZE: %d \n", getName(), this, data );
			
			if ( (data < 5) || (data > 8) )
				ret = kIOReturnBadArgument;
			else
			{
				
				port->CharLength = data;
			}
            if( setSerialConfiguration() ){
                DEBUG_IOLog(4,"%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
            }
            break;
			
		case PD_RS232_E_STOP_BITS:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_RS232_E_STOP_BITS\n", getName(), this );
			if ( (data < 0) || (data > 20) )
				ret = kIOReturnBadArgument;
			else
			{
				port->StopBits = data;
			}
            if( setSerialConfiguration() ){
                DEBUG_IOLog(4,"%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
            }
            break;
			
		case PD_E_RXQ_FLUSH:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RXQ_FLUSH \n", getName(), this );
		    flush( &port->RX );
            //            state = maskMux(state, generateRxQState( port ), (PD_S_RXQ_MASK | kRxAutoFlow));
            //            delta |= PD_S_RXQ_MASK | kRxAutoFlow;
			break;
			
		case PD_E_RX_DATA_INTEGRITY:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RX_DATA_INTEGRITY\n", getName(), this );
			if ( (data != PD_RS232_PARITY_DEFAULT) &&  (data != PD_RS232_PARITY_ANY) )
				ret = kIOReturnBadArgument;
			else
				port->RX_Parity = data;
			break;
			
		case PD_E_RX_DATA_RATE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RX_DATA_RATE\n", getName(), this );
			if ( data )
				ret = kIOReturnBadArgument;
			break;
			
		case PD_E_RX_DATA_SIZE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RX_DATA_SIZE\n", getName(), this );
			if ( data )
				ret = kIOReturnBadArgument;
			break;
			
		case PD_RS232_E_RX_STOP_BITS:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_RS232_E_RX_STOP_BITS \n", getName(), this );
			if ( data )
				ret = kIOReturnBadArgument;
			break;
			
		case PD_E_TXQ_FLUSH:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_TXQ_FLUSH\n", getName(), this );
			break;
			
		case PD_RS232_E_LINE_BREAK:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_RS232_E_LINE_BREAK\n", getName(), this );
            state &= ~PD_RS232_S_BRK;
            delta |= PD_RS232_S_BRK;
            if (data)
            {
                port->BreakState = true;
            } else {
                port->BreakState = false;
            }
            setBreak(data);
            setStateGated(state, delta, port);
			break;
			
		case PD_E_DELAY:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_DELAY\n", getName(), this );
            if (port->BreakState)					// It's the break delay in micro seconds
            {
                IOSleep(data/1000);
            } else {
                port->CharLatInterval = long2tval(data * 1000);
            }
			break;
			
		case PD_E_RXQ_SIZE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RXQ_SIZE\n", getName(), this );
			break;
			
		case PD_E_TXQ_SIZE:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_TXQ_SIZE\n", getName(), this );
			break;
			
		case PD_E_RXQ_HIGH_WATER:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RXQ_HIGH_WATER \n", getName(), this );
            //            state = maskMux(state, generateRxQState( port ), (PD_S_RXQ_MASK | kRxAutoFlow));
            //            delta |= PD_S_RXQ_MASK | kRxAutoFlow;
			break;
			
		case PD_E_RXQ_LOW_WATER:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_RXQ_LOW_WATER \n", getName(), this );
            //            state = maskMux(state, generateRxQState( port ), (PD_S_RXQ_MASK | kRxAutoFlow));
            //            delta |= PD_S_RXQ_MASK | kRxAutoFlow;
			break;
			
		case PD_E_TXQ_HIGH_WATER:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_TXQ_HIGH_WATER \n", getName(), this );
			break;
			
		case PD_E_TXQ_LOW_WATER:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - PD_E_TXQ_LOW_WATER \n", getName(), this );
			break;
			
		default:
			DEBUG_IOLog(4,"%s(%p)::executeEvent - unrecognized event \n", getName(), this );
			ret = kIOReturnBadArgument;
			break;
	}
	
    state |= state;/* ejk for compiler warnings. ?? */
	changeState( port, state, delta );
    
	return ret;
    
}/* end executeEvent */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::requestEvent
//
//		Inputs:		event - The event
//					refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument
//					data - any data associated with the event
//
//		Desc:		call requestEventGated through the command gate.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::requestEvent(UInt32 event, UInt32 *data, void *refCon)
{
    IOReturn 	ret;
    
	DEBUG_IOLog(4,"%s(%p)::requestEvent\n", getName(), this);
    
    retain();
    ret = fCommandGate->runAction(requestEventAction, (void *)event, (void *)data, (void *)refCon);
    release();
    
    return ret;
    
}/* end requestEvent */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::requestEventAction
//
//		Desc:		Dummy pass through for requestEventGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::requestEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *)
{
	DEBUG_IOLog(4,"me_nozap_driver_PL2303::requestEventAction\n");
    
#if defined(__x86_64__)
    return ((me_nozap_driver_PL2303 *)owner)->requestEventGated((UInt64)arg0, (UInt32 *)arg1, (void *)arg2);
#else
    return ((me_nozap_driver_PL2303 *)owner)->requestEventGated((UInt32)arg0, (UInt32 *)arg1, (void *)arg2);
#endif
}/* end requestEventAction */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::requestEvent
//
//      Inputs:     event - The event, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnBadArgument, data - any data associated with the event
//
//      Desc:       requestEvent processes the specified event as an immediate request and
//                  returns the results in data.  This is primarily used for getting link
//                  status information and verifying baud rate and such.
//
//					Queue access requires this be on the command gate.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::requestEventGated( UInt32 event, UInt32 *data, void *refCon )
{
    PortInfo_t  *port = (PortInfo_t *) refCon;
    IOReturn    returnValue = kIOReturnSuccess;
	
    DEBUG_IOLog(4,"%s(%p)::requestEventGated\n", getName(), this);
	
    if ( data == NULL ) {
		DEBUG_IOLog(4,"%s(%p)::requestEvent - data is null\n", getName(), this );
		returnValue = kIOReturnBadArgument;
	}
	else
	{
		switch ( event )
		{
			case PD_E_ACTIVE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_ACTIVE\n", getName(), this);
				*data = bool(readPortState( port ) & PD_S_ACTIVE);
				break;
				
			case PD_E_FLOW_CONTROL:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_FLOW_CONTROL\n", getName(), this);
				*data = port->FlowControl;
				break;
				
			case PD_E_DELAY:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_DELAY\n", getName(), this);
				*data = tval2long( port->CharLatInterval )/ 1000;
				break;
				
			case PD_E_DATA_LATENCY:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_DATA_LATENCY\n", getName(), this);
				*data = tval2long( port->DataLatInterval )/ 1000;
				break;
				
			case PD_E_TXQ_SIZE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_TXQ_SIZE\n", getName(), this);
				*data = getQueueSize( &port->TX );
				break;
				
			case PD_E_RXQ_SIZE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RXQ_SIZE\n", getName(), this);
				*data = getQueueSize( &port->RX );
				break;
				
			case PD_E_TXQ_LOW_WATER:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_TXQ_LOW_WATER\n", getName(), this);
				*data = 0;
				returnValue = kIOReturnBadArgument;
				break;
				
			case PD_E_RXQ_LOW_WATER:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RXQ_LOW_WATER\n", getName(), this);
				*data = 0;
				returnValue = kIOReturnBadArgument;
				break;
				
			case PD_E_TXQ_HIGH_WATER:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_TXQ_HIGH_WATER\n", getName(), this);
				*data = 0;
				returnValue = kIOReturnBadArgument;
				break;
				
			case PD_E_RXQ_HIGH_WATER:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RXQ_HIGH_WATER\n", getName(), this);
				*data = 0;
				returnValue = kIOReturnBadArgument;
				break;
				
			case PD_E_TXQ_AVAILABLE:
				*data = freeSpaceinQueue( &port->TX );
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_TXQ_AVAILABLE size: %x\n", getName(), this, *data );
				break;
				
			case PD_E_RXQ_AVAILABLE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RXQ_AVAILABLE\n", getName(), this);
				*data = usedSpaceinQueue( &port->RX );
				break;
				
			case PD_E_DATA_RATE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_DATA_RATE\n", getName(), this);
				*data = port->BaudRate << 1;
				break;
				
			case PD_E_RX_DATA_RATE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RX_DATA_RATE\n", getName(), this);
				*data = 0x00;
				break;
				
			case PD_E_DATA_SIZE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_DATA_SIZE\n", getName(), this);
				*data = port->CharLength << 1;
				break;
				
			case PD_E_RX_DATA_SIZE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RX_DATA_SIZE\n", getName(), this);
				*data = 0x00;
				break;
				
			case PD_E_DATA_INTEGRITY:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_DATA_INTEGRITY\n", getName(), this);
				*data = port->TX_Parity;
				break;
				
			case PD_E_RX_DATA_INTEGRITY:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_E_RX_DATA_INTEGRITY\n", getName(), this);
				*data = port->RX_Parity;
				break;
				
			case PD_RS232_E_STOP_BITS:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_RS232_E_STOP_BITS\n", getName(), this);
				*data = port->StopBits << 1;
				break;
				
			case PD_RS232_E_RX_STOP_BITS:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_RS232_E_RX_STOP_BITS\n", getName(), this);
				*data = 0x00;
				break;
				
			case PD_RS232_E_XON_BYTE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_RS232_E_XON_BYTE\n", getName(), this);
				*data = port->XONchar;
				break;
				
			case PD_RS232_E_XOFF_BYTE:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_RS232_E_XOFF_BYTE\n", getName(), this);
				*data = port->XOFFchar;
				break;
				
			case PD_RS232_E_LINE_BREAK:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_RS232_E_LINE_BREAK\n", getName(), this);
				*data = bool(readPortState( port ) & PD_RS232_S_BRK);
                
				break;
				
			case PD_RS232_E_MIN_LATENCY:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - PD_RS232_E_MIN_LATENCY\n", getName(), this);
				*data = bool( port->MinLatency );
				break;
				
			default:
				DEBUG_IOLog(4,"%s(%p)::requestEvent - unrecognized event\n", getName(), this);
				returnValue = kIOReturnBadArgument;
				break;
		}
    }
	
    return kIOReturnSuccess;
    
}/* end requestEvent */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::enqueueEvent
//
//      Inputs:     event - The event, data - any data associated with the event,
//                                              sleep - true (wait for it), false (don't), refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//      Desc:       Only used for set/reset break
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::enqueueEvent( UInt32 event, UInt32 data, bool sleep, void *refCon)
{
	DEBUG_IOLog(2,"%s(%p)::enqueueEvent event: %p \n", getName(), this, data);
	PortInfo_t  *port = (PortInfo_t *) refCon;
    IOReturn    ret = kIOReturnSuccess;
    UInt32      state, delta;
    
    delta = 0;
    state = readPortState( port );
    
    
    if ( (state & PD_S_ACQUIRED) == 0 ){
		return kIOReturnNotOpen;
	}
	
    switch ( event )
	{
		case PD_RS232_E_LINE_BREAK:
			DEBUG_IOLog(2,"%s(%p)::enqueueEvent - PD_RS232_E_LINE_BREAK\n", getName(), this );
            state &= ~PD_RS232_S_BRK;
            delta |= PD_RS232_S_BRK;
            if (data)
            {
                port->BreakState = true;
            } else {
                port->BreakState = false;
            }
            setBreak(data);
            setStateGated(state, delta, port);
			break;
		case PD_E_DELAY:
			DEBUG_IOLog(2,"%s(%p)::enqueueEvent - PD_E_DELAY time: %d \n", getName(), this, data );
            if (port->BreakState)					// It's the break delay in micro seconds
            {
                IOSleep(data/1000);
            } else {
                port->CharLatInterval = long2tval(data * 1000);
            }
			break;
		default:
			DEBUG_IOLog(2,"%s(%p)::enqueueEvent - unrecognized event \n", getName(), this );
			ret = kIOReturnBadArgument;
			break;
	}
	
    state |= state;/* ejk for compiler warnings. ?? */
	changeState( port, state, delta );
    
	return ret;
    
	return kIOReturnSuccess;
    
    
}/* end enqueueEvent */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::dequeueEvent
//
//      Inputs:     sleep - true (wait for it), false (don't), refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//      Desc:       Not used by this driver.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::dequeueEvent( UInt32 *event, UInt32 *data, bool sleep, void *refCon )
{
	DEBUG_IOLog(4,"%s(%p)::dequeueEvent\n", getName(), this);
    
    PortInfo_t *port = (PortInfo_t *) refCon;
    
    if ( (event == NULL) || (data == NULL) )
		return kIOReturnBadArgument;
	
    if ( readPortState( port ) & PD_S_ACTIVE )
	{
#ifdef FIX_PARITY_PROCESSING
        *event = nextEvent(refCon);
        
        if(*event == PD_E_EOQ)
            return kIOReturnSuccess;
        
        UInt8 Value;
        UInt8 rtn = getBytetoQueue(&fPort->RX, &Value);
        if(rtn != kIOReturnSuccess)
            return rtn;
        *data = Value;
        
        DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueEvent held=[0x%X]\n", Value );
        
        if(Value == 0xff) {
            while(getBytetoQueue(&fPort->RX, &Value) == kQueueEmpty){};
            DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueEvent purged=[0x%X]\n", Value );
        }
        
        if(*event == PD_E_INTEGRITY_ERROR) {
            getBytetoQueue(&fPort->RX, &Value); // Purge marker
            DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueEvent purged=[0x%X]\n", Value );
            while(getBytetoQueue(&fPort->RX, &Value) == kQueueEmpty)
                IOSleep(BYTE_WAIT_PENALTY); // in case it is not yet cool
            DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueEvent purged=[0x%X]\n", Value );
        }
#endif
		return kIOReturnSuccess;
	}
	
    return kIOReturnNotOpen;
    
}/* end dequeueEvent */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::enqueueData
//
//		Inputs:		buffer - the data
//					size - number of bytes
//					sleep - true (wait for it), false (don't)
//					refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument or value returned from watchState
//					count - bytes transferred
//
//		Desc:		set up for enqueueDataGated call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::enqueueData(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *refCon)
{
    IOReturn 	ret;
    
    if (count == NULL || buffer == NULL)
        return kIOReturnBadArgument;
	
    retain();
	ret = fCommandGate->runAction(enqueueDataAction, (void *)buffer, (void *)size, (void *)count, (void *)sleep);
    release();
	
    return ret;
	
}/* end enqueueData */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::enqueueDatatAction
//
//		Desc:		Dummy pass through for equeueDataGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::enqueueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
#if defined(__x86_64__)
    return ((me_nozap_driver_PL2303 *)owner)->enqueueDataGated((UInt8 *)arg0, (UInt64)arg1, (UInt32 *)arg2, (bool)arg3);
#else
    return ((me_nozap_driver_PL2303 *)owner)->enqueueDataGated((UInt8 *)arg0, (UInt32)arg1, (UInt32 *)arg2, (bool)arg3);
#endif
}/* end enqueueDataAction */

/****************************************************************************************************/
//
//
//      Method:     me_nozap_driver_PL2303::enqueueData
//
//      Inputs:     buffer - the data, size - number of bytes, sleep - true (wait for it), false (don't),
//                                                                                      refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess or value returned from watchState, count - bytes transferred,
//
//      Desc:       enqueueData will attempt to copy data from the specified buffer to
//                  the TX queue as a sequence of VALID_DATA events.  The argument
//                  bufferSize specifies the number of bytes to be sent.  The actual
//                  number of bytes transferred is returned in count.
//                  If sleep is true, then this method will sleep until all bytes can be
//                  transferred.  If sleep is false, then as many bytes as possible
//                  will be copied to the TX queue.
//                  Note that the caller should ALWAYS check the transferCount unless
//                  the return value was kIOReturnBadArgument, indicating one or more
//                  arguments were not valid.  Other possible return values are
//                  kIOReturnSuccess if all requirements were met.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::enqueueDataGated( UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep)
{
    UInt32      state = PD_S_TXQ_LOW_WATER;
    IOReturn    rtn = kIOReturnSuccess;
	
    DEBUG_IOLog(1,"%s(%p)::enqueueDataGated (bytes: %d)\n", getName(), this,size);
    /*
     #ifdef DEBUG
     UInt8 *buf;
     UInt32 buflen;
     buflen = size;
     buf = buffer;
     
     
     while ( buflen ){
     unsigned char c = *buf;
     DEBUG_IOLog(1,"[%02x] ",c);
     buf++;
     buflen--;
     }
     
     #endif
     */
    if ( fTerminate ){
		IOLog("%s(%p)::enqueueDataGated fTerminate set\n", getName(), this);
		
		return kIOReturnOffline; }
	
    if ( count == NULL || buffer == NULL ){
		IOLog("%s(%p)::enqueueDataGated buffer empty\n", getName(), this);
		
		return kIOReturnBadArgument;}
	
    *count = 0;
	
    if ( !(readPortState( fPort ) & PD_S_ACTIVE) ){
		IOLog("%s(%p)::enqueueDataGated port not open\n", getName(), this);
		
		return kIOReturnNotOpen;
	}
    
	/* OK, go ahead and try to add something to the buffer  */
    *count = addtoQueue( &fPort->TX, buffer, size );
    checkQueues( fPort );
	
	/* Let the tranmitter know that we have something ready to go   */
    setUpTransmit( );
	
	/* If we could not queue up all of the data on the first pass and   */
	/* the user wants us to sleep until it's all out then sleep */
	
    while ( (*count < size) && sleep )
	{
		state = PD_S_TXQ_LOW_WATER;
		rtn = watchStateGated( &state, PD_S_TXQ_LOW_WATER );
		if ( rtn != kIOReturnSuccess )
		{
			IOLog("%s(%p)::enqueueDataGated - interrupted\n", getName(), this);
			return rtn;
		}
		
		*count += addtoQueue( &fPort->TX, buffer + *count, size - *count );
		checkQueues( fPort );
		
		/* Let the tranmitter know that we have something ready to go.  */
		
		setUpTransmit( );
	}/* end while */
    
    DEBUG_IOLog(4,"%s(%p)::enqueueDataGateda - Enqueue\n", getName(), this);
    
    return kIOReturnSuccess;
    
}/* end enqueueData */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::dequeueData
//
//		Inputs:		size - buffer size
//					min - minimum bytes required
//					refCon - the Port (not used)
//
//		Outputs:	buffer - data returned
//					min - number of bytes
//					Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
//
//		Desc:		set up for enqueueDataGated call.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon)
{
    IOReturn 	ret;
	DEBUG_IOLog(4,"%s(%p)::dequeueData\n", getName(), this);
    
    if ((count == NULL) || (buffer == NULL) || (min > size))
        return kIOReturnBadArgument;
	
	retain();
    ret = fCommandGate->runAction(dequeueDataAction, (void *)buffer, (void *)size, (void *)count, (void *)min);
    release();
	
    return ret;
	
	
}/* end dequeueData */

/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::dequeueDatatAction
//
//		Desc:		Dummy pass through for equeueDataGated.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::dequeueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
	DEBUG_IOLog(4,"me_nozap_driver_PL2303::dequeueDataAction\n");
    
#if defined(__x86_64__)
    return ((me_nozap_driver_PL2303 *)owner)->dequeueDataGated((UInt8 *)arg0, (UInt64)arg1, (UInt32 *)arg2, (UInt64)arg3);
#else
    return ((me_nozap_driver_PL2303 *)owner)->dequeueDataGated((UInt8 *)arg0, (UInt32)arg1, (UInt32 *)arg2, (UInt32)arg3);
#endif
}/* end dequeueDataAction */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::dequeueData
//
//      Inputs:     size - buffer size, min - minimum bytes required, refCon - the Port
//
//      Outputs:    buffer - data returned, min - number of bytes
//                  Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
//
//      Desc:       dequeueData will attempt to copy data from the RX queue to the
//                  specified buffer.  No more than bufferSize VALID_DATA events
//                  will be transferred. In other words, copying will continue until
//                  either a non-data event is encountered or the transfer buffer
//                  is full.  The actual number of bytes transferred is returned
//                  in count.
//                  The sleep semantics of this method are slightly more complicated
//                  than other methods in this API. Basically, this method will
//                  continue to sleep until either min characters have been
//                  received or a non data event is next in the RX queue.  If
//                  min is zero, then this method never sleeps and will return
//                  immediately if the queue is empty.
//                  Note that the caller should ALWAYS check the transferCount
//                  unless the return value was kIOReturnBadArgument, indicating one or
//                  more arguments were not valid.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::dequeueDataGated( UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min )
{
    IOReturn    rtn = kIOReturnSuccess;
    UInt32      state = 0;
    CirQueue *Queue;
    
    DEBUG_IOLog(4,"%s(%p)::dequeueDataGated\n", getName(), this);
    
    /* Check to make sure we have good arguments.   */
    if ( (count == NULL) || (buffer == NULL) || (min > size) )
        return kIOReturnBadArgument;
    
    /* If the port is not active then there should not be any chars.    */
    *count = 0;
    if ( !(readPortState( fPort ) & PD_S_ACTIVE) )
        return kIOReturnNotOpen;
    
    Queue = &fPort->RX;
    
#if FIX_PARITY_PROCESSING
    while (*count < size) {
        UInt8 Value;
        if(peekBytefromQueue(Queue, &Value, 1) != kQueueEmpty && Value == 0xff) {
            if (peekBytefromQueue(Queue, &Value, 2) != kQueueEmpty && Value == 0x00) {
                checkQueues( fPort );
                return kIOReturnSuccess;
            }
        }
        if( (rtn = getBytetoQueue(Queue, &Value)) != kQueueNoError) {
            if(rtn == kQueueEmpty)
                break;
            IOLog("%s(%p)::dequeueDataGated - INTERRUPTED while reading\n", getName(), this );
            return rtn;
        }
        DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueDataGated held=[0x%X]\n", Value );
        if(Value == 0xff) {
            while(getBytetoQueue(Queue, &Value) == kQueueEmpty){}; // Read double 0xff
            DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueDataGated purged=[0x%X]\n", Value );
        }
        *(buffer++) = Value;
        ++(*count);
    }
#else
    /* Get any data living in the queue.    */
    *count = removefromQueue( Queue, buffer, size );
#endif
    
    checkQueues( fPort );
    while ( (min > 0) && (*count < min) )
    {

        
        /* Figure out how many bytes we have left to queue up */
        DEBUG_IOLog(4,"%s(%p)::dequeueDataGated - min: %d count: %d size: %d SizeQueue: %d InQueue: %d \n", getName(), this,min,*count, (size - *count), Queue->Size, Queue->InQueue );
        
#if FIX_PARITY_PROCESSING
        /* Always prefer waiting for HIGH_WATER to waiting a little bit more for not empty queue */
        state = PD_S_RXQ_HIGH_WATER;
        rtn = watchStateGated( &state, PD_S_RXQ_EMPTY | PD_S_RXQ_HIGH_WATER);
        if(!(state & PD_S_RXQ_HIGH_WATER))
            IOSleep(BYTE_WAIT_PENALTY);
#else
        state = 0;
        rtn = watchStateGated( &state, PD_S_RXQ_EMPTY);
#endif
        
        if ( rtn != kIOReturnSuccess )
        {
            IOLog("%s(%p)::dequeueDataGated - INTERRUPTED\n", getName(), this );
            //			LogData( kUSBIn, *count, buffer );
            return rtn;
        }
        /* Try and get more data starting from where we left off */
#if FIX_PARITY_PROCESSING
        while (*count < size) {
            UInt8 Value;
            if(peekBytefromQueue(Queue, &Value, 1) != kQueueEmpty && Value == 0xff) {
                if (peekBytefromQueue(Queue, &Value, 2) != kQueueEmpty && Value == 0x00) {
                    DEBUG_IOLog(4,"%s(%p)::dequeueDataGated Parity error on queue -->Out Dequeue\n", getName(), this);
                    checkQueues( fPort );
                    return kIOReturnSuccess;
                }
            }
            if( (rtn = getBytetoQueue(Queue, &Value)) != kQueueNoError) {
                if(rtn == kQueueEmpty)
                    break;
                IOLog("%s(%p)::dequeueDataGated - INTERRUPTED while reading\n", getName(), this );
                return rtn;
            }
            DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueDataGated held=[0x%X]\n", Value );
            if(Value == 0xff) {
                while(getBytetoQueue(Queue, &Value) == kQueueEmpty){}; // Read double 0xff
                DATA_IOLog(2,"me_nozap_driver_PL2303::dequeueDataGated purged=[0x%X]\n", Value );
            }
            *(buffer++) = Value;
            ++(*count);
        }
#else
        count_read = removefromQueue( &fPort->RX, buffer + *count, (size - *count) );
        *count += count_read;
#endif
        checkQueues( fPort );
        
    }/* end while */
    
    DEBUG_IOLog(4,"%s(%p)::dequeueDataGated -->Out Dequeue\n", getName(), this);
    
    return kIOReturnSuccess;
    
}/* end dequeueData */


/****************************************************************************************************/
//
//		Method:		me_nozap_driver_PL2303::getState
//
//		Inputs:		refCon - the Port (not used)
//
//		Outputs:	Return value - port state
//
//		Desc:		Get the state for the port.
//
/****************************************************************************************************/

UInt32 me_nozap_driver_PL2303::getState(void *refCon)
{
	DEBUG_IOLog(6,"%s(%p)::getState\n", getName(), this);
    
	PortInfo_t  *port = (PortInfo_t *) refCon;
    UInt32      state;
    
    checkQueues( port );
	
    state = readPortState( port ) & EXTERNAL_MASK;
    
    DEBUG_IOLog(6,"%s(%p)::getState-->State: %x\n", getName(), this, state );
    
    return state;
}/* end getState */




/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::StartTransmission
//
//      Inputs:     control_length - Length of control data
//                  control_buffer - Control data
//                  data_length - Length of raw data
//                  data_buffer - raw data
//
//      Outputs:    Return code - kIOReturnSuccess
//
//      Desc:       Start the transmisson. If both control and data length is zero then only
//                  the change byte will be sent.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::startTransmit(UInt32 control_length, UInt8 *control_buffer, UInt32 data_length, UInt8 *data_buffer)
{
    IOReturn    ior;
    
	DEBUG_IOLog(1,"%s(%p)::StartTransmit\n", getName(), this);
	if ( data_length != 0 )
	{
		bcopy(data_buffer, &fPipeOutBuffer[0], data_length);
	}
    
    // add up the total length to send off to the device
    fCount = control_length + data_length;
    fpPipeOutMDP->setLength( fCount );
	
    fWriteActive = true;
	changeState( fPort, PD_S_TX_BUSY ,PD_S_TX_BUSY );
	
    //    UInt32      state, delta;
    //    delta = 0;
    //    state = readPortState( port );
    //	state &= ~PD_S_TX_BUSY;
    //	delta |= PD_S_TX_BUSY;
    //	setStateGated(state, delta, port);
	
	
#ifdef DATALOG
	UInt8 *buf;
	UInt32 buflen;
	buflen = fCount;
	buf = &fPipeOutBuffer[0];
	
	DATA_IOLog(1,"me_nozap_driver_PL2303: Send (bytes %d): ",fCount);
	while ( buflen ){
		unsigned char c = *buf;
		DATA_IOLog(1,"[%02x] ",c);
		buf++;
		buflen--;
	}
    
#endif
    ior = fpOutPipe->Write( fpPipeOutMDP, 1000, 1000, &fWriteCompletionInfo );  // 1 second timeouts
    DEBUG_IOLog(1,"%s(%p)::StartTransmit return value %d\n", getName(), this, ior);
    return ior;
    
}/* end StartTransmission */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::dataWriteComplete
//
//      Inputs:     obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//
//      Outputs:    None
//
//      Desc:       BulkOut pipe (Data interface) write completion routine
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::dataWriteComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
    
    me_nozap_driver_PL2303  *me = (me_nozap_driver_PL2303*)obj;
	DEBUG_IOLog(1,"me_nozap_driver_PL2303::dataWriteComplete return code c: %d, fcount: %d,  remaining: %d\n", rc, me->fCount,remaining );
    
    // Boolean done = true;                // write really finished?  // use is commented out below.
    me->fWriteActive = false;
    // BJA we zijn nu klaar dus zet TX BUSY weer uit
    me->changeState( me->fPort, 0, PD_S_TX_BUSY );
	me->fPort->AreTransmitting = false;
	if (me->fTerminate)
        return;
	
    
    
    // in a transmit complete, but need to manually transmit a zero-length packet
    // if it's a multiple of the max usb packet size for the bulk-out pipe (64 bytes)
    
    if ( rc == kIOReturnSuccess )   // If operation returned ok
    {
        
        //		if ( me->fCount > 0 )                       // Check if it was not a zero length write
        //		{
        
        //			if ( (me->fCount % 64) == 0 )               // If was a multiple of 64 bytes then we need to do a zero length write
        //			{
        //				me->changeState( me->fPort, PD_S_TX_BUSY ,PD_S_TX_BUSY );
        //				me->fWriteActive = true;
        //				me->fpPipeOutMDP->setLength( 0 );
        //				me->fCount = 0;
        //				me->fpOutPipe->Write( me->fpPipeOutMDP,1000,1000, &me->fWriteCompletionInfo );
        //				done = false;               // don't complete back to irda quite yet
        //			}
        //		}
		
		me->setUpTransmit();						// just to keep it going??
    }
    
    return;
    
}/* end dataWriteComplete */



/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::interruptReadComplete
//
//      Inputs:     obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//                                                                                  (whose idea was that?)
//
//      Outputs:    None
//
//      Desc:       Interrupt pipe read. Interrupts are used for reading handshake signals. see linux driver
//
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::interruptReadComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	DEBUG_IOLog(1,"me_nozap_driver_PL2303::interruptReadComplete" );
	UInt8 status_idx = kUART_STATE;
	UInt8 length = INTERRUPT_BUFF_SIZE;
	UInt32 stat = 0;
    me_nozap_driver_PL2303  *me = (me_nozap_driver_PL2303*)obj;
	PortInfo_t            *port = (PortInfo_t*)param;
    UInt32      dLen;
	
    if ( rc == kIOReturnSuccess )   /* If operation returned ok:    */
	{
		if ( (me->fpDevice->GetVendorID() == SIEMENS_VENDOR_ID ) && (me->fpDevice->GetProductID() == SIEMENS_PRODUCT_ID_X65) ) {
            status_idx = 0;
            length = 1;
            DEBUG_IOLog( 3, "me_nozap_driver_PL2303::interruptReadComplete interrupt Buff size = 1\n");
        }
		dLen = length - remaining;
    	if (dLen != length)
		{
			DEBUG_IOLog(1,"me_nozap_driver_PL2303::interruptReadComplete wrong buffersize");
		} else {
            
            
			UInt8 *buf;
			UInt32 buflen;
			buflen = dLen;
			buf = &me->fpinterruptPipeBuffer[0];
#ifdef DATALOG
            
			DATA_IOLog(1,"me_nozap_driver_PL2303: Interrupt: ");
			unsigned char c = buf[status_idx];
		    DATA_IOLog(1,"[%02x] ",c);
#endif
			me->fPort->lineState = buf[status_idx];
            
			if (buf[status_idx] & kCTS) stat |= PD_RS232_S_CTS;
			if (buf[status_idx] & kDSR) stat |= PD_RS232_S_DSR;
			if (buf[status_idx] & kRI)  stat |= PD_RS232_S_RI;
			if (buf[status_idx] & kDCD) stat |= PD_RS232_S_CAR;
            // ++ Parity check
            if(buf[status_idx] & kParityError) {
#if FIX_PARITY_PROCESSING
                DEBUG_IOLog(5,"me_nozap_driver_PL2303::interruptReadComplete PARITY ERROR\n");
                me->addBytetoQueue(&me->fPort->RX, 0xff); // Internal parity error marker
                me->addBytetoQueue(&me->fPort->RX, 0x00); // Internal parity error marker
#else
                DEBUG_IOLog(5,"me_nozap_driver_PL2303::interruptReadComplete PARITY ERROR (ignored)\n");
#endif
            }
            me->setStateGated( stat, kHandshakeInMask , port); // refresh linestate in State
            
		}
		
	    /* Queue the next interrupt read:   */
		
		me->fpInterruptPipe->Read( me->fpinterruptPipeMDP, &me->finterruptCompletionInfo, NULL );
        
#if FIX_PARITY_PROCESSING
        me->checkQueues( port );
#endif
    } else {
        DEBUG_IOLog(1,"me_nozap_driver_PL2303::interruptReadComplete wrong return code: %p", rc );
	}
    return;
}/* end interruptReadComplete */



/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::dataReadComplete
//
//      Inputs:     obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//
//      Outputs:    None
//
//      Desc:       BulkIn pipe (Data interface) read completion routine
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::dataReadComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	DEBUG_IOLog(4,"me_nozap_driver_PL2303::dataReadComplete\n");
    me_nozap_driver_PL2303  *me = (me_nozap_driver_PL2303*)obj;
    PortInfo_t      *port = (PortInfo_t*)param;
    UInt16          dtlength;
    IOReturn        ior = kIOReturnSuccess;
    if ( rc == kIOReturnSuccess )   /* If operation returned ok:    */
	{
		me->fReadActive = false;
		dtlength = USBLapPayLoad - remaining;
		if ( dtlength > 0 )
		{
#ifdef DATALOG
            
			UInt8 *buf;
			UInt32 buflen;
			buflen = dtlength;
			buf = &me->fPipeInBuffer[0];
			DATA_IOLog(1,"me_nozap_driver_PL2303: Receive: ");
			while ( buflen ){
				unsigned char c = *buf;
				DATA_IOLog(1,"[%02x] ",c);
				buf++;
				buflen--;
			}
            
#endif
            
#if FIX_PARITY_PROCESSING
            if ( !(me->fPort && me->fPort->serialRequestLock ) ) goto Fail;
            DEBUG_IOLog(2,"me_nozap_driver_PL2303::dataReadComplete IOLockLock( port->serialRequestLock );\n" );
            
            IOLockLock( me->fPort->serialRequestLock );
            
            clock_get_system_nanotime(&me->_fReadTimestampSecs, &me->_fReadTimestampNanosecs);
            
            DEBUG_IOLog(2,"me_nozap_driver_PL2303::dataReadComplete IOLockUnLock( port->serialRequestLock ); kQueueNoError\n" );
            
            IOLockUnlock( me->fPort->serialRequestLock);
#endif
			ior = me->addtoQueue( &me->fPort->RX, &me->fPipeInBuffer[0], dtlength );
		}
		
		/* Queue the next read 	*/
		ior = me->fpInPipe->Read( me->fpPipeInMDP, &me->fReadCompletionInfo, NULL );
	    
		if ( ior == kIOReturnSuccess )
		{
			me->fReadActive = true;
			me->checkQueues( port );
			return;
		} else {
			DEBUG_IOLog(4,"me_nozap_driver_PL2303::dataReadComplete dataReadComplete - queueing bulk read failed\n");
		}
		
	} else {
    Fail:
		/* Read returned with error */
		DEBUG_IOLog(4,"me_nozap_driver_PL2303::dataReadComplete - io err %x\n",rc );
		
	}
	
    return;
    
}/* end dataReadComplete */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::allocateRingBuffer
//
//      Inputs:     Queue - the specified queue to allocate, BufferSize - size to allocate
//
//      Outputs:    return Code - true (buffer allocated), false (it failed)
//
//      Desc:       Allocates resources needed by the queue, then sets up all queue parameters.
//
/****************************************************************************************************/

bool me_nozap_driver_PL2303::allocateRingBuffer( CirQueue *Queue, size_t BufferSize )
{
    UInt8       *Buffer;
	
	// Size is ignored and kMaxCirBufferSize, which is 4096, is used.
	// BJA Hack
#define kCirBufferSize 1
    DEBUG_IOLog(4,"%s(%p)::allocateRingBuffer\n", getName(), this );
    Buffer = (UInt8*)IOMalloc( kMaxCirBufferSize );
	
    initQueue( Queue, Buffer, kMaxCirBufferSize );
	
    if ( Buffer )
		return true;
	
    return false;
    
}/* end allocateRingBuffer */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::freeRingBuffer
//
//      Inputs:     Queue - the specified queue to free
//
//      Outputs:    None
//
//      Desc:       Frees all resources assocated with the queue, then sets all queue parameters
//                  to safe values.
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::freeRingBuffer( CirQueue *Queue )
{
    DEBUG_IOLog(4,"%s(%p)::freeRingBuffer\n", getName(), this );
    if( !(Queue->Start) )  goto Bogus;
    
    IOFree( Queue->Start, Queue->Size );
    closeQueue( Queue );
	
Bogus:
    return;
    
}/* end freeRingBuffer */




/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::SetSpeed
//
//      Inputs:     brate - the requested baud rate
//
//      Outputs:    return word - baud coding
//
//      Desc:       Set the baudrate for the device
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::setSerialConfiguration( void )
{
	IOReturn rtn;
	IOUSBDevRequest request;
	char * buf;
    DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration baudrate: %d \n", getName(), this, fPort->BaudRate );
	buf = (char *)IOMalloc( 10 );
	memset(buf, 0x00, 0x07);
    
    fCurrentBaud = fPort->BaudRate;
    
    switch (fPort->BaudRate){
		case 75:
			fBaudCode = kLinkSpeed75;     // 0x01
			break;
		case 150:
			fBaudCode = kLinkSpeed150;     // 0x01
			break;
		case 300:
			fBaudCode = kLinkSpeed300;     // 0x01
			break;
		case 600:
			fBaudCode = kLinkSpeed600;     // 0x01
			break;
		case 1200:
			fBaudCode = kLinkSpeed1200;     // 0x01
			break;
		case 1800:
			fBaudCode = kLinkSpeed1800;     // 0x01
			break;
		case 2400:
			fBaudCode = kLinkSpeed2400;     // 0x01
			break;
		case 3600:
			fBaudCode = kLinkSpeed3600;     // 0x01
			break;
		case 4800:
			fBaudCode = kLinkSpeed4800;     // 0x01
			break;
		case 7200:
			fBaudCode = kLinkSpeed7200;     // 0x01
			break;
		case 9600:
			fBaudCode = kLinkSpeed9600;     // 0x02
			break;
		case 19200:
			fBaudCode = kLinkSpeed19200;    // 0x03
			break;
		case 38400:
			fBaudCode = kLinkSpeed38400;    // 0x04
			break;
		case 57600:
			fBaudCode = kLinkSpeed57600;    // 0x05
			break;
		case 115200:
			fBaudCode = kLinkSpeed115200;   // 0x06
			break;
		case 230400:
			fBaudCode = kLinkSpeed230400;   // 0x07
			break;
		case 460800:
			fBaudCode = kLinkSpeed460800;  // 0x08
			break;
		case 614400:
			fBaudCode = kLinkSpeed614400;  // 0x08
			break;
		case 921600:
			fBaudCode = kLinkSpeed921600;  // 0x08
			break;
		case 1228800:
			fBaudCode = kLinkSpeed1228800;  // 0x08
			break;
		case 1843200:
			fBaudCode = kLinkSpeed1843200;  // 0x08
			break;
		case 2457600:
			fBaudCode = kLinkSpeed2457600;  // 0x08
			break;
		case 3000000:
			fBaudCode = kLinkSpeed3000000;  // 0x08
			break;
		case 6000000:
			fBaudCode = kLinkSpeed6000000;  // 0x08
			break;
			
            // Other baudrates may be depend on the model (see manual on page 19)
            // I changed the error into a warning...
		default:
			IOLog("%s(%p)::setSerialConfiguration - Requesting non standard baud rate\n", getName(), this);
			fBaudCode = fPort->BaudRate;
			break;
    }
	
	if(fBaudCode) {
		buf[0] = fBaudCode & 0xff;
		buf[1] = (fBaudCode >> 8) & 0xff;
		buf[2] = (fBaudCode >> 16) & 0xff;
		buf[3] = (fBaudCode >> 24) & 0xff;
	}
	
    switch (fPort->StopBits) {
        case 0:
            buf[4] = 0;
            break;
            
        case 2:
            buf[4] = 0; // 1 stop bit
            break;
            
        case 3:
            buf[4] = 1; // 1.5 stop bits
            break;
            
        case 4:
            buf[4] = 2; // 2 stop bits
            break;
            
        default:
            buf[4] = 0;
            break;
    }
	DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - StopBits: %d \n", getName(), this,  buf[4]);
	
	
    switch(fPort->TX_Parity)
    {
        case PD_RS232_PARITY_NONE:
            buf[5] = 0;
			DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - PARITY_NONE \n", getName(), this);
            break;
            
        case PD_RS232_PARITY_ODD:
            buf[5] = 1;
			DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - PARITY_ODD \n", getName(), this);
            break;
            
        case PD_RS232_PARITY_EVEN:
            buf[5] = 2;
			DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - PARITY_EVEN \n", getName(), this);
            break;
            
        case PD_RS232_PARITY_MARK:
			buf[5] = 3;
			DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - PARITY_MARK \n", getName(), this);
			break;
			
		case PD_RS232_PARITY_SPACE:
			buf[5] = 4;
			DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - PARITY_SPACE \n", getName(), this);
			break;
			
        default:
			buf[5] = 0;
			DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - PARITY_NONE \n", getName(), this);
    }
	
	if (fPort->CharLength >= 5 && fPort->CharLength <= 8){
		buf[6] = fPort->CharLength;
    }
	DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - Bits: %d \n", getName(), this,  buf[6]);
	
	request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    request.bRequest = SET_LINE_REQUEST;
	request.wValue =  0;
	request.wIndex = 0;
	request.wLength = 7;
	request.pData = buf;
	rtn =  fpDevice->DeviceRequest(&request);
	DEBUG_IOLog(3,"%s(%p)::setSerialConfiguration - return: %p \n", getName(), this,  rtn);
	IOFree( buf, 10 );
	
    
    
	return rtn;
}/* end SetSpeed */


/* QueuePrimatives  */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::AddBytetoQueue
//
//      Inputs:     Queue - the queue to be added to
//
//      Outputs:    Value - Byte to be added, Queue status - full or no error
//
//      Desc:       Add a byte to the circular queue.
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::addBytetoQueue( CirQueue *Queue, char Value )
{
    /* Check to see if there is space by comparing the next pointer,    */
    /* with the last, If they match we are either Empty or full, so     */
    /* check the InQueue of being zero.                 */
    DEBUG_IOLog(4,"me_nozap_driver_PL2303(%p)::AddBytetoQueue\n", this );
	
    if ( !(fPort && fPort->serialRequestLock ) ) goto Fail;
	DEBUG_IOLog(2,"me_nozap_driver_PL2303::addBytetoQueue IOLockLock( port->serialRequestLock );\n" );
	
    IOLockLock( fPort->serialRequestLock );
	
    if ( (Queue->NextChar == Queue->LastChar) && Queue->InQueue ) {
		DEBUG_IOLog(2,"me_nozap_driver_PL2303::addBytetoQueue IOLockUnLock( port->serialRequestLock ); kQueueFull\n" );
        
		IOLockUnlock( fPort->serialRequestLock);
		return kQueueFull;
	}
	
    *Queue->NextChar++ = Value;
    Queue->InQueue++;
	
	/* Check to see if we need to wrap the pointer. */
	
    if ( Queue->NextChar >= Queue->End )
		Queue->NextChar =  Queue->Start;
    
	DEBUG_IOLog(2,"me_nozap_driver_PL2303::addBytetoQueue IOLockUnLock( port->serialRequestLock ); kQueueNoError\n" );
	
    IOLockUnlock( fPort->serialRequestLock);
    return kQueueNoError;
    
Fail:
    return kQueueFull;       // for lack of a better error
    
}/* end AddBytetoQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::GetBytetoQueue
//
//      Inputs:     Queue - the queue to be removed from
//
//      Outputs:    Value - where to put the byte, Queue status - empty or no error
//
//      Desc:       Remove a byte from the circular queue.
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::getBytetoQueue( CirQueue *Queue, UInt8 *Value )
{
    DEBUG_IOLog(4,"%s(%p)::GetBytetoQueue\n", getName(), this );
	
    if( !(fPort && fPort->serialRequestLock) ) goto Fail;
	DEBUG_IOLog(2,"me_nozap_driver_PL2303::getBytetoQueue IOLockLock( port->serialRequestLock ); \n" );
    
    IOLockLock( fPort->serialRequestLock );
	
	/* Check to see if the queue has something in it.   */
	
    if ( (Queue->NextChar == Queue->LastChar) && !Queue->InQueue ) {
		DEBUG_IOLog(2,"me_nozap_driver_PL2303::getBytetoQueue IOLockUnLock( port->serialRequestLock ); kQueueEmpty\n" );
        
		IOLockUnlock(fPort->serialRequestLock);
		return kQueueEmpty;
	}
    
#if FIX_PARITY_PROCESSING
    // If queue has only one byte, check with timestamp, to allow cooldown grace period
    if(Queue->InQueue == 1) {
        clock_sec_t			secs;
        clock_nsec_t        nanosecs;
        clock_get_system_nanotime(&secs, &nanosecs);
        if( secs == _fReadTimestampSecs && nanosecs < _fReadTimestampNanosecs + LAST_BYTE_COOLDOWN ) {
            // Pretend it is empty
            DEBUG_IOLog(2,"me_nozap_driver_PL2303::getBytetoQueue IOLockUnLock( port->serialRequestLock ); (cooldown - queue empty)\n" );
            
            IOLockUnlock(fPort->serialRequestLock);
            return kQueueEmpty;
        }
    }
#endif
    
    *Value = *Queue->LastChar++;
    Queue->InQueue--;
	
	/* Check to see if we need to wrap the pointer. */
	
    if ( Queue->LastChar >= Queue->End )
		Queue->LastChar =  Queue->Start;
	
	DEBUG_IOLog(2,"me_nozap_driver_PL2303::getBytetoQueue IOLockUnLock( port->serialRequestLock ); kQueueNoError\n" );
	
    IOLockUnlock(fPort->serialRequestLock);
    return kQueueNoError;
    
Fail:
    return kQueueEmpty;          // can't get to it, pretend it's empty
    
}/* end GetBytetoQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::peekBytefromQueue
//
//      Inputs:     Queue - the queue to be peeked into, offset - index of byte to be peeked
//
//      Outputs:    Value - where to put the byte, Queue status - empty or no error
//
//      Desc:       Peek a byte from the circular queue.
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::peekBytefromQueue( CirQueue *Queue, UInt8 *Value, size_t offset = 0)
{
    DEBUG_IOLog(4,"%s(%p)::peekBytefromQueue\n", getName(), this );
    
    if( !(fPort && fPort->serialRequestLock) ) goto Fail;
	DEBUG_IOLog(2,"me_nozap_driver_PL2303::peekBytefromQueue IOLockLock( port->serialRequestLock ); \n" );
    
    IOLockLock( fPort->serialRequestLock );
    
	/* Check to see if the queue has something in it.   */
    
    if ( ((Queue->NextChar == Queue->LastChar) && !Queue->InQueue) || Queue->InQueue <= offset ) {
		DEBUG_IOLog(2,"me_nozap_driver_PL2303::peekBytefromQueue IOLockUnLock( port->serialRequestLock ); kQueueEmpty\n" );
        
		IOLockUnlock(fPort->serialRequestLock);
		return kQueueEmpty;
	}
    
    if(Queue->LastChar + offset >= Queue->End) {
        *Value = *( Queue->Start + (offset - (Queue->End - Queue->LastChar)) );
    } else
        *Value = Queue->LastChar[offset];
    
	DEBUG_IOLog(2,"me_nozap_driver_PL2303::peekBytefromQueue IOLockUnLock( port->serialRequestLock ); kQueueNoError\n" );
    
    IOLockUnlock(fPort->serialRequestLock);
    
	DEBUG_IOLog(5,"me_nozap_driver_PL2303::peekBytefromQueue offset = %u [0x%02x]\n", (unsigned) offset, *Value );
    return kQueueNoError;
    
Fail:
    return kQueueEmpty;          // can't get to it, pretend it's empty
    
}/* end peekBytefromQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::InitQueue
//
//      Inputs:     Queue - the queue to be initialized, Buffer - the buffer, size - length of buffer
//
//      Outputs:    Queue status - queueNoError.
//
//      Desc:       Pass a buffer of memory and this routine will set up the internal data structures.
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::initQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size )
{
    DEBUG_IOLog(4,"%s(%p)::InitQueue\n", getName(), this );
    
    Queue->Start    = Buffer;
    Queue->End      = (UInt8*)((size_t)Buffer + Size);
    Queue->Size     = Size;
    Queue->NextChar = Buffer;
    Queue->LastChar = Buffer;
    Queue->InQueue  = 0;
	
    IOSleep( 1 );
    
    return kQueueNoError ;
    
}/* end InitQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::CloseQueue
//
//      Inputs:     Queue - the queue to be closed
//
//      Outputs:    Queue status - queueNoError.
//
//      Desc:       Clear out all of the data structures.
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::closeQueue( CirQueue *Queue )
{
    DEBUG_IOLog(4,"%s(%p)::CloseQueue\n", getName(), this );
	
    Queue->Start    = 0;
    Queue->End      = 0;
    Queue->NextChar = 0;
    Queue->LastChar = 0;
    Queue->Size     = 0;
	
    return kQueueNoError;
    
}/* end CloseQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::Flush
//
//      Inputs:     Queue - the queue to be flushesd
//
//      Outputs:    Queue status - queueNoError.
//
//      Desc:       Clear queue
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::flush( CirQueue *Queue )
{
    DEBUG_IOLog(4,"%s(%p)::flush\n", getName(), this );
	
    Queue->NextChar = Queue->LastChar = Queue->Start;
	
    return kQueueNoError;
    
}/* end CloseQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::AddtoQueue
//
//      Inputs:     Queue - the queue to be added to, Buffer - data to add, Size - length of data
//
//      Outputs:    BytesWritten - Number of bytes actually put in the queue.
//
//      Desc:       Add an entire buffer to the queue.
//
/****************************************************************************************************/

size_t me_nozap_driver_PL2303::addtoQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size )
{
    size_t      BytesWritten = 0;
    DEBUG_IOLog(4,"%s(%p)::AddtoQueue\n", getName(), this );
	
    while ( freeSpaceinQueue( Queue ) && (Size > BytesWritten) )
	{
#if FIX_PARITY_PROCESSING
        if(*Buffer == 0xff)
            addBytetoQueue(Queue, 0xff);
#endif
		addBytetoQueue( Queue, *Buffer++ );
		BytesWritten++;
	}
	
    return BytesWritten;
    
}/* end AddtoQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::RemovefromQueue
//
//      Inputs:     Queue - the queue to be removed from, Size - size of buffer
//
//      Outputs:    Buffer - Where to put the data, BytesReceived - Number of bytes actually put in Buffer.
//
//      Desc:       Get a buffers worth of data from the queue.
//
/****************************************************************************************************/

size_t me_nozap_driver_PL2303::removefromQueue( CirQueue *Queue, UInt8 *Buffer, size_t MaxSize )
{
    size_t      BytesReceived = 0;
    UInt8       Value;
    DEBUG_IOLog(4,"%s(%p)::RemovefromQueue\n", getName(), this );
    
    while( (MaxSize > BytesReceived) && (getBytetoQueue(Queue, &Value) == kQueueNoError) )
	{
		*Buffer++ = Value;
		BytesReceived++;
	}/* end while */
	
    return BytesReceived;
    
}/* end RemovefromQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::FreeSpaceinQueue
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    Return Value - Free space left
//
//      Desc:       Return the amount of free space left in this buffer.
//
/****************************************************************************************************/

size_t me_nozap_driver_PL2303::freeSpaceinQueue( CirQueue *Queue )
{
    size_t  retVal = 0;
    DEBUG_IOLog(6,"%s(%p)::FreeSpaceinQueue\n", getName(), this );
	
    if( !(fPort && fPort->serialRequestLock ) ) goto Fail;
	DEBUG_IOLog(6,"me_nozap_driver_PL2303::freeSpaceinQueue IOLockLock( port->serialRequestLock );\n");
    
	IOLockLock( fPort->serialRequestLock );
	
    retVal = Queue->Size - Queue->InQueue;
 	DEBUG_IOLog(6,"me_nozap_driver_PL2303::freeSpaceinQueue IOLockUnLock( port->serialRequestLock );\n");
    
    IOLockUnlock(fPort->serialRequestLock);
    
Fail:
    return retVal;
    
}/* end FreeSpaceinQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::UsedSpaceinQueue
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    UsedSpace - Amount of data in buffer
//
//      Desc:       Return the amount of data in this buffer.
//
/****************************************************************************************************/

size_t me_nozap_driver_PL2303::usedSpaceinQueue( CirQueue *Queue )
{
    DEBUG_IOLog(6,"%s(%p)::UsedSpaceinQueue\n", getName(), this );
    
    return Queue->InQueue;
    
}/* end UsedSpaceinQueue */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::GetQueueSize
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    QueueSize - The size of the queue.
//
//      Desc:       Return the total size of the queue.
//
/****************************************************************************************************/

size_t me_nozap_driver_PL2303::getQueueSize( CirQueue *Queue )
{
    DEBUG_IOLog(4,"%s(%p)::GetQueueSize\n", getName(), this );
    
    return Queue->Size;
    
}/* end GetQueueSize */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::GetQueueStatus
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    Queue status - full, empty or no error
//
//      Desc:       Returns the status of the circular queue.
//
/****************************************************************************************************/

QueueStatus me_nozap_driver_PL2303::getQueueStatus( CirQueue *Queue )
{
    if ( (Queue->NextChar == Queue->LastChar) && Queue->InQueue )
        return kQueueFull;
    else if ( (Queue->NextChar == Queue->LastChar) && !Queue->InQueue )
        return kQueueEmpty;
    
    return kQueueNoError ;
    
} /* end GetQueueStatus */

/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::CheckQueues
//
//      Inputs:     port - the port to check
//
//      Outputs:    None
//
//      Desc:       Checks the various queue's etc and manipulates the state(s) accordingly
//
/****************************************************************************************************/

void me_nozap_driver_PL2303::checkQueues( PortInfo_t *port )
{
    unsigned long   Used;
    unsigned long   Free;
    unsigned long   QueuingState;
    unsigned long   DeltaState;
    UInt32	SW_FlowControl;
    UInt32	RTS_FlowControl;
    UInt32	DTR_FlowControl;
    DEBUG_IOLog(6,"%s(%p)::CheckQueues\n", getName(), this );
	
    // Initialise the QueueState with the current state.
	
    QueuingState = readPortState( port );
    
    Used = usedSpaceinQueue( &port->TX );
    Free = freeSpaceinQueue( &port->TX );
    if ( Free == 0 )
	{
        
		QueuingState |=  PD_S_TXQ_FULL;
		QueuingState &= ~PD_S_TXQ_EMPTY;
	}
	else if ( Used == 0 )
	{
		QueuingState &= ~PD_S_TXQ_FULL;
		QueuingState |=  PD_S_TXQ_EMPTY;
	}
	else
	{
		QueuingState &= ~PD_S_TXQ_FULL;
		QueuingState &= ~PD_S_TXQ_EMPTY;
	}
    
	
	/* Check to see if we are below the low water mark. */
    if ( Used < port->TXStats.LowWater )
		QueuingState |=  PD_S_TXQ_LOW_WATER;
	else QueuingState &= ~PD_S_TXQ_LOW_WATER;
	
    if ( Used > port->TXStats.HighWater )
        
		QueuingState |= PD_S_TXQ_HIGH_WATER;
	else QueuingState &= ~PD_S_TXQ_HIGH_WATER;
    
    
	/* Check to see if there is anything in the Receive buffer. */
    Used = usedSpaceinQueue( &port->RX );
    Free = freeSpaceinQueue( &port->RX );
	
    if ( Free == 0 )
	{
		QueuingState |= PD_S_RXQ_FULL;
		QueuingState &= ~PD_S_RXQ_EMPTY;
	}
	else if ( Used == 0 )
	{
		QueuingState &= ~PD_S_RXQ_FULL;
		QueuingState |= PD_S_RXQ_EMPTY;
	}
	else
	{
		QueuingState &= ~PD_S_RXQ_FULL;
		QueuingState &= ~PD_S_RXQ_EMPTY;
	}
	
    
    
    SW_FlowControl  = port->FlowControl & PD_RS232_A_RXO;
    RTS_FlowControl = port->FlowControl & PD_RS232_A_RTS;
    DTR_FlowControl = port->FlowControl & PD_RS232_A_DTR;
	
	/* Check to see if we are below the low water mark. */
    
    if (Used < port->RXStats.LowWater)			    // if under low water mark, release any active flow control
    {
        if ((SW_FlowControl) && (port->xOffSent))	    // unblock xon/xoff flow control
        {
            DEBUG_IOLog(1,"XON AAN :(\n");
            
            port->xOffSent = false;
            addBytetoQueue(&(port->TX), port->XONchar);
            setUpTransmit( );
        }
		if (RTS_FlowControl && !port->RTSAsserted)	    // unblock RTS flow control
        {
            port->RTSAsserted = true;
            port->State |= PD_RS232_S_RFR;
        }
		if (DTR_FlowControl && !port->DTRAsserted)	    // unblock DTR flow control
        {
            port->DTRAsserted = true;
            
            port->State |= PD_RS232_S_DTR;
        }
        QueuingState |= PD_S_RXQ_LOW_WATER;
    } else {
        QueuingState &= ~PD_S_RXQ_LOW_WATER;
    }
    
	// Check to see if we are above the high water mark
    
    if (Used > port->RXStats.HighWater)			    // if over highwater mark, block w/any flow control thats enabled
    {
        if ((SW_FlowControl) && (!port->xOffSent))
        {
            DEBUG_IOLog(1,"XOFF AAN :(\n");
			
            port->xOffSent = true;
            addBytetoQueue(&(port->TX), port->XOFFchar);
            setUpTransmit( );
        }
		if (RTS_FlowControl && port->RTSAsserted)
        {
            port->RTSAsserted = false;
            port->State &= ~PD_RS232_S_RFR;			    // lower RTS to hold back more rx data
        }
        if (DTR_FlowControl && port->DTRAsserted)
        {
            port->DTRAsserted = false;
            port->State &= ~PD_RS232_S_DTR;
        }
        QueuingState |= PD_S_RXQ_HIGH_WATER;
    } else {
        QueuingState &= ~PD_S_RXQ_HIGH_WATER;
		port->aboveRxHighWater = false;
    }
    
	/* Figure out what has changed to get mask.*/
    DeltaState = QueuingState ^ readPortState( port );
    changeState( port, QueuingState, DeltaState );
	
    return;
    
}/* end CheckQueues */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::SetUpTransmit
//
//      Inputs:
//
//      Outputs:    return code - true (transmit started), false (transmission already in progress)
//
//      Desc:       Setup and then start transmisson on the channel specified
//
/****************************************************************************************************/

bool me_nozap_driver_PL2303::setUpTransmit( void )
{
    size_t      count = 0;
    size_t      data_Length = 0;
    UInt8       *TempOutBuffer;
	
	DEBUG_IOLog(2,"%s(%p)::SetUpTransmit\n", getName(), this);
    
	//  If we are already in the cycle of transmitting characters,
	//  then we do not need to do anything.
	
    if ( fPort->AreTransmitting == TRUE )
		return false;
	
	
    //if ( GetQueueStatus( &fPort->TX ) != queueEmpty )
    if (usedSpaceinQueue(&fPort->TX) > 0)
	{
		//data_Length = fIrDA->TXBufferAvailable();
        
		data_Length = MAX_BLOCK_SIZE; // remove maximum 10 bytes from queue
		if ( data_Length == 0 )
		{
			DEBUG_IOLog(4,"%s(%p)::SetUpTransmit - No space in TX buffer available\n", getName(), this);
            
			return false;
		}
		
		if ( data_Length > MAX_BLOCK_SIZE )
		{
			data_Length = MAX_BLOCK_SIZE;
		}
		
		TempOutBuffer = (UInt8*)IOMalloc( data_Length );
		if ( !TempOutBuffer )
		{
			DEBUG_IOLog(4,"%s(%p)::SetUpTransmit - buffer allocation problem\n", getName(), this);
			return false;
		}
		bzero( TempOutBuffer, data_Length );
		
		// Fill up the buffer with 1 character from the queue
		//		count = removefromQueue( &fPort->TX, TempOutBuffer, data_Length );
		// BJA Aanpassing stuurt karakter voor karakter
		count = removefromQueue( &fPort->TX, TempOutBuffer, 1 );
		
		fPort->AreTransmitting = TRUE;
		changeState( fPort, PD_S_TX_BUSY, PD_S_TX_BUSY );
		
		startTransmit(0, NULL, count, TempOutBuffer );      // do the "transmit" -- send to IrCOMM
        //BJA Dit is niet goed, we moeten dit uitzetten als we een ack hebben van de pl2303, dus datawritecomplete
        //		changeState( fPort, 0, PD_S_TX_BUSY );
        //		fPort->AreTransmitting = false;
		
		IOFree( TempOutBuffer, data_Length );
		
		// We potentially removed a bunch of stuff from the
		// queue, so see if we can free some thread(s)
		// to enqueue more stuff.
		
		checkQueues( fPort );
    }
	
    return true;
    
}/* end SetUpTransmit */


/****************************************************************************************************/
//
//      Method:     me_nozap_driver_PL2303::setControlLines
//
//      Inputs:     the Port and state
//
//      Outputs:    IOReturn
//
//      Desc:       set control lines of the serial port ( DTR and RTS )
//
/****************************************************************************************************/
IOReturn me_nozap_driver_PL2303::setControlLines( PortInfo_t *port ){
	UInt32 state = port->State;
	IOReturn rtn;
	IOUSBDevRequest request;
    
    DEBUG_IOLog(4,"%s(%p)::setControlLines state %p \n", getName(), this, state );
	
    UInt8 value=0;
    
    if (state & PD_RS232_S_DTR)  { value |= kCONTROL_DTR;
	    DEBUG_IOLog(5,"setControlLines DTR ON \n" );
    }
    if (state & PD_RS232_S_RFR)  {value |= kCONTROL_RTS;
        DEBUG_IOLog(5,"setControlLines RTS ON \n" );
    }
	
    
	request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    request.bRequest = SET_CONTROL_REQUEST;
	request.wValue =  value;
	request.wIndex = 0;
	request.wLength = 0;
	request.pData = NULL;
	rtn =  fpDevice->DeviceRequest(&request);
	DEBUG_IOLog(4,"%s(%p)::setControlLines - return: %p \n", getName(), this,  rtn);
	
	return rtn;
}/* end setControlLines */



// generateRxQState() : Called to generate the status bits for queue control.
// This routine should be called any time an enqueue/dequeue boundary is crossed
// or any of the queue level variables are changed by the user.
// WARNING: {BIGGEST_EVENT ≤ LowWater ≤ (HighWater-BIGGEST_EVENT)} and
//	{(LowWater-BIGGEST_EVENT) ≤ HighWater ≤ (size-BIGGEST_EVENT)} must be enforced.


UInt32 me_nozap_driver_PL2303::generateRxQState( PortInfo_t *port )
{
    IOLog("%s(%p)::generateRxQState\n", getName(), this );
    
    UInt32 state = port->State & (kRxAutoFlow | kTxAutoFlow);
    UInt32 fifostate = port->State & ( kRxQueueState );
    state = maskMux(state, (UInt32)fifostate >> PD_S_RX_OFFSET, PD_S_RXQ_MASK);
    switch (fifostate) {
        case (PD_S_RXQ_EMPTY | PD_S_RXQ_LOW_WATER) :
        case PD_S_RXQ_LOW_WATER :
            if ( port->FlowControl & PD_RS232_A_RFR) {
                state |= PD_RS232_S_RFR;
            } else if ( port->FlowControl & PD_RS232_A_RXO) {
                state |= PD_RS232_S_RXO;
                switch ( port->RXOstate ) {
                    case kXOffSent :
                    case kXO_Idle :	port->RXOstate=kXOnNeeded;	break;
                    case kXOffNeeded :	port->RXOstate=kXOnSent;	break;
                    default :		break;
                }                    
            } else if ( port->FlowControl & PD_RS232_A_DTR) {
                state |= PD_RS232_S_DTR;
                
            }
            break;
        case PD_S_RXQ_HIGH_WATER :
        case (PD_S_RXQ_HIGH_WATER | PD_S_RXQ_FULL) :
            if ( port->FlowControl & PD_RS232_A_RFR) {
                state &= ~PD_RS232_S_RFR;
            } else if ( port->FlowControl & PD_RS232_A_RXO) {
                state &= ~PD_RS232_S_RXO;
                switch ( port->RXOstate ) {
                    case kXOnSent :
                    case kXO_Idle :	port->RXOstate=kXOffNeeded;	break;
                    case kXOnNeeded :	port->RXOstate=kXOffSent;	break;
                    default :		break;
                }
            } else if ( port->FlowControl & PD_RS232_A_DTR) {
                state &= ~PD_RS232_S_DTR;
            }
            break;
        case 0 : break;
    }
    
    return state;
}

/****************************************************************************************************/
//
//		Function:	SetBreak
//
//		Inputs:		Channel - The port
//				break - true(send break), false(clear break)
//
//		Outputs:	
//
//		Desc:		Set and clear line break.
//
/****************************************************************************************************/

IOReturn me_nozap_driver_PL2303::setBreak( bool data){
	UInt16 value;
	IOReturn rtn;
	IOUSBDevRequest request;
	
    
	DEBUG_IOLog(4,"%s(%p)::setBreak - data: %p \n", getName(), this,  data);
    
	if (data == 0)
		value = BREAK_OFF;
	else
		value = BREAK_ON;
    
	request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    request.bRequest = BREAK_REQUEST;
	request.wValue =  value; 
	request.wIndex = 0;
	request.wLength = 0;
	request.pData = NULL;
    
	rtn =  fpDevice->DeviceRequest(&request);
	DEBUG_IOLog(4,"%s(%p)::setBreak - return: %p \n", getName(), this,  rtn);
	return rtn;
}
