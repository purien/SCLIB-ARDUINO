/*
  SCLib4c.h - Smart Card library
  Copyright (C) 2022 Pascal Urien . All right reserved.

  This library is a modified version of
  Smart Card library Copyright (c) 2012 Frank Bargstedt.
  
*/

#ifndef SCLib4c_h
#define SCLib4c_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include "WProgram.h"
#endif


//#define C_PLATFORM 3

#if defined(C_PLATFORM)
#else

#if defined (ARDUINO_ARCH_STM32F1) 
#define C_PLATFORM 4
#endif

#if defined(__AVR_ATmega256__)
#define C_PLATFORM 3
#endif

#if defined(C_PLATFORM)
#else
#define C_PLATFORM 1
#endif

#if   C_PLATFORM == 1  //STD  CLK=TIMER1 Time=TIMER0, reso=4us - jitter = 10us

#elif C_PLATFORM == 2  //ATMEGA256
#define ATMEGA256 
#define TIMERX5
// CLK= TIMER1 or CLK= TIMER5 if UseTimer5= true

#elif C_PLATFORM == 3  //ATMEGA256 + TIMER4 (16 bits)
#define C_ATMEGA256    //ATMEGA256 
#define TIMERX5
// CLK= TIMER1 or CLK= TIMER5 if UseTimer5= true
#define  TIMERX
#define  TIMERX4

#elif C_PLATFORM == 4 
#define C_STM32
#include "HardwareTimer.h"
#define  TIMERX
#define  C_STM32_TIMERX    1
#define  C_STM32_PRESCALE  9
#define  C_STM32_TIMER_NUM 3
#define  ASTM32


#endif
#endif

#ifdef TIMERX

#ifdef  TIMERX4
#define MICROS() (((unsigned long)TCNT4))
#define ResetTIMERX() TCNT4=0
#define GetTIMERX() (unsigned long)((unsigned long)0xFFFF&(unsigned long)TCNT4)
#define  FTX 16
#define _ETU _Setu
#endif

#ifdef  TIMERX2
#define MICROS() (((unsigned long)TCNT2))
#define ResetTIMERX() TCNT2=0
#define GetTIMERX() (unsigned long)((unsigned long)0xFF&(unsigned long)TCNT2)
#define _ETU _Setu2
#define FTX 2
#define 
#endif

#ifdef   ASTM32
#define  MICROS()      (unsigned long)timer->getCount()
#define  ResetTIMERX() timer->pause();timer->refresh();timer->resume()
#define  GetTIMERX()   (unsigned long)((unsigned long)0xFFFF&(unsigned long)timer->getCount())
#define  FTX    8
#define _ETU _Setu
#endif

#else
#define  MICROS() micros()
#define _ETU _etu
#define FTX 1
#endif 


// Currently if you comment out the SC_DEBUG define, the dumpHEX function
// will not be available anymore to reduce the size of the sclib binary
#define SC_DEBUG

// Uncomment this to activate debug pin toggle on DEFAULT_DEBUG_PIN (Normally 13)
// #define USE_DEBUG_PIN
#define DEFAULT_DEBUG_PIN 13
// https://www.arduino.cc/en/tutorial/blink


//
// NO NEED TO CHANGE ANY DEFINES / CODE BELOW THIS LINE, UNLESS YOU KNOW WHAT YOU ARE DOING ;-.)
//


// CLK Values f >= 1MHz, leads to asynchronous transmission, otherwise synchronous (CLK_10KHZ)
typedef enum {
  CLK_1MHZ,
  CLK_2MHZ,
  CLK_2DOT5MHZ,
  CLK_4MHZ,
  CLK_NO_CLK,
} frequency_t;

// Callback func pointer
typedef void (*SCLibVoidFuncPtr)(void);

// Default Config
#define DEFAULT_CLK_PIN              9
#define DEFAULT_ETU                372

// Maximum BYTES in ATR
#define MAX_ATR_BYTES 32



#define TTB 1000 // tb  >= 400/f
#define TTC 100  // tc >= 400/f
// Maximum time of etu's to wait
#define MAX_WAIT_TIME    40000

// APDU Command struct

typedef struct {
  /* CLASS byte of command */
  uint8_t  cla;
  /* INS byte of command */
  uint8_t  ins;
  /* Parameter 1 of command */
  uint8_t  p1;
  /* Parameter 2 of command */
  uint8_t  p2;
  /* pointer to data buffer containing "data_size" bytes 
     (Can be NULL to signal no data required) */
  uint8_t* data_buf;
  /* number of bytes in data_buf is ignored, if data_buf is NULL. 
     Can also be 0 to signal no data is required. */
  uint16_t data_size;
  /* max. number of bytes for response size */
  uint16_t resp_size;
  uint16_t resp_length;

} APDU_t;


class SmartCardReader4c
{
  public:
    // Constructors to define communication pins
    /**
     * Creates a basic smart card reader object, with mandatory
     * pins assigned to establish smart card communication.
     *
     * c7_io               - IO line, connected to Card (C7)
     * c2_rst              - RST for Card (C2)
     * c1_vcc              - Vcc for Card (C1)
     * c3_clk              - clk signal to SC (C3) - (timer1 / pin9 used)
     * card_present_invert - Use inverted off signal (LOW Card available - HIGH Card removed)
     */
    SmartCardReader4c(uint8_t c7_io, uint8_t c2_rst, uint8_t c1_vcc,uint8_t c3_clk);
    
    /**
     * Is card inserted?
     *
     * return true if card is inserted, otherwise false.
     */
    boolean cardInserted();
    
    /**
     * Timeout occured, with last communication command?
     *
     * return true if timeout occured, otherwise false.
     */
    boolean timeout();
    
    /**
     * Start activation sequence and return ATR result in buf (buf should be at least 32 Bytes to be able to
     * receive maximum ATR response.
     *
     * return number of received bytes, or 0 if error occured (timeout etc.) 
     */
    uint16_t activate(uint8_t* buf, uint16_t buf_size);
    
    /**
     * Start deactivation sequence
     */
    void deactivate();
    
    /**
     * Get current ETU
     */
    uint16_t getCurrentETU();
    
    /**
     * Change guard time in micro seconds (Should be at least 9600)
     */
    void setGuardTime(unsigned long t);
    
    /**
     * Don't care about parity errors, as some cards screw it up on purpose as they use odd parity ;-)
     */
    void ignoreParityErrors(boolean in);
    
    /**
     * Set callback, to be noticed if time out occurs ...
     */
    void setTimeOutCallback(SCLibVoidFuncPtr cb);

	int fclk  = 2000;
	int lasterror=0;
	int pbufs = 0   ;
    uint8_t bufs[32];
	boolean use_ts=false;
	int fscale=1;
	
    uint16_t _WarmReset(uint8_t* buf, uint16_t buf_size);

 


    /**
     * Include TS byte in ATR (Only valid for ASync Cards)
     */
    void setIncludeTSinATR(boolean include);
    
  
    /**
     * Used to caculate ADPU crc value
     *
     * startValue should normaly be 0, only if sequend calls on a data structure are made the curent value 
     *            is used as input for the calculation of the subsequend call.
     * buf        pointer to the APDU command
     * size       number of bytes used for EDC calculation
     *
     * returns the current ECU value.
     */
    uint8_t calcEDC(uint8_t startValue, uint8_t *buf, uint16_t size);
    /**
     * Used to send APDU to smart card.
     *
     * command pointer to APDU command structure
     *
     * send if true data will be send, from command->data_buf (command->data_size bytes)
     *      to smartcard, otherwise data will be received and put into command->data_buf.
     *
     * maxwaits number of additional wait cycles sclib will wait for the card to respond.
     *          maxwaits shoud not exceed ~5 (If any value higher than 1~2 is needed there
     *          might other problem, with the communication/timing) 
     *
     * returns SW1/SW2 byte or 0, if an error occured. 
     */
    uint16_t sendAPDU(APDU_t* command, boolean send=true, uint16_t maxwaits=0); // CJT added maxwaits


    /**
     * Send data to smartcard
     *
     * Return true, if the receiver signaled a parity error
     */
    boolean sendBytes(uint8_t* buf, uint16_t count);
    boolean sendBytesStd(uint8_t* buf, uint16_t count);
    
	boolean _sendByteStd_T1(uint8_t out);
    boolean _sendByte_T1(uint8_t out);
	boolean _sendASyncBytes_T1(uint8_t* buf, uint16_t count);
    boolean _sendASyncBytesStd_T1(uint8_t* buf, uint16_t count);


    /**
     * Receive data from SmartCard until buffer full or timeout
     *
     * Return number of received bytes
     */    
    uint16_t receiveBytes(uint8_t* buf, uint16_t buf_size);
    uint16_t receiveBytesStd(uint8_t* buf, uint16_t buf_size);


    int  _receiveByte_T1(uint8_t* buf, unsigned long timeout);
    bool _receiveDataBits_T1(uint8_t* buf, unsigned long startTime, uint8_t count);
    int  _receiveByteStd_T1(uint8_t* buf, unsigned long timeout);
    bool _receiveDataBitsStd_T1(uint8_t* buf, unsigned long startTime, uint8_t count);

    uint16_t _receiveASyncBytes_T1(uint8_t* buf, uint16_t buf_size,unsigned long bwt,unsigned long cwt);
    uint16_t _receiveASyncBytesStd_T1(uint8_t* buf, uint16_t buf_size,unsigned long bwt,unsigned long cwt);


    #if defined(SC_DEBUG)
    // Just some debug function 
    void dumpHEX(uint8_t* values, uint16_t size);
    #endif

    #ifdef C_STM32
    HardwareTimer * pwmtimer = NULL; //new HardwareTimer((uint8_t) 3);
    HardwareTimer * timer    = NULL; 
    #endif


  //private:
    

    // Internal receive functions
    int  _receiveByte(uint8_t* buf, unsigned long timeout);
    int  _receiveByteStd(uint8_t* buf, unsigned long timeout);
    uint8_t _receiveTSByte();
    bool _receiveDataBits(uint8_t* buf, unsigned long startTime, uint8_t count=8);
    bool _receiveDataBitsStd(uint8_t* buf, unsigned long startTime, uint8_t count=8);
    boolean _sendByte(uint8_t out);
    boolean _sendByteStd(uint8_t out);
 
    void _activateHW();

    /**
     * Init PINs and start CLK, if defined
     */
    void _init(frequency_t freq);

    void _timeOutoccured();
    
    #ifdef TIMERX
    void _stopTIMERX();
    void _startTIMERX();
    #endif

 

    /**
     * Send bytes in async mode
     */
    boolean _sendASyncBytes(uint8_t* buf, uint16_t buf_size);
    boolean _sendASyncBytesStd(uint8_t* buf, uint16_t buf_size);

    /**
     * Receive bytes is async mode
     */
    uint16_t _receiveASyncBytes(uint8_t* buf, uint16_t buf_size)   ;
    uint16_t _receiveASyncBytesStd(uint8_t* buf, uint16_t buf_size);

    /**
     * Activate asynchron card
     */
    uint16_t _activateASynchronCard(uint8_t* buf, uint16_t buf_size);

    /**
     * member to hold include TS in ATR value
     */
    boolean _includeTSinATR;


    #if defined(USE_DEBUG_PIN)
    // Toggle debug pin
    void _toggleDebugPin();
    #endif
    
    // PIN configuration
    uint8_t _io_in_pin;  // (INPUT/OUTPUT) : I/O pin to communication with Smart Card
    uint8_t _rstin_pin;  // (OUTPUT)       : LOW: reset active.  HIGH: inactive.
    uint8_t _cmdvcc_pin; // (OUTPUT)       : LOW: Activation sequence initiated HIGH: No activation in progress
    uint8_t _clk_pin;    // (OUTPUT)       : CLK signal for Smart Card (NOT_A_PIN, when no CLK generation)

    /////////////////////////   
	boolean UseTimer5= false;
	/////////////////////////

    // Internal configuration
    boolean _ignoreParity;   // We don't care about screwd up parity from the smartcard
	frequency_t _clkFrequency; // See CLK_* defines
    uint16_t _etu;
    uint16_t _Setu;
    uint16_t _Setu2;
    uint16_t _initial_etu;
    uint8_t  _ocra1;
    SCLibVoidFuncPtr _timeOutCB;
    // Some timing constants
    unsigned long _guardTime;
    // Work Waiting Time (960 * D * Wi) etus
    unsigned long _wwt;
    // Maximum Waiting time (WWT + (D * 480)) etus
    unsigned long _max_wwt;
	// T=1 CWT
	unsigned long _cwt;
	// T=1 BWT
	unsigned long _bwt;
    // T=1 BGT
    unsigned long _bgt;
	// T=1 CRC
    boolean _crc=false;
    
	unsigned long tb= TTB;
    unsigned long tc= TTC;
	unsigned long tmwt= MAX_WAIT_TIME;
	

    
    // Internal state vars
    boolean  _activated;
    boolean  _high_active;
    boolean  _timeout;
   
};

#endif
