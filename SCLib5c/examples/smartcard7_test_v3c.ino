#include <Arduino.h>
#include <EEPROM.h>
#include <SCLib5c.h>

// https://forum.arduino.cc/t/need-help-uploading-sketch-to-leonardo-pro-micro-board-without-bootloader/650579/8
// C:\arduino-1.8.9\hardware\arduino\avr\board.txt

////////////////////
// #define MINIPRO
// #define ATMEGA2560
#define LEONARDO
////////////////////

bool auto_p = true;
#define ATRVERBOSE


#define MINIPRO_CLK 16;//8 ou 16MHZ

#define  LED 5 //13
#define  BUTTON 4

#define STD

#define FREE_MEM_SIZE 0

#define SIZE_APDU_BUF 261
#define SIZE_RESP_BUF 258+6 
#define SIZE_BUF0     220
#define SIZE_BUF1     SIZE_APDU_BUF+SIZE_RESP_BUF

bool    mycrc= false;

bool    fwarm=false;
uint8_t last_send_sequence = 0    ;
uint8_t last_received_sequence = 0;
uint8_t finject=0;
uint8_t t1_error=0;
#define T1_ERROR_MAX 1
bool t1_parity_error=false;
uint8_t lastpcb=0;

typedef struct
{
        /* Prologue field (NAD, PCB, LEN) */
        uint8_t nad;
        uint8_t pcb;
        uint8_t len;
        /* Epilogue field (EDC) */
        union {
                uint8_t  edc_lrc;
                uint16_t edc_crc;
        };
        uint8_t edc_type; /* Type of used EDC */
        /* Information field (APDU) is handled by the lower layer */
        uint8_t *data;
} SC_TPDU;



#ifdef STD
int    MyF = 4000  ;
bool   mypts=true  ;
uint8_t myptcol=1   ;
uint8_t myifs= (uint8_t)254 ;
char   MyTA = (char)0x11; 
#define MYBAUD 19200
 // TA1=(char)0x11, F=372;  D=1;
 // TA1=(char)0x12; F=372;  D=2;
 // TA1=(char)0x13; F=372;  D=4;
 // TA1=(char)0x14; F=372;  D=8 ;
 #else
int     MyF =  4000  ;
bool    mypts= false ;
uint8_t myptcol=0    ;
uint8_t myifs= (uint8_t)254 ;
char   MyTA = (char)0x11; //(char)0x12; (char)0x11;
//#define MYBAUD 115200
#define MYBAUD 19200
 // TA1=(char)0x11, F=372;  D=1;
 // TA1=(char)0x12; F=372;  D=2;
 // TA1=(char)0x13; F=372;  D=4;
 // TA1=(char)0x14; F=372;  D=8 ;
 #endif

bool   fdebug=false;//true;//false;
#ifdef MINIPRO
int    base = 1024;
#elif defined(LEONARDO)
int base = 1536;
#else
int base= 1024;
#endif


boolean flocal=true;//false; false= mode X de reception

#define SRMAX 32
char * toram(PROGMEM char * s)
{ static char sr[SRMAX];
  sr[0]=0;
  if (strlen_P(s) < SRMAX)
  strcpy_P(sr,s);
  return sr;
}

#define NPERM 1
#define NPERM2 2*NPERM

#define EEPROM_SIZE  1024
#define REG_SIZE      256
#ifdef MINIPRO
#define SRAM_SIZE    2048
#else
#define SRAM_SIZE    2560
#endif
#define FLASH_SIZE  32768

//LEONARDO has 2,5KB SRAM=> 35840 + 512= 36352
// 35840 = (32+2+1) *1024
// 33792 + 220 + 800 = 34812

//36467, 36887

#ifdef MINIPRO
#define PRIME   35879  // SAFE PRIME
#define Q       17939 //  SOPHIE GERMAIN PRIME PRIME=2Q+1
#else
#define PRIME   36887   // SAFE PRIME
#define Q       18443  //  SOPHIE GERMAIN PRIME PRIME=2Q+1
#endif

// PRIME mod 8 = 7
// in Z/pZ*, q-1 generators, g= p - ((2**k) mod p), k from 1 to q-1

// number of bits for generators
#define NBITS 16
// Address Range  16 bits
#define USHORT uint16_t 
// Computing Range 2x16 = 32bits
#define ULONG  uint32_t

/*
 * keccak code
 * From https://github.com/brainhub/SHA3IUF/, Aug 2015. Andrey Jivsov. crypto@brainhub.org
 * Based on code from http://keccak.noekeon.org/ .
 * Adapted to AVR by Pascal Urien
 */

/* 'Words' here refers to uint64_t   200 bytes */
#define SHA3_KECCAK_SPONGE_WORDS \
  (((1600)/8/*bits to byte*/)/sizeof(uint64_t))

typedef struct sha3_context_ {
  uint64_t saved;             /* the portion of the input message that we didn't consume yet */
  union 
  {  /* Keccak's state */
    uint64_t s[SHA3_KECCAK_SPONGE_WORDS];
    uint8_t sb[SHA3_KECCAK_SPONGE_WORDS * 8]; // => digest 200 octets
  };
  uint32_t  byteIndex;         /* 0..7--the next byte after the set one
                                   (starts from 0; 0--none are buffered) */
  uint32_t wordIndex;         /* 0..24--the next word to integrate input
                                   (starts from 0) */
  uint32_t capacityWords;     /* the double size of the hash output in
                                   words (e.g. 16 for Keccak 512) */
} sha3_context;

typedef union hdata_ {
  sha3_context sha3; // 220 bytes
  struct
  { char prefix[8]  ;
    char digest[32 ];
    char mybuf[168] ;
    uint16_t code[6];
  } buf; // 220 bytes
  char mybuf[SIZE_BUF1];
} hdata;


/* generally called after SHA3_KECCAK_SPONGE_WORDS-ctx->capacityWords words
   are XORed into the state s
*/
void keccakf(uint64_t s[25]);
void sha3_Init256(sha3_context * priv);
void sha3_Update(sha3_context *priv, void const *bufIn, size_t len);
void const * sha3_Finalize(sha3_context * priv);


union    
{
  uint32_t v32    ;
  uint16_t v16[2] ;
  uint8_t  v8[4]  ;
} time1;


uint32_t gettime1()
{ uint16_t v16 ;

  v16 = TCNT1  ;

  if (v16 < time1.v16[0])
    time1.v16[1] += 1 ;

  time1.v16[0] = v16;

  return time1.v32;
}


void delay1(uint32_t adelay)
{
  if (adelay == 0)
  {
    time1.v32= 0 ;
    TCCR1A  = 0  ;
    TCCR1B  = 0  ;
    TCNT1   = 0xFFF0  ; // wait 16*4=64 us
    TCCR1B |=  3 ; // prescale 64

    while (TCNT1 != 0);
    return ;
  }

  uint32_t v32 = gettime1();

  while (true)
  {
    uint32_t t32 = gettime1() ;
    if ((t32 - v32) >= adelay)
      break;
  }
}

typedef  union  ui16_
{ uint16_t v  ;
  uint8_t b[2];
} ui16 ;


typedef  union  ui32_
{ uint32_t v   ;
  uint8_t b[4] ;
} ui32 ;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

uint16_t getstack(void)
{ char c;
  return (uint16_t)&c ;
}

extern int  __heap_start;

//#define V1
#define V2


/*
?
A 00A4040006010203040500
A 00200001083030303030303030
A 008700000A

A 0087000004

A 0081000000
A 0089000000
A 0082000000


A 008700000A
A 0081000300
A 0089000300
A 0084060343
A 00800003201234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF
A 00C0000048
S 00B0000000
A 0084060043
A 00800000201234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF

A 00D00000F0A55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55A
A 00B00000F0
*/

/*
 Not all pins on the Mega and Mega 2560 boards support change interrupts, 
 so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 
 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68),A15 (69). 
 Not all pins on the Leonardo and Micro boards support change interrupts, 
 so only the following can be used for RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 */

#ifdef LEONARDO
#include <SoftwareSerial.h>
// SoftwareSerial mySerial (rxPin, txPin); //rx=MOSI tx=MISO
// SoftwareSerial myserial2(16,14); // 16=RX=MOSI, 14=TX=MISO
SoftwareSerial myseriale(10,A0); // 51=RX=MOSI, 50=TX=MISO
#define MySerial2 myseriale
#define MySerial  Serial
//#define MySerial  myserial2
#else
#define MySerial2 Serial
#define MySerial  Serial
#endif

char histlen=0;
char histbytes[15];



// If you are using a Arduino Mega compatible board you need to change the SC_C2_CLK to 11 as the TIMER1A
// is used for asynchronous clock generation (1MHz with just plain arduino code is no fun ;-) )
// and the SC_C1_VCC can be changed to any other "free" digital pin

#ifdef ATMEGA2560
#define SC_C7_IO     26 // 39// 42 // 38
#define SC_C2_RST    24 // 41 //40
#define SC_C1_VCC    22 // 43 //5   // 42
#define SC_C2_CLK    11  // timer 1 
//#define SC_C2_CLK    46  // Timer 5 
#else


#ifdef MINIPRO
#define SC_C2_RST    7  // 14   // D10  // 38  //  Vert
#define SC_C1_VCC    6  // 15   // D11  // 40  //  Rouge/Orange 
#define SC_C7_IO     8   // 12   //  D8  // 36  //  Jaune 
#define SC_C2_CLK    9   // 13   //  D9  // 11  //  Marron // OC1A   NANO=> D9 p13
#elif defined(LEONARDO)
#define SC_C2_RST    7  // 14   // D10  // 38  //  Vert
#define SC_C1_VCC    6  // 15   // D11  // 40  //  Rouge/Orange 
#define SC_C7_IO     8   // 12   //  D8  // 36  //  Jaune 
#define SC_C2_CLK    9   // 13   //  D9  // 11  //  Marron // OC1A   NANO=> D9 p13
#else
#define SC_C2_RST    10  // 14   // D10  // 38  //  Vert
#define SC_C1_VCC    11  // 15   // D11  // 40  //  Rouge/Orange 
#define SC_C7_IO     8   // 12   //  D8  // 36  //  Jaune 
#define SC_C2_CLK    9   // 13   //  D9  // 11  //  Marron // OC1A   NANO=> D9 p13
#endif
#endif
 
// Create SmartCardReader object for further use
SmartCardReader5c sc(SC_C7_IO, SC_C2_RST, SC_C1_VCC,SC_C2_CLK);


void reboot(int reason)
{
  noInterrupts();
  asm // HOT RESTART
  (  // status register
    "jmp __dtors_end \n\t"
   );
   
}


//char   apdu_buf[260],resp_buf[258] ;
char *apdu_buf= NULL; // (char *)&__heap_start ;
char *resp_buf= NULL; //((char *)&__heap_start) + SIZE_APDU_BUF ;
hdata cs  ; // 220 octets
char *  MyBuf1= NULL;// apdu_buf ;
char *  MyBuf0= NULL;

const char* cOK="OK";
const char* cERROR="!";
const char* cnok="ERROR";
bool tmode=false;

bool ablink(int bmax)
{ int ct=0;

 while(ct < bmax)
 { digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
    ct++;
    int val = digitalRead(BUTTON);
    if (val == 0)
    { return true;
      break;
    }
  }
  return false;    
}

void setup() 
{ int ct=0;
  MySerial.begin(MYBAUD);
  #ifdef LEONARDO
  MySerial2.begin(MYBAUD);
  #endif
 
  sc.fscale= 1;
  #ifdef MINIPRO
  sc.fscale= 16/ MINIPRO_CLK;
  #endif

  pinMode(LED, OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);

  while(ct < 2)
  { digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
    ct++;
    int val = digitalRead(BUTTON);
    if (val == 0)
    { digitalWrite(LED, HIGH);
      tmode=true;
      break;
    }
    
  }
 
  
#ifdef ATMEGA2560
MySerial.println("Atmega2560");
#endif
  
  apdu_buf=  cs.mybuf ;
  resp_buf=  apdu_buf+ + SIZE_APDU_BUF ;
  MyBuf1=    apdu_buf   ;
  MyBuf0=    (char *)&cs;
  
  if (fdebug) 
  { MySerial.print(toram(PSTR("csSize= ")));MySerial.println(sizeof(cs));
    MySerial.print(toram(PSTR("@apdu_buf= ")));MySerial.println((int)apdu_buf);
    MySerial.print(toram(PSTR("@resp_buf= ")));MySerial.println((int)resp_buf);
    MySerial.print(toram(PSTR("@myBuf0= ")));MySerial.println((int)MyBuf0);
    MySerial.print(toram(PSTR("@myBuf1= ")));MySerial.println((int)MyBuf1);
    MySerial.print(toram(PSTR("@base= ")));MySerial.println(base);
    MySerial.print(toram(PSTR("@heap= ")));MySerial.println((int)&__heap_start); 
    MySerial.print(toram(PSTR("@stack= ")));MySerial.println(getstack());
   }
  

  if (fdebug) CheckSC(mypts,fwarm) ;
  sc.deactivate();

 
}

const char SelectAid[]   = {(char)0x00, (char)0xA4, (char)0x04, (char)0x00, (char)0x06, (char)0x01, (char)0x02, (char)0x03, (char)0x04, (char)0x05, (char)0x00};
//char VerifyPinUser[]     = {(char)0x00, (char)0x20, (char)0x00, (char)0x00, (char)0x04, (char)0x30, (char)0x30, (char)0x30, (char)0x30};

bool f_adm = false,f_user = false, f_user2 = false;
bool smartcard_on = false;

bool startse(bool mode)
{ int err;
  bool stat;

  smartcard_on = false;
  f_adm = f_user = f_user2 = false;

  stat = CheckSC(mypts,fwarm) ;

  if (!stat) return false;

  err = send_apdu((char *)SelectAid, sizeof(SelectAid), (char *)resp_buf,false,true);
  if (err != 2)
    return false;
  if ( (resp_buf[0] != (char)0x90) || (resp_buf[1] != (char)0x00) )
    return false;

  smartcard_on = true;
  f_adm = f_user = f_user2 = false;

  return true;
}

bool stopse()
{ sc.deactivate();

  smartcard_on = false;
  f_adm = f_user = f_user2 = false;

  return true;
}



void loop()
{ char *cmd= NULL ;
  char *opt= NULL,v[3],c[1];
  int i,err,pt;
  uint16_t  s_end = 0  ;
  char *MyBuf= cs.mybuf;
  boolean fserial2=false;
  
  int nb= readBuffer(MyBuf,SIZE_BUF1,'\n');
  #ifdef LEONARDO
  if (nb <= 0) 
  { nb= readBuffer2(MyBuf,SIZE_BUF1,'\n');
    if (nb <= 0) return ;
    fserial2=true;
  }
  #else
  if (nb <= 0) return ;
  #endif
  
  cmd = strtok(MyBuf,toram(PSTR(" \r\n")));
  
  if (cmd == NULL) 
  { if (fserial2) MySerial2.println(toram(PSTR("Ready"))); 
    else          MySerial.println(toram(PSTR("Ready"))); 
    return;
  }
  
  opt = strtok(NULL,toram(PSTR(" \r\n")));
  pt =  strlen(cmd);

  if       ((pt==1)&&(cmd[0]=='h')) 
  { if (fserial2) MySerial2.println((int)&__heap_start); else MySerial.println((int)&__heap_start);
    return;
  }
  else if  ((pt==1)&&(cmd[0]=='s')) 
  { if (fserial2) MySerial2.println((int)&s_end); else MySerial.println((int)&s_end);
    return;
  }
  else if  ((pt==1)&&(cmd[0]=='f')) 
  { dump_full(MyBuf,fserial2);
    return;
  }
  else if  (cmd[0]=='m')
  {  uint32_t x = atoi(cmd+1);
     dump_ram(MyBuf,x,fserial2);
     return;
  }

  else if (strcmp_P(cmd,PSTR("user")) == 0)
  { 
    if ((opt == NULL) || (strlen(opt) != 4))
    { MySerial.println(cnok);
      return;
    }
    
    stopse();
    digitalWrite(LED, LOW);
    
    if (!smartcard_on)
    { if (!startse(false)) 
      { MySerial.println(cnok);
        return;
      }
    }
        
    apdu_buf[0]=0;apdu_buf[1]=0x20;apdu_buf[2]=0;apdu_buf[3]=0;apdu_buf[4]=4;
    for (i = 0; i < 4; i++) apdu_buf[5 + i] = opt[i];
    err = send_apdu(apdu_buf,9,(char*)resp_buf,false,true);

    if ( (err != 2) || ( (resp_buf[0] != (char)0x90) || (resp_buf[1] != (char)0x00)) )
    { MySerial.println(cnok);
      stopse();
      return;
    }

    digitalWrite(LED, HIGH);
    f_user = true;
    smartcard_on = true;
  
    MySerial.println(cOK);
    return;
  }

    else if (strcmp_P(cmd,PSTR("on"))==0)
   { sc.deactivate();
     stopse();
     if (opt != NULL)
     {
      err= Ascii2bin(opt);
      if (err != 1)  { MySerial.println(cERROR); return;}
      MyTA= *opt ;
      mypts=true ;
     }
    
     if (CheckSC(mypts,fwarm)) { smartcard_on=true;MySerial.println(cOK); digitalWrite(LED, HIGH);return;}
     else                { smartcard_on=false;MySerial.println(cERROR);  digitalWrite(LED, LOW);return;}
     
   }
   
   else if  (strcmp_P(cmd,PSTR("off"))==0)
   {  sc.deactivate();
      stopse();
      digitalWrite(LED, LOW);
      MySerial.println(cOK);
      return;
   }

  else if ( (strcmp_P(cmd, PSTR("binder")) == 0) ||  (strcmp_P(cmd, PSTR("derive"))==0) )
  { bool fbind=true;
    
    if (strcmp_P(cmd, PSTR("derive"))==0) 
    { fbind=false;
      if (!ablink(50))
      { MySerial.println(cnok);
        return;
      }
    }

     if (opt == NULL) 
     { MySerial.println(cnok);
       return;
     }
     nb = Ascii2bin(opt);
     if ( (nb <= 0) || (nb > 32) ) 
     { MySerial.println(cnok);
       return;
     }

     strcpy(resp_buf,opt);

     for (i = 0; i < (32 - nb); i++)
     apdu_buf[5 + i] = 0;
     for (i = 0; i < nb; i++)
     apdu_buf[5 + i + (32 - nb)] = resp_buf[i];

      apdu_buf[0]= 0x00; apdu_buf[1]= 0x85;
      if (fbind)
      { apdu_buf[2]= 0x00;apdu_buf[3]= 0x0C;} // IM_Finished
      else       
      { apdu_buf[2]= 0x00;apdu_buf[3]= 0x0E;} // IM_Derived
      apdu_buf[4]= 32;
      
      err = send_apdu((char *)apdu_buf,5+32,(char *)resp_buf,false,true);

      if ( (err != 34) || (resp_buf[err - 2] != (char)0x90) || (resp_buf[err - 1] != (char)0x00) )
      { MySerial.println(cnok);
        return;
      }

      Bin2ascii(resp_buf, err - 2 , apdu_buf);
      
      MySerial.println(apdu_buf);
      return;

  }

  else if (strcmp_P(cmd, PSTR("getpk")) == 0)
  {  
      apdu_buf[0]= (byte)0x00; apdu_buf[1]= (byte)0x84;
      apdu_buf[2]= (byte)0x06; apdu_buf[3]= (byte)0xFE;
      apdu_buf[4]= (byte)0x43;
      
      err = send_apdu((char *)apdu_buf,5,(char *)resp_buf,false,true);

      if ( (err != 69) || (resp_buf[err - 2] != (char)0x90) || (resp_buf[err - 1] != (char)0x00) )
      { MySerial.println(cnok);
        return;
      }
      Bin2ascii(resp_buf, err - 2 , apdu_buf);
      
      MySerial.println(apdu_buf);
      return;

  }

  else if (strcmp_P(cmd, PSTR("getcert")) == 0)
  {  
      apdu_buf[0]= (byte)0x00; apdu_buf[1]= (byte)0x8E;
      apdu_buf[2]= (byte)0x00; apdu_buf[3]= (byte)0x00;
      apdu_buf[4]= (byte)0x40;
      
      err = send_apdu((char *)apdu_buf,5,(char *)resp_buf,false,true);

      if ( (err != 66) || (resp_buf[err - 2] != (char)0x90) || (resp_buf[err - 1] != (char)0x00) )
      { MySerial.println(cnok);
        return;
      }
      Bin2ascii(resp_buf, err - 2 , apdu_buf);
      
      MySerial.println(apdu_buf);
      return;

  }

  else if (strcmp_P(cmd, PSTR("auth")) == 0) 
  { 
     if (opt == NULL) 
     { MySerial.println(cnok);
       return;
     }
     nb = Ascii2bin(opt) ;
     strcpy(resp_buf,opt);
     
     for (i = 0; i < nb; i++)
     apdu_buf[5 + i] = resp_buf[i];
  

      apdu_buf[0]= (byte)0x00; apdu_buf[1]= (byte)0x80;
      apdu_buf[2]= (byte)0x21; apdu_buf[3]= (byte)0xFE; 
      apdu_buf[4]= (byte)nb;
      
      err = send_apdu((char *)apdu_buf,5+nb,(char *)resp_buf,false,true);

      if ( (err < 2) || (resp_buf[err - 2] != (char)0x90) || (resp_buf[err - 1] != (char)0x00) )
      { MySerial.println(cnok);
        return;
      }

      Bin2ascii(resp_buf, err - 2 , apdu_buf);
      
      MySerial.println(apdu_buf);
      return;
  }

  else if  (strcmp_P(cmd,PSTR("bmac"))==0)
  {  if (opt == NULL) 
     { MySerial.println(cERROR);
       return ;
     }
     else
     { 
      uint32_t ct=0;
       //memset(MyBuf,0,SIZE_BUF1);
      bmac(atoi(opt),&ct);
      bin2ascii(cs.buf.digest,32,cs.buf.mybuf);
      MySerial.print(ct)  ;
      MySerial.print(" ") ;
      MySerial.println(cs.buf.mybuf);
      return;
     
     }
  }
  
  else if ( (cmd[0]=='b') && (opt == NULL) )
  { uint32_t x = atoi(cmd+1);
    if (x >0) base = x ;
    if (fserial2) MySerial2.println(base);
    else          MySerial.println(base) ;
    return;
  }

   else if  (strcmp_P(cmd,PSTR("hist"))==0)
   { if (histlen <=0)  
     { MySerial.println(cERROR); return;}
     
     for(char i=0;i<histlen;i++)
     { Bin2ascii(histbytes+i,1, v);
       MySerial.print(v);
     }
     MySerial.println();
     return;
    }

   else if (strcmp_P(cmd,PSTR("reboot"))==0)
   {
     reboot(0);
     return;
   }
   else if (strcmp_P(cmd,PSTR("nodebug"))==0)
   { fdebug=false;
     MySerial.println(cOK); 
     return;
   }
   
   else if (strcmp_P(cmd,PSTR("debug"))==0)
   { fdebug=true;
     MySerial.println(cOK); 
     return;
   }
   

  else if (tmode)
  {
   if (strcmp_P(cmd,PSTR("test"))==0)
   { int myct=0;
     int maxi=1;
     if (opt != NULL) maxi=atoi(opt);
     while(myct++ < maxi)
     {
     strcpy_P(MyBuf,PSTR("0085000C201234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF"));
     err = Ascii2bin(MyBuf);
     if (err <=0)  { MySerial.println(cERROR); return;}
     err= send_apdu(apdu_buf,err,resp_buf,false,true);
     if (err <=0)  { MySerial.print(cERROR); MySerial.print(" "); MySerial.println(myct); return;}
     for(int i=0;i<err;i++)
     { Bin2ascii(resp_buf+i, 1, v);
       MySerial.print(v);
     }
     }
     MySerial.println();
     return;
   }


   else if ( (strcmp_P(cmd,PSTR("F"))==0) && (opt != NULL) )
   { MyF= atoi(opt);
     MySerial.println(cOK); 
     return;
   }

      else if (strcmp_P(cmd,PSTR("t0"))== 0)
   { myptcol=(char)0 ;
     MySerial.println(cOK); 
     return;
   }

   else if (strcmp_P(cmd,PSTR("t1"))== 0)
   { myptcol=(char)1 ;
     MySerial.println(cOK); 
     return;
   }

   else if (strcmp_P(cmd,PSTR("ptcol"))== 0)
   { MySerial.println(0xFF & myptcol); 
     return;
   }

     else if(strcmp_P(cmd,PSTR("finject"))==0)
   { 
     if (opt == NULL)
     { MySerial.println(finject); 
       return;
     }
     err = atoi(opt) ;
     finject=err     ;
     MySerial.println(cOK); 
     return;
   }



   else if( (strcmp_P(cmd,PSTR("pts"))==0) && (opt != NULL) )
   { 
     err = atoi(opt);
     MyTA = 0x10 + err;
     mypts=true;
     MySerial.println(cOK); 
     return;
   }

    else if (strcmp(cmd,"nopts")== 0)
   { mypts=false ;
     MySerial.println(cOK); 
     return;
   }

   
    else if  (( (strcmp_P(cmd,PSTR("A"))==0) || (strcmp_P(cmd,PSTR("S"))==0))  && (opt != NULL) )
   { bool fdata=false;
     if (strcmp_P(cmd,PSTR("S"))==0) fdata=true ;
     strcpy(MyBuf,opt);
     err = Ascii2bin(MyBuf);
     if (err <=0)  { MySerial.println(cERROR); return;}
     err= send_apdu(apdu_buf,err,resp_buf,fdata,true);
     if (err <=0)  { MySerial.println(cERROR); return;}
     for(int i=0;i<err;i++)
     { Bin2ascii(resp_buf+i, 1, v);
       MySerial.print(v);
     }
     MySerial.println();
     return;
    }

    else if ( (strcmp_P(cmd,PSTR("ewrite"))==0) && (opt != NULL) )
    { err = atoi(opt);
      if ( (err<0) || (err >= EEPROM_SIZE) )
      { MySerial.println(cERROR); 
        return;
      }
      nb=0;
      opt= strtok(NULL,toram(PSTR("\r\n")));
      if (opt != NULL) 
      { strcpy(MyBuf,opt)   ;
        nb= Ascii2bin(MyBuf);
      }
      if ( (opt == NULL) || (nb<=0) || ((err+nb)>EEPROM_SIZE) )
      { MySerial.println(cERROR); 
        return;
      }
      for(i=0;i<nb;i++)
      EEPROM.update(err+i,(byte)MyBuf[i]);
      MySerial.println(cOK); 
      return;
    }


else if (strcmp_P(cmd,PSTR("eread"))==0) 
{
     for(int i=0;i<EEPROM_SIZE;i++)
     { err = EEPROM.read(i);
       c[0]= 0xFF & err;
       Bin2ascii(c, 1, v);
       MySerial.print(v);
     }
     MySerial.println();
     return;
}   

  }
  
  MySerial.println(cERROR);
   
 
  
}

int readBufferX(char* buf,int count,char deb)
{
  int pos=0,len=0;
  char c;
  byte nb;
  bool found = false, fdata=false;

  if (deb == (char)'X') fdata=true;
  
  while (pos < (count-1) )
  {
    //if (MySerial.available() <= 0)
    //continue ;

    nb =  MySerial.readBytes(&c, 1) ;

    // if (nb == 0) continue;
    if (nb <= 0) return -1; // timeout (1000ms default)

    if (pos == 0)
    { len = 0xff & c ; len = 0xFF00 & (len << 8);  pos++; continue;}
    else if (pos == 1)
    { len |= (0xFF & c);  pos++; continue;}
    else
    { buf[(pos-2)] = c ; pos++;}
    
    if ((pos-2) == len)
    { found = true;
      break;
    }
  }

  if (!found)
  return 0;

  if (len > 260)
  return 0;
  
  memmove(apdu_buf,buf,len);
  int err= send_apdu(apdu_buf,len,resp_buf,fdata,true);
  if (err <=0)  { MySerial.write((byte)0xFF);
                  MySerial.write((byte)0);
                  MySerial.write((byte)0);
                  return 0;
                 }
    MySerial.write(deb);
    MySerial.write((byte)(0xFF & err>>8));
    MySerial.write((byte)(0xFF & err));
    for(int i=0;i<err;i++) MySerial.write((byte)resp_buf[i]);
  
    return 0;
}


int readBuffer(char* buf, int count, char delim)
{
  static int pos = 0;
  char c;
  byte nb;
  bool found = false;

  while (pos < (count - 1) )
  {
    //if (MySerial.available() <= 0) return 0;
    int err= MySerial.available();
    if (err <= 0)
    { if (flocal)      return 0;
      else if (pos==0) return 0;
    }
     nb =  MySerial.readBytes(&c, 1) ;

    //if (nb == 0) return 0;
    if (nb <= 0)  // timeout
    { pos=0;
      return -1;
    }

    if (pos ==0)
    { if      (c== '?') {flocal=true; return 0;}
      else if (c== '#') {flocal=false; return 0;}
      else if ((c==(char)'x') || (c==(char)'X')) 
      { int err= readBufferX(buf,count-1,c);
        return err;
      }
    }
    
    buf[pos++] = c ;
 
    if (c == delim )
    { found = true;
      break;
    }
  }

  buf[pos] = '\0';
  nb = pos ;
  pos = 0;
  if (found) return nb;

  return 0;
}

int readBuffer2(char* buf, int count, char delim)
{
  static int pos = 0;
  char c;
  byte nb;
  bool found = false;

  while (pos < (count - 1) )
  {
    int err= MySerial2.available();
    if (err <= 0) return 0;
 
     nb =  MySerial2.readBytes(&c, 1) ;

    if (nb <= 0)  // timeout
    { pos=0;
      return -1;
    }

    buf[pos++] = c ;
 
    if (c == delim )
    { found = true;
      break;
    }
  }

  buf[pos] = '\0';
  nb = pos ;
  pos = 0;
  if (found) return nb;

  return 0;
}



void dump_ram(char *ptrd,int adr,bool f2)
{ char * p;
 
 for (int i=adr;i<(adr+16);i+=16)
{ if (i<10)        { if (f2) MySerial2.print("000"); else MySerial.print("000");}
  else if (i<100)  { if (f2) MySerial2.print("00") ; else MySerial.print("00");}
  else if (i<1000) { if (f2) MySerial2.print("0")  ; else MySerial.print("0");}
  if (f2) { MySerial2.print(i); MySerial2.print(" ");}
  else    { MySerial.print(i); MySerial.print(" ");}
  p = (char *) (i+REG_SIZE)  ;
  bin2ascii(p,16,ptrd) ;   
  if (f2) MySerial2.print(ptrd  ) ;
  else    MySerial.print(ptrd  ) ;
  }
  if (f2) MySerial2.println();
  else    MySerial.println();
} 

void dump_full(char *ptrd,bool f2)
{char * p;
 for (int i=0;i<1024;i+=16)
 { 
  p = (char *) (i+base+REG_SIZE)  ;
  bin2ascii(p,16,ptrd) ;   
  if (f2)  MySerial2.print(ptrd) ;
  else     MySerial.print(ptrd);
 }
 if (f2) MySerial2.println();
 else    MySerial.println();
} 







boolean is_hexa(char *s)
{ int nb,i;
  char c;
  nb=strlen(s);
  for (int i=0;i<nb;i++)  
  if (isDigit(s[i]) == 0) return false;

  return true;
  
}

int isDigit(char c)
{ if (((int)c >= (int)'0') && ((int)c <= (int)'9')) return (1);
  if (((int)c >= (int)'A') && ((int)c <= (int)'F')) return (1);
  if (((int)c >= (int)'a') && ((int)c <= (int)'f')) return (1);
  return (0);
}


int Ascii2bin(char *Data_In)
{ int deb = -1, fin = -1, i, j = 0, nc, iCt = 0, len;
  long decimal_value = 0;
  char c;
  char *data_in = NULL;
  char *data_out = NULL;

  data_out = data_in = Data_In;

  len = (int) strlen(Data_In);

  for (i = 0; i < len; i++)
  { if      ( (deb == -1) && (isDigit(data_in[i])) )             {
      iCt = 1;
      deb = i;
    }
    else if ( (deb != -1) && (iCt == 1) && (isDigit(data_in[i])) ) {
      iCt = 2;
      fin = i;
    }

    if (iCt == 2)
    { c = data_in[fin + 1];
      data_in[deb + 1] = data_in[fin];
      data_in[deb + 2] = 0;

      decimal_value = strtol(&data_in[deb], NULL, 16);
      data_in[fin + 1] = c;

      c = (char)(0xFF & decimal_value);

      data_out[j++] = c ;
      deb = fin = -1; iCt = 0 ;
    }
  }

  return (j);
}


void Bin2ascii(char *bin, int len, char *buffer)
{ int i;
  for (i = 0; i < len; i++)
  {
    char nib1 = (bin[i] >> 4) & 0x0F;
    char nib2 = (bin[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}



void bin2ascii(char *bin, int len, char *buffer)
{ int i;
  for (i = 0; i < len; i++)
  {
    char nib1 = (bin[i] >> 4) & 0x0F;
    char nib2 = (bin[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'a' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'a' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}



char * bmac(int32_t seed, uint32_t *ct)
{
  uint8_t   buf[1] ;
  USHORT v         ;
  USHORT kv=0      ;
  ULONG  kq[NPERM2] ;
  ULONG  g[NPERM2]  ;
  ULONG  p2[NBITS] ;
  ULONG  gi[NBITS] ;
  USHORT skey[NPERM2];
  ULONG  x, y, bitn  ;
  int32_t a =  16807, m = 2147483647, q = 127773, r = 2836 , hi = 0, lo = 0, test = 0; // seed=1234
  uint8_t * ptsram = NULL ;
  bool      tohash = false;
  uint32_t tt = 0 ;
  USHORT i = 0    ;
  struct
  {
    uint16_t  m_begin = 0;
    uint16_t   m_end  = 0;
  }
  sctx;
  uint16_t  s_end = 0  ;

 
  ///////////////////////////////////////////////////////////////////////
  //  cbi (TIMSK0, TOIE0); // disable Timer0 !!! delay() is now not available
  ///////////////////////////////////////////////////////////////////////

  ///////////////////////////
  // power of 2 modulo PRIME
  ///////////////////////////
  p2[0] = 2;
  for (uint8_t n = 1; n <= (NBITS-1); n++)
  p2[n] = (p2[n - 1] * p2[n - 1]) % ((ULONG)PRIME);

  ////////////////
  noInterrupts();
  ///////////////

  s_end = (int)&s_end   ;
  sctx.m_begin        =  (uint16_t)&__heap_start;
  sctx.m_end          =  (uint16_t)&__heap_start + (uint16_t)FREE_MEM_SIZE  ;

  bool cfirst = true;

cloop:

  ////////////////
  noInterrupts();
  ///////////////


  // Stephen K.Park and Keith W.Miller
  // Random Numbers Generators: Good Ones Are Hard to Find
  // Communication of ACM October 1998, Volume 31, Number 10, pp1192-1201
  //
  // Fill free SRAM with pseudo random value
  // compute skey and kq pseudo random values


  //if (cfirst)
  {
  
  for (uint16_t ii = sctx.m_begin ; ii < (sctx.m_end + 4*NPERM) ; ii++)
  { hi = seed  / q ;
    lo = seed  % q ;
    test = a * lo - r * hi;
    if (test > 0) seed = test ;
    else          seed = test + m ;

    if (ii < sctx.m_end)
    {
      if (cfirst)
        *((char*)ii) = seed & 0xFF ;
    }

    else if ( (ii - sctx.m_end) < NPERM2 )
      kq[ii - sctx.m_end]=   (ULONG)(1 + (seed % (int32_t)(Q - 1))) ;
   
    else
      skey[ii - sctx.m_end - NPERM2]=   (USHORT)(1 + (seed % (int32_t)(PRIME - 1))) ;
 
  }

  for (uint16_t ii = 0 ; ii < (SIZE_BUF1-SIZE_BUF0) ; ii++)
  { hi = seed  / q ;
    lo = seed  % q ;
    test = a * lo - r * hi;
    if (test > 0) seed = test ;
    else          seed = test + m ;
    cs.mybuf[ii+SIZE_BUF0] = seed & 0xFF ;
  }
  
  
  }


  
  

  cfirst = false ;

  for (uint8_t j=0;j<NPERM2;j++)
  {
    //compute  2**kq mod PRIME
    bitn = kq[j];
    y = 1;
    for (uint8_t n = 1; n <= NBITS; n++)
    { if ( (bitn & 0x1) == 0x1)  y = (y * p2[n - 1]) % (ULONG)PRIME;
      bitn = bitn >> 1;
    }

    // g = PRIME - (2**kq mod PRIME)
    g[j] = y ;
    g[j] = (ULONG)PRIME - y ;
  }


  //////////////////////////
  delay1(0); // init timer1
  //////////////////////////

  sha3_Init256(&cs.sha3);

  kv = 0;
  
  for (uint8_t j = 0; j <NPERM; j++)
  {
    x = (ULONG)skey[2*j+1];

    gi[0] = g[2*j]; // G2j
    // compute square power of G2j
    for ( uint8_t n = 1; n <= (NBITS-1); n++)
      gi[n] = (gi[n - 1] * gi[n - 1]) % (ULONG)PRIME;

    for ( i = 1; i < (USHORT)PRIME; i++)
    { tohash = false;

      x = (x * g[2*j + 1]) % (ULONG)PRIME; // G2j+1
      bitn = x;
      y = 1;

      for (uint8_t n = 1; n <= NBITS; n++)
      { if ( (bitn & 0x1) == 0x1)  y = (y * gi[n - 1]) % (ULONG)PRIME;
        bitn = bitn >> 1;
      }

      v = (USHORT)(y - 1);

      if (v < FLASH_SIZE)
      { tohash = true;
        buf[0] = pgm_read_byte_near(v);
      }

      else if ( (v >= FLASH_SIZE) &&  (v < (FLASH_SIZE+SRAM_SIZE)) )
      {
        v = v - FLASH_SIZE + REG_SIZE ;
        if (  ( ((uint16_t)v >=  sctx.m_begin) && ((uint16_t)v < sctx.m_end))  || ( ((uint16_t)v >= (uint16_t)&cs) && ((uint16_t)v < ((uint16_t)&cs + (uint16_t)sizeof(cs)))) )
        { tohash = true   ;
          ptsram =  (uint16_t)v ;
          buf[0] = *ptsram ;
        }
      }

      else if ( (v >= (FLASH_SIZE+SRAM_SIZE)) && (v < (FLASH_SIZE+SRAM_SIZE+EEPROM_SIZE)) )
      {
        v -= (FLASH_SIZE+SRAM_SIZE); 
        tohash = true;
        buf[0] = EEPROM.read((uint16_t)v);
      }



      if (tohash)
      { sha3_Update(&cs.sha3, buf, 1);
        kv++;
      }

      ///////////////
      tt = gettime1();
      ///////////////

    }

  }

  sha3_Finalize(&cs.sha3) ;

  tt = gettime1();
  *ct = tt; 

  ///////////////
   interrupts();
  ///////////////

  

  if (fdebug) 
  {
  MySerial.println(toram(PSTR("OK")));
  MySerial.print(toram(PSTR("heap= ")));
  MySerial.println(sctx.m_begin); // Heap
  MySerial.print(toram(PSTR("stack= ")));
  MySerial.println(s_end);        //Stack
  MySerial.print(toram(PSTR("nvalue= ")));MySerial.print(kv); 
  MySerial.print(toram(PSTR(" (should be "))); MySerial.print(FLASH_SIZE+EEPROM_SIZE+FREE_MEM_SIZE+sizeof(cs));MySerial.println(toram(PSTR(")")));
  MySerial.println(toram(PSTR("Generators:")));
  for ( uint8_t ii = 0; ii < NPERM2; ii++)
  MySerial.println(g[ii]);      // Generators gi
  MySerial.println(toram(PSTR("s values:")));  
  for ( uint8_t ii= 0; ii < NPERM2; ii++)
  MySerial.println(skey[ii]);   // skey values
  MySerial.println(toram(PSTR("Generator exponents:")));    
  for ( uint8_t ii = 0; ii < NPERM2; ii++)
  MySerial.println(kq[ii]);  // generators exponent values
  MySerial.println(toram(PSTR("bMAC:")));
  bin2ascii(cs.buf.digest, 32, cs.buf.mybuf);
  MySerial.println(cs.buf.mybuf) ;
  MySerial.println(toram(PSTR("Computing time: (unit=4us= 64/16MHz)")));
  MySerial.println(tt);
  MySerial.print(tt*4); MySerial.println(toram(PSTR(" us")));
  }
 
  // for(uint8_t ii=0;ii<4;ii++) cs.buf.digest[31-ii] ^= ((ui32 *)&tt)->b[ii];}
  // MySerial.println("bMAC_TS:");
  // bin2ascii(cs.buf.digest, 32, cs.buf.mybuf);
  // MySerial.println(cs.buf.mybuf) ;
  // Serial.end();
  // goto cloop;

  return cs.buf.digest;

}







#define SHA3_ASSERT( x )
#define SHA3_USE_KECCAK
#define SHA3_CONST(x) x

#ifndef SHA3_ROTL64
#define SHA3_ROTL64(x, y) \
  (((x) << (y)) | ((x) >> ((sizeof(uint64_t)*8) - (y))))
#endif

#define KECCAK_ROUNDS 24



// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/
// The following code WILL work, even if locally defined within a function:
// const static char long_str[] PROGMEM = "Hi, I would like to tell you a bit about myself.\n"

const static uint64_t keccakf_rndc[] PROGMEM = {
  SHA3_CONST(0x0000000000000001UL), SHA3_CONST(0x0000000000008082UL),
  SHA3_CONST(0x800000000000808aUL), SHA3_CONST(0x8000000080008000UL),
  SHA3_CONST(0x000000000000808bUL), SHA3_CONST(0x0000000080000001UL),
  SHA3_CONST(0x8000000080008081UL), SHA3_CONST(0x8000000000008009UL),
  SHA3_CONST(0x000000000000008aUL), SHA3_CONST(0x0000000000000088UL),
  SHA3_CONST(0x0000000080008009UL), SHA3_CONST(0x000000008000000aUL),
  SHA3_CONST(0x000000008000808bUL), SHA3_CONST(0x800000000000008bUL),
  SHA3_CONST(0x8000000000008089UL), SHA3_CONST(0x8000000000008003UL),
  SHA3_CONST(0x8000000000008002UL), SHA3_CONST(0x8000000000000080UL),
  SHA3_CONST(0x000000000000800aUL), SHA3_CONST(0x800000008000000aUL),
  SHA3_CONST(0x8000000080008081UL), SHA3_CONST(0x8000000000008080UL),
  SHA3_CONST(0x0000000080000001UL), SHA3_CONST(0x8000000080008008UL)
};

const static uint8_t  keccakf_rotc[KECCAK_ROUNDS] PROGMEM = {
  1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 2, 14, 27, 41, 56, 8, 25, 43, 62,
  18, 39, 61, 20, 44
};

const static uint8_t  keccakf_piln[KECCAK_ROUNDS] PROGMEM = {
  10, 7, 11, 17, 18, 3, 5, 16, 8, 21, 24, 4, 15, 23, 19, 13, 12, 2, 20,
  14, 22, 9, 6, 1
};


void keccakf(uint64_t s[25])
{
  int i, j, round    ;
  uint64_t t, bc[5], v;

  for (round = 0; round < KECCAK_ROUNDS; round++) {

    /* Theta */
    for (i = 0; i < 5; i++)
      bc[i] = s[i] ^ s[i + 5] ^ s[i + 10] ^ s[i + 15] ^ s[i + 20];

    for (i = 0; i < 5; i++) {
      t = bc[(i + 4) % 5] ^ SHA3_ROTL64(bc[(i + 1) % 5], 1);
      for (j = 0; j < 25; j += 5)
        s[j + i] ^= t;
    }

    /* Rho Pi */
    t = s[1];
    for (i = 0; i < 24; i++) {
      j = pgm_read_byte(keccakf_piln + i);
      bc[0] = s[j];
      s[j] = SHA3_ROTL64(t, pgm_read_byte(keccakf_rotc + i));
      t = bc[0];
    }

    /* Chi */
    for (j = 0; j < 25; j += 5) {
      for (i = 0; i < 5; i++)
        bc[i] = s[j + i];
      for (i = 0; i < 5; i++)
        s[j + i] ^= (~bc[(i + 1) % 5]) & bc[(i + 2) % 5];
    }

    /* Iota */
    //s[0] ^= keccakf_rndc[round];
    memcpy_P(&v, keccakf_rndc + round, 8);
    s[0] ^=  v ;


  }
}

/* *************************** Public Inteface ************************ */

/* For Init or Reset call these: */
void sha3_Init256(sha3_context * priv)
{
  sha3_context *ctx = (sha3_context *) priv;
  memset(ctx, 0, sizeof(sha3_context));
  
  ctx->capacityWords = 2 * 256 / (8 * sizeof(uint64_t));
}

void sha3_Update(sha3_context *priv, void const *bufIn, size_t len)
{
  sha3_context *ctx = (sha3_context *) priv;

  /* 0...7 -- how much is needed to have a word */
  unsigned int old_tail = (8 - ctx->byteIndex) & 7;

  size_t words;
  unsigned int tail;
  size_t i;

  const uint8_t *buf = bufIn;

  SHA3_ASSERT(ctx->byteIndex < 8);
  SHA3_ASSERT(ctx->wordIndex < sizeof(ctx->s) / sizeof(ctx->s[0]));

  if (len < old_tail) {
    /* have no complete word or haven't started
                                   the word yet */
    // SHA3_TRACE("because %d<%d, store it and return", (unsigned)len,(unsigned)old_tail);
    /* endian-independent code follows: */

    while (len--)
      ctx->saved |= (uint64_t) (*(buf++)) << ((ctx->byteIndex++) * 8);
    SHA3_ASSERT(ctx->byteIndex < 8);
    return;
  }

  if (old_tail) {             /* will have one word to process */
    // SHA3_TRACE("completing one word with %d bytes", (unsigned)old_tail);
    /* endian-independent code follows: */
    len -= old_tail;
    while (old_tail--)
      ctx->saved |= (uint64_t) (*(buf++)) << ((ctx->byteIndex++) * 8);

    /* now ready to add saved to the sponge */
    ctx->s[ctx->wordIndex] ^= ctx->saved;
    SHA3_ASSERT(ctx->byteIndex == 8);
    ctx->byteIndex = 0;
    ctx->saved = 0;
    if (++ctx->wordIndex ==
        (SHA3_KECCAK_SPONGE_WORDS - ctx->capacityWords)) {
      keccakf(ctx->s);
      ctx->wordIndex = 0;
    }
  }

  /* now work in full words directly from input */

  SHA3_ASSERT(ctx->byteIndex == 0);

  words = len / sizeof(uint64_t);
  tail = (uint32_t)(len - words * sizeof(uint64_t));

  for (i = 0; i < words; i++, buf += sizeof(uint64_t)) {
    const uint64_t t = (uint64_t) (buf[0]) |
                       ((uint64_t) (buf[1]) << 8 * 1) |
                       ((uint64_t) (buf[2]) << 8 * 2) |
                       ((uint64_t) (buf[3]) << 8 * 3) |
                       ((uint64_t) (buf[4]) << 8 * 4) |
                       ((uint64_t) (buf[5]) << 8 * 5) |
                       ((uint64_t) (buf[6]) << 8 * 6) |
                       ((uint64_t) (buf[7]) << 8 * 7);
#if defined(__x86_64__ ) || defined(__i386__)
    SHA3_ASSERT(memcmp(&t, buf, 8) == 0);
#endif
    ctx->s[ctx->wordIndex] ^= t;
    if (++ctx->wordIndex ==
        (SHA3_KECCAK_SPONGE_WORDS - ctx->capacityWords)) {
      keccakf(ctx->s);
      ctx->wordIndex = 0;
    }
  }

  // SHA3_TRACE("have %d bytes left to process, save them", (unsigned)tail);

  /* finally, save the partial word */
  SHA3_ASSERT(ctx->byteIndex == 0 && tail < 8);
  while (tail--) {
    // SHA3_TRACE("Store byte %02x '%c'", *buf, *buf);
    ctx->saved |= (uint64_t) (*(buf++)) << ((ctx->byteIndex++) * 8);
  }
  SHA3_ASSERT(ctx->byteIndex < 8);
  // SHA3_TRACE("Have saved=0x%016" PRIx64 " at the end", ctx->saved);
}



void const * sha3_Finalize(sha3_context * priv)
{
  sha3_context *ctx = (sha3_context *) priv;

  // SHA3_TRACE("called with %d bytes in the buffer", ctx->byteIndex);

  /* Append 2-bit suffix 01, per SHA-3 spec. Instead of 1 for padding we
     use 1<<2 below. The 0x02 below corresponds to the suffix 01.
     Overall, we feed 0, then 1, and finally 1 to start padding. Without
     M || 01, we would simply use 1 to start padding. */


  /* For testing the "pure" Keccak version */
  ctx->s[ctx->wordIndex] ^=
    (ctx->saved ^ ((uint64_t) ((uint64_t) 1 << (ctx->byteIndex * 8))));

  ctx->s[SHA3_KECCAK_SPONGE_WORDS - ctx->capacityWords - 1] ^=
    SHA3_CONST(0x8000000000000000UL);
  keccakf(ctx->s);

  /* Return first bytes of the ctx->s.
     This conversion is not needed for little-endian platforms */

  /*
      uint32_t  i;
      for(i = 0; i < SHA3_KECCAK_SPONGE_WORDS; i++) {
          const uint32_t  t1 = (uint32_t) ctx->s[i];
          const uint32_t  t2 = (uint32_t) ((ctx->s[i] >> 16) >> 16);
          ctx->sb[i * 8 + 0] = (uint8_t) (t1);
          ctx->sb[i * 8 + 1] = (uint8_t) (t1 >> 8);
          ctx->sb[i * 8 + 2] = (uint8_t) (t1 >> 16);
          ctx->sb[i * 8 + 3] = (uint8_t) (t1 >> 24);
          ctx->sb[i * 8 + 4] = (uint8_t) (t2);
          ctx->sb[i * 8 + 5] = (uint8_t) (t2 >> 8);
          ctx->sb[i * 8 + 6] = (uint8_t) (t2 >> 16);
          ctx->sb[i * 8 + 7] = (uint8_t) (t2 >> 24);
  */


  return (ctx->sb);
}

void myPrintf(char *str, char *vli, int size)
{ int i;
  char buf[80];

  if (size <= 0) return ;
  
  sprintf(buf, "%s: %d", str,size);
  MySerial.println(buf);
  buf[0] = 0;
  for (i = 0; i < size; ++i)
  {
    sprintf(&buf[strlen(buf)], "%02X", 0xFF & (unsigned)vli[i]);
    if (i % 32 == 31)
    {  MySerial.println(buf);
      buf[0] = 0;
    }
  }

  i--;
  if ((i % 32) != 31)
     MySerial.println(buf);
}

void dumpHEX(uint8_t* values, uint16_t size) {
  if (values != NULL && size > 0) {
    char ascii[17];
    for(uint16_t row=0; row<(size + 15)/16; row++) {
      // Print Adress
      if (row==0)
        MySerial.print("0");
      MySerial.print(row * 16, HEX);
      MySerial.print("|");

      // Prefill ascii
      for(int i=0; i<16; i++)
        ascii[i] = '.';
      ascii[16] = (char)0x00;
      // colums
      for(uint16_t pos=row*16; pos<(row + 1) * 16; pos++ ) {
        if(pos < size) {
          if(values[pos] < 0x10)
            MySerial.print("0");
          MySerial.print(values[pos], HEX);
          if(isPrintable(values[pos]))
            ascii[pos - row*16] = (char)values[pos];
        } else {
          MySerial.print("  ");
        }
        MySerial.print(" ");
      }
      MySerial.print("'");
      MySerial.print(ascii);
      MySerial.println("'");
    }
  }
}

bool PTS(uint8_t TA, uint8_t ptcol)
{
  uint8_t buf[4] =  {(uint8_t)0xFF, (uint8_t)0x10, (uint8_t)0x11, (uint8_t)0 };
  uint8_t buf2[4];
  int nb;

  buf[1] |= ptcol;
  buf[2]  = TA  ;

  buf[3] = buf[2] ^ buf[1] ^ buf[0] ;

  delayMicroseconds(sc._guardTime);

  for (uint8_t i=0; i < 4; i++)
  { 

//sc._sendByte(buf[i]);
sc._sendByteStd(buf[i]);
    
    delayMicroseconds(sc._guardTime);
  }

  for (uint8_t i = 0; i < 4; i++) buf2[i] = (char)0xA5;

  nb = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
// if (sc._receiveByte((uint8_t*)(buf2 + i), sc._max_wwt) != 0)  break;
   if (sc._receiveByteStd((uint8_t*)(buf2 + i), sc._max_wwt) != 0)  break;

    nb++;
  }

if (fdebug)
{
dumpHEX((uint8_t*)buf, 4);
if (nb >0) dumpHEX((uint8_t*)buf2, nb);
}

  if (nb != 4) 
    return false;
  
   for (uint8_t i=0;i<4;i++)
   if (buf[i] != buf2[i])
   return false;
   
   return true;
 

}

#define NTD 4
typedef struct ifd_atr_info {
    /* The following contain -1 if the field wasn't present */
    int  TA[NTD];
    int  TB[NTD];
    int  TC[NTD];
    int supported_protocols;
    int  default_protocol  ;
    int  t1;
    int  t15;
  } ifd_atr_info_t;
  
int ifd_atr_parse(ifd_atr_info_t * info, unsigned char *atr, unsigned int len);

int ifd_atr_parse(ifd_atr_info_t * info, unsigned char *atr, unsigned int len)
{
  unsigned int m, n, k=0,i,lenh=0;
  int fTCK=0;

  /* Initialize the atr_info struct */
  memset(info, 0, sizeof(*info));
  info->default_protocol = -1;
  for (n = 0; n < NTD; n++) {
    info->TA[n] = -1;
    info->TB[n] = -1;
    info->TC[n] = -1;
    info->t1= -1;
    info->t15= -1;
  }

  if (len < (unsigned int)(2 + ((atr[1] & 0x0f)) ))
    return -1; // ERROR_INVALID_ATR;

  /* Ignore hystorical bytes */
  lenh = atr[1] & 0x0f;
  len -= lenh;

  for (m = 0, n = 2; n < len; m++) {
    unsigned int TDi;

    /* TA1, TA2, TA3, TA4 are legal, TA5 wouldn't be */
    if (m >= NTD)
      return -2 ; //ERROR_INVALID_ATR;

    TDi = atr[n - 1]; // n=2 => T0
    if (n != 2) 
    { unsigned int prot;
      prot = TDi & 0x0f;
      if ((prot == 15) &&  (info->t15 == -1) )          info->t15= m-1 ;
      if ( (prot == 1) &&  (info->t1 == -1) && (m>1) )  info->t1 = m-1 ;
      if (info->default_protocol < 0) info->default_protocol = prot;
      info->supported_protocols |= (1 << prot);
    }

    //k = ifd_count_bits(TDi & 0xF0);

    k=0;
    if (TDi & 0x10) k++;
    if (TDi & 0x20) k++;
    if (TDi & 0x40) k++;
    if (TDi & 0x80) k++;

    // n-1+k  0...len-1  n-1+k <= len-1 n+k <= len

    if ((n + k) > len) 
    return -3; //ERROR_INVALID_ATR;
     
    if (TDi & 0x10)
    info->TA[m] = atr[n++];
    if (TDi & 0x20)
    info->TB[m] = atr[n++];
    if (TDi & 0x40)
    info->TC[m] = atr[n++];
    if (!(TDi & 0x80)) // TDi (i= m+1) is absent end of ATR
    {
      /* If the ATR indicates we support anything in addition to T=0, 
       * there'll be a TCK byte
       * at the end of the string.
       * For now, simply chop it off. Later we may want to verify it.
       */
      // If only T=0 is indicated, possibly by default, byte TCK shall be absent. 
      if (info->supported_protocols & ~0x1)
      { fTCK=1; len--;}

      // n= len
      // MAIS si il y a un byte TCK en plus (non STD) n = len-1

      break;
    }
    n++;
  }

  /* ATR didn't list any supported protocols, so we default to T=0 */
  if (info->supported_protocols == 0) 
  {
    info->supported_protocols = 0x01;
    info->default_protocol = 0; //PROTOCOL_T0;
  }

  if (fTCK)
  { k=0;
      for(i=1;i<(len+lenh);i++)
      k= k ^ atr[i];

    if (k != (uint16_t)atr[len+lenh])
      return -10; // TCK ERROR
   }

  // return number of extra bytes after historical bytes
  return fTCK + len - n;
}

void FiDi(unsigned char ta,int * Fi,int *Di)
{ unsigned char D,F;
  F = (unsigned char)(ta >> 4) ;
  D = (unsigned char)(ta & 0xF);
  *Fi=-1;
  *Di=-1;

  switch (F)
  { case 0:
    case 1:
    *Fi=372;break;
    break;
    case 2:
    *Fi=558;break;
    case 3:
    *Fi= 744;break;
    case 4:
    *Fi= 1116;break;
    case 5:
    *Fi= 1488;break;
    case 6:
    *Fi= 1860;break;
    case 7:
    case 8:
    break;
    case 9:
    *Fi=512;break;
    case 10:
    *Fi=758;break;
    case 11:
    *Fi=1024;break;
    case 12:
    *Fi=1536;break;
    case 13:
    *Fi=2048;break;
    case 14:
    case 15:
    default:
    break;
    break;
    
  }

   switch (D)
  { case 0:
    break;
    case 1:
    *Di=1;break;
    break;
    case 2:
    *Di=2;break;
    case 3:
    *Di=4;break;
    case 4:
    *Di=8;break;
    case 5:
    *Di=16;break;
    case 6:
    *Di=32;break;
    case 7:
    break;
    case 8:
    *Di=12;break;
    case 9:
    *Di=20;break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    default:
    break;
 }
 
}

bool CheckSC(bool pts,bool warm)
{ uint8_t data[MAX_ATR_BYTES];
  uint16_t atr_received = 0;
  int fs=2000, D = 1, F = 372, N = 0;
  int CWI = 13, BWI = 4, WI = 10;
  int Fi=-1,Di=-1;
  ifd_atr_info_t info;
  uint8_t cardifs=0;
  uint8_t ptsct=0 ;
  uint8_t ptcol=0 ;

  if (fdebug)MySerial.println();
 
  // Parameter N is the extra guardtime used to send characters from the interface device to the card. 
  // No extra guardtime is used to send characters from the card to the interface device. 
  // The default value is N = 0.
  // In the range 0 to 254, N indicates that, before being ready to receive the next character, the card requires the
  // following delay from the leading edge of the previous character (sent either by the card or by the interface device).
  // 12 etu + (Q x N/F)
  // In the formula, Q shall take either one of the two values:
  // - F/D, i.e., the values used for computing the etu, if T=l5 is absent in the Answer-to-Reset;
  // - Fi/Di if T=l5 is present in the Answer-to-Reset.
  // etu = Fi/Di/fs, 12 etu + N. etu
  
  fs= sc.fclk = MyF;
  mycrc=false;
 
 // F/D/fs
 // fs=1  F=512 D=8   etu=64
 // fs=1  F=512 D=16  etu=32
 // fs=2  F=512 D=4   etu=64
 // fs=2  F=512 D=8   etu=32
 // fs=4  F=512 D=2   etu=64
 // fs=4  F=512 D=4   etu=32


  sc.ignoreParityErrors(false);
  if (!warm) atr_received  = sc.activate(data, MAX_ATR_BYTES)   ;
  else       
  { // warm reset
    pinMode(SC_C7_IO,INPUT) ;  
    sc.use_ts= true;
    atr_received  = sc._WarmReset(data, MAX_ATR_BYTES) ;
    sc.use_ts= false;
    // use ts to measure _etu, or compute _etu from clock frequency
    // compute  _guardTime,  _wwt, _max_wwt; 

    /*
    pinMode(SC_C7_IO, INPUT) ;  
    digitalWrite( SC_C2_RST, LOW );
    delayMicroseconds(1000);
    digitalWrite(SC_C2_RST,HIGH);
    data[0]=0xFF;
    int v = digitalRead(SC_C7_IO); 
    int err= sc._receiveByteStd(data,40000) ;
    sc.lasterror= 0;
    uint8_t err= sc._receiveTSByte();
    MySerial.println(sc._etu);
    MySerial.println(v);
    MySerial.println(data[0],HEX);
    MySerial.println(err,HEX);
    MySerial.println(sc.lasterror);
    */
  }
  
  int ftck= -1;
  if (atr_received >0) ftck= ifd_atr_parse(&info,data,atr_received);
  if (ftck >=0) 
  { ftck= ifd_atr_parse(&info,data,atr_received);

    #ifdef ATRVERBOSE
    if (fdebug)
    {
    MySerial.print("T");MySerial.print(info.default_protocol);MySerial.print(" {");
    if (info.supported_protocols & 0x1) MySerial.print("T0 ");
    if (info.supported_protocols & 0x2) MySerial.print("T1"); 
    MySerial.println("}");
    }
    #endif

    if (auto_p)
    {
    if (info.supported_protocols & 0x1)      ptcol=0;
    else if (info.supported_protocols & 0x2) ptcol=1; 
    else ptcol=0 ;
    myptcol=ptcol;
    }
    else
    ptcol=myptcol;
    
    for (int i=0;i<NTD;i++)
    { if (info.TA[i] != -1) 
      { 
        #ifdef ATRVERBOSE
        if (fdebug) {MySerial.print("TA"); MySerial.print(i+1);MySerial.print("= ");MySerial.println(info.TA[i],HEX);}
        #endif
        if (i == 0)
        { Fi = 0x0F & (info.TA[i]>>4) ;
          Di = 0x0F & info.TA[i];
          #ifdef ATRVERBOSE
          if (fdebug) {MySerial.print("Fi=");MySerial.print(Fi);MySerial.print(" Di=");MySerial.println(Di);}
          #endif
        }
      }
      if (info.TB[i] != -1) 
      { 
        #ifdef ATRVERBOSE
        if (fdebug) {MySerial.print("TB"); MySerial.print(i+1);MySerial.print("= ");MySerial.println(info.TB[i],HEX);}
        #endif
      }
      if (info.TC[i] != -1) 
      { 
        #ifdef ATRVERBOSE
        if (fdebug) {MySerial.print("TC"); MySerial.print(i+1);MySerial.print("= ");MySerial.println(info.TC[i],HEX);}
        #endif
        
        if (i == 0)
        { N = 0xFF & info.TC[i];
          #ifdef ATRVERBOSE
          if (fdebug) {MySerial.print("N="); MySerial.println(N);};
          #endif
          if (N==255) N=0;
        }
      }
    }
    if (info.t1 != -1) 
    { 
      #ifdef ATRVERBOSE
      if (fdebug) {MySerial.print("T1i=");MySerial.println(info.t1+1);}
      #endif
      
     if ( ((info.t1+1) > 1) && ((info.t1+1) < NTD) )
     {
      if (info.TA[info.t1+1] != -1)
      { cardifs = 0xFF & info.TA[info.t1+1];
        #ifdef ATRVERBOSE
        if (fdebug) {MySerial.print("IFS=");MySerial.println(cardifs);}
        #endif
        //myifs=cardifs ;
      }
      if (info.TB[info.t1+1] != -1)
      { CWI = 0x0F & info.TB[info.t1+1];
        #ifdef ATRVERBOSE
        if (fdebug) {MySerial.print("CWI=");MySerial.println(CWI);}
        #endif
        BWI = 0x0F & (info.TB[info.t1]>>4);
        #ifdef ATRVERBOSE
        if (fdebug) {MySerial.print("BWI=");MySerial.println(BWI);}
        #endif
      }
      if (info.TC[info.t1+1] != -1)
      { if (info.TC[info.t1+1] & 0x01) { mycrc= true ;
                                         #ifdef ATRVERBOSE
                                         if (fdebug) MySerial.println("CRC");
                                         #endif
                                       }
        else                           { mycrc=false ;
                                         #ifdef ATRVERBOSE
                                         if (fdebug) MySerial.println("EDC");
                                         #endif
                                       }
      }
    }
    }

    if (info.t15 != -1) 
    { 
      #ifdef ATRVERBOSE
      if (fdebug) {MySerial.print("T15i=");MySerial.println(info.t15+1);}
      #endif
    }
  
    
  }
  
 #ifdef ATRVERBOSE
 if (fdebug)
  {
  MySerial.print("SC Frequency = ");
  MySerial.print(sc.fclk/sc.fscale);
  MySerial.println(" KHz");
  MySerial.print("TCK = ");
  MySerial.println(ftck);
  }
  #endif
 
  if (ftck < 0)
  {
  histlen=atr_received;
  for (uint8_t i=0;i<histlen; i++)
    histbytes[i] = data[i];
  }
  else
  { histlen= 0x0F & data[1];
    for (uint8_t i=0;i<histlen; i++)
    histbytes[i] = data[atr_received-histlen-ftck+i];
  }
  
  if ( (atr_received > 0) && (pts == true) )
  { 
    pts=false;
    while (!pts && (ptsct < 1) )
    {
     pts= PTS(MyTA,ptcol);
     if (!pts) // pts fail => try warm reset
     {  ptsct++;
        atr_received = sc._WarmReset(data, MAX_ATR_BYTES);
        if (atr_received <= 0) break ;
     }
    }

    if (!pts)
    { 
      #ifdef ATRVERBOSE
      if (fdebug)  MySerial.println("\nNo PTS");
      #endif
      atr_received = sc.activate(data,MAX_ATR_BYTES);
      pts=false;
    }
    else
    { 
      #ifdef STM32
      if (fdebug) MySerial.println("PTS OK");
      #endif
      pts = true ;
    }
    
  }

 // update pts for next restart
 mypts=pts; 

if ( atr_received > 0)
{
if (fdebug) dumpHEX(data, atr_received);

    // The specific interface byte TC(2) codes the integer value WI over the eight bits; 
    // the null value is reserved for future use.
    // If no TC(2) appears in the Answer-to-Reset, then the default value is WI = 10.
    // The interval between the leading edge of any character sent by the card 
    // and the leading edge of the previous character (sent either by the card 
    // or by the interface device) shall not exceed 960 x WI x (Fi / f). 
    // This maximum delay is named the work waiting time.

    //   T=0
    //   WWT = WI . 960 . F/f secondes, le temps d'attente maximum d'une réponse carte par le lecteur.

    // T=1
    // CWT = (2**CWI + 11)etu  CWI=0, 12, 1+8+1+2 Character Waiting Time
    // BWT = (2**BWI X 960 X 372 / fs) seconde + 11 etu // 5,713,920 /fs  + 11 etu
    // BGT is defined as the minimum delay between the leading edges 
    // of two consecutive characters sent in opposite directions. 
    // The value of BGT shall be 22 etu.


    /*
    if (fdebug)
    {
    MySerial.print("etu =");
    MySerial.print(sc._etu);
    MySerial.print(" guardTime=");
    MySerial.print(sc._guardTime);
    MySerial.print(" wwt=");
    MySerial.print(sc._wwt);
    MySerial.print(" max_wwt=");
    MySerial.println(sc._max_wwt);
    }
    */


    if (!pts)
    { F=372; D=1;}

    else
    { /*
      if      (MyTA == (char)0x11) { F=372;  D=1;}
      else if (MyTA == (char)0x12) { F=372;  D=2;}
      else if (MyTA == (char)0x13) { F=372;  D=4;}
      else if (MyTA == (char)0x14) { F=372;  D=8;}
      */
      FiDi(MyTA,&F,&D);
    }

    if (fdebug)
    {
    MySerial.print("F= ");
    MySerial.print(F);
    MySerial.print(" D= ");
    MySerial.println(D);
    }

     sc._etu       = (long) ((long)sc.fscale * F*(long)1000) / (long)D  / (long)fs ; // (us) =  F/D/fs (en MHz), F=372, D=1 fs=2

#ifdef TIMERX4
     sc._Setu      = (long) ((long)sc.fscale * F*(long)1000*(long)16) / (long)D  / (long)fs ; // (us) 
#endif

#ifdef TIMERX2
     sc._Setu2     = (long) ((long)sc.fscale * F*(long)1000*(long)2)  / (long)D  / (long)fs ; // (us) =  F/D/fs (en MHz), F=372, D=1 fs=2
#endif

#ifdef ASTM32
     sc._Setu     = (long) ((long)sc.fscale * (long)FTX* F *(long)1000) / (long)D  / (long)fs ; // (us) =  F/D/fs (en MHz), F=372, D=1 fs=2
#endif
     
     sc._guardTime = ((long)2 + (long)N) * (long)sc._etu;
     sc._wwt       = (long)WI * (long)960 * (long)D * (long)sc._etu ;
     sc._max_wwt   = sc._wwt ;

     sc._cwt = ((long)12 * sc._etu) ;
     sc._bwt = (long)(10000000L)   ;//(long)(5000000L);
     sc._bgt = 1L*((long)22 * sc._etu);
     if (myptcol != (char)0) 
     { sc._guardTime= (2L * sc._etu);// 10 + 2
       sc._cwt = (((long)1<<CWI)+ 11L) * sc._etu ;
       last_send_sequence = 0    ;
       last_received_sequence = 0;
     }

     if (fdebug)
     { MySerial.print("etu =")  ;
       MySerial.println(sc._etu);
       
     #ifdef ATRVERBOSE
        
    #ifdef ASTM32
     MySerial.print("Super_etu= ");
     MySerial.println(sc._Setu);
    #endif
       
      #ifdef TIMERX4
       MySerial.print("Super_etu= ");
       MySerial.println(sc._Setu);
      #endif
     
     #ifdef TIMERX2
     MySerial.print("Super_etu=");
     MySerial.println(sc._Setu2);
     #endif
     
    MySerial.print("guardTime= ");
    MySerial.print(sc._guardTime);
    if (myptcol == (char)0)
    { 
    MySerial.print(" wwt=");
    MySerial.print(sc._wwt);
    MySerial.print(" max_wwt=");
    MySerial.println(sc._max_wwt);
    }
    else
    {
    MySerial.print(" cwt= ");
    MySerial.print(sc._cwt);
    MySerial.print(" bwt=");
    MySerial.print(sc._bwt);  
    MySerial.print(" bgt=");
    MySerial.println(sc._bgt);  
    }
#endif
   }
        
}
  else
  {
   //if (fdebug) MySerial.println("no smartcard ...");
   sc.deactivate();
   return false;
  }

  if (myptcol == 1) ifst1(myifs);
  return true;
}

/*
void myPrintf(char *str, char *vli, int size)
{ int i;
  char buf[80];

  if (size <= 0) return ;
  
  sprintf(buf, "%s: %d", str,size);
  MySerial.println(buf);
  buf[0] = 0;
  for (i = 0; i < size; ++i)
  {
    sprintf(&buf[strlen(buf)], "%02X", 0xFF & (unsigned)vli[i]);
    if (i % 32 == 31)
    {  MySerial.println(buf);
      buf[0] = 0;
    }
  }

  i--;
  if ((i % 32) != 31)
     MySerial.println(buf);
}
*/

/********** T=1 TPDU block ********************/
#define TPDU_T1_DATA_MAXLEN  254

#define PCB_M_POS   5
#define PCB_M_MSK   (0x1 << PCB_M_POS)
#define PCB_M_NO_CHAIN    (0x0 << PCB_M_POS)
#define PCB_M_CHAIN   (0x1 << PCB_M_POS)

#define PCB_IBLOCK_POS    7
#define PCB_IBLOCK_MSK    (0x1 << PCB_IBLOCK_POS)
#define PCB_IBLOCK    (0x0 << PCB_IBLOCK_POS)

#define PCB_RBLOCK_POS    6
#define PCB_RBLOCK_MSK    (0x3 << PCB_RBLOCK_POS)
#define PCB_RBLOCK    (0x2 << PCB_RBLOCK_POS)

#define PCB_SBLOCK_POS    6
#define PCB_SBLOCK_MSK    (0x3 << PCB_SBLOCK_POS)
#define PCB_SBLOCK    (0x3 << PCB_SBLOCK_POS)

#define PCB_ISEQ_NUM_POS  6
#define PCB_ISEQ_NUM_MASK (0x1 << PCB_ISEQ_NUM_POS)
#define PCB_ISEQ_NUM0   (0x0 << PCB_ISEQ_NUM_POS)
#define PCB_ISEQ_NUM1   (0x1 << PCB_ISEQ_NUM_POS)

#define PCB_RSEQ_NUM_POS  4
#define PCB_RSEQ_NUM_MASK (0x1 << PCB_RSEQ_NUM_POS)
#define PCB_RSEQ_NUM0   (0x0 << PCB_RSEQ_NUM_POS)
#define PCB_RSEQ_NUM1   (0x1 << PCB_RSEQ_NUM_POS)

#define PCB_ERR_POS   0
#define PCB_ERR_MASK    (0x3 << PCB_ERR_POS)
#define PCB_ERR_NOERR   (0x0 << PCB_ERR_POS)
#define PCB_ERR_EDC   (0x1 << PCB_ERR_POS)
#define PCB_ERR_OTHER   (0x2 << PCB_ERR_POS)

/* SBLOCK types */
#define SBLOCK_TYPE_MSK   0x3f
#define SBLOCK_RESYNC_REQ 0x00
#define SBLOCK_RESYNC_RESP  0x20
#define SBLOCK_CHANGE_IFS_REQ 0x01
#define SBLOCK_CHANGE_IFS_RESP  0x21
#define SBLOCK_ABORT_REQ  0x02
#define SBLOCK_ABORT_RESP 0x22
#define SBLOCK_WAITING_REQ  0x03
#define SBLOCK_WAITING_RESP 0x23
#define SBLOCK_VPP_ERR_RESP 0x24

#define EDC_TYPE_LRC    0
#define EDC_TYPE_CRC    1

/////////////////////////////////////////////////////////////
/***************** T=1 case ********************************/
/* Compute the checksum (LRC) of a TPDU */
uint8_t SC_TPDU_T1_lrc(SC_TPDU *tpdu){
  unsigned int i;
  uint8_t lrc = 0;

  if(tpdu == NULL){
    return 0;
  }

  lrc ^= tpdu->nad;
  lrc ^= tpdu->pcb;
  lrc ^= tpdu->len;
  for(i = 0; i < tpdu->len; i++){
    lrc ^= tpdu->data[i];
  }

  return lrc;
}

/* Compute the checksum (CRC-16) of a TPDU */
/* [RB] TODO: check the CRC-16 algorithm ... */
#define CRC_BLOCK(in, crc, poly) do {     \
  unsigned int j;         \
  uint32_t data;          \
  data = (in);          \
  for(j = 0; j < 8; j++){       \
    if(((crc) & 0x0001) ^ (data & 0x0001)){ \
      (crc) = ((crc) >> 1) ^ (poly);  \
    }         \
    else{         \
      (crc) >>= 1;      \
    }         \
    data >>= 1;       \
  }           \
} while(0);

uint16_t SC_TPDU_T1_crc(SC_TPDU *tpdu){
  unsigned int i;
  uint32_t poly = 0x8408; /* CCIT polynomial x16 + x12 + x5 + 1 */
  uint32_t crc  = 0xffff;

  if(tpdu == NULL){
    return 0;
  }

  CRC_BLOCK(tpdu->nad, crc, poly);
  CRC_BLOCK(tpdu->pcb, crc, poly);
  CRC_BLOCK(tpdu->len, crc, poly);
  for(i = 0; i < tpdu->len; i++){
    CRC_BLOCK(tpdu->data[i], crc, poly);
  }

  crc = ~crc;
  crc = (crc << 8) | ((crc >> 8) & 0xff);

  return (uint16_t)crc;
}

/* Compute the checksum of a TPDU */
void SC_TPDU_T1_checksum_compute(SC_TPDU *tpdu, int fcrc)
{

  if(tpdu == NULL)
  {
    return;
  }

  /* The method used for the checksum depends on ATR byte (LRC or CRC). Default is LRC.
   * TCi (i>2) contains this information.
   */
  tpdu->edc_type = EDC_TYPE_LRC;
  if (fcrc) 
  tpdu->edc_type = EDC_TYPE_CRC;
  

  if(tpdu->edc_type == EDC_TYPE_LRC){
    /* LRC is the xor of all the bytes of the TPDU */
    tpdu->edc_lrc = SC_TPDU_T1_lrc(tpdu);
    return;
  }
  else{
    /* CRC type */
    tpdu->edc_crc = SC_TPDU_T1_crc(tpdu);
  }

  return;
}
/* Check the checksum of a TPDU */
int SC_TPDU_T1_checksum_check(SC_TPDU *tpdu){

  if(tpdu == NULL){
    return 0;
  }

  if(tpdu->edc_type == EDC_TYPE_LRC){
    /* LRC is the xor of all the bytes of the TPDU */
    if(tpdu->edc_lrc != SC_TPDU_T1_lrc(tpdu)){
      return 0;
    }
  }
  else{
    /* CRC type */
    if(tpdu->edc_crc != SC_TPDU_T1_crc(tpdu)){
      return 0;
    }
  }

  return 1;
}


void testcrc()
{ //char testcrc[5]= {0xA5,0x81,0x00,0xB2,0x65};
  SC_TPDU tpdu  ;
  char crc[2]   ;
  tpdu.nad=0xA5 ;
  tpdu.pcb=0x81 ;     
  tpdu.len=0x00 ;
  tpdu.data= (uint8_t *)NULL ;
  SC_TPDU_T1_checksum_compute(&tpdu,1);
  crc[0] = 0xFF & tpdu.edc_crc >> 8  ;
  crc[1] = 0xFF & tpdu.edc_crc ;
  //B265
  myPrintf("CRC",crc,2);
}


int ifst1(uint8_t ifs)
{ uint8_t p1[6] ;
  int i,ledc=1  ;
  SC_TPDU tpdu  ;
  
  delayMicroseconds(sc._bgt);

  if (mycrc) ledc++;
 
  tpdu.nad=0;
  tpdu.pcb=0xC1     ; //IFS REQUEST
  tpdu.len=0x01     ;
  tpdu.data= (uint8_t *)&ifs  ;
  SC_TPDU_T1_checksum_compute(&tpdu,mycrc);

  p1[0]=tpdu.nad;
  p1[1]=tpdu.pcb;
  p1[2]=tpdu.len;
  p1[3]=tpdu.data[0];
  if (!mycrc) p1[4]=tpdu.edc_lrc;
  else
  {p1[4] =tpdu.edc_crc >> 8  ;
   p1[5] =tpdu.edc_crc & 0xFF;
  }

   if (fdebug) myPrintf("TxT1",(char*)p1,4+ledc);
   sc._sendASyncBytes_T1(p1,4+ledc); 
   int result= sc._receiveASyncBytes_T1(p1,4+ledc,sc._bwt,sc._cwt);
   if (fdebug) myPrintf("RxT1",(char *)p1,result);
   delayMicroseconds(sc._bgt);

   return 0;
  
}


/* Push a TPDU on the line */
int SC_push_TPDU_T1(SC_TPDU *tpdu)
{
int err=0; // 0= OK, 1=ERROR
int ledc=1   ;
uint8_t p1[5];

if (mycrc) ledc++;

p1[0]=tpdu->nad;
p1[1]=tpdu->pcb;
p1[2]=tpdu->len;
if (!mycrc)
p1[3]=tpdu->edc_lrc;
else
{p1[3] =tpdu->edc_crc >> 8  ;
 p1[4] =tpdu->edc_crc & 0xFF;
}
   if (fdebug) myPrintf("TxT1",(char *)p1,3+ledc);

   if ((finject & (uint8_t)0x1) == (uint8_t)0x1)
   {
     finject &= (uint8_t)0xFE ;
     p1[3] ^= (uint8_t)1;
   }

   sc._sendASyncBytes_T1(p1,3);
   sc._sendASyncBytes_T1((uint8_t *)tpdu->data,(uint16_t)tpdu->len);
   sc._sendASyncBytes_T1(p1+3,(uint16_t)ledc);
 
   lastpcb= tpdu->pcb;

return err;
}

/* Pull a TPDU from the line */
int SC_pull_TPDU_T1(SC_TPDU *tpdu,uint8_t bwt_factor)
{
int err=0; //0 =OK 1=ERROR
uint16_t result=0 ;
int t1rx=0;
int ledc=1;
uint8_t p1[5];

if (mycrc) ledc++;

 t1rx=254+4; //Max + 4
 if (mycrc) t1rx += 1;

  
    // uint16_t _receiveASyncBytes_T1(uint8_t* buf, uint16_t buf_size,unsigned long bwt,unsigned long cwt);
   sc.t1_parity_err=0   ;
   t1_parity_error=false;
   result= sc._receiveASyncBytes_T1((uint8_t*)tpdu->data,t1rx,sc._bwt,sc._cwt);
   delayMicroseconds(sc._bgt);
   if (sc.t1_parity_err != 0) t1_parity_error=true;

   if (result > 0)
   {
   tpdu->nad = tpdu->data[0];
   lastpcb= tpdu->pcb = tpdu->data[1];
   tpdu->len = tpdu->data[2];
   if (mycrc) 
   { tpdu->edc_crc  = (0xFF00 & (tpdu->data[3+tpdu->len]<<8)) ;
     tpdu->edc_crc |= (0x00FF &  tpdu->data[4+tpdu->len]) ;
   }
   else
   tpdu->edc_lrc = tpdu->data[3+tpdu->len] ;

   if ((finject & (uint8_t)0x2) == (uint8_t)0x2)
   {
     finject &= (uint8_t)0xFD   ;  // 1101
     tpdu->edc_lrc ^= (uint8_t)1;
   }

   
   memmove(tpdu->data,tpdu->data+3,tpdu->len);
   lastpcb =  tpdu->pcb; ;

   p1[0]=tpdu->nad;
   p1[1]=tpdu->pcb;
   p1[2]=tpdu->len;
   if (!mycrc)
   p1[3]=tpdu->edc_lrc;
   else
   { p1[3] =tpdu->edc_crc >> 8  ;
     p1[4] =tpdu->edc_crc & 0xFF;
   }

   if (fdebug) myPrintf("RxT1",(char *)p1,3+ledc);
   return 0;
   }

lastpcb=0xFF;
return 1    ;
}
/*** T=1 helpers for block types and error handling ***/
int SC_TPDU_T1_is_IBLOCK(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return 0;
  }
  return ((tpdu->pcb & PCB_IBLOCK_MSK) == PCB_IBLOCK);
}

int SC_TPDU_T1_is_RBLOCK(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return 0;
  }
  return ((tpdu->pcb & PCB_RBLOCK_MSK) == PCB_RBLOCK);
}

int SC_TPDU_T1_is_SBLOCK(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return 0;
  }
  return ((tpdu->pcb & PCB_SBLOCK_MSK) == PCB_SBLOCK);
}

void SC_TPDU_T1_set_IBLOCK(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return;
  }
  tpdu->pcb |= PCB_IBLOCK;
  return;
}

void SC_TPDU_T1_set_RBLOCK(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return;
  }
  tpdu->pcb |= PCB_RBLOCK;
  return;
}

void SC_TPDU_T1_set_SBLOCK(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return;
  }
  tpdu->pcb |= PCB_SBLOCK;
  return;
}

uint8_t SC_TPDU_T1_get_sequence(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return 0xff;
  }
  if(SC_TPDU_T1_is_IBLOCK(tpdu)){
    return ((tpdu->pcb & PCB_ISEQ_NUM_MASK) >> PCB_ISEQ_NUM_POS);
  }
  else if(SC_TPDU_T1_is_RBLOCK(tpdu)){
    return ((tpdu->pcb & PCB_RSEQ_NUM_MASK) >> PCB_RSEQ_NUM_POS);
  }
  else{
    /* Should not happen */
    return 0xff;
  }
}

int SC_TPDU_T1_is_sequence_ok(SC_TPDU *tpdu, uint8_t sequence_number){
  return (SC_TPDU_T1_get_sequence(tpdu) == sequence_number);
}


int SC_TPDU_T1_set_sequence(SC_TPDU *tpdu, uint8_t sequence_number){
  if(tpdu == NULL){
    return -1;
  }
  if(SC_TPDU_T1_is_IBLOCK(tpdu)){
    tpdu->pcb |= (sequence_number << PCB_ISEQ_NUM_POS);
    return 0;
  }
  else if(SC_TPDU_T1_is_RBLOCK(tpdu)){
    tpdu->pcb |= (sequence_number << PCB_RSEQ_NUM_POS);
    return 0;
  }
  else{
    return -1;
  }
}

uint8_t SC_TPDU_T1_RBLOCK_get_error(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return 0xff;
  }
  /* Return an error if this is not an RBLOCK */
  if(!SC_TPDU_T1_is_RBLOCK(tpdu)){
    return 0xff;
  }
  return (tpdu->pcb & PCB_ERR_MASK);
}
/* Send an error frame with the given error and the given frame sequence */
int SC_TPDU_T1_send_error(uint8_t pcb_err, uint8_t sequence_number, int fcrc)
{
  SC_TPDU err_tpdu;

  err_tpdu.nad = 0;

  /* PCB is an RBLOCK */
  err_tpdu.pcb = 0;
  SC_TPDU_T1_set_RBLOCK(&err_tpdu);
  SC_TPDU_T1_set_sequence(&err_tpdu, sequence_number);
  /* Put the error field */
  err_tpdu.pcb |= pcb_err;
  /* No data */
  err_tpdu.len  = 0;
  err_tpdu.data = NULL;
  /* Compute the checksum */
  SC_TPDU_T1_checksum_compute(&err_tpdu,fcrc);

  /* Send the error on the line */
  SC_push_TPDU_T1(&err_tpdu);

  return 0;
}

/* Get the type of an SBLOCK */
uint8_t SC_TPDU_T1_SBLOCK_get_type(SC_TPDU *tpdu){
  if(tpdu == NULL){
    return 0xff;
  }

  /* Return an error if this is not an SBLOCK */
  if(!SC_TPDU_T1_is_SBLOCK(tpdu)){
    return 0xff;
  }

  return (tpdu->pcb & SBLOCK_TYPE_MSK);
}

int SC_TPDU_T1_SBLOCK_get_waiting_time(SC_TPDU *tpdu, uint8_t *waiting_time){
  if((tpdu == NULL) || (waiting_time == NULL)){
    goto err;
  }

  *waiting_time = 0;
  /* Sanity check: is this an SBLOCK? */
  if(!SC_TPDU_T1_is_SBLOCK(tpdu)){
    goto err;
  }
  /* The waiting time should be encoded in a one byte data field
   * as a multiple of the BWT (Block Waiting Time).
   */
  if((tpdu->len != 1) || (tpdu->data == NULL)){
    goto err;
  }
  *waiting_time = tpdu->data[0];

  return 0;
err:
  return -1;
}
/* Get the new IFS asked by an SBLOCK */
int SC_TPDU_T1_SBLOCK_get_new_ifs(SC_TPDU *tpdu, uint8_t *new_ifs){
  if((tpdu == NULL) || (new_ifs == NULL)){
    goto err;
  }

  *new_ifs = 0;
  /* Sanity check: is this an SBLOCK? */
  if(!SC_TPDU_T1_is_SBLOCK(tpdu)){
    goto err;
  }
  /* The new IFS should be encoded in a one byte data field.
   */
  if((tpdu->len != 1) || (tpdu->data == NULL)){
    goto err;
  }
  *new_ifs = tpdu->data[0];

  return 0;
err:
  return -1;
}


/* Send an SBLOCK */
int SC_TPDU_T1_send_sblock(uint8_t sblock_type, uint8_t *data, uint8_t size,int fcrc)
{
  SC_TPDU s_tpdu;

  if(data == NULL)
  {
    return -1;
  }

  s_tpdu.nad = 0;

  /* PCB is an SBLOCK */
  s_tpdu.pcb = 0;
  SC_TPDU_T1_set_SBLOCK(&s_tpdu);
  /* Set the SBLOCK type */
  s_tpdu.pcb |= sblock_type;
  /* Is there data? No data */
  if(size > TPDU_T1_DATA_MAXLEN){
    /* Sanity check */
    return 0;
  }
  s_tpdu.len  = size;
  s_tpdu.data  = data;
  /* Compute the checksum */
  SC_TPDU_T1_checksum_compute(&s_tpdu,fcrc);

  /* Send the SBLOCK on the line */
  SC_push_TPDU_T1(&s_tpdu);

  return 0;
}

// i block number of size ifsc
int SC_APDU_prepare_buffer(uint8_t *apdu, int len,uint8_t **buffer_send, int i, int ifsc,int * ret)
{  int size=0;
   
   if ( (i*ifsc) >= len ) {*ret=1;return 0;};
   
   *ret=0;
    size= len-(i*ifsc);
  if (size > ifsc) size= ifsc;
  *buffer_send= apdu+(i*ifsc);
  return size;
}


void SC_delay_BGT()
{ //delayMicroseconds(sc._bgt);
}

/* Send APDU in T=1 and get the response
 * [RB] FIXME: for now, this is a basic yet straightforward way of handling T=1. Some
 * error/corner cases are not implemented yet! However, this should work
 * for the basic interactions with cards we need.
 */

uint16_t SC_send_APDU_T1(uint8_t *apdu, unsigned int lena,uint8_t *resp,int ifsc, int fcrc)
{
  /* Create an IBLOCK in order to send our APDU */
  SC_TPDU tpdu_send;
  SC_TPDU tpdu_rcv ;
  unsigned int i, num_iblocks; //, bwi, cwi;
  /* Internal working buffers.
   * Note: we can work with only one buffer for both send and receive, but
   * this is cleaner to split those two for debug purposes. Additionally, the
   * buffers size is 254 bytes maximum imposed by the standard, which is reasonable
   * on our STM32F4 platform.
   */
  uint8_t *buffer_send ;
  
  unsigned int encapsulated_apdu_len=0;
  unsigned int received_size=0;
  uint8_t bwt_factor = 1; /* BWT factor for waiting time extension */
  t1_error=0;

  if( (apdu == NULL) || (resp == NULL) )
  {   lastpcb= 0x1F;
      goto err;
  }
  

  /* Compute the length we will have to send */
  encapsulated_apdu_len = lena; 

  /* How much IBLOCKS do we need? */

  num_iblocks = (encapsulated_apdu_len /ifsc) + 1;
  if(((encapsulated_apdu_len % ifsc) == 0) && (encapsulated_apdu_len != 0)){
    num_iblocks--;
  }
    
  //CWT CWT_character_wait_time
  //BWT BWT_block_wait_time

  /* Sanity zeroize */
  memset(&tpdu_rcv, 0, sizeof(tpdu_rcv))  ;
  memset(&tpdu_send, 0, sizeof(tpdu_send));

  /* NAD is always zero in our case (no multi-slaves) */
  tpdu_send.nad = 0; 

  /* Send all the IBLOCKS */
  for(i=0; i < num_iblocks; i++){
    int ret;
    tpdu_send.pcb = 0;
    /* PCB is an IBLOCK */
    SC_TPDU_T1_set_IBLOCK(&tpdu_send);
    if(i != (num_iblocks-1)){
      /* Blocks are chained except for the last one */
      tpdu_send.pcb |= PCB_M_CHAIN;
    }
    /* Set the sequence number */
    SC_TPDU_T1_set_sequence(&tpdu_send, last_send_sequence);
    /* Flip the sequence number for the next block to send */
    last_send_sequence = (last_send_sequence + 1) % 2;
    
    /* Compute the length to send and prepare the buffer */
    tpdu_send.len = SC_APDU_prepare_buffer(apdu,lena,&buffer_send,i,ifsc, &ret);
    
    if (ret)
    {   lastpcb= 0x1F;
      goto err;
    }

    /* Adapt the data pointer */
    tpdu_send.data = buffer_send;

    /* Compute the checksum */
    SC_TPDU_T1_checksum_compute(&tpdu_send,fcrc);

SEND_TPDU_AGAIN_CMD:
    /* Reset the BWT factor to 1 */
    bwt_factor = 1;
    /* Send the TPDU */
    if(SC_push_TPDU_T1(&tpdu_send))
    {   
      lastpcb= 0x1F;
      goto err;
    }
RECEIVE_TPDU_AGAIN_CMD:
    /* Get the ACK from the card */
    //tpdu_rcv.data = buffer_recv;
    tpdu_rcv.data = resp + received_size ;
    tpdu_rcv.len = 0;
    tpdu_rcv.edc_type = tpdu_send.edc_type;
        
    if(!SC_pull_TPDU_T1(&tpdu_rcv, bwt_factor))
    {
    /* If the checksum of the received block is wrong, 
       send an error R block and receive again */
      
       //SC_print_TPDU(&tpdu_rcv);

      if ( (!SC_TPDU_T1_checksum_check(&tpdu_rcv)) || t1_parity_error)
      {   
        t1_error++;
        if (t1_error > T1_ERROR_MAX) { lastpcb= 0x01; 
                                       goto err; }
        /* Wait a bit and send a parity error */
        SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
        
        SC_TPDU_T1_send_error(PCB_ERR_EDC, SC_TPDU_T1_get_sequence(&tpdu_rcv),fcrc);
        goto RECEIVE_TPDU_AGAIN_CMD;
      }
      /* If we have an error, send again */
      if(SC_TPDU_T1_is_RBLOCK(&tpdu_rcv) && (SC_TPDU_T1_RBLOCK_get_error(&tpdu_rcv) != PCB_ERR_NOERR))
      {
        /* Check the sequence number */
        if(SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, SC_TPDU_T1_get_sequence(&tpdu_send)))
        {

            t1_error++;
            if (t1_error > T1_ERROR_MAX) {lastpcb= tpdu_rcv.pcb;goto err; }

          /* Genuine error, send again */
          SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
          goto SEND_TPDU_AGAIN_CMD;
        }
        /* Unexpected error */
        //printf("[Smartcard T=1] Unexpected case: received error block with bad sequence number ...\n");
        lastpcb= tpdu_rcv.pcb;
        goto err;
      }
      /* Check that this is the ACK we are waiting for */
      if(i != (num_iblocks - 1)){
        /* This is not the last block, we should receive a R type block with a last transmitted I Block sequence + 1 */
        if(!SC_TPDU_T1_is_RBLOCK(&tpdu_rcv) || !SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, (SC_TPDU_T1_get_sequence(&tpdu_send) + 1) % 2))
        {   lastpcb= 0x03;
          /* This is not what we expect */
          //printf("[Smartcard T=1] Unexpected case: received other block than expected RBLOCK, or bad sequence number ...\n");
          goto err;
        }
      }
      else{
        /* This is the last block, we should receive at least one I type block with a last I Block received sequence + 1 value
         */
        if(!SC_TPDU_T1_is_IBLOCK(&tpdu_rcv) || !SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, last_received_sequence)){
          /* If we have an error, send again */
          if(SC_TPDU_T1_is_RBLOCK(&tpdu_rcv) && (SC_TPDU_T1_RBLOCK_get_error(&tpdu_rcv) != PCB_ERR_NOERR)){
            /* Check the sequence number */
            if(SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, SC_TPDU_T1_get_sequence(&tpdu_send)))
            {   
              t1_error++;
                    if (t1_error > T1_ERROR_MAX) { lastpcb= 0x04; goto err; }

              /* Genuine error, send again */
              SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
              goto SEND_TPDU_AGAIN_CMD;
            }
            /* Unexpected error */
            //printf("[Smartcard T=1] Unexpected case: received error block with bad sequence number ...\n");
            lastpcb= 31;
            goto err;
          }
          /* If this is something else, fallback to our error case ... */
          if(SC_TPDU_T1_is_SBLOCK(&tpdu_rcv)){
            /* If this is an SBLOCK we should not receive, this is an error ... */
            if((SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_RESYNC_REQ) || (SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_RESP))
            {   lastpcb= 0x05;
              //printf("[Smartcard T=1] Unexpected SBLOCK reveived from smartcard (SBLOCK_RESYNC_REQ or SBLOCK_WAITING_RESP)\n");
              goto err;
            }
            /* If this is a Request Waiting Time extension, answer and go back to waiting our response ... */
            if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_REQ){
              /* Get the expected waiting time in number of BWT */
              if(SC_TPDU_T1_SBLOCK_get_waiting_time(&tpdu_rcv, &bwt_factor))
              { lastpcb= 0x06;
                goto err;
              }
              /* Acknowledge the waiting time */
              SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
              SC_TPDU_T1_send_sblock(SBLOCK_WAITING_RESP, tpdu_rcv.data, tpdu_rcv.len, fcrc);
              goto RECEIVE_TPDU_AGAIN_CMD;
            }
            if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_CHANGE_IFS_REQ){
              /* Get the new IFSC */
              uint8_t new_ifsc;
              if(SC_TPDU_T1_SBLOCK_get_new_ifs(&tpdu_rcv, &new_ifsc))
              { lastpcb= 0x07;
                goto err;
              }
              if((new_ifsc == 0) || (new_ifsc == 255))
              {  lastpcb= 0x08;
                //printf("[Smartcard T=1] Bad value for IFSC asked with SBLOCK_CHANGE_IFS_REQ = %d\n", new_ifsc);
                goto err;
              }
              else{
                /* Record the new current IFSC in our ATR context: this will be actibe
                 * from now on for the next transactions.
                 */
                ifsc = new_ifsc;
                /* Acknowledge the new IFSC */
                SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
                SC_TPDU_T1_send_sblock(SBLOCK_CHANGE_IFS_RESP, tpdu_rcv.data, tpdu_rcv.len, fcrc);
                /* Go back to waiting the IBlock */
                goto RECEIVE_TPDU_AGAIN_CMD;
              }
            }
            /* Else, fallback to error since SBLOCKS are not fully implemented */
            //printf("[Smartcard T=1] S blocks automaton not fully implemented yet!\n");
             lastpcb= 9;goto err;
          }
          //printf("[Smartcard T=1] Unexpected case: received other block than expected IBLOCK, or bad sequence number ...\n");
          lastpcb= 10;goto err;
        }
        /* Now get out and receive other necessary I blocks */
      } // last block
    }
    else{
      /* Error pulling the response ... */
      // printf("[Smartcard T=1] TPDU response reception error 1 ...\n");
      lastpcb= 11;goto err;
    }
  } // next I-Bloc
  /* If we are here, we have received at least one IBlock. We have to check
   * if more blocks have to be received.
   */
  /* Reset the BWT factor to 1 */
  bwt_factor = 1;
  received_size += tpdu_rcv.len;
  tpdu_rcv.data = resp + received_size ;
    
  while(1)
  { // Flip the last reception sequence counter 
    last_received_sequence = (last_received_sequence + 1) % 2;
       
    /* More IBlocks are to be received */
    if((tpdu_rcv.pcb & PCB_M_MSK) == PCB_M_CHAIN){
      SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
      /* Send an ACK with the received IBlock sequence + 1 */
      SC_TPDU_T1_send_error(PCB_ERR_NOERR, (SC_TPDU_T1_get_sequence(&tpdu_rcv)+1)%2, fcrc);
RECEIVE_TPDU_AGAIN_RESP:
      /* Receive the new block */
      if(SC_pull_TPDU_T1(&tpdu_rcv, bwt_factor))
      {
        //printf("[Smartcard T=1] TPDU response reception error 2 ...\n");
       lastpcb= 12;
        goto err;
      }

            //SC_print_TPDU(&tpdu_rcv);

      /* If the checksum of the received block is wrong, send an error R block and receive again */
      if(!SC_TPDU_T1_checksum_check(&tpdu_rcv)){
        /* Wait a bit and send a parity error */
        SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
        SC_TPDU_T1_send_error(PCB_ERR_EDC, SC_TPDU_T1_get_sequence(&tpdu_rcv), fcrc);
        goto RECEIVE_TPDU_AGAIN_RESP;
      }
      /* If this is not an IBlock, check if this is an SBLOCK and perform the appropriate action.
       * [RB] TODO: handle the *full* resync automaton here instead of aborting ...
       */
      if(!SC_TPDU_T1_is_IBLOCK(&tpdu_rcv))
      {
        if(SC_TPDU_T1_is_SBLOCK(&tpdu_rcv)){
          /* If this is an SBLOCK we should not receive, this is an error ... */
          if((SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_RESYNC_REQ) || (SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_RESP))
          {  lastpcb= 13;
            //printf("[Smartcard T=1] Unexpected SBLOCK received from smartcard (SBLOCK_RESYNC_REQ or SBLOCK_WAITING_RESP)\n");
            goto err;
          }
          /* If this is a Request Waiting Time extension, answer and go back to waiting our response ... */
          if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_REQ){
            /* Get the expected waiting time in number of BWT */
            if(SC_TPDU_T1_SBLOCK_get_waiting_time(&tpdu_rcv, &bwt_factor))
            { lastpcb= 14;
              goto err;
            }
            /* Acknowledge the waiting time */
            SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
            SC_TPDU_T1_send_sblock(SBLOCK_WAITING_RESP, tpdu_rcv.data, tpdu_rcv.len, fcrc);
            goto RECEIVE_TPDU_AGAIN_RESP;
          }
          if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_CHANGE_IFS_REQ){
            /* The smartcard is negotiating a new IFSC, modify it */
            /* Get the new IFSC */
            uint8_t new_ifsc;
            if(SC_TPDU_T1_SBLOCK_get_new_ifs(&tpdu_rcv, &new_ifsc))
            {   lastpcb= 15;
              goto err;
            }
            if((new_ifsc == 0) || (new_ifsc == 255))
            {
              //printf("[Smartcard T=1] Bad value for IFSC asked with SBLOCK_CHANGE_IFS_REQ = %d\n", new_ifsc);
              lastpcb= 16;
              goto err;
            }
            ifsc = new_ifsc;
            /* Acknowledge the new IFSC */
            SC_delay_BGT(); /* Wait for the standardized Block Guard Time (22 ETU by default) */
            SC_TPDU_T1_send_sblock(SBLOCK_CHANGE_IFS_RESP, tpdu_rcv.data, tpdu_rcv.len, fcrc);
          }
          /* Else, fallback to error since SBLOCKS are not fully implemented */
          //printf("[Smartcard T=1] S blocks automaton not fully implemented yet!\n");

          lastpcb= 17;goto err;
        }
        else
        {
          //printf("[Smartcard T=1] TPDU response reception error: expected IBLOCK but got something else ...\n");
          lastpcb= 18;goto err;
        }
      } // end of I Block received
      /* Reset the BWT factor to 1 after each I-Block received */
      bwt_factor = 1;
      received_size += tpdu_rcv.len;
      tpdu_rcv.data =  resp + received_size ;

    }
    else
    {
      /* We are finished, nothing more to get. Get out. */
      break;
    }
  }// While(1)
  
  /* We have received everything, format the response */
  return received_size ;

err:
  
  /* We have an error, clean up stuff */
  return 0;
}

#define BADSW1 0x6F
int send_apdu(char *req, int asize, char *resp, bool foutput,bool first)
{
  boolean do_verbose = fdebug ;
  boolean f_wr = true;
  APDU_t   command;
  int i, PSIZE = 16;
  char v[5];
  unsigned long t1,t2;
  int t1rx=0;
  uint16_t result ;
    
 if (do_verbose)
  { MySerial.print("Tx: ");
    for (i = 0; i < asize; i++)
    { sprintf(v, "%02X", 0xFF & req[i]);
      MySerial.print(v);
      if ((i % PSIZE) == (PSIZE - 1) ) {
        MySerial.println();
        MySerial.print("    ");
      }
    }
    if ((i % PSIZE) != 0) MySerial.println();
  }

  if (myptcol == (char)0)
  {  
  command.cla       = (unsigned char)req[0] ;
  command.ins       = (unsigned char)req[1] ;
  command.p1        = (unsigned char)req[2] ;
  command.p2        = (unsigned char)req[3] ;
  command.data_size = (unsigned char) req[4] & 0xFF  ;
  if (asize == 5)
  { f_wr = false ;
    command.data_buf  = (unsigned char *)resp;
    if (foutput) command.data_size= 256;
  }
  else
   command.data_buf  = (unsigned char *)&req[5];
 
  command.resp_size = 0x100 ;
  }

  else // T=1
  { t1rx=254+4; //Max + 4
    if (mycrc) t1rx += 1;
  }

  //  uint16_t sc.sendAPDU(APDU_t* command, boolean send=true, uint16_t maxwaits=0); 
  // return 0 if error

  if (myptcol == (char)0) 
  { if (do_verbose) t1=micros();
    result = sc.sendAPDU(&command, f_wr, 10000); // timeout 10s
    if (result == 0)
    { command.resp_length = 0;
      result=0x6F00;
    }
  }
  else // t=1
  { if (do_verbose) t1=micros();

    // return size of response, at least 2
    result= SC_send_APDU_T1((uint8_t *)req,asize,(uint8_t *)resp,myifs,mycrc);
    t1rx=result;

    if (result <= 1)
    { command.resp_length=0;resp[0]=BADSW1;resp[1]=lastpcb ;t1rx=2;
      result =  0xFF00 & resp[0]<<8; 
      result |= 0xFF   & resp[1];
      goto sendend;
    }
    result =  0xFF00 & (resp[t1rx-2]<<8); // SW1 SW2
    result |= 0xFF   & resp[t1rx-1];
    command.resp_length = 0xFF & (t1rx-2);
   }

  sendend:
  
  if (do_verbose) t2=micros();

  if (do_verbose)
  { 
    if (myptcol == (char)0)
    { MySerial.print("APDU Error = ");MySerial.println(sc.lasterror);
    for (i = 0; i < sc.pbufs; i++)
    { sprintf(v, "%02X", 0xFF & sc.bufs[i]);
      MySerial.print(v);
      if ((i % PSIZE) == (PSIZE - 1) ) 
      {
      MySerial.println();
      }
    }
    if ((i % PSIZE) != 0) MySerial.println();
    }
  }

  if (do_verbose)
  { MySerial.print("Rx[");MySerial.print((t2-t1)/1000);MySerial.println("ms]: ");
    
    if ( ((!f_wr) || (myptcol != (char)0)) &&  (command.resp_length != 0) )
    { MySerial.print("    ");
      for (i = 0; i < command.resp_length; i++)
      { sprintf(v, "%02X", 0xFF & resp[i]);
        MySerial.print(v);
        if ((i % PSIZE) == (PSIZE - 1) ) 
        {
          MySerial.println();
          MySerial.print("    ");
        }
      }
    }

    if ( (command.resp_length != 0) && ((i % PSIZE) != 0) )
    {
      MySerial.println();
      MySerial.print("    ");
    }
    
    MySerial.println(result, HEX); // SW1 SW2
    MySerial.println();
  }


  if (myptcol != (char)0)
  { 
    return t1rx ;
  }

  if (!f_wr)
    asize = command.resp_length ;
  else
    asize = 0;

  resp[asize] =   (result >> 8) & 0xFF ;
  resp[asize + 1] = result & 0xFF;

  if (first)
  {
  if ((asize == 0) && (resp[0] == (char)0x61) )
  {
    if (resp[1]== 0) foutput=true;
    else             foutput=false;
    req[0]=0;
    req[1]=0xC0;
    req[2]=0;
    req[3]=0;
    req[4]=resp[1];
    
    return send_apdu(req,5,resp,foutput,false);
    
  }

 if ((asize == 0) && (resp[0] == (char)0x6C) )
  {
    if (resp[1]== 0) foutput=true;
    else             foutput=false;
    req[0]=command.cla;
    req[1]=command.ins;
    req[2]=command.p1;
    req[3]=command.p2;
    req[4]=resp[1];
    
    return send_apdu(req,5,resp,foutput,false);
    
  }
 }
 
  return  asize + 2;

}
