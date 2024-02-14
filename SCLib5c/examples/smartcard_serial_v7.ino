/*
  smartcard_serial_v7.ino - Smart Card library version 7
  Copyright (c) 2024 Pascal Urien
*/

#include <Arduino.h>
#include <SCLib5c.h>

bool    fdebug=false ; 
bool    flocal=true; // false= modeX en reception
#define MY_PTCOL 0 
bool    auto_p=true  ;

#define MINIPRO_CLK 16 //8 or 16 MHZ

///////////////////
// #define STM32
#define WROOM32
// #define MINIPRO
// #define ATMEGA2560
// #define LEONARDO
// default NANO
////////////////////

// TA1=(char)0x11, F=372;  D=1;
// TA1=(char)0x12; F=372;  D=2;
// TA1=(char)0x13; F=372;  D=4;
// TA1=(char)0x14; F=372;  D=8 ;

#ifdef STM32
#define  LED PC13
#define  MYBAUD 115200 
#define  MY_FREQ  6000   
#define  MY_PTS   true   
#define  MY_TA    0x12 
#define  MySerial Serial1
#define  ATRVERBOSE

#elif defined(WROOM32)
#define  LED BUILTIN_LED 
#define  MYBAUD   115200 
#define  MY_FREQ  4000   
#define  MY_PTS   true   
#define  MY_TA    0x11 
//By default, only one Serialx instance is available mapped to the generic Serial name.
#define MySerial Serial
#define ATRVERBOSE

#else
#if defined(LEONARDO)
#define LED 13
#else
#define LED 13
#endif

#define  MYBAUD  19200
#define  MY_FREQ  4000   
#define  MY_PTS   true   
#define  MY_TA    0x11 
#define  MySerial Serial
#endif

uint16_t MyF = MY_FREQ    ;
bool     mypts=MY_PTS     ;
uint8_t  MyTA =MY_TA      ;
uint8_t  myptcol=MY_PTCOL ; // 0=>T0 or 1=>T1

bool    mycrc= false ;
uint8_t myifs= 254   ;
uint8_t lastpcb=0    ;

uint8_t last_send_sequence = 0    ;
uint8_t last_received_sequence = 0;
uint8_t finject=0;
uint8_t t1_error=0;
uint8_t T1_ERROR_MAX=3;
bool t1_parity_error=false;

uint8_t histlen=0   ;
uint8_t histbytes[15];

typedef struct
{
 /* Prologue field (NAD, PCB, LEN) */
 uint8_t nad;
 uint8_t pcb;
 uint8_t len;
 /* Epilogue field (EDC) */
 union 
 { uint8_t  edc_lrc;
   uint16_t edc_crc;
 };
 uint8_t edc_type; /* Type of used EDC */
 /* Information field (APDU) is handled by the lower layer */
 uint8_t *data;
} SC_TPDU;

/*
?  ENTER => mode LOCAL
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
A 00D00000FFA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA5

A 00B00000F0
S 00B0000000
*/

#ifdef ATMEGA2560
#define SC_C7_IO     4 
#define SC_C2_RST    8  
#define SC_C1_VCC    10 
#define SC_C2_CLK    11  // timer 1 
//#define SC_C2_CLK  46  // Timer 5
//#define LED   3
//#define BPUSH 5
#define LED_ON   HIGH
#define LED_OFF  LOW  
#define ATRVERBOSE

#elif defined(STM32)
#define SC_C2_RST    PC15  
#define SC_C1_VCC    PC14  
#define SC_C7_IO     PB5    
#define SC_C2_CLK    PA6  
#define LED_ON   LOW
#define LED_OFF  HIGH
#define ATRVERBOSE

#elif defined(WROOM32)
#define SC_C2_RST    GPIO_NUM_17  
#define SC_C1_VCC    GPIO_NUM_16  
#define SC_C7_IO     GPIO_NUM_32      
#define SC_C2_CLK    GPIO_NUM_25  
#define LED_ON   HIGH
#define LED_OFF  LOW
#define ATRVERBOSE

#elif defined(MINIPRO)
#define SC_C2_RST    7  
#define SC_C1_VCC    6  
#define SC_C7_IO     8   
#define SC_C2_CLK    9  
#define LED_ON   HIGH
#define LED_OFF  LOW 

#elif defined(LEONARDO)
#define SC_C2_RST    7  
#define SC_C1_VCC    6  
#define SC_C7_IO     8   
#define SC_C2_CLK    9  
#define LED_ON   HIGH
#define LED_OFF  LOW 

#else // Arduino Nano
#define SC_C2_RST    10  
#define SC_C1_VCC    11  
#define SC_C7_IO     8   
#define SC_C2_CLK    9  
#define LED_ON   HIGH
#define LED_OFF  LOW  
#endif

/*
?
A 00A4040006010203040500
A 00200001083030303030303030

A 008700000A
A 0087000004

A 0081000000
A 0089000000
A 0082000000

A 0081000300
A 0089000300
A 0082000300

A 0084060343
A 00800003201234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF
A 00C0000048
S 00B0000000
A 0084060043
A 00800000201234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF

A 00D00000F0A55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55A
A 00B00000F0
A 00D00000FFA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA5
A 00B00000FF
S 00B0000000

*/

// Create SmartCardReader object for further use
SmartCardReader5c sc((uint8_t)SC_C7_IO, (uint8_t)SC_C2_RST, (uint8_t)SC_C1_VCC,(uint8_t)SC_C2_CLK);

#define SIZE_APDU_BUF   3+261+2
#define SIZE_RESP_BUF   258+16 
#define SIZE_BUF1       SIZE_APDU_BUF+SIZE_RESP_BUF

char *apdu_buf= NULL ; 
char *resp_buf= NULL ; 
char mybuf[SIZE_BUF1];
 
const char* cOK="OK";
const char* cERROR="ERROR";

void setup() 
{
  MySerial.begin(MYBAUD);
  while (!MySerial);

  sc.fscale=1;
  sc.UseTimer5= false;
  
  #ifdef MINIPRO
  sc.fscale= 16/ MINIPRO_CLK;
  #endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LED_ON);
  delay(30);
  digitalWrite(LED, LED_OFF);
  delay(30);
  digitalWrite(LED, LED_ON);
  delay(30);
  digitalWrite(LED, LED_OFF);

if (fdebug)
{
#ifdef ATMEGA2560
MySerial.println("Atmega2560");

#elif defined (STM32)
MySerial.println("STM32");

#elif defined(MINIPRO)
MySerial.println("MINIPRO");

#elif defined(LEONARDO)
MySerial.println("LEONARDO");

#elif defined(WROOM32)
MySerial.println("ESP32");

#else
MySerial.println("NANO");

#endif
}
 
 apdu_buf=  mybuf ;
 resp_buf=  mybuf + SIZE_APDU_BUF + 3 ;

 //if (fdebug) CheckSC(mypts) ;
 sc.deactivate();

}

void loop()
{ char *cmd= NULL ;
  char *opt= NULL, v[3];
  int i,err,pt;
  char *MyBuf= mybuf;
  
  int nb= readBuffer(MyBuf,SIZE_BUF1,'\n');
  if (nb <= 0) return ;
  
  cmd = strtok(MyBuf, " \r\n");
  if (cmd == NULL) { MySerial.println("Ready"); return;}
  opt = strtok(NULL,"\r\n");
  pt = strlen(cmd);

 
   if (strcmp_P(cmd,PSTR("test"))==0)
   { int myct=0;
     int maxi=1;
     if (opt != NULL) maxi=atoi(opt);

     strcpy_P(MyBuf,PSTR("00A4040006010203040500"));
     err = Ascii2bin(MyBuf);
     err= send_apdu(apdu_buf,err,resp_buf,false,true);
     if (err <=0)  { MySerial.println(cERROR); return;}

     strcpy_P(MyBuf,PSTR("00200001083030303030303030"));
     err = Ascii2bin(MyBuf);
     err= send_apdu(apdu_buf,err,resp_buf,false,true);
     if (err <=0)  { MySerial.println(cERROR); return;}

     strcpy_P(MyBuf,PSTR("0081000000"));
     err = Ascii2bin(MyBuf);
     err= send_apdu(apdu_buf,err,resp_buf,false,true);
     if (err <=0)  { MySerial.println(cERROR); return;}

     strcpy_P(MyBuf,PSTR("0089000000"));
     err = Ascii2bin(MyBuf);
     err= send_apdu(apdu_buf,err,resp_buf,false,true);
     if (err <=0)  { MySerial.println(cERROR); return;}
   
     strcpy_P(MyBuf,PSTR("0082000000"));
     err = Ascii2bin(MyBuf);
     err= send_apdu(apdu_buf,err,resp_buf,false,true);
     if (err <=0)  { MySerial.println(cERROR); return;}  
     
     while(myct++ < maxi)
     {
     strcpy_P(MyBuf,PSTR("00800000201234567890ABCDEF1234567890ABCDEF1234567890ABCDEF1234567890ABCDEF"));
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

  else if (strcmp_P(cmd,PSTR("F"))==0) 
   { 
     if (opt==NULL)
     {
      MySerial.println(MyF); 
      return;
     }
     MyF= atoi(opt);
     MySerial.println(cOK); 
     return;
   }

   else if (strcmp_P(cmd,PSTR("ifs"))==0)
   {  
    if (opt == NULL)
    {
    MySerial.println(myifs);
    return;  
    }
    myifs= (uint8_t)atoi(opt);
    if (myptcol != (char)0)
     ifst1((uint8_t)myifs);
     MySerial.println(cOK); 
     return;
   }


   else if(strcmp_P(cmd,PSTR("retry"))==0)
   { 
     if (opt == NULL)
     { MySerial.println(T1_ERROR_MAX); 
       return;
     }
     err = atoi(opt) ;
     T1_ERROR_MAX=err     ;
     MySerial.println(cOK); 
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

   else if(strcmp_P(cmd,PSTR("ta"))==0) 
   {
   if (opt == NULL)
   { MySerial.println(0xFF & MyTA, HEX);
     return;
   }
   strcpy(MyBuf,opt);
   err = Ascii2bin(MyBuf);
   if (err <=0)  { MySerial.println(cERROR); return;}
   MyTA= MyBuf[0];
   MySerial.println(cOK); 
   return;
   }

   else if(strcmp_P(cmd,PSTR("pts"))==0)
   { 
     if (opt == NULL)
     { MySerial.println((int)MyTA-(int)0x10); 
       return;
     }
      
     err = atoi(opt)   ;
     MyTA = 0x10 + err ;
     mypts=true;
     MySerial.println(cOK); 
     return;
   }

    else if (strcmp_P(cmd,PSTR("nopts"))== 0)
   { mypts=false ;
     MySerial.println(cOK); 
     return;
   }

   else if (strcmp_P(cmd,PSTR("ptcol"))== 0)
   { MySerial.println(0xFF & myptcol); 
     return;
   }

   else if (strcmp_P(cmd,PSTR("t0"))== 0)
   { myptcol=(char)0 ;
     auto_p=false    ;
     MySerial.println(cOK); 
     return;
   }

   else if (strcmp_P(cmd,PSTR("t1"))== 0)
   { myptcol=(char)1 ;
     auto_p=false;
     MySerial.println(cOK); 
     return;
   }
   
   else if ( (strcmp_P(cmd,PSTR("on"))==0) || (strcmp_P(cmd,PSTR("w"))==0) )
   { bool fwarm=false;
     if (strcmp_P(cmd,PSTR("w"))==0) fwarm=true;
     if (opt != NULL)
     {
      err= Ascii2bin(opt);
      if (err != 1)  { MySerial.println(cERROR); return;}
      MyTA= *opt ;
      mypts=true ;
     }

     bool stat=false;
     unsigned long t1=micros();
     stat= CheckSC(mypts,fwarm);
     unsigned long t2=micros();

     if (fdebug){MySerial.print(t2-t1); 
                 MySerial.println("us");}
     
     if (stat) { MySerial.println(cOK); digitalWrite(LED, LED_ON);return;}
     else      { MySerial.println(cERROR);  digitalWrite(LED, LED_OFF);return;}
   }
   
   else if  (strcmp_P(cmd,PSTR("off"))==0)
   {  sc.deactivate();
      MySerial.println(cOK);
      digitalWrite(LED, LED_OFF);
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

  else if  (strcmp_P(cmd,PSTR("hist"))==0)
   { if (histlen <=0)  
     { MySerial.println(cERROR); return;}
     
     for(char i=0;i<histlen;i++)
     { Bin2ascii(((char *)histbytes)+i,1, v);
       MySerial.print(v);
     }
     MySerial.println();
     return;
    }
 
   else 
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

  if (len > 261)
  return 0;
  
  memmove(apdu_buf,buf,len);
  int err= send_apdu(apdu_buf,len,resp_buf,fdata,true);
  if (err <=0)  { MySerial.write((byte)0xFF);
                  MySerial.write((byte)0x00);
                  MySerial.write((byte)0x00);
                  return 0;
                 }
    MySerial.write((byte)deb);
    MySerial.write((byte)(0xFF & err>>8));
    MySerial.write((byte)(0xFF & err));
    for(int i=0;i<err;i++) MySerial.write(resp_buf[i]);
  
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
