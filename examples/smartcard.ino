#include <Arduino.h>
#include <SCLib4.h>

bool    fdebug=true; 
bool    flocal=true; // false= modeX en reception
char    myptcol=0;   // 0=>T0 or 1=>T1

#define MINIPRO_CLK 16 //8 or 16 MHZ

///////////////////
#define STM32
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
#define LED PC13
#define MYBAUD 115200 
int    MyF = 4000  ;
bool   mypts=true  ;
char   MyTA = (char)0x12;
//By default, only one Serialx instance is available mapped to the generic Serial name.
#define MySerial Serial1

#else
#if defined(LEONARDO)
#define LED 13
#else
#define LED 13
#endif

#define MYBAUD 19200
int     MyF = 4000   ;
bool    mypts=true   ;
char    MyTA = (char)0x11; 
#define MySerial Serial
#endif

int     ptsct=0;
bool    mycrc= false;
bool    pNR=false;
bool    pNS=false;
uint8_t myifs= 254;

char histlen=0    ;
char histbytes[15];

#ifdef ATMEGA2560
#define SC_C7_IO     26 
#define SC_C2_RST    24 
#define SC_C1_VCC    22 
#define SC_C2_CLK    11  // timer 1 
//#define SC_C2_CLK  46  // Timer 5 

#elif defined(STM32)
#define SC_C2_RST    PC15  
#define SC_C1_VCC    PC14  
#define SC_C7_IO     PB5    
#define SC_C2_CLK    PA6  
#define LED_ON   LOW
#define LED_OFF  HIGH

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

? ENTER => Local Mode
# ENTER =>  External Mode
A 00A4040006010203040500
A 00200001083030303030303030
A 00D00000F0A55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55A
A 00B00000F0
A 00D00000FFA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA55AA5
A 00B00000FF
S 00B0000000

*/

// Create SmartCardReader object for further use
SmartCardReader4 sc(SC_C7_IO, SC_C2_RST, SC_C1_VCC,SC_C2_CLK);

#define SIZE_APDU_BUF   3+260+2
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

  sc.fscale=1;
  sc.UseTimer5= false;
  
  #ifdef MINIPRO
  sc.fscale= 16/ MINIPRO_CLK;
  #endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LED_OFF);
  delay(30);
  digitalWrite(LED, LED_ON);
  delay(30);
  digitalWrite(LED, LED_OFF);
  delay(30);

#ifdef ATMEGA2560
MySerial.println("Atmega2560");

#elif defined (STM32)
MySerial.println("STM32");

#elif defined(MINIPRO)
MySerial.println("MINIPRO");

#elif defined(LEONARDO)
MySerial.println("LEONARDO");

#else
MySerial.println("NANO");

#endif
  
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

  /*
  if (fdebug) 
  { MySerial.println(cmd);
    if (opt != NULL) MySerial.println(opt);
  }
  */


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

   else if ( (strcmp_P(cmd,PSTR("F"))==0) && (opt != NULL) )
   { MyF= atoi(opt);
     MySerial.println(cOK); 
     return;
   }

   else if ( (strcmp_P(cmd,PSTR("ifs"))==0) && (opt != NULL) )
   { if (myptcol != (char)0)
     ifst1((uint8_t)atoi(opt));
     MySerial.println(cOK); 
     return;
   }



   else if( (strcmp_P(cmd,PSTR("ta"))==0) && (opt != NULL) )
   {
   strcpy(MyBuf,opt);
   err = Ascii2bin(MyBuf);
   if (err <=0)  { MySerial.println(cERROR); return;}
   MyTA= MyBuf[0];
   MySerial.println(cOK); 
   return;
   }

   else if( (strcmp_P(cmd,PSTR("pts"))==0) && (opt != NULL) )
   { 
     err = atoi(opt);
     MyTA = 0x10 + err    ;
     mypts=true;
     MySerial.println(cOK); 
     return;
   }

    else if (strcmp_P(cmd,PSTR("nopts"))== 0)
   { mypts=false ;
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
     { Bin2ascii(histbytes+i,1, v);
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

  if (len > 260)
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


bool PTS(char TA)
{
  //char buf[] =  {(char)0xFF, (char)0x70, (char)0x11, (char)0, (char)0, (char)0 };
  char buf[4] =  {(char)0xFF, (char)0x10, (char)0x11, (char)0 };
  char buf2[4];
  int nb;

  buf[1] |= myptcol;
  buf[2]  = TA ;

  buf[3] = buf[2] ^ buf[1] ^ buf[0] ;


  delayMicroseconds(sc._guardTime);

  for (int i = 0; i < 4; i++)
  { 

#if defined(V1)
sc._sendByte(buf[i]);
#else
sc._sendByteStd(buf[i]);
#endif
    
    delayMicroseconds(sc._guardTime);
  }

  for (int i = 0; i < 4; i++) buf2[i] = (char)0xA5;

  nb = 0;
  for (int i = 0; i < 4; i++)
  {
#if defined(V1)
if (sc._receiveByte((uint8_t*)(buf2 + i), sc._max_wwt) != 0)  break;
#else
if (sc._receiveByteStd((uint8_t*)(buf2 + i), sc._max_wwt) != 0)  break;
#endif 
    
    nb++;
  }

if (fdebug)
{
#if defined(SC_DEBUG)
                sc.dumpHEX((uint8_t*)buf, 4);
     if (nb >0) sc.dumpHEX((uint8_t*)buf2, nb);
#else
#endif
}

  if (nb != 4) 
    return false;
  
   for (int i=0;i<4;i++)
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
    *Di=8;break;
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
{ uint8_t data[MAX_ATR_BYTES ];
  uint16_t atr_received = 0;
  int fs=2000, D = 1, F = 372, N = 0;
  int CWI = 13, BWI = 4, WI = 10;
  int Fi=-1,Di=-1;
  ifd_atr_info_t info;

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
  { pinMode(SC_C7_IO,INPUT) ;  
    sc.use_ts= true;
    atr_received  = sc._WarmReset(data, MAX_ATR_BYTES);
    sc.use_ts= false;
  }
  
  int ftck= -1;
  if (atr_received >0) ftck= ifd_atr_parse(&info,data,atr_received);
  if (ftck >=0) 
  { ftck= ifd_atr_parse(&info,data,atr_received);

    if (fdebug)
    {
    MySerial.print("T");MySerial.print(info.default_protocol);MySerial.print(" {");
    if (info.supported_protocols & 0x1) MySerial.print("T0 ");
    if (info.supported_protocols & 0x2) MySerial.print("T1"); 
    MySerial.println("}");
    }
    
    for (int i=0;i<NTD;i++)
    { if (info.TA[i] != -1) 
      { if (fdebug) {MySerial.print("TA"); MySerial.print(i+1);MySerial.print("= ");MySerial.println(info.TA[i],HEX);}
        if (i == 0)
        { Fi = 0x0F & (info.TA[i]>>4) ;
          Di = 0x0F & info.TA[i];
          if (fdebug) {MySerial.print("Fi=");MySerial.print(Fi);MySerial.print(" Di=");MySerial.println(Di);}
        }
      }
      if (info.TB[i] != -1) 
      { if (fdebug) {MySerial.print("TB"); MySerial.print(i+1);MySerial.print("= ");MySerial.println(info.TB[i],HEX);}
      }
      if (info.TC[i] != -1) 
      { if (fdebug) {MySerial.print("TC"); MySerial.print(i+1);MySerial.print("= ");MySerial.println(info.TC[i],HEX);}
        if (i == 0)
        { N = 0xFF & info.TC[i];
          if (fdebug) {MySerial.print("N="); MySerial.println(N);};
          if (N==255) N=0;
        }
      }
    }
    if (info.t1 != -1) 
    { if (fdebug) {MySerial.print("T1i=");MySerial.println(info.t1+1);}
     if ( ((info.t1+1) > 1) && ((info.t1+1) < NTD) )
     {
      if (info.TA[info.t1+1] != -1)
      { myifs = 0xFF & info.TA[info.t1+1];
        if (fdebug) {MySerial.print("IFS=");MySerial.println(myifs);}
      }
      if (info.TB[info.t1+1] != -1)
      { CWI = 0x0F & info.TB[info.t1+1];
        if (fdebug) {MySerial.print("CWI=");MySerial.println(CWI);}
        BWI = 0x0F & (info.TB[info.t1]>>4);
        if (fdebug) {MySerial.print("BWI=");MySerial.println(BWI);}
      }
      if (info.TC[info.t1+1] != -1)
      { if (info.TC[info.t1+1] & 0x01) { mycrc= true ;if (fdebug) MySerial.println("CRC");}
        else                           { mycrc=false ;if (fdebug) MySerial.println("EDC");}
      }
    }
    }

    if (info.t15 != -1) 
    { if (fdebug) {MySerial.print("T15i=");MySerial.println(info.t15+1);}
    }
    

    
  }
  

 if (fdebug)
  {
  MySerial.print("SC Frequency = ");
  MySerial.print(sc.fclk/sc.fscale);
  MySerial.println(" KHz");
  MySerial.print("TCK = ");
  MySerial.println(ftck);
  }

  
  ptsct=0;

  if (ftck < 0)
  {
  histlen=atr_received;
  for (char i=0;i<histlen; i++)
    histbytes[i] = data[i];
  }
  else
  { histlen= 0x0F & data[1];
    for (char i=0;i<histlen; i++)
    histbytes[i] = data[atr_received-histlen-ftck+i];
  }
  
  if ( (atr_received > 0) && (pts == true) )
  { 
    pts=false;
    while (!pts && (ptsct < 1) )
    {
     pts= PTS(MyTA);
     if (!pts)
     {  ptsct++;
        atr_received = sc._WarmReset(data, MAX_ATR_BYTES);
        if (atr_received <= 0) break;
     }
    }

    if (!pts)
    { if (fdebug)  MySerial.println("\nNo PTS");
      atr_received = sc.activate(data, MAX_ATR_BYTES);
      pts=false;
    }
    else
    { if (fdebug) MySerial.println("PTS OK");
      pts = true ;
    }
    
  }

 mypts=pts; 

  if ( atr_received > 0)
  {
#if defined(SC_DEBUG)
    if (fdebug) sc.dumpHEX(data, atr_received);
#else
    if (fdebug) { MySerial.println(); MySerial.print(atr_received); MySerial.println(" bytes ATR received from card.");}
#endif

    // The specific interface byte TC(2) codes the integer value WI over the eight bits; the null value is reserved for future use.
    // If no TC(2) appears in the Answer-to-Reset, then the default value is WI = 10.
    // The interval between the leading edge of any character sent by the card and the leading edge of the previous character (sent either by the card or by the interface device)
    // shall not exceed 960 x WI x (Fi / f). This maximum delay is named the work waiting time.

    //   T=0
    //   WWT = WI . 960 . F/f secondes, le temps d'attente maximum d'une réponse carte par le lecteur.

    //   T=1
    //   CWT = (2**CWI + 11)etu  CWI=0, 12, 1+8+1+2 Character Waiting Time
    //   BWT = (2**BWI X 960 X 372 / fs) Sec + 11 etu // 5,713,920 /fs  + 11 etu
    //   BGT is defined as the minimum delay between the leading edges of two consecutive characters 
    //   sent in opposite directions. The value of BGT shall be 22 etu.


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

#if defined(V2)
     sc._Setu      = (long) ((long)sc.fscale * F*(long)1000*(long)16) / (long)D  / (long)fs ; // (us) =  F/D/fs (en MHz), F=372, D=1 fs=2
     sc._Setu2     = (long) ((long)sc.fscale * F*(long)1000*(long)2)  / (long)D  / (long)fs ; // (us) =  F/D/fs (en MHz), F=372, D=1 fs=2
#endif

//#ifdef STM32_TIMERX
#ifdef ASTM32
     sc._Setu     = (long) ((long)sc.fscale * (long)FTX* F *(long)1000) / (long)D  / (long)fs ; // (us) =  F/D/fs (en MHz), F=372, D=1 fs=2
#endif
     
     sc._guardTime = ((long)2 + (long)N) * (long)sc._etu;
     sc._wwt       = (long)WI * (long)960 * (long)D * (long)sc._etu ;
     sc._max_wwt   = sc._wwt ;

     sc._cwt = ((long)8 * sc._etu);
     sc._bwt = (long)(5000000L);
     sc._bgt = ((long)22 * sc._etu);
     if (myptcol != (char)0) 
     { sc._guardTime= ((long)1 * sc._etu);// 10 + 1
       pNS=false;
     }

    if (fdebug)
    {  MySerial.print("etu =");
       MySerial.print(sc._etu);
    }
    
#if defined(V2)   
    if (fdebug)
    { 
    #ifdef TIMERX4
    MySerial.print(" Super_etu =");
    MySerial.print(sc._Setu);
    #endif
    #ifdef TIMERX2
    MySerial.print("Super_etu2 =");
    MySerial.print(sc._Setu2);
    #endif
    }
#endif    

    //#ifdef STM32_TIMERX
    #ifdef ASTM32
    if (fdebug)
    { MySerial.print(" Super_etu =");
      MySerial.print(sc._Setu);
    }
    #endif

 

    if (fdebug)
    {
    MySerial.print(" guardTime=");
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
    MySerial.print(" cwt=");
    MySerial.print(sc._cwt);
    MySerial.print(" bwt=");
    MySerial.print(sc._bwt);  
    MySerial.print(" bgt=");
    MySerial.println(sc._bgt);  
    }

    }
    

  }
  else
  {
   if (fdebug) MySerial.println("no smartcard ...");
   sc.deactivate();
   return false;
  }

  if (myptcol != (char)0) ifst1(myifs);
  

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

uint8_t edc_crc(bool fcrc,bool first, bool last, uint8_t *data, int len, uint8_t *value)
{ static uint16_t current_crc_value;
  static uint8_t  edc;

  if (!fcrc)
  {  if (first) edc= 0;
     for(int i=0;i<len;i++) edc = edc ^ data[i];
     value[0]= edc ;
     return 1;
  }

  if (first) current_crc_value = 0xFFFF;
  for (int i = 0; i < len; i++ )
  {
    current_crc_value ^= ((uint16_t)data[i] & 0xFF);
    for (int j = 0; j < 8; j++)
    {
      if ((current_crc_value & 1) != 0)
      current_crc_value = (current_crc_value >> 1) ^ (uint16_t)0x8408;
      
      else
      current_crc_value = current_crc_value >> 1;
     }
  }

  if (!last)
  return 2;
  
  current_crc_value = ~current_crc_value   ;
  value[0]= 0xFF & current_crc_value       ;
  value[1]= 0xFF & (current_crc_value >> 8);
  return 2;
   
}



int ifst1(uint8_t ifs)
{ uint8_t pt1[6];
  int i,ledc=0;

  delayMicroseconds(sc._bgt);

  pt1[0]=0;
  pt1[1]=0xC1 ; //IFS REQUEST
  pt1[2]=0x01 ;
  pt1[3]=ifs  ;

 /*
 if (!mycrc)
 { uint8_t edc=0 ;
   for(i=0;i<4;i++) edc = edc ^ pt1[i];
   pt1[4]= edc ;
   ledc=1;
 }  
 */

 ledc =edc_crc(mycrc,true,true,pt1,4,pt1+4) ;  

    if (fdebug) myPrintf("TxT1",(char*)pt1,4+ledc);
    sc._sendASyncBytes_T1(pt1,4+ledc); 
    int result= sc._receiveASyncBytes_T1(pt1,4+ledc,sc._bwt,sc._cwt);
    if (fdebug) myPrintf("RxT1",(char *)pt1,result);
    //if (result != (4+ledc)) return -1 ;

    delayMicroseconds(sc._bgt);

    return 0;
  
}

int sendt1(char *req, int asize, char *resp,uint8_t ifs)
{ char pt1[5],edc=0,*pfrag=NULL;
  int lfrag,i,ledc=0,len,result=0;
  bool last=false;

  len=asize ;
  pfrag= req;

  while(true)
  {
    if (len <= (int)ifs) 
    { lfrag = len; last=true;}
    else
    { lfrag= (int)ifs;}
    
    pt1[0]=0  ;
    if   (!pNS) { pt1[1]=(char)0x00; pNS=true;}
    else        { pt1[1]=(char)0x40; pNS=false;}

    char pcb = pt1[1]>>2;

    if (!last) 
    pt1[1] |= (char)0x20;
    
    pt1[2] = 0xFF & lfrag;

    /*                        
    if (!mycrc)
    { edc=0 ;
      ledc=1;
      for(i=0;i<3;i++)     edc = edc ^ pt1[i]  ;
      for(i=0;i<lfrag;i++) edc = edc ^ pfrag[i];
      pt1[3]= edc      ;
    }  
    */

    ledc = edc_crc(mycrc,true,false,(uint8_t *)pt1,3,(uint8_t *)pt1+3);
    ledc = edc_crc(mycrc,false,true,(uint8_t *)pfrag,lfrag,(uint8_t *)pt1+3);



   if (fdebug) myPrintf("TxT1",pt1,3+ledc);

   sc._sendASyncBytes_T1((uint8_t *)pt1,3);
   sc._sendASyncBytes_T1((uint8_t *)pfrag,(uint16_t)lfrag);
   sc._sendASyncBytes_T1((uint8_t *)(pt1+3),(uint16_t)ledc);

   if (last) 
   break;

   pfrag += lfrag;
   len-= lfrag;

   result= sc._receiveASyncBytes_T1((uint8_t*)pt1,3+ledc,sc._bwt,sc._cwt);
  
   if (fdebug) myPrintf("RxT1",pt1,result);
   
   if (result != (3+ledc)) return -1 ;
   if ( (pt1[1] & 0xEF) == (char)0x80) // MORE
   { char R= pt1[1] & (char)0x10; // Block R - N(R)
     if (R == pcb) 
     return -1 ;
   }
   else
   return -1;
   
   delayMicroseconds(sc._bgt);

  }

  return 0;
}


int send_apdu(char *req, int asize, char *resp, bool foutput,bool first)
{
  boolean do_verbose = fdebug ;
  boolean f_wr = true;
  APDU_t   command;
  int i, PSIZE = 16;
  char v[5];
  unsigned long t1,t2;
  char *pt1=NULL;
  int   t1len=0,t1rx=0;
  uint16_t result ;
  char edc=0;
  bool t1first=true;
  int  ptrt1=0;
  
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
    
    /*
    memmove(req+3,req,asize);
    pt1= req ;
    
    pt1[0]=0  ;
    if   (!pNS) { pt1[1]=(char)0x00; pNS=true;}
    else        { pt1[1]=(char)0x40; pNS=false;}
    pt1[2] = 0xFF & asize;
                         
    if (!mycrc)
    { edc=0;
      for(i=0;i<3+asize;i++) edc = edc ^ pt1[i];
      pt1[asize+3]= edc;
      t1len= asize + 4 ;
     }
     
     */

   }
  
  if (myptcol == (char)0) 
  { if (do_verbose) t1=micros();
    result = sc.sendAPDU(&command, f_wr, 10000); // timeout 10s
   
  }
  else
  { t1first= true;
    ptrt1=0;
loop_t1:       
    sc.lasterror=0;
    if (do_verbose) 
    { if (t1first) 
      { t1=micros(); 
       //t1first=false;
      }
      else {t2=micros();MySerial.print((t2-t1)/1000);MySerial.println("ms");}
      // myPrintf("TxT1",pt1,t1len);
    }

    if (t1first)
    { t1first=false; 
      //sendt1(pt1+3,t1len-4,resp,254);
      result= sendt1(req,asize,resp,myifs);
      if (result != 0) return -1;
    }
    else
    {  delayMicroseconds(sc._bgt);
       sc._sendASyncBytes_T1((uint8_t *)pt1,(uint16_t) t1len);
    }
    
    pt1= resp + ptrt1;
    result= sc._receiveASyncBytes_T1((uint8_t*)pt1,(uint16_t)t1rx,sc._bwt,sc._cwt);
    if (result > 0)  
    {if (do_verbose) myPrintf("RxT1",pt1,result);
     
     if (pt1[1] == (char)0xC3) // WTX Request
     { pt1[1]= 0xE3; // WTX response
       t1len=result;

       /*
       if (!mycrc)
       { edc=0;
         for(i=0;i<result-1;i++) edc = edc ^ pt1[i];
         pt1[result-1]= edc;
       }*/

       if (!mycrc) edc_crc(mycrc,true,true,(uint8_t *)pt1,result-1,(uint8_t *)pt1+result-1);
       else        edc_crc(mycrc,true,true,(uint8_t *)pt1,result-2,(uint8_t *)pt1+result-2);
       goto loop_t1;
     
     }

      if ( (pt1[1] & 0xBF) == (char)0x20) // MORE
      { char pcb= pt1[1] & (char)0x40; // N(R)
        
        if   (pcb == (char)0x40) pcb = (char)0x80 ;
        else pcb = (char)0x90 ;
        
        int  lfrag= 0xff & pt1[2] ;
        memmove(resp+ptrt1,pt1+3,lfrag);
        ptrt1 += lfrag  ;
        pt1= resp + ptrt1;
        pt1[0]=0;
        pt1[1]=pcb; // R Block
        pt1[2]= 0 ;
        if (!mycrc) 
        { pt1[3]=  pt1[1];
          t1len= 4;
        }
        goto loop_t1;
         
      }
      
      if ( (pt1[1] & 0xBF) != (char)0)
      {  if (do_verbose) MySerial.println("\n!!! send_apdu T=1 ERROR !!!\n");
         
         //myPrintf("RxT1",pt1,result);
         
         delay(1000);
         return -1;
      }

      int  lfrag= 0xff & pt1[2] ;
      memmove(resp+ptrt1,pt1+3,lfrag);
      ptrt1 += lfrag  ;
      t1rx= ptrt1     ;
      result =  0xFF00 & (resp[t1rx-2]<<8);
      result |= 0xFF   & resp[t1rx-1];
      command.resp_length = 0xFF & (t1rx-2);
     
    }
    
  }
  
  if (do_verbose) t2=micros();

  if (do_verbose)
  { MySerial.print("APDU Error = ");MySerial.println(sc.lasterror);
    if (myptcol == (char)0)
    {
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


  if (result == 0)
  {
    MySerial.println("\n!!! send_apdu ERROR !!!\n");
    delay(1000);
    // reboot();
    return -1;
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
    
    MySerial.println(result, HEX);
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
