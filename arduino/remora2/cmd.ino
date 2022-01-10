 #define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
  short *pCV;
  short n;
  char s[22];
  unsigned int tday;
  unsigned int tmonth;
  unsigned int tyear;
  unsigned int thour;
  unsigned int tmin;
  unsigned int tsec;

  pCV = (short*)pCmd;

  n = strlen(pCmd);
  if(n<2) return TRUE;

  switch(*pCV)
  {                     
    // Set Real Time Clock
    case ('T' + ('M'<<8)):
    {
         //set time
//         Serial.print("Time ");
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
//         Serial.print(tyear); Serial.print(" ");
//         Serial.print(tmonth);Serial.print(" ");
//         Serial.print(tday);Serial.print(" ");
//         Serial.print(thour);Serial.print(" ");
//         Serial.print(tmin);Serial.print(" ");
//         Serial.print(tsec);
         setTime2(thour,tmin,tsec,tday,tmonth,tyear); 
 //        Serial.println(" set");
         break;
     }

      // calibration mode
       case ('C' + ('C'<<8)):
      {
        delayRecPlayDays = 0;
        delayMotion = 0;
        minPlayBackInterval = 2;
        break;
      }
      // settings 0
      case ('S' + ('0'<<8)):
      {
        delayMotion = 0;
        delayRecPlayDays = 1;
        break;
      }

      // settings 1
      case ('S' + ('1'<<8)):
      {
        delayMotion = 1;
        delayRecPlayDays = 1;
        break;
      } 

      // settings 2
      case ('S' + ('2'<<8)):
      {
        delayMotion = 21;
        delayRecPlayDays = 23;
        break;
      } 

    // Playback
    case ('P' + ('I'<<8)):
    {
      sscanf(&pCmd[3],"%d",&minPlayBackInterval);
      break;
    }

    // default simulateDepth = 0
    case ('S' + ('D'<<8)):
    {
      simulateDepth = 1;
      break;
    }
  } 
  return TRUE;
}

boolean loadScript()
{
  char s[30];
  char c;
  short i;

  File file;
  unsigned long TM_byte;
  int comment_TM = 0;

  // Read card setup.txt file to set date and time, recording interval
 file=sd.open("setup.txt");
 if(file)
 {
   do{
        i = 0;
        s[i] = 0;
        do{
            c = file.read();
            if(c!='\r') s[i++] = c;
            if(c=='T') 
            {
              TM_byte = file.position() - 1;
              comment_TM = 1;
            }
            if(i>29) break;
          }while(c!='\n');
          s[--i] = 0;
          if(s[0] != '/' && i>1)
          {
            ProcCmd(s);
          }
      }while(file.available());
      file.close();  
      
      // comment out TM line if it exists
      if (comment_TM)
      {
//        Serial.print("Comment TM ");
//        Serial.println(TM_byte);
        file = sd.open("setup.txt", FILE_WRITE);
        file.seek(TM_byte);
        file.print("//");
        file.close();
      }
      
  }
  else
  {   
  //  Serial.println("no file");
    return 0;
  }
 return 1;  
}
