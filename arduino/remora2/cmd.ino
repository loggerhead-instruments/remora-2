 #define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
  short *pCV;
  short n;
  long lv1;
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

      case ('C' + ('P'<<8)):
      {
         sscanf(&pCmd[3],"%d",&lv1);
         clockprescaler=lv1;
         break; 
      }
                
      case ('S' + ('T'<<8)):
      {
        //start time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         startTime = RTCToUNIXTime(tyear, tmonth, tday, thour, tmin, tsec);
         break;
      } 

      case ('D' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        long delaySeconds = lv1 * SECONDS_IN_DAY;
        //start time
        readRTC();
        startTime = t + delaySeconds;
        break;
      } 

    // RD: Record duration in minutes  
    case ('R' + ('D'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      recDur = lv1;
//      Serial.print("Rec dur:");
//      Serial.println(recDur);
      break;
    }

    // RI: Record interval in seconds
    case ('R' + ('I'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      recInt = lv1;
//      Serial.print("Rec int:");
//      Serial.println(recInt);
      break;
    } 

    case ('I' + ('S'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      imuSrate = lv1;
      break;
    }

    case ('P' + ('S'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      sensorSrate = lv1;
      break;
    }

    // disable LED
    case ('L' + ('D'<<8)):
    {
      LED_EN = 0;
      break;
    }

    // Playback
    //default minPlayBackInterval = 480; // keep playbacks from being closer than x minutes
    case ('P' + ('I'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      minPlayBackInterval = lv1;
      break;
    }

    //default playBackDepthThreshold = 275.0; // tag must be deeper than this depth to trigger threshold
    case ('P' + ('T'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      playBackDepthThreshold = lv1;
      break;
    }

    //default ascentDepthTrigger = 100.0; // tag must be rising faster than this rate per 3 minutes
    case ('R' + ('A'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      ascentRateTrigger = lv1;
      break;
    }

    //default maxPlayBacks = 80;
    case ('P' + ('M'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      maxPlayBacks = lv1;
      break;
    }
    // default simulateDepth = 0
    case ('S' + ('D'<<8)):
    {
      simulateDepth = 1;
      break;
    }

    //float delayRecPlayDays = 0.0; // delay record/playback for x days.
    case ('D' + ('P'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      delayRecPlayDays = (float) lv1;
      break;
    }

    //byte recMinutes
    case ('R' + ('M'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      recMinutes = lv1;
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
