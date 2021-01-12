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
	if(n<2) 
          return TRUE;

	switch(*pCV)
	{                     
    // disable LED
    case ('L' + ('D'<<8)):
    {
      LEDSON = 0;
      break;
    }
  
    case ('S' + ('G'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        gainSetting = (unsigned int) lv1;
        if((gainSetting<0) | (gainSetting>15)) gainSetting = 4;
        EEPROM.write(14, gainSetting); //byte
        break;
      }
      
      case ('N' + ('D'<<8)):
      {
        noDC = 1;
        break;
      }
      
    // Sample rate in Hz
    case ('H' + ('Z'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      // lhi_fsamps[9] = {8000, 16000, 32000, 44100, 48000, 96000, 200000, 250000, 300000};
      switch(lv1){
        case 8000: isf = 0; break;
        case 16000: isf = 1; break;
        case 32000: isf = 2; break;
        case 44100: isf = 3; break;
        case 48000: isf = 4; break;
        case 96000: isf = 5; break;
        case 192000: isf = 6; break;
      }
      break;
    } 
	}	
	return TRUE;
}

boolean LoadScript()
{
  char s[30];
  char c;
  short i;

#if USE_SDFS==1
  FsFile file;
#else
  File file;
#endif
  unsigned long TM_byte;
  int comment_TM = 0;

  // Read card setup.txt file to set date and time, recording interval
  sd.chdir(); // only to be sure to star from root
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
        Serial.print("Comment TM ");
        Serial.println(TM_byte);
        
        sd.chdir(); // only to be sure to star from root
        file = sd.open("setup.txt", FILE_WRITE); // WMXZ check mod
        file.seek(TM_byte);
        file.print("//");
        file.close();

      }
      
  }
  else
  {   
    Serial.println("setup.txt not opened");
    display.println("no setup file");
    return 0;
  }
 return 1;	
}
