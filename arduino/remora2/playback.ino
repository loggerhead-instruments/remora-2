uint8_t timeoutCounter = 0;

void checkPlay(){

  if((depth < 1.0) & (playState==0)) {
    maxDepth = depth; // reset maxDepth when each dive ends
  }
  
  if(depth > maxDepth) {
    maxDepth = depth; // track maximum depth
  }
  
  // check if after exceeding playback depth, came shallow enough to allow another playback
  if(playBackDepthExceeded==2){
    if(depth < playBackResetDepth){
      maxDepth = depth;
      playBackDepthExceeded = 0;
      //Serial.println("D");
    }
  }

  // playState State = 0
  // waiting for playback algorithm to be satisfied to trigger playback
  // prevent from playing back more than once per x minutes
  if(playState==0){
    if((depth > playBackDepthThreshold) & (playBackDepthExceeded==0)) {
      playBackDepthExceeded = 1;  // check if went deeper than a certain depth
    }

    // Trigger record if on ascent came up enough
    if ((playBackDepthExceeded==1) & (maxDepth - depth > ascentRecordTrigger) & (nPlayed < maxPlayBacks) & (REC_STATE==0)) {
      digitalWrite(REC_POW, HIGH); // turn on recorder
      digitalWrite(REC_ST, HIGH);  // start recording
      playTime = t; // update here as well so timeout below doesn't kick out
      REC_STATE = 1;
    }

    // Trigger playback if on ascent came up enough
    if ((playBackDepthExceeded==1) & (maxDepth - depth > ascentDepthTrigger) & (nPlayed < maxPlayBacks)) {
        playState = 1;
        digitalWrite(PLAY_POW, HIGH);  // play will start automatically
        PLAY_STATE = 1;
        playTime = t;
        playBackDepthExceeded = 2;
        nPlayed += 1;
        //Serial.println("P");
    }
  }

  // playState state = 1
  // turn off playback board when playing done
  // give 10 seconds before checking this because takes some time for play board to wake
  if((playState==1) & (t > playTime + 10)){
    int playStatusPin = analogRead(PLAY_STATUS);
   // Serial.println(playStatusPin);
    if(playStatusPin < 200){
      digitalWrite(PLAY_POW, LOW);
      PLAY_STATE = 0;
      playState = 2;
    }
  }

  // in case playback fails, turn off play board after 1 minute
  if((playState==1) & (t > playTime + 60)){
    digitalWrite(PLAY_POW, LOW);
    PLAY_STATE = 0;
    playState = 2;
  }

  // playState state = 2
  
  if(playState==2){
    // wait to turn off record recMinutesAfterPlay started
    byte minutesAfterPlay = (t - playTime) / 60;
    if(minutesAfterPlay >= recMinutesAfterPlay){
      digitalWrite(REC_ST, LOW); // stop recording
      timeoutCounter++;
      if(digitalRead(REC_STATUS)==0 | (timeoutCounter > 10)){
        digitalWrite(REC_POW, LOW);
        REC_STATE = 0;
        playState = 3;
        maxDepth = 0; // reset maxDepth    
        timeoutCounter = 0;
      } 
    }
  }
}
