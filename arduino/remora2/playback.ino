int checkPlay(){
  if(depth > maxDepth) {
    maxDepth = depth; // track maximum depth
  }

  if(depth < 20.0) maxDepth = depth; // reset maxDepth when each dive ends

  
  // check if after exceeding playback depth, came shallow enough to allow another playback
  if(playBackDepthExceeded==2){
    if(depth < playBackResetDepth){
      maxDepth = depth;
      playBackDepthExceeded = 0;
      //Serial.println("D");
    }
  }

  // playNow State = 0
  // waiting for playback algorithm to be satisfied to trigger playback
  // prevent from playing back more than once per x minutes
  if(playNow==0){
    if((depth > playBackDepthThreshold) & (playBackDepthExceeded==0) & (((t - playTime)/60) > minPlayBackInterval)) {
      playBackDepthExceeded = 1;  // check if went deeper than a certain depth
    }

    // Trigger record if on ascent came up enough
    if ((playBackDepthExceeded==1) & (maxDepth - depth > ascentRecordTrigger) & (nPlayed < maxPlayBacks) & (REC_STATE==0)) {
      digitalWrite(REC_POW, HIGH); // turn on recorder
      digitalWrite(REC_ST, HIGH);  // start recording
      REC_STATE = 1;
    }

    // Trigger playback if on ascent came up enough
    if ((playBackDepthExceeded==1) & (maxDepth - depth > ascentDepthTrigger) & (nPlayed < maxPlayBacks)) {
        playNow = 1;
        digitalWrite(PLAY_POW, HIGH);  // play will start automatically
        PLAY_STATE = 1;
        playTime = t;
        playBackDepthExceeded = 2;
        nPlayed += 1;
        //Serial.println("P");
    }
  }

  // playNow state = 1
  // turn off playback board when playing done
  // give 10 seconds before checking this because takes some time for play board to wake
  if((playNow==1) & (t > playTime + 10)){
    int playStatusPin = analogRead(PLAY_STATUS);
   // Serial.println(playStatusPin);
    if(playStatusPin < 200){
      digitalWrite(PLAY_POW, LOW);
      PLAY_STATE = 0;
      playNow = 2;
    }
  }

  // playNow state = 2
  if(playNow==2){
    // wait to turn off record recMinutesAfterPlay started
    byte minutesAfterPlay = (t - playTime) / 60;
    if(minutesAfterPlay >= recMinutesAfterPlay){
      digitalWrite(REC_ST, LOW); // stop recording
      // wait for file to close
      if(digitalRead(REC_STATUS)==0){
        digitalWrite(REC_POW, LOW);
        REC_STATE = 0;
        playNow = 0;
      }
    }
  }

  return REC_STATE;
}
