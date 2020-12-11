void checkPlay(){
  if(depth > maxDepth) {
    maxDepth = depth; // track maximum depth
  }

  // waiting for playback algorithm to be satisfied to trigger playback
  // prevent from playing back more than once per x seconds
  if(playNow==0){
      if((depth > playBackDepthThreshold) & (playBackDepthExceeded==0) & (t - playTime > minPlayBackInterval)) {
      playBackDepthExceeded = 1;  // check if went deeper than a certain depth
//      if(printDiags){
//        Serial.print("Playback depth exceeded: ");
//        Serial.println(playBackDepthThreshold);
//      } 
    }
  
    // check if after exceeding playback depth, came shallow enough to allow another playback
    if(playBackDepthExceeded==2){
      if(depth < playBackResetDepth){
        maxDepth = depth;
        playBackDepthExceeded = 0;
//        if(printDiags){
//          Serial.print("Reset depth: ");
//          Serial.println(playBackResetDepth);
//        }
      }
    }
  
    // Trigger playback if on ascent came up enough
    if ((playBackDepthExceeded==1) & (maxDepth - depth > ascentDepthTrigger) & (nPlayed < maxPlayBacks)) {
 //       playBackOn();
        playNow = 1;
        playTime = t + 2; // wait 2 seconds for playback board to power up
        playBackDepthExceeded = 2;
//        if(printDiags) {
//          Serial.print("Ascent trigger");
//          Serial.println(ascentDepthTrigger);
//          Serial.print("Trigger playback ");
//          Serial.println(nPlayed);
//        } 
    }
  }

  // trigger playback after delay
  if((playNow==1) & (t >= playTime)) {
     playNow = 2;
//      playTrackNumber(trackNumber);
//      trackNumber += 1;
//      if(trackNumber >= nPlayBackFiles) trackNumber = 0;
//      nPlayed++;
  }

  // turn off playback board
  if(playNow==2){
      if (t-playTime > longestPlayback){
//        playBackOff();
        playNow = 0;
      }
  }
}
