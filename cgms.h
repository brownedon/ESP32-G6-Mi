//
//cgms
//
RTC_SLOW_ATTR struct readings {
  long seconds;
  int glucose;
};

RTC_SLOW_ATTR struct readings reading;
RTC_SLOW_ATTR struct readings readings_arr[3];

RTC_SLOW_ATTR int alertCount = 0;
RTC_SLOW_ATTR int showReadings = 0;

void initReadings() {
  reading.seconds = 0;
  reading.glucose = 0;
  for (int i = 0; i < 3; i++ ) {
    readings_arr[i].seconds = reading.seconds;
    readings_arr[i].glucose = reading.glucose;
  }
}

void missedReadingsMsg() {
  message[0] = 0x05;
  message[1] = 0x01;
  message[2] = 0x4d;//M
  message[3] = 0x69;//i
  message[4] = 0x73;//s
  message[5] = 0x73;//s
  message[6] = 0x20;
  message[7] = 0x20;
  message[8] = 0x20;
  message[9] = 0x20;
  message[10] = 0x20;
  message[11] = 0x20;

  message_len = 12;
  newValue = true;
}


void addReading(int glucose, long seconds) {

  Serial.println("addReading");

  //move everything in the array over 1 place
  //then add the new values
  for (int i = 2; i > 0; i-- ) {
    readings_arr[i].seconds = readings_arr[i - 1].seconds;
    readings_arr[i].glucose = readings_arr[i - 1].glucose;
  }

  readings_arr[0].seconds = seconds;
  readings_arr[0].glucose = glucose;

}


float getSlopeGlucose() {
  Serial.println("getSlopeGlucose");
  int counter = 0;
  float sumx = 0.0, sumy = 0.0, sum1 = 0.0, sum2 = 0.0;

  for (int i = 0; i < 3; i++ ) {
    if (readings_arr[i].glucose > 20) {
      counter++;
      Serial.print("Seconds "); Serial.println(readings_arr[i].seconds);
      Serial.print("Glucose "); Serial.println(readings_arr[i].glucose);
      sumx = sumx + (float)(readings_arr[i].seconds / 60 );
      sumy = sumy + (float)(readings_arr[i].glucose);
    }
  }

  float xmean = sumx / counter;
  float ymean = sumy / counter;

  for (int i = 0; i < counter; i ++) {
    if (readings_arr[i].glucose > 20) {
      sum1 = sum1 + ( (float)(readings_arr[i].seconds / 60) - xmean) * ((float)(readings_arr[i].glucose) - ymean);
      sum2 = sum2 + pow(((float)(readings_arr[i].seconds / 60 ) - xmean), 2);
    }
  }

  // derive the least squares equation
  if (sum2 == 0) {
    return 0;
  }
  Serial.print(sum1); Serial.print(":"); Serial.println(sum2);
  return (sum1 / sum2);
};


void handle_glucose(int GLUCOSE, long timestamp) {
  Serial.println("+++++++++++++CGMS+++++++++++++");
  Serial.println("handle_glucose");
  Serial.println(GLUCOSE);
  addReading( GLUCOSE, timestamp);

  Slope = getSlopeGlucose();
  int timeToLimit = 0;
   EST_GLUCOSE = 0;
  char c_glucose[10];

  // what glucose MIGHT be right now, assuming 15 minute delay
  EST_GLUCOSE = GLUCOSE + (Slope * 15);

  //stop estimating if it's really low or really high.
  if (EST_GLUCOSE < 40 || EST_GLUCOSE > 300) {
    EST_GLUCOSE = GLUCOSE;
  }

  //rising, how long until 180
  if (Slope > 0 && GLUCOSE < 180) {
    timeToLimit = abs((180 - GLUCOSE) / Slope);
    //since the dex is ~15 minutes behind reality
    timeToLimit = timeToLimit - 15;
  }

  //falling, how long until 80
  if (Slope < 0 && GLUCOSE > 80) {
    timeToLimit = abs((GLUCOSE - 80) / Slope);
    timeToLimit = timeToLimit - 15;
  }

  if (timeToLimit < 0) {
    timeToLimit = 1;
  }

  if (timeToLimit > 99) {
    timeToLimit = 0;
  }

  Serial.print("AlertCount:"); Serial.println(alertCount);

  uint8_t msgType = 0x05;  //double vibrate then go away
  message[2] = 0xFF;
  message[3] = 0x20;
  message[4] = 0x20;
  message[5] = 0x20;
  message[6] = 0x20;
  message[7] = 0x20;
  message[8] = 0x20;
  message[9] = 0x20;
  message[10] = 0x20;
  message[11] = 0x20;
  Serial.print("Est Glucose ");
  Serial.println(EST_GLUCOSE);
  //
  if (EST_GLUCOSE < 80  && alertCount == 0 && EST_GLUCOSE > 65) {
    if (alertCount == 0) {
      //msgType = 0x03;
      Alert = 1;
      alertCount++;
    } else {
      msgType = 0x05;
      alertCount++;
      if (alertCount >= 6) {
        alertCount = 0;
      }
    }
  }

  if (EST_GLUCOSE < 60) {
    if (alertCount == 0) {
      // msgType = 0x03;
      Alert = 1;
      alertCount++;
    } else {
      msgType = 0x05;
      alertCount++;
      if (alertCount >= 4) {
        alertCount = 0;
      }
    }
  }

  if (EST_GLUCOSE > 180 ) {
    if (alertCount == 0) {
      //msgType = 0x03;
      Alert = 1;
      alertCount++;
    } else {
      msgType = 0x05;
      alertCount++;
      if (alertCount >= 24) {
        alertCount = 0;
      }
    }
  }

  if (EST_GLUCOSE > 80 && EST_GLUCOSE < 180) {
    alertCount = 0;
    msgType = 0x05;
  }

  if (abs(Slope) >= 3) {
    //msgType = 0x03;
    Alert = 1;
  }


  sprintf(c_glucose, "%d", EST_GLUCOSE);
  message[2] = c_glucose[0];
  message[3] = c_glucose[1];
  if (strlen(c_glucose) > 2) {
    message[4] = c_glucose[2];
  }

  //if ((EST_GLUCOSE < 90 || EST_GLUCOSE > 160) || (sqrt(pow(Slope, 2)) > 1.5)) {
    showReadings = 0;
    if (timeToLimit == 0 || timeToLimit == 1) {
      // to do add slope to message
      if (abs(Slope) > 0.1) {
        float value = abs(Slope);
        int left_part, right_part;
        char buffer[50];
        sprintf(buffer, "%2.1lf", value);
        sscanf(buffer, "%d.%d", &left_part, &right_part);
        sprintf(c_glucose, "%d", left_part);
        message[6] = c_glucose[0];
        message[7] = 0x2e; //decimal point
        sprintf(c_glucose, "%d", right_part);
        message[8] = c_glucose[0];
      }
    } else {
      message[6] = 0x20;
      sprintf(c_glucose, "%d", timeToLimit);
      message[7] = c_glucose[0];
      message[8] = c_glucose[1];
      message[9] = 0x20;
    }
  //}
  /* else {
    //send notification only once
    if (showReadings != 1) {
      //send Good
      message[2] = 0x47;
      message[3] = 0x6F;
      message[4] = 0x6F;
      message[5] = 0x64;

      showReadings = 1;
    } else {
      message[2] = 0xff;
    }
  }*/

  if (readings_arr[0].glucose == NULL) {
    initReadings();
  }
  if (readings_arr[0].glucose != 0 && readings_arr[1].glucose != 0) {
    if (abs(readings_arr[0].glucose - readings_arr[1].glucose) > 25) {
      message[11] = 0x3f;  //?
    }
  }

  if (Slope > 0) {
    message[10] = 0x2b; //+
  };
  if (Slope < 0) {
    message[10] = 0x2d; //-
  }

  Serial.print("Slope:"); Serial.println(Slope);
  Serial.print("TimeToLimit:"); Serial.println(timeToLimit);

  message[0] = msgType;
  message[1] = 0x01;

  if (message[2] != 0xff) {
    newValue = true;
    message_len = 12;
  }
  Serial.println("+++++++++++++END CGMS+++++++++++++");
}
