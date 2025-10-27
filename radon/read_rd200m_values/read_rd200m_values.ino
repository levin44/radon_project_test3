/*
// Modified by: Nisal v0.0.1
// ChtGt 2025-10-11
//working but gives wrpmg 1.76pCi for correct 37bq/m3 
//    FILE: read_rd200m_values.ino
//  AUTHOR: Kyuho Kim (ekyuho@gmail.com)
// CREATED: April 10, 2018

// Released to the public domain
*/
// https://www.slideshare.net/radonFTlabkorea/ftlabdatasheet-rd200-mv12eng 

#define INTERVAL 60  //sec
#include <Rd200m.h>
Rd200m radon(&Serial2);

void getit() {
  Serial.print(String(radon.value()) + String(" pCi"));
  String s = radon.status();
  if (s != "") Serial.println(String(" (")+ s +String(")"));
  else Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, 16, 17);
  Serial.println("\nRadon Sensor RD200M V1.0 April 10, 2018");
  radon.debug(0);
  radon.onPacket(getit);
}

void loop() {   
  static unsigned mark = 0;
  
  if (millis() > mark) {
	  mark = millis() + (INTERVAL * 1000);
	  radon.request();
  }
  radon.update();
}
