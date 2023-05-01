#include <EITKitArduino.h>

EITKitArduino *eit = nullptr;

void setup() {
  //eit.take_measurements(AD, AD, 16);
  eit = new EITKitArduino(16,1,4, AD, AD, true);
}

void loop() {
  // Example for taking measurements 
  // Should return origin frame and frame measurements
  // in Serial
  if (!eit) {
    Serial.println("EIT not set!");
  } else {
    eit->take_measurements(AD, AD);
    // Example for checking attributes of EITKitArduino
//    Serial.println(eit->get_num_bands()); 
  }
  delay(5);

}
