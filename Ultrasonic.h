#ifndef Ultrasonic_h
#define Ultrasonic_h

class Ultrasonic {
  public:
    Ultrasonic(int trigPin, int echoPin, int dcPin);
    int getDistance();
  private:
    int _echoPin;  // 12
    int _trigPin;  // 11
    int _dcPin;    // 4
    long microsecondsToMillimeters(long microseconds);
};

#endif
