#define SOUND_SPEED 0.0343
const int LEFT_TRIG = 5;
const int LEFT_ECHO = 18;
const int CENTER_TRIG = 17;
const int CENTER_ECHO = 19;
const int RIGHT_TRIG = 16;
const int RIGHT_ECHO = 21;

float measureDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0)
    return -1.0;
  return (duration * SOUND_SPEED / 2.0) / 100.0; // Convert cm to meters
}

void setup()
{
  Serial.begin(115200);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(CENTER_TRIG, OUTPUT);
  pinMode(CENTER_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(CENTER_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);
}

void loop()
{
  unsigned long timestamp = millis();
  
  float left = measureDistance(LEFT_TRIG, LEFT_ECHO);
  delay(20);
  float center = measureDistance(CENTER_TRIG, CENTER_ECHO);
  delay(20);
  float right = measureDistance(RIGHT_TRIG, RIGHT_ECHO);
  
  Serial.print("ULTRA,");
  Serial.print(timestamp);
  Serial.print(",");
  Serial.print(left, 3);
  Serial.print(",");
  Serial.print(center, 3);
  Serial.print(",");
  Serial.println(right, 3);
  
  delay(30);
}