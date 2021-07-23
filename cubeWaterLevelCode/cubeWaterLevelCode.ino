#define echoPin 6 
#define trigPin 5 
#define ledPin 13 

long duration; 
int distance; 

void setup() 
{
  pinMode(ledPin, OUTPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  Serial.begin(9600); 
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); 
}
void loop() 
{
  digitalWrite(ledPin, HIGH);    
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; 
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  digitalWrite(ledPin, LOW);    
  delay(10000);               
}
