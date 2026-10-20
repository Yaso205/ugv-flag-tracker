// Define pins based on your circuit connections
const int trigPin1 = 11;  // Trigger pin for Sensor 1
const int echoPin1 = 12;  // Echo pin for Sensor 1
const int trigPin2 = 8;  // Trigger pin for Sensor 2
const int echoPin2 = 9;  // Echo pin for Sensor 2

// Array configuration
const int arraySize = 50; // Maximum capacity for stored readings
int sensor1Readings[arraySize];
int sensor2Readings[arraySize];
int currentIndex = 0;

unsigned long previousMillis = 0;
const long interval = 2000; // 2 seconds interval
bool runningState = true;

void setup() {
  Serial.begin(9600);
  
  // Initialize sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  Serial.println("System started. Press ENTER in the Serial Monitor to stop.");
}

void loop() {
  // Check if the program has been stopped by pressing Enter
  if (!runningState) {
    return; 
  }

  // Check if user pressed Enter in the Serial Monitor
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == '\n' || incomingChar == '\r') {
      runningState = false;
      Serial.println("\n--- Program Stopped by User ---");
      printFinalArrays();
      return;
    }
  }

  // Handle the 2-second non-blocking interval
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read distance from both sensors
    int dist1 = getDistance(trigPin1, echoPin1);
    int dist2 = getDistance(trigPin2, echoPin2);

    // Store readings into separate arrays if space permits
    if (currentIndex < arraySize) {
      sensor1Readings[currentIndex] = dist1;
      sensor2Readings[currentIndex] = dist2;

      // Print current readings
      Serial.print("Index [");
      Serial.print(currentIndex);
      Serial.print("] -> Sensor 1: ");
      Serial.print(dist1);
      Serial.print(" cm | Sensor 2: ");
      Serial.print(dist2);
      Serial.println(" cm");

      currentIndex++;
    } else {
      Serial.println("Arrays are full! Stopping data collection.");
      runningState = false;
      printFinalArrays();
    }
  }
}

// Function to trigger sensor and calculate distance in centimeters
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout to prevent hanging
  int distance = duration * 0.034 / 2;
  
  // Return 0 if out of range or no echo received
  if (distance <= 0 || distance > 400) {
    return 0;
  }
  return distance;
}

// Function to display final arrays when stopped
void printFinalArrays() {
  Serial.println("\nFinal Recorded Data:");
  Serial.print("Sensor 1 Array: [");
  for (int i = 0; i < currentIndex; i++) {
    Serial.print(sensor1Readings[i]);
    if (i < currentIndex - 1) Serial.print(", ");
  }
  Serial.println("]");

  Serial.print("Sensor 2 Array: [");
  for (int i = 0; i < currentIndex; i++) {
    Serial.print(sensor2Readings[i]);
    if (i < currentIndex - 1) Serial.print(", ");
  }
  Serial.println("]");
}