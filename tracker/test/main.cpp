/*  PulseSensor Starter Project and Signal Tester
 *  The Best Way to Get Started  With, or See the Raw Signal of, your PulseSensor.com™ & Arduino.
 *
 *  Here is a link to the tutorial
 *  https://pulsesensor.com/pages/code-and-guide
 *
 *  WATCH ME (Tutorial Video):
 *  https://www.youtube.com/watch?v=RbB8NSRa5X4
 *
 *
-------------------------------------------------------------
1) This shows a live human Heartbeat Pulse.
2) Live visualization in Arduino's Cool "Serial Plotter".
3) Blink an LED on each Heartbeat.
4) This is the direct Pulse Sensor's Signal.
5) A great first-step in troubleshooting your circuit and connections.
6) "Human-readable" code that is newbie friendly."

*/
#include <Arduino.h>
#include <stack>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


String s = "google.com/maps/dir/";

unsigned long interval = 180000;
static const uint32_t GPSBaud = 9600;
unsigned long previousMillis = 0;
int data_counter;
TinyGPSPlus gps;// The TinyGPSPlus object
SoftwareSerial ss(18, 19);// The serial connection to the GPS device

const size_t BUFSIZE = 300;
char f_buffer[BUFSIZE];
float *f_buf = (float*)f_buffer;

int PulseSensorPurplePin = 36;   
int LED13 = 2;   
int Signal;            
int Threshold = 2020; 
int count = 9;
unsigned long starttime = 0;
unsigned long lastmsg = 0;
double heartrate = 0;


struct Queue {
  int front, rear, capacity;
  uint16_t* queue;
  Queue(int c)
  {
    front = rear = 0;
    capacity = c;
    queue = new uint16_t;
  }

  ~Queue() { delete[] queue; }

  // function to delete an element
  // from the front of the queue
  void queueDequeue()
  {
    // if queue is empty
    if (front == rear) {
      printf("\nQueue is  empty\n");
      return;
    }

    // shift all the elements from index 2 till rear
    // to the left by one
    else {
      for (int i = 0; i < rear - 1; i++) {
        queue[i] = queue[i + 1];
      }

      // decrement rear
      rear--;
    }
    return;
  }

  // function to insert an element
  // at the rear of the queue
  void queueEnqueue(uint16_t data)
  {
    // check queue is full or not
    if (capacity == rear) {
      this->queueDequeue();
      return;
    }

    // insert element at the rear
    else {
      queue[rear] = data;
      rear++;
    }
    return;
  }   

  float queueAverage()
  {
    uint32_t sum = 0;
    for(int i = 0; i < this->capacity; i++) {
      sum += queue[i]; 
    }
    return sum/(this->capacity);
  }

  uint16_t queueDiff()
  {
    if(queue[1]==0) {return 1;};
    return queue[0] - queue[1];
  }
};
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
void send_gps_data()
{
  if (gps.location.lat() == 0 || gps.location.lng() == 0)
  {
    Serial.println("Return Executed");
    return;
  }

  data_counter++;

  Serial.print("Latitude (deg): ");
  f_buf[data_counter] = gps.location.lat();
  Serial.println(f_buf[data_counter]);

  Serial.print("Longitude (deg): ");
  f_buf[data_counter + 1] = gps.location.lng();
  Serial.println(f_buf[data_counter + 1]);

  Serial.println(data_counter);
  Serial.println();

  s += String(gps.location.lat(), 6);
  s += ",";
  s += String(gps.location.lng(), 6);
  s += "/";

  Serial.println(s);

  if (data_counter >= 3)
  {
    data_counter = 0;

    Serial.println("Sending Message");

    ss.println("AT+CMGF=1\r");
    delay(1000);

    ss.println("AT+CNMI=2,2,0,0,0\r");
    delay(1000);

    // ss.print("AT+CMGS=\"+60182030696\"\r"); //Replace this with your mobile number
    // ss.print("AT+CMGS=\"+601116034382\"\r"); //Replace this with your mobile number
    ss.print("AT+CMGS=\"+60194421397\"\r");
    delay(1000);
    ss.print(s + "\nLast Lat: "+ String(gps.location.lat()) + "\nLast Long: "+ String(gps.location.lng()));
    ss.print("\n Heart Rate: "+ String(heartrate));
    // ss.print(" Last Lat: "+ String(gps.location.lat()));
    // ss.print(" Last Long: "+ String(gps.location.lng()));
    ss.write(0x1A);
    delay(1000);
    s = "google.com/maps/dir/";
  }
}
void send_abnormal_heartrate(){
  if(millis()-lastmsg>60000){
  Serial.println("Sending Message");

    ss.println("AT+CMGF=1\r");
    delay(1000);

    ss.println("AT+CNMI=2,2,0,0,0\r");
    delay(1000);

    // ss.print("AT+CMGS=\"+60182030696\"\r"); //Replace this with your mobile number
    // ss.print("AT+CMGS=\"+601116034382\"\r"); //Replace this with your mobile number
    ss.print("AT+CMGS=\"+60194421397\"\r");
    delay(1000);
    ss.print("WARNING!! ABNORMAL HEART RATE\ngoogle.com/maps/dir/"+String(gps.location.lat(), 6)+","+String(gps.location.lng(), 6) + "/ \nLast Lat: "+ String(gps.location.lat()) + "\nLast Long: "+ String(gps.location.lng()));
    ss.print("\n Heart Rate: "+ String(heartrate));
    // ss.print(" Last Lat: "+ String(gps.location.lat()));
    // ss.print(" Last Long: "+ String(gps.location.lng()));
    ss.write(0x1A);
    delay(1000);
    }
}
void send_abnormal_location(){
    if(millis()-lastmsg>60000){
  Serial.println("Sending Message");

    ss.println("AT+CMGF=1\r");
    delay(1000);

    ss.println("AT+CNMI=2,2,0,0,0\r");
    delay(1000);

    // ss.print("AT+CMGS=\"+60182030696\"\r"); //Replace this with your mobile number
    // ss.print("AT+CMGS=\"+601116034382\"\r"); //Replace this with your mobile number
    ss.print("AT+CMGS=\"+60194421397\"\r");
    delay(1000);
    ss.print("WARNING!! OUTSIDE QUARANTINE\ngoogle.com/maps/dir/"+String(gps.location.lat(), 6)+","+String(gps.location.lng(), 6) + "/ \nLast Lat: "+ String(gps.location.lat()) + "\nLast Long: "+ String(gps.location.lng()));
    ss.print("\n Heart Rate: "+ String(heartrate));
    // ss.print(" Last Lat: "+ String(gps.location.lat()));
    // ss.print(" Last Long: "+ String(gps.location.lng()));
    ss.write(0x1A);
    delay(1000);
    }
}

//  Variables
bool counted = false;
Queue q(50);
Queue t(2);


// The SetUp Function:
void setup() {
  pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
  Serial.begin(115200);         // Set's up Serial Communication at certain speed.
  pinMode(5, INPUT_PULLUP); 
  pinMode(4, OUTPUT);
  ss.begin(GPSBaud);
  
  Serial.println("Starting...");
  ss.println("\r");
  ss.println("AT\r");
  delay(10);

  ss.println("\r");
  ss.println("AT+GPS=1\r");

  delay(100);
  ss.println("AT+CREG=2\r");
  delay(6000);

  //ss.print("AT+CREG?\r");
  ss.println("AT+CGATT=1\r");
  delay(6000);

  ss.println("AT+CGDCONT=1,\"IP\",\"WWW\"\r");
  delay(6000);

  // ss.println("AT+LOCATION=1\r");
  ss.println("AT+CGACT=1,1\r");
  delay(6000);

  //Initialize ends
  //Initialize GPS
  ss.println("\r");
  ss.println("AT+GPS=1\r");
  delay(1000);

  //ss.println("AT+GPSMD=1\r");   // Change to only GPS mode from GPS+BDS, set to 2 to revert to default.
  ss.println("AT+GPSRD=10\r");
  delay(100);

  // set SMS mode to text mode
  ss.println("AT+CMGF=1\r");
  delay(1000);

  //ss.println("AT+LOCATION=2\r");

  Serial.println("Setup Executed");
}

// The Main Loop Function
void loop() {
  (digitalRead(5)==0)? digitalWrite(4, HIGH): digitalWrite(4, LOW);

  // while (millis()<starttime+5000){ 
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
  // Serial.println(Signal); 
  q.queueEnqueue(Signal);
  float wave = q.queueAverage();                                              // Assign this value to the "Signal" variable.
  Serial.printf("%.2f %.2lf %d %d\n", wave, heartrate, starttime, millis());
  
                   // Send the Signal value to Serial Plotter.


   if(wave > Threshold && counted == false){                          // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
     digitalWrite(LED13,HIGH);
    counted = true;
    count++;
    heartrate = 60000/(millis()-starttime);
   } 
   else if((wave < Threshold) && counted) {
    counted=false;
    digitalWrite(LED13,LOW);                //  Else, the sigal must be below "550", so "turn-off" this LED.
    starttime = millis();
   }


   smartDelay(2000);

if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
    
  unsigned long currentMillis = millis();
if ((unsigned long)(currentMillis - previousMillis) >= 30000) {
    Serial.println(gps.location.lat());
    Serial.println(gps.location.lng());}


if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    // Serial.println(gps.location.lat());
    // Serial.println(gps.location.lng());
    send_gps_data();
    previousMillis = currentMillis;
  }

if((heartrate<60 ||heartrate>150)&& heartrate!=0){
send_abnormal_heartrate();
lastmsg=millis();
}
if(((gps.location.lat()<3.04 || gps.location.lat()>3.06)&& gps.location.lat()!=0)||((gps.location.lng()<101.71 || gps.location.lng()>101.73)&& gps.location.lng()!=0)){
send_abnormal_location();
lastmsg=millis();
}
// }

// heartrate = count*60000/(millis()+1);
// Serial.print ("BPM = ");
// Serial.println (heartrate);                        // Display BPM in the Serial Monitor
// Serial.println ();
// count = 0;
// starttime=millis();
// heartrate = 60000/t.queueDiff();
}