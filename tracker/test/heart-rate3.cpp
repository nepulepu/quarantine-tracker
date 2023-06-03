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


//  Variables
int PulseSensorPurplePin = 36;   
int LED13 = 2;   
int Signal;            
int Threshold = 2020; 
int count = 9;
unsigned long starttime = 0;
double heartrate = 0;
bool counted = false;
Queue q(50);
Queue t(2);


// The SetUp Function:
void setup() {
  pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
   Serial.begin(115200);         // Set's up Serial Communication at certain speed.

}

// The Main Loop Function
void loop() {
  
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
// }

// heartrate = count*60000/(millis()+1);
// Serial.print ("BPM = ");
// Serial.println (heartrate);                        // Display BPM in the Serial Monitor
// Serial.println ();
// count = 0;
// starttime=millis();
// heartrate = 60000/t.queueDiff();
}