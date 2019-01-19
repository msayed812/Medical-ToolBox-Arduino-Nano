//DH 11 Inclusions
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//DH 11 Definitions
#define DHTPIN            7         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT11     // DHT 11 
#define cor_factor        1
DHT_Unified dht(DHTPIN, DHTTYPE);

//HB Definitions
#define PROCESSING_VISUALIZER 1
#define SERIAL_PLOTTER        2

static int outputType = SERIAL_PLOTTER;

// Blueooth Commands
#define RT_comm     1
#define BT_comm     2
#define HB_comm     3
#define start_comm  4
#define exit_comm   5


// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime  = 0;           // used to find IBI
volatile int P =512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 530;                // used to find instant moment of heart beat, seeded
volatile int amp = 0;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

//pins connections
int POW_led  = 6;
int HB_led   = 5;
int BT_led   = 4;
int RT_led   = 3;
int buzz     = 8; 
int lm35     = A1;  // Connect LM35 pin 2 to Arduino pin A1//

//control variables
int dataIn  = 0;
int dataOut = 0;

// HB Variables
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat

// LM35 Variables 
int val     = 0;
float mv    = 0;
int cel     = 0;
int farh    = 0;
int max_cel = 39;
int min_cel = 32;

// HB Variables 
int max_hb = 100;
int min_hb = 60;

// Functions prototypes
void ambient_sys();
void body_sys();
void HB_sys();
void serialOutput();
void serialOutputWhenBeatHappens();
void sendDataToSerial(char symbol, int data );
void interruptSetup();
ISR(TIMER2_COMPA_vect);
void starting();
void start_menue();
void exiting();
void emerg(char c, char m);

void setup() {
  Serial.begin(9600);
  
  //Initialize device.
  dht.begin();
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS
  // Pin config.
  pinMode(POW_led, OUTPUT);
  pinMode(HB_led, OUTPUT);
  pinMode(BT_led, OUTPUT);
  pinMode(RT_led, OUTPUT);
  pinMode(buzz, OUTPUT);
}

void loop() {
  if(Serial.available()>0)
  {
    dataIn = Serial.read(); // reading the data received from the bluetooth module
    if (dataIn == RT_comm)           //DH11
    {
      ambient_sys();
    }
    else if (dataIn == BT_comm)      //LM35
    {
      body_sys();
    }
    else if (dataIn == HB_comm)       //HBS
    {
      HB_sys();
    }
    else if (dataIn == start_comm)     //start
    {
      starting();
    }
    else if (dataIn == exit_comm)       //exit
    {
      exiting();
    }
  }
}

void ambient_sys()
{
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("**********************************************");
    Serial.println("Error reading Ambient temperature!");
    delay(250);
  }
  else 
  {
    Serial.println("**********************************************");
    Serial.print("Ambient TEMP:      ");
    Serial.print(event.temperature);
    Serial.println(" *C");
    delay(100);
  }
  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) 
  {
    Serial.println("Error reading humidity!");
    delay(150);
  }
  else 
  {
    Serial.print("Ambient Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println(" %");
    delay(150);
  } 
}

void body_sys()
{
  for(int i = 0; i < 32; i++)  //filter for smoothing the output
  {
    val += analogRead(lm35);
  }
  val = val / 32;
  mv = ( val/1023.00 ) * 5000; //convert the analog signal into mv Form
  cel = (mv/10) + cor_factor;  
  farh = (cel*9)/5 + 32;
  Serial.println("**********************************************");
  Serial.print("Body TEMP = ");
  Serial.print(cel);
  Serial.print(" *C  ||  ");
  Serial.print(farh);
  Serial.println(" farh");
  delay(200);

  if(cel > max_cel)
    emerg('T','H');
  if(cel < min_cel)
    emerg('T','L');
}

void HB_sys()
{
  serialOutput();
  if (QS == true){     // A Heartbeat Was Found
                       // BPM and IBI have been Determined
                       // Quantified Self "QS" true when arduino finds a heartbeat
        serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
        QS = false;                      // reset the Quantified Self flag for next time
  }

  delay(20);                             //  take a break
}

////////
///////  All Serial Handling Code,
///////  It's Changeable with the 'outputType' variable
///////  It's declared at start of code.
///////
void serialOutput(){   // Decide How To Output Serial.
  switch(outputType){
    case PROCESSING_VISUALIZER:
      sendDataToSerial('S', Signal);     // goes to sendDataToSerial function
      break;
    case SERIAL_PLOTTER:  // open the Arduino Serial Plotter to visualize these data
      Serial.print("Heart Beat : ");
      Serial.print(BPM);
      Serial.println(" BPM");
      //Serial.print(",");
      //Serial.print(IBI);
      //Serial.print(",");
      //Serial.println(Signal);
      break;
    default:
      break;
  }
}

//void serialOutput(){   // Decide How To Output Serial.
//  switch(outputType){
//    case PROCESSING_VISUALIZER:
//      sendDataToSerial('S', Signal);     // goes to sendDataToSerial function
//      break;
//    case SERIAL_PLOTTER:  // open the Arduino Serial Plotter to visualize these data
//      Serial.println("**********************************************");
//      Serial.print("Heart Rate = ");
//      Serial.print(BPM);
//      Serial.println(" BPM");
//      if(BPM > max_hb)
//        emerg('H','H');
//      if(BPM < min_hb)
//        emerg('H','L');
//        
//        
////      Serial.print(",");
////      Serial.print(IBI);
////      Serial.print(",");
////      Serial.println(Signal);
//      break;
//    default:
//      break;
//  }
//}

//  Decides How To OutPut BPM and IBI Data
void serialOutputWhenBeatHappens()
{
  switch(outputType)
  {
    case PROCESSING_VISUALIZER:    // find it here https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer
      sendDataToSerial('B',BPM);   // send heart rate with a 'B' prefix
      sendDataToSerial('Q',IBI);   // send time between beats with a 'Q' prefix
      break;

    default:
      break;
  }
}

//  Sends Data to Pulse Sensor Processing App, Native Mac App, or Third-party Serial Readers.
void sendDataToSerial(char symbol, int data )
{
    Serial.print(symbol);
    Serial.println(data);
}

void interruptSetup() // CHECK OUT THE Timer_Interrupt_Notes TAB FOR MORE ON INTERRUPTS 
{  
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED
}

// THIS IS THE TIMER 2 INTERRUPT SERVICE ROUTINE.
// Timer 2 makes sure that we take a reading every 2 miliseconds
ISR(TIMER2_COMPA_vect)                        // triggered when Timer2 counts to 124
{                         
  cli();                                      // disable interrupts while we do this
  Signal = analogRead(pulsePin);              // read the Pulse Sensor
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                        // T is the trough
      T = Signal;                         // keep track of lowest point in pulse wave
    }
  }

  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }


      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 530;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }

  sei();                                   // enable interrupts when youre done!
}// end isr


void starting()
{
  start_menue();
  int t = 250;
  int replay = 4;
  digitalWrite(buzz, HIGH);
  for(int i = 0; i < replay; i++)
  {
    digitalWrite(POW_led, HIGH);
    delay(t);
    digitalWrite(POW_led, LOW);
    digitalWrite(HB_led, HIGH);
    delay(t);
    digitalWrite(HB_led, LOW);
    digitalWrite(BT_led, HIGH);
    delay(t);
    digitalWrite(BT_led, LOW);
    digitalWrite(RT_led, HIGH);
    digitalWrite(buzz, LOW);
    delay(t);
    digitalWrite(RT_led, LOW);
  }
  digitalWrite(POW_led, HIGH);
}

void start_menue()
{
  Serial.println("***************************************************");
  Serial.println("***************************************************");
  Serial.println("Welcome To The Bio-Mechatronics Lab");
  Serial.println("Designers: Team of Mechatronics Engineering Ain Shams University");
  Serial.println("***************************************************");
  Serial.println("***************************************************");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Please press the button corrsponding to the test desired");
  Serial.println(",then place the sensor and wait for a bit...");
}

void exiting()
{
   Serial.println(" ");
   Serial.println(" ");
   Serial.println("***************************************************");
   Serial.println("***************************************************");
   Serial.println("***************************************************");
   Serial.print("Good Bye...");
   Serial.print("See you soon...");
}

void emerg(char c, char m)
{
  int t = 100;
  int replay = 7;
  if(c == 'T')
  {
    if(m == 'H')
    {
      for(int i = 0; i < replay; i++)
      {
        digitalWrite(buzz,    HIGH);
        digitalWrite(POW_led, HIGH);
        digitalWrite(BT_led,  LOW);
        
        delay(t);
        digitalWrite(buzz,    LOW);
        digitalWrite(POW_led, LOW);
        digitalWrite(BT_led,  HIGH);
        delay(t);
      }
    }
    else if(m == 'L')
    {
      for(int i = 0; i < replay; i++)
      {
        digitalWrite(buzz,   HIGH);
        digitalWrite(RT_led, HIGH);
        digitalWrite(BT_led, LOW);
        delay(t);
        digitalWrite(buzz,   LOW);
        digitalWrite(RT_led, LOW);
        digitalWrite(BT_led, HIGH);
        delay(t);
      }
    }
  }
  
  else if(c == 'H')
  {
    if(m == 'H')
    {
      for(int i = 0; i < replay; i++)
      {
        digitalWrite(buzz,    HIGH);
        digitalWrite(POW_led, HIGH);
        digitalWrite(HB_led,  LOW);
        
        delay(t);
        digitalWrite(buzz,    LOW);
        digitalWrite(POW_led, LOW);
        digitalWrite(HB_led,  HIGH);
        delay(t);
      }
    }
    else if(m == 'L')
    {
      for(int i = 0; i < replay; i++)
      {
        digitalWrite(buzz,   HIGH);
        digitalWrite(RT_led, HIGH);
        digitalWrite(HB_led, LOW);
        delay(t);
        digitalWrite(buzz,   LOW);
        digitalWrite(RT_led, LOW);
        digitalWrite(HB_led, HIGH);
        delay(t);
      }
    }
  }
  digitalWrite(POW_led, HIGH);
  digitalWrite(HB_led, LOW);
  digitalWrite(BT_led, LOW);
  digitalWrite(RT_led, LOW);  
}
