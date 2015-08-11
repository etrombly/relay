#include <avr/wdt.h>
#include <avr/sleep.h>
#include <device.h>
#include <RFM69.h>
#include <SPI.h>
#include <Time.h>

typedef struct
{
  Message message;
  uint8_t count;
  int timestamp;
} Packet;

/**************************************
CONFIGURATION PARAMETERS
**************************************/

#define NODEID 2 				// unique node ID within the closed network
#define GATEWAYID 1				// node ID of the Gateway is always 1
#define NETWORKID 1				// network ID of the network
#define ENCRYPTKEY "1234567891011121" 		// 16-char encryption key; same as on Gateway!
#define DEBUG					// uncomment for debugging
#define VERSION "RELAY V1.0"			// this value can be queried as device 3
#define SERIAL_BAUD 9600

/**************************************
Wireless settings
Match frequency to the hardware version of the radio
**************************************/

//#define FREQUENCY RF69_433MHZ
//#define FREQUENCY RF69_868MHZ
#define FREQUENCY RF69_915MHZ

#define IS_RFM69HW 				// uncomment only for RFM69HW!
#define PROMISCUOUS_MODE false
#define ACK_TIME 30 				// max # of ms to wait for an ack
#define RETRIES 5                               //number of tx retries

/**************************************
device settings
**************************************/

#define BTN_INT 1                               //interrupt number
#define BTN_PIN 3                               //button pin
#define RELAY 9                                 //relay pin

/**************************************
global variables
**************************************/
int 	TXinterval = 20;			// periodic transmission interval in seconds
bool    toggle = true;
bool    updatesSent = false;
time_t  start = now();
uint8_t id = 1;

volatile bool btnPressed = false;
volatile long lastBtnPress = -1;		// timestamp last buttonpress

const Message DEFAULT_MSG = {1, 0, 0, 0, 0, 0, VERSION};

/**************************************
configure devices
**************************************/

//Device name(devID, tx_periodically, read_function, optional_write_function)

Device uptimeDev(0, false, readUptime);
Device txIntDev(1, false, readTXInt, writeTXInt);
Device rssiDev(2, false, readRSSI);
Device verDev(3, false);
Device voltDev(4, false, readVoltage);
Device toggleDev(6, false, readToggle, writeToggle);
Device relayDev(17, false, readRelay, writeRelay);

static Device devices[] = {uptimeDev, txIntDev, rssiDev, verDev,
                    voltDev, toggleDev, relayDev};

RFM69 radio;

Packet sendBuffer[10];

/*******************************************
put non-system read/write functions here
********************************************/

void readRelay(Message *mess){
  digitalRead(RELAY) ? mess->intVal = 1 : mess->intVal = 0;
}

void writeRelay(const Message *mess){
  digitalWrite(RELAY, mess->intVal);
}

/*********************************************
Setup
*********************************************/

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup() {
  //set all pins as input with pullups, floating pins can waste power
  DDRD &= B00000011;       // set Arduino pins 2 to 7 as inputs, leaves 0 & 1 (RX & TX) as is
  DDRB = B00000000;        // set pins 8 to 13 as inputs
  PORTD |= B11111100;      // enable pullups on pins 2 to 7, leave pins 0 and 1 alone
  PORTB |= B11111111;      // enable pullups on pins 8 to 13

  //if debug enabled, initialize serial
  #ifdef DEBUG
    Serial.begin(SERIAL_BAUD);
    Serial.println("Serial initialized");
  #endif

  //initialize IO
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);

  //initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.rcCalibration();
  #ifdef IS_RFM69HW
    radio.setHighPower(); 				// only for RFM69HW!
  #endif
  radio.encrypt(ENCRYPTKEY);				// set radio encryption
  radio.promiscuous(PROMISCUOUS_MODE);			// only listen to closed network

  #ifdef DEBUG
    Serial.print("Node Software Version ");
    Serial.println(VERSION);
    Serial.print("\nTransmitting at ");
    Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(" Mhz...");
  #endif

  //setup interrupt for button
  attachInterrupt(BTN_INT, buttonHandler, LOW);

  //send wakeup message
  Message wakeup = DEFAULT_MSG;
  wakeup.devID = 99;
  queueMsg(&wakeup);
}

void loop() {
  //Serial.println(freeRam());

  Message reply = DEFAULT_MSG;

  if (radio.receiveDone()){
    if (radio.DATALEN != sizeof(Message)){
      Serial.println("INVALID PACKET");
    }else{
      Message mess = *(Message*)radio.DATA;
      bool match = false;
      reply.nodeID = radio.SENDERID;
      reply.packetID = mess.packetID;

      //check if message is a acknowledgement
      if(mess.cmd == 2){
        for(int i = 0; i < 10; i++){
          if(sendBuffer[i].message.packetID == mess.packetID){
            Serial.print("Ack received for ");
            Serial.println(sendBuffer[i].message.packetID);
            sendBuffer[i].message.packetID = NULL;
            break;
          }
        }
      }else{
        //check if message is for any devices registered on node
        for (int i = 0; i < sizeof(devices) / sizeof(Device); i++){
          if (mess.devID == devices[i].id){
            match = true;
            reply.devID = devices[i].id;
            //write for cmd 0
            if (mess.cmd == 0){
              devices[i].write(&mess);
              #ifdef DEBUG
              Serial.print("writing node ");
              Serial.print(mess.nodeID);
              Serial.print(" dev ");
              Serial.println(mess.devID);
              #endif
            }
            devices[i].read(&reply);
            #ifdef DEBUG
            Serial.print("reading node ");
            Serial.print(reply.nodeID);
            Serial.print(" dev ");
            Serial.println(reply.devID);
            #endif
            queueMsg(&reply);
          }
        }
        //invalid device id in message
        if (!match){
          reply.devID = 92;
          queueMsg(&reply);
        }
      }
    }
  }

  //check if any devices needs to transmit periodic info
  if (!updatesSent && now() % TXinterval == 0){
    Serial.println("Sending periodic updates");
    for (int i = 0; i <= sizeof(devices) / sizeof(Device); i++){
      if (devices[i].setTX){
        reply = DEFAULT_MSG;
        reply.packetID = id++;
        reply.devID = devices[i].id;
        devices[i].read(&reply);
        queueMsg(&reply);
      }
    }
    updatesSent = true;
  }else if(now() % TXinterval != 0){
    updatesSent = false;
  }

  //if button was pressed and enabled toggle the relay
  if (btnPressed && toggle){
    digitalWrite(RELAY, !digitalRead(RELAY));
    reply = DEFAULT_MSG;
    reply.packetID = id++;
    reply.devID = 17;
    relayDev.read(&reply);
    queueMsg(&reply);
  }
  btnPressed = false;

  txRadio();

  //Disabling sleep for now, was making comms too unreliable
  /*#ifdef DEBUG
    //make sure serial tx is done before sleeping
    Serial.flush();
    while ((UCSR0A & _BV (TXC0)) == 0){}
  #endif
  */

  //put chip to sleep until button is pressed, packet is RX, or watchdog timer fires
  //sleep();
}

void txRadio(){
  for(int i = 0; i < 10; i++){
    if(sendBuffer[i].message.packetID != 0){
      if(sendBuffer[i].count == 4 || (sendBuffer[i].count > 0 && now() > sendBuffer[i].timestamp + 1)){
        Serial.print("Sending message ");
        Serial.print("  ");
        Serial.print(sendBuffer[i].message.packetID);
        Serial.print("  ");
        Serial.println(sendBuffer[i].message.devID);
        radio.send(GATEWAYID, &sendBuffer[i].message, sizeof(sendBuffer[i].message));
        sendBuffer[i].count--;
        sendBuffer[i].timestamp = now();
      }else if(sendBuffer[i].count < 1){
        Serial.print("Dropping message ");
        Serial.println(sendBuffer[i].message.packetID);
        sendBuffer[i].message.packetID = NULL;
        sendBuffer[i].count = NULL;
        sendBuffer[i].timestamp = NULL;
      }
    }
  }
}

void queueMsg(Message * mess){
  for(int i = 0; i < 10; i++){
    if(sendBuffer[i].message.packetID == NULL){
      sendBuffer[i].message = *mess;
      sendBuffer[i].count = 4;
      sendBuffer[i].timestamp = now();
      Serial.print(" message ");
      Serial.print(sendBuffer[i].message.devID);
      Serial.println(" queued...");
      return;
    }
  }
  Serial.println("Send buffer full");
}

void readUptime(Message *mess){
  mess->intVal = (now() - start) / 60;
}

void readTXInt(Message *mess){
  mess->intVal = TXinterval;
}

void writeTXInt(const Message *mess){
  TXinterval = mess->intVal;
  if (TXinterval <10 && TXinterval !=0) TXinterval = 10;	// minimum interval is 10 seconds
}

void readRSSI(Message *mess){
  mess->intVal = radio.RSSI;
}

void readVoltage(Message *mess){
  long result;					// Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);					// Wait for Vref to settle
  ADCSRA |= _BV(ADSC);				// Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; 			// Back-calculate in mV
  mess->fltVal = float(result/1000.0);		// Voltage in Volt (float)
}

void readToggle(Message *mess){
  toggle ? mess->intVal = 1 : mess->intVal = 0;
}

void writeToggle(const Message *mess){
  mess->intVal ? toggle = true: toggle = false;
}

void sleep(){
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable();
  sleep_mode();
  sleep_disable();
}

void buttonHandler(){
  if (lastBtnPress != now()){
    lastBtnPress = now();
    btnPressed = true;
  }
}
