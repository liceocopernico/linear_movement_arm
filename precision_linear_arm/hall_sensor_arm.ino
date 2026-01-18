#include <mwc_stepper.h>
#include <EEPROM.h>
#include "./eepromutils.h"

#define EN_PIN 4
#define DIR_PIN 5
#define STEP_PIN 6
#define PULSE 6400
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 0
#define RPM 50
#define RPM1 40
#define STOP_ONE 2
#define STOP_TWO 3
#define VALIDFLAG 125
#define DELTAT 250


const int cmd_HANDSHAKE = 'h';
const int cmd_GO_HOME1 = 'w';
const int cmd_GO_HOME2 = 'x';
const int cmd_CALIBRATE = 'c';
const int cmd_PING = 'p';
const int cmd_MOVE_FORWARD = 'f';
const int cmd_MOVE_BACKWARD = 'b';
const int cmd_SEND_MOVEMENT_FORWARD = 'm';
const int cmd_SEND_MOVEMENT_BACKWARD= 'q';
const int cmd_GET_POSITION = 'g';
const int cmd_GET_MAX_POSITION ='a';
const int cmd_GO_POSITION = 'u';
const int cmd_GET_STOP_STATUS ='e';
const int cmd_SET_MAX_POSITION = 't';
const int cmd_SET_POSITION = 'k';
const int cmd_IS_CALIBRATED = 'r';
const int cmd_REBOOT = 'z';
const int cmd_DEBUG = 'd';

const int one_turn=6400;
const int min_step=80;
const float resolution=0.1;

String data;
volatile bool running = true;
char command;
bool writeflag=false;
bool debug=false;
long  lastwrite=0;
long  move_position=0;
unsigned int   current_address=0;

struct arm_configuration {
  int validdata;                    
  bool calibrated;
  long position;                   // last cart position
  long max_position;
  
};

arm_configuration myarm;

MWCSTEPPER nema23(EN_PIN, DIR_PIN, STEP_PIN);

void print_data(){
    Serial.print("Valid flag: ");
    Serial.println(myarm.validdata);
    Serial.print("Calibration status: ");
    Serial.println(myarm.calibrated);
    Serial.print("Current position: ");
    Serial.println(myarm.position);
    Serial.print("Max position: ");
    Serial.println(myarm.max_position);
}

template <class T> void debug_print(T param)
{
   if(debug){
    Serial.print(param);
   }
}
template <class T> void debug_println(T param)
{
   if(debug){
    Serial.println(param);
   }
}

void set_defaults(){
    myarm.validdata=0;
    myarm.calibrated=false;
    myarm.position=0;
    myarm.max_position=0;
}

void invalidate_eeprom(){
    int datasize;
    datasize = sizeof(myarm);
    myarm.validdata=0;//invalidate current data location
    EEPROM_writeAnything(current_address, myarm);//write invalidated data
    current_address += datasize;
    debug_print("Eeprom invalidated!");
}

void write_eeprom(){
    EEPROM_writeAnything(current_address, myarm);
    writeflag=false;
}

void setup() {
  int datasize;    // will hold size of the struct myarm
  int nlocations;
  current_address=0;
  bool found=false;
  lastwrite=millis();
  Serial.begin(115200);
  nema23.init();
  nema23.active(true);

  pinMode(STOP_ONE, INPUT);
  attachInterrupt(digitalPinToInterrupt(STOP_ONE), homestop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STOP_TWO), homestop, CHANGE);
  running=digitalRead(STOP_ONE)||digitalRead(STOP_TWO);
  
  datasize = sizeof(myarm);
  nlocations = EEPROM.length() / datasize;

  for (int lp1 = 0; lp1 < nlocations; lp1++) {
    int addr = lp1 * datasize;
    EEPROM_readAnything(addr, myarm);
    
    if (myarm.validdata==VALIDFLAG) {
      
      current_address = addr;
      found = true;
      break;
    }
  }

  if (found){
    EEPROM_readAnything(current_address, myarm);//read current data
    myarm.validdata=0;//invalidate current data location
    EEPROM_writeAnything(current_address, myarm);//write invalidated data
    current_address += datasize;
    
    if (current_address >= (nlocations * datasize)) {
      current_address = 0; //loop back eeprom locations
    }
    myarm.validdata = VALIDFLAG;
    EEPROM_writeAnything(current_address, myarm);
    print_data();
  }else{
    debug_println("No valid data found");
    print_data();
    set_defaults();
    write_eeprom();
  }
  Serial.flush();
}

int move(bool rotation,uint8_t rpm,unsigned long microsteps,uint8_t stop,uint16_t pulse){

          if ((myarm.validdata==125)&& (!writeflag)){
              invalidate_eeprom();
          }
   
          long inc =(rotation)?1:-1;

          running=digitalRead(stop);
          move_position=myarm.position+inc*microsteps;
          
          if ((move_position > myarm.max_position)|| (move_position<0)){
              Serial.println("Cannot move");
              return 0;
            }

          if (running) {
                nema23.active(false);
                nema23.set(rotation, rpm, pulse);
                for (unsigned long i = 0; i < microsteps; i++){
                    if(running){
                      myarm.position+=inc;
                      nema23.run();
                  }
	         }
           nema23.active(true);
          }
          writeflag=true;
          return 1;
}


void go_home(char side, bool rotation1,bool rotation2,uint8_t rpm_coarse,uint8_t rpm_fine,uint16_t pulse, uint8_t stop){
    
    if ((myarm.validdata==125)&& (!writeflag)){
              invalidate_eeprom();
          }
    
  
    running=digitalRead(stop);
    long inc;
    inc = (side=='m')?-1:1;

    if (side=='m'){
      Serial.println("Going home, motor side.");
    }else{
      Serial.println("Going home, other side.");
    }
    if (running) {
      nema23.active(false);
      nema23.set(rotation1, rpm_fine, pulse);
      while (running){
            myarm.position+=inc;
            nema23.run();
        }
      running=true;
      nema23.set(rotation2, rpm_coarse, pulse);
      delayMicroseconds(2000);
      for (int i = 0; i < 8*min_step; i++){
                myarm.position-=inc;
                nema23.run();
             
	     }
      nema23.active(true);
      
    }
    writeflag=true;
  }



void loop() {


if( (writeflag) && (millis()-lastwrite>DELTAT)){ 

      myarm.validdata=VALIDFLAG;
      debug_print("Write data to eeprom at address: ");
      debug_println(current_address);
      write_eeprom();
      lastwrite=millis();
      writeflag=false;

}
  

if (Serial.available()) {
    command = Serial.read();
    switch (command) {
      case cmd_REBOOT:
        reboot();
        break;
      case cmd_IS_CALIBRATED:
        Serial.println(myarm.calibrated);
        Serial.println("executed");
        break;
      case cmd_SET_POSITION:
        data=Serial.readString();
        myarm.position=data.toInt();
        writeflag=true;
        Serial.println("executed");
        break;
      case cmd_SET_MAX_POSITION:
        data=Serial.readString();
        myarm.max_position=data.toInt();
        writeflag=true;
        Serial.println("executed");
        break;
      case cmd_GET_STOP_STATUS:
        Serial.print(digitalRead(STOP_ONE));
        Serial.print(",");
        Serial.println(digitalRead(STOP_TWO));
        Serial.println("executed");
        break;  

      case cmd_GO_POSITION:
          data=Serial.readString();
          move_position=data.toInt();
          
          if ((move_position<=myarm.max_position)&& (move_position>=0)){
              if (move_position>=myarm.position){
                  move(CLOCKWISE,RPM,move_position-myarm.position,STOP_TWO,PULSE);
              }else{
                  move(COUNTERCLOCKWISE,RPM,myarm.position-move_position,STOP_ONE,PULSE);
              }

          }
          Serial.println("executed");
        break;
      case cmd_GET_POSITION:
        Serial.println(myarm.position);
        Serial.println("executed");
        break;
      case cmd_GET_MAX_POSITION:
        Serial.println(myarm.max_position);
        Serial.println("executed");
        break;
      
      case cmd_CALIBRATE:
         
         go_home('m',COUNTERCLOCKWISE,CLOCKWISE,RPM,RPM1,PULSE,STOP_ONE);
         myarm.position=0;
         go_home('o',CLOCKWISE,COUNTERCLOCKWISE,RPM,RPM1,PULSE,STOP_TWO);
         myarm.max_position=myarm.position;
         Serial.print("Max Position: ");
         Serial.println(myarm.max_position);
         Serial.println("executed");
         myarm.calibrated=true;
         
         break;
      
      case cmd_GO_HOME1:
         go_home('m',COUNTERCLOCKWISE,CLOCKWISE,RPM,RPM1,PULSE,STOP_ONE);
         Serial.println("executed");
         Serial.flush();
         break;    
      
      case cmd_GO_HOME2:
        go_home('o',CLOCKWISE,COUNTERCLOCKWISE,RPM,RPM1,PULSE,STOP_TWO);
        Serial.println("executed");
        Serial.flush();
        break;    

      case cmd_MOVE_FORWARD:
        if (is_not_calibrated()){break;}
        Serial.println("Move forward 1 turn");
        move(CLOCKWISE,RPM,one_turn,STOP_TWO,PULSE);
        Serial.println("executed");
        break;

      case cmd_SEND_MOVEMENT_FORWARD:
          if (is_not_calibrated()){break;}
          data=Serial.readString();
          move(CLOCKWISE,RPM,floor((data.toFloat())/resolution)*min_step,STOP_TWO,PULSE);
          Serial.println("executed");        
        break;

      case cmd_SEND_MOVEMENT_BACKWARD:
          if (is_not_calibrated()){break;}
          data=Serial.readString();
          move(COUNTERCLOCKWISE,RPM,floor((data.toFloat())/resolution)*min_step,STOP_ONE,PULSE);
          Serial.println("executed");     
        break;

       case cmd_MOVE_BACKWARD:
        if (is_not_calibrated()){break;}        
        Serial.println("Move backwards 1 turn");
        move(COUNTERCLOCKWISE,RPM,one_turn,STOP_ONE,PULSE);
        Serial.println("executed");
        break; 
   
      case cmd_HANDSHAKE:
        handshake();
        Serial.println("executed");
        break;
      
      case cmd_PING:
        Serial.println("PONG");
        Serial.println("executed");
        break;

      case cmd_DEBUG:
        debug=!debug;

        if (is_not_calibrated()){
          Serial.println("Not calibrated");
        }else{
          Serial.println("Calibrated");
        }

        if (debug){
          debug_println("Debug enabled");
        } else{
           Serial.println("Debug disabled");
        }
        Serial.println("executed");
        break;      
      
      }
  }

}


bool is_not_calibrated(){
    if (!myarm.calibrated){
            Serial.println("not calibrated");
            Serial.println("executed");
        } 
    return !myarm.calibrated;
}

void reboot() {
  asm volatile("jmp 0");
}

void homestop() {
 running=false;
}

void handshake() {
  Serial.println('R');
  Serial.flush();
}