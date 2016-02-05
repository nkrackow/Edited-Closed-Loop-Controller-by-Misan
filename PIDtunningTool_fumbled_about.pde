/* I needs Ramp Response */





int Speed=500;     // define for Ramp speed in Steps per Second VERY aproximately as it just changes a delay length
int Steps=250;    // define length of Ramp in Steps
boolean to_ramp_or_not_to_ramp=true;    // DO A BARREL ROLL...?


// simple tool for testing the PID response using some of the features of new firmware (S command returns the last encoder positions from last X command
// by misan 2015
// it is a loop of moves X100 .. X0 .. X100 .. X0 ...
// "P",  "I" or "D" keys increase existing PID values
// p i d keys decrease respective values
// it draws "almost" in real time motor response

import java.util.*; 
import processing.serial.*;

Serial myPort;      // The serial port

float kp=2, ki=0.125, kd=0.01;
void setup() {
  size(1200, 800);
  PFont myFont = createFont(PFont.list()[2], 14);
  textFont(myFont);
  frameRate(1);
  // List all the available serial ports:
  println(Serial.list());
  String portName = Serial.list()[1];
  myPort = new Serial(this, portName, 115200);
}

void draw() {
  if(to_ramp_or_not_to_ramp==true){
    strokeWeight(1);
    background(255,100); stroke(255,0,0);
    line(0,800,1000,200); 
    line(1000,200,1200,200);
    strokeWeight(.1);
    for(int i=1;i<12;i++) line(i*100,0,i*100,800);
    delay(500);
    stroke(0,0,255); strokeWeight(2); 
    while(myPort.available()>0) print(char(myPort.read()));
    do_a_Ramp(Steps,Speed);
    delay(500);
    myPort.write('S');
    myPort.write('\r');
    myPort.write('\n');
    delay(200);
    String s="";
    while(myPort.available()>0) s+=char(myPort.read()); 
    
    
    Scanner sc = new Scanner(s);
    
    float xscale=(600/((float(Steps)/float(Speed))/0.002))*(float(13)/float(9));
    
    int x=0; 
    while(sc.hasNextInt()) {
      point((x++)*xscale, 800-2.4*sc.nextInt());
    }
    delay(500);
  }
  else{
    strokeWeight(1);
    background(255,100); stroke(255,0,0);
    line(0,200,1200,200);
    strokeWeight(.1);
    for(int i=1;i<12;i++) line(i*100,0,i*100,800);
    delay(500);
    stroke(0,0,255); strokeWeight(2); 
    while(myPort.available()>0) print(char(myPort.read()));
    
    myPort.write('X');
    myPort.write('1');
    myPort.write('0');  
    myPort.write('0');
    myPort.write('\r');
    myPort.write('\n');
    delay(500);
    
    myPort.write('S');
    myPort.write('\r');
    myPort.write('\n');
    delay(200);
    String s="";
    while(myPort.available()>0) s+=char(myPort.read()); 
    
    Scanner sc = new Scanner(s);
    
    int x=0; 
    while(sc.hasNextInt()) {
      point((x++)*4, 800-6*sc.nextInt());
    }
      
  }

  myPort.write('X');
  //myPort.write('0'); 
  //myPort.write('0');  
  myPort.write('0'); 
  myPort.write('\r');
  myPort.write('\n');
  delay(500);
}

void keyPressed() {
  if(key=='P') {
    kp+=1; 
      myPort.write('P');
      String out=""+kp; for(int i=0; i<out.length();i++) myPort.write(out.charAt(i));
      myPort.write('\r');
      myPort.write('\n');
      println("P="+kp);
  } 
  if(key=='p')  {
    kp-=1; kp=max(0,kp);
      myPort.write('P');
      String out=""+kp; for(int i=0; i<out.length();i++) myPort.write(out.charAt(i));
      myPort.write('\r');
      myPort.write('\n');
      println("P="+kp);
  } 
  if(key=='D') 
   {
    kd+=0.01; 
      myPort.write('D');
      String out=""+kd; for(int i=0; i<out.length();i++) myPort.write(out.charAt(i));
      myPort.write('\r');
      myPort.write('\n');
      println("D="+kd);
  } 
  if(key=='d')  {
    kd-=0.01; kd=max(0,kd);
      myPort.write('D');
      String out=""+kd; for(int i=0; i<out.length();i++) myPort.write(out.charAt(i));
      myPort.write('\r');
      myPort.write('\n');
      println("D="+kd);
  } 
  if(key=='I')  {
    ki*=2; 
      myPort.write('I');
      String out=""+ki; for(int i=0; i<out.length();i++) myPort.write(out.charAt(i));
      myPort.write('\r');
      myPort.write('\n');
      println("I="+ki);
  } 
  if(key=='i')  {
    ki/=2; 
      myPort.write('I');
      String out=""+ki; for(int i=0; i<out.length();i++) myPort.write(out.charAt(i));
      myPort.write('\r');
      myPort.write('\n');
      println("I="+ki);
  } 
}
        
        
/* new code */
        
void do_a_Ramp(int Steps, int Speed){
  int delaytime=1000/Speed;            // delaytime after each Step according to Speed
  myPort.write('N');                   // Stets the 'p' couter that monitors the positions in the Controller Firmware to 0
  for(int i=0; i<Steps; i++){          // ofc the rest of the code here takes no time to run and my Speed calculation is absolutely precise!
    myPort.write('+');
    delay(delaytime);
    //println("Step="+i);
  }
}
  
  
