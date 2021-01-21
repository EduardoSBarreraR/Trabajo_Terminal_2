import controlP5.*;
import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import processing.serial.*;    // Importing the serial library to communicate with the Arduino 

Serial myPort;      // Initializing a vairable named 'myPort' for serial communication

ControlP5 cp5;
PFont font,font2;
PImage ipn,upiita;

// Kinect Library object
Kinect kinect;

String recived_data ;   // Variable for changing the background color
String[] datos_rec;

boolean recived;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

int CurX;
int CurY;

float Xc,Yc,Zc;
float Xr,Yr,Zr;
float[] qs = new float[7];
int[] q = new int[7];

float Xo,Yo,Zo;

PVector v;

int myColor = color(0);

int c1,c2;

float n,n1;

float O1=2.5;
float O2=4.5;
float l1=32;
float l2=31.5;
float l3=30;

float temp1=0,temp2=0,temp3=0,peso=0;

void setup() {
  size(880,480);
 
  myPort  =  new Serial (this, "COM5",  250000); // Set the com port and the baud rate according to the Arduino IDE 
  myPort.bufferUntil ( '\n' );   // Receiving the data  
  noStroke();
  cp5 = new ControlP5(this);
  font = createFont("Impact",18);
  font2 = loadFont("Square721BT-Roman-48.vlw");
  
  // create a new button with name 'buttonA'
  cp5.addButton("Activar")
  .setBroadcast(false)
     .setValue(0)
     .setPosition(710,290)
     .setSize(80,50)
     .setFont(font)
     .setBroadcast(true)
     ;
     
      // create a new button with name 'buttonA'
  cp5.addButton("Salir")
  .setBroadcast(false)
     .setValue(0)
     .setPosition(10,420)
     .setSize(80,50)
     .setFont(font)
     .setBroadcast(true)
     ;
     
  ipn = loadImage("ipn_inv.png");
  upiita = loadImage("upiita_inv.png");
      kinect = new Kinect(this);
 
  kinect.initDepth();
   kinect.enableColorDepth(true);

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  

}

void draw() {
  background(myColor);
  // background(0);
 image(kinect.getDepthImage(), 0, 0);
 
 image(ipn,650,10,60,90);
 image(upiita,780,10,80,90);
 textAlign(CENTER);
 textSize(12);
 text("SISTEMA ROBOTICO DE UN ROVER \n DE EXPLORACION PARA LA \n OBTENCION DE MUESTRAS  \n GEOLOGICAS", 760, 120);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

CurX = mouseX;
CurY = mouseY;

 int offset = CurX + CurY*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      v = depthToWorld(CurX, CurY, rawDepth);
      textAlign(LEFT);
      textFont(font2);
      textSize(15);
      text("COORDENADAS", 650, 210);
      text(" X: " + Xr +"\n Y: "+Yr+"\n Z: "+Zr, 650, 230);
      textSize(17);
      text("Ultimo peso medido: "+peso+" [g]", 640, 370);
      textSize(15);
      text("Temperatura del sistema: \n"+temp1+" Segundo motor \n"+temp2+" Fuente de energia \n"+temp3+" Tarjeta de procesamiento", 640, 400);
      
}

public void controlEvent(ControlEvent theEvent) {
  if(theEvent.getController().getName() == "Activar"){
    myPort.clear();

myPort.write(q[0]+","+q[1]+","+q[6]+","+q[3]+","+q[4]+","+q[5]+"a") ; 
println(q[0]+","+q[1]+","+q[6]+","+q[3]+","+q[4]+","+q[5]+"a");
    // myPort.write(75+","+ -16+","+ -7 + ","+8+","+25+","+15+"\n");
}
else if(theEvent.getController().getName() == "Salir"){
myPort.clear();
myPort.stop();
exit();

}
  }
  

void mouseClicked() {
  
  if((CurX<640) && (CurY<480)){
 Xc=v.x+(v.x/15);
 Yc=v.y+(v.y/15);
 Zc=v.z+(v.z*7/129);  
 
 Xr=-11.9+Xc;
 Yr=14-Yc;
 Zr=34-Zc;
 
 qs =   cinematicaInversa(Xr,Yr,Zr);
for (int i = 0; i < qs.length; i++) { // For each element,
    q[i] = int(round((qs[i] * 180/PI))); // divide the value by 2
  }
  if (qs[3]<90) qs[3]=qs[3]+360;
println("Angulos: " + q[0] + ", " + q[1] + ", "+ q[6] +", " + q[3] + ", " + q[4] + ", "+ q[5] );
}
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Only needed to make sense of the ouput depth values from the kinect
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

// Drawing the result vector to give each point its three-dimensional space
  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d)*100;
  result.y = (float)((y - cy_d) * depth * fy_d)*100;
  result.z = (float)(depth)*100;
  return result;
}

void serialEvent  (Serial myPort) {
recived_data  =  myPort.readStringUntil ( '\n' )  ;  // leer puerto serial
println(recived_data);
datos_rec = split(recived_data,',');
temp1=float(datos_rec[0]);
temp2=float(datos_rec[1]);
temp3=float(datos_rec[2]);
peso=float(datos_rec[3]);
recived = true;
} 

float[] cinematicaInversa(float X, float Y, float Z)
{
  X=X-(0);
  Y=Y-(0);
  Z=Z-(-20);
  float [][] R = {{-1,0,0},{0,1,0},{0,0,-1}};
  float r = sqrt(sq(X)+sq(Y))-O2;
  float s = Z-O1;
  float c = sq(s) + sq(r);
  float Dd=(c-sq(l1)-sq(l2))/(2*l1*l2);
  float[] res = new float[7]; // q1,q2,q3,q4,q5,q6,gamma
   res[0]=atan2(Y,X);
   //println(res[0]);
   res[2]=atan2(-sqrt(1-sq(Dd)),Dd);
   res[1]=atan2(s,r)-atan2(l2*sin(res[2]),l1+l2*cos(res[2]));
   res[4]=-atan2((sqrt(1-sq((R[0][2])*cos(res[0])*cos(res[1]+res[2])+(R[1][2])*sin(res[0])*cos(res[1]+res[2])+(R[2][2])*sin(res[1]+res[2])))), ((R[0][2])*cos(res[0])*cos(res[1]+res[2]) + (R[1][2])*sin(res[0])*cos(res[1]+res[2]) + (R[2][2])*sin(res[1]+res[2]) ));
   res[3]=atan2( (R[2][2])*cos(res[1]+res[2])-(R[0][2])*cos(res[0])*sin(res[1]+res[2])-(R[1][2])*sin(res[0])*sin(res[1]+res[2]), (sqrt(sq(sin(res[4]))-sq((R[2][2])*cos(res[1]+res[2])-(R[0][2])*cos(res[0])*sin(res[1]+res[2])-(R[1][2])*sin(res[0])*sin(res[1]+res[2])))));

   if(floor(abs(res[3]))==0){
   res[3]=-PI/2;
   }
   res[5]=atan2((R[0][1])*cos(res[0])*cos(res[1]+res[2])+(R[1][1])*cos(res[1]+res[2])*sin(res[0])+(R[2][1])*sin(res[1]+res[2]),abs(sqrt(sq(sin(res[4])) - sq((R[0][1])*cos(res[0])*cos(res[1]+res[2])+(R[1][1])*cos(res[1]+res[2])*sin(res[0])+(R[2][1])*sin(res[1]+res[2])))));
   res[6]=res[2]+res[1]; // Gamma, o Angulo absoluto del segundo eslabon respecto a la horizontal.    
   //println(X," ",Y," ",Z);
   for (int i = 0; i < qs.length; i++) { // For each element,
    q[i] = int(round((res[i] * 180/PI))); // divide the value by 2
  }
  println("Angulos: " + q[0] + ", " + q[1] + ", "+ q[2] +", " + q[3] + ", " + q[4] + ", "+ q[5]+ ", "+ q[6] );
    return res;
}
