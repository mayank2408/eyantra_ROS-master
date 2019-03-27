/*
* Team Id:eYRC#3848
* Author List:Hardik Aggarwal,Mayank Gupta,Aarush Gupta
* Filename: MARTIAN
* Theme: Explorer Bot
* Functions: velocity_right,velocity_left,MotorA_enc_isr,MotorB_enc_isr,messageCb,pid
* Global Variables: left_out,right_out,left_in,right_in,left_inp,right_inp,speed_wish_right,speed_wish_left,DistancePerCount,consKi,consKp,consKd,last_time1,
current_time1,last_time2,current_time2,tick_x, tick_y,v_left,v_right, deltaLeft,deltaRight,iteration_time,last_iteration,enc_countA,enc_countB, error_prior,
integral,derivative,output,time1,left_time,yo,t,p,time123
*/ 
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

double left_out,right_out=0;
double left_in,right_in,left_inp,right_inp=0;
double speed_wish_right=0, speed_wish_left=0;
double DistancePerCount = (0.204035) / 1680;
double consKp=5.15, consKi=0.1, consKd=0.008;
double last_time1,current_time1,last_time2,current_time2=millis();
double tick_x, tick_y=0;
double v_left,v_right=0;
double deltaLeft,deltaRight=0,iteration_time=0,last_iteration=0;
volatile long enc_countA = 0,enc_countB = 0;
double error_prior=0.0, integral=0, derivative=0.0, output=0.0;  
double time1;
double left_time=millis();
double yo=0;
int t=0;
int p=0;
double time123;

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define OUTPUT_READABLE_QUATERNION

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


std_msgs::Int32 enc_msg1;
std_msgs::Int32 enc_msg2;
std_msgs::Float32 enc_msg3;

ros::Publisher encoder_L("left_enc", &enc_msg1);
ros::Publisher encoder_R("right_enc", &enc_msg2);
ros::Publisher gyro("yaw", &enc_msg3);
ros::NodeHandle nh;
/*
* Class Name:motor
* Input:pin numbers through which motor is connected to arduino,pwm value to be set of individual motor 
* Output: sets the desired pwm
* Logic: we wanted to make the code generic and easy for debugging by changing the connection of motor by making a object for motor  
* Example Call: if we want to set the motor connection to pins 25,24 and pwm of that motor to pint 7,then we can
 invoking this class as motor(25,24,7)
*/
class motor
{
  int m1;
  int m2;
  int pwm;
  public:
  motor(int x,int y,int z)
  {
    m1=x;
    m2=y;
    pwm=z;
  }
  void setpwm(int k)     //this method sets the pwm of the respective motor
  {analogWrite(pwm,k);}
    
  void forward()       //this method makes motor to rotate forward
  {
    digitalWrite(m1,HIGH);
    digitalWrite(m2,LOW);
  }
  void brake()        // this method makes motor to stop rotating
  {
    digitalWrite(m1,LOW);
    digitalWrite(m2,LOW);
  }
  void backward()    //this method makes motor to rotate backward
  {
    digitalWrite(m1,LOW);
    digitalWrite(m2,HIGH);
  }
};

motor motor_l(22,23,4); //we create object for left motor by providing pin numbers for motor and pwm 
motor motor_r(24,25,6); //we create object for right motor by providing pin numbers for motor and pwm 
/*
* Function Name:velocity_right
* Input: none
* Output: returns the computed velocity of right wheel
* Logic: we calculate velocity by using difference from encoder ticks and difference in time from previous iteration 
* Example Call: a=velocity_right() gives the velocity of right wheel
*/
double velocity_right()
{
  current_time1 = millis();
  if ((current_time1 - last_time1)==0) 
  {v_left=0;}
  else
  {
    deltaLeft = enc_countA - tick_x;
    v_left = ((deltaLeft * DistancePerCount) / (current_time1 - last_time1))*1000;
  }
  tick_x=enc_countA;
  last_time1=current_time1;
  
  return v_left ;
}

/*
* Function Name:velocity_left
* Input: none
* Output: returns the computed velocity of left wheel
* Logic: we calculate velocity by using difference from encoder ticks and difference in time from previous iteration 
* Example Call: a=velocity_left() gives the velocity of the left wheel
*/
double velocity_left(){
  current_time2 = millis();
  if ((current_time2 - last_time2)==0) 
  {v_right=0;}
  else
  {
    deltaRight = enc_countB - tick_y;
    v_right = ((deltaRight * DistancePerCount) / (current_time2 - last_time2))*1000;
  }
  tick_y=enc_countB;
  last_time2=current_time2;
  
  return v_right ;
}
/*
* Function Name:MotorA_enc_isr
* Input: none
* Output: it does not return anything just increments or decrements the ticks according to the movement of wheel
* Logic:we created lookup_table for the cases whether wheel is rotating forward or backward and increment or decrement values of ticks accordingly 
* Example Call: MotorA_enc_isr() increments or decrements the value of enc_countA
*/
void MotorA_enc_isr() 
{
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valA= 0;  
  enc_valA = enc_valA << 2;
  uint8_t tw=digitalRead(19);
  uint8_t thr=digitalRead(18)<<1;
  uint8_t sample= (tw | thr);
  enc_valA = enc_valA | sample;
  enc_countA = enc_countA + lookup_table[enc_valA & 0b1111];
}
 /*
* Function Name:MotorA_enc_isr
* Input: none
* Output: it does not return anything just increments or decrements the ticks according to the movement of wheel
* Logic:we created lookup_table for the cases whether wheel is rotating forward or backward and increment or decrement values of ticks accordingly 
* Example Call: MotorA_enc_isr() increments or decrements the value of enc_countB
*/   
void MotorB_enc_isr() 
{
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valB= 0;  
  enc_valB = enc_valB << 2;
  uint8_t fr=digitalRead(2);
  uint8_t fv=digitalRead(3)<<1;
  uint8_t sample= (fr | fv);
  enc_valB = enc_valB | sample;
  enc_countB = enc_countB + lookup_table[enc_valB & 0b1111];
}
/*
 * Function Name:pid
 * Input: val,setpoint
 * Output: it returns the required velocity
 * Logic:we use the pid control system to calculate the required velocity using the difference between the computed velocity and input velocity
 * Example Call: a=pid(0.21,0.22) puts the value of required velocity in a
 */
double pid(double val, double setpoint)
{ 
  float error=0;
  iteration_time=millis();
  time1=(iteration_time-last_iteration)/1000;
  error=setpoint-val;
  integral = integral + error*time1;
  if(time1==0)
  {derivative=0;}
  else
  {derivative = (error-error_prior)/time1;}
  output = (consKp*error) +(consKi*integral) +(consKd*derivative)+val;
  error_prior = error;
  last_iteration=iteration_time;
}
/*
 * Function Name:messageeCb
 * Input: cmd_vel
 * Output:it converts angular and linear velocity into velocity of respective wheels
 * Logic: we are using differential drive model for moving our robot so we have to convert angular and linear velocity to velocity of individual wheels
 * Example Call: messageCb(cmd_vel)
 */
void messageCb( const geometry_msgs::Twist& cmd)
{
  float lin=cmd.linear.x;
  float ang=cmd.angular.z;
  speed_wish_right = (2*lin+ang*0.19)/2;
  speed_wish_left = ((2*lin-ang*0.19)/2)*1.005;
    if (lin==0 and (ang<0.6 and ang>-0.6)){      // checks for angular speed because speed_wish_right and speed_wish_left less than this does not map to correct pwm and robot does not rotate  
    speed_wish_right=speed_wish_right*1.7;
    speed_wish_left=speed_wish_left*1.7;}
 }

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb ); //subscribes to twist message coming from move_base

void setup()
{
  attachInterrupt(digitalPinToInterrupt(19),MotorA_enc_isr,CHANGE); //interrupts used for encoders
  attachInterrupt(digitalPinToInterrupt(18),MotorA_enc_isr,CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),MotorB_enc_isr,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),MotorB_enc_isr,CHANGE);
  pinMode(6, OUTPUT);
  pinMode(4, OUTPUT);
  nh.initNode();
  nh.advertise(encoder_L);
  nh.advertise(encoder_R);
  nh.advertise(gyro);
  nh.subscribe(sub);

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(57600);
    while (!Serial); 

    // initialize device
    mpu.initialize();
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(67);         
    mpu.setYGyroOffset(9);
    mpu.setZGyroOffset(19);
    mpu.setZAccelOffset(1461);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
  
  }

void loop()
{

  
    if (t==0) {
      left_time=millis();}
    if (!dmpReady) return;  // wait for MPU interrupt or extra packet(s) available
    while (fifoCount < packetSize) {
      if (left_time>15000)  // wait for mpu6050 for 15 sec at start because of garbage values at starting
      {
      static double temp=ypr[0];  //store the garbage vlue at the end of 15 seconds 
      yo=ypr[0] - temp;           //minus the garbage values or offset from future values
      double x2=2466.026,x1=-311.658,x0=27.589;  //coefficients for calculating the pwm for the given required velocity;these were obtained by using polyval function in octave
      left_in = velocity_left(); //computes the current velocity of left wheel
      right_in= velocity_right();//computes the current velocity of left wheel

      enc_msg3.data = -yo; //equating the yaw values from MPU6050 to the enc_msg3.data for publishing,minus because gyro placed on robot in reverse direction
      gyro.publish( &enc_msg3);//publishing yaw
      enc_msg1.data = enc_countB;
      encoder_L.publish( &enc_msg1 );//publishing encoder data
      enc_msg2.data =enc_countA;
      encoder_R.publish( &enc_msg2 );//publishing encoder data

      left_out=abs(pid(left_in,speed_wish_left));//computes the required velocity for left wheel 
      right_out=abs(pid(right_in,speed_wish_right));//computes the required velocity for right wheel 
      int pwm_l=(pow(left_out,2)*x2)+(left_out*x1)+x0; //calculating pwm using required velocity
      int pwm_r=(pow(right_out,2)*x2)+(right_out*x1)+x0;////calculating pwm using required velocity
      if (left_out<0.01)pwm_l=0;       //checks for minimum and maximum value of the pwm
      if (right_out<0.01)pwm_r=0;
      if(pwm_l>255)pwm_l=255;
      if(pwm_r>255)pwm_r=255;
      if (speed_wish_right<0)motor_r.backward(); //if required velocity -ve then rotate backward
      else motor_r.forward();
      if (speed_wish_left<0)motor_l.backward();
      else motor_l.forward();
      motor_l.setpwm(pwm_l);             //sets pwm for wheels respectively
      motor_r.setpwm(pwm_r);
      t=1;
      nh.spinOnce();
      delay(10);
  }
        nh.spinOnce();
        fifoCount = mpu.getFIFOCount();
    }
    
  
    // check for overflow 
    if ( fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else  {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;



       #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif 
    }
    
}
