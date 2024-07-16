Line Follower Robot Controlled by PID with 
Embedded Systems Software 
Ömer BOZKURT, Mahmut Hilmi KAYA 
Department of Electrical-Electronics Engineering, Abdullah Gül University 
Kayseri, Turkey 
Email : omer.bozkurt@agu.edu.tr, mahmuthilmi.kaya@agu.edu.tr   
Abstract— In this article, PID control, embedded software 
and improvements of Line-follower robot are discussed and 
explained. The line data was obtained using the reflected sensor. 
A PID control algorithm has been developed to process the data 
received from the sensor and extract a meaningful output to be 
converted into motion. The output from the algorithm is used in 
the control of dc motors.  After the hardware and software 
developments to be used in the robot were completed, the 
robot was designed. Stabilization settings were made in the 
algorithm by testing the robot's ability to perform straight lines, 
bends, and sharp turns over 90 degrees. As a result, the line 
follower robot has stably followed the line under different 
conditions. 
Keywords— DC-motor control, STM32F103C8, PID Control, 
Embedded Software, QTR8 
I. INTRODUCTION  
The aim of this project is to implement embedded system 
and feedback control courses in real life and to improve the 
skills of the students in these areas.  The name of the project 
is Line-Follower Robot. The robot should follow the line. 
The robot must follow the black line, distinguishing black 
and white colours. The working principle of the robot is that 
the data received from the optical sensor used for line 
tracking shows the position of the line. In order to understand 
the position of the line from the sensor data, directly 
proportional values are assigned to the data of eight different 
optical sensors on the sensor according to their distance from 
the midpoint. The number of sensors used is divided by the 
sensor data, and the line is provided to create different 
outputs at different points. This received position 
information creates a proportional error for the PID control 
algorithm. 
PID is a feedback control mechanism.  PID control 
algorithm calculate the error which is difference between 
desired error and current error in certain time. The aim of 
PID is reducing the error value at least minimum in certain 
time.  Before PID calculations the error should defined. The 
error is the difference between reference value and obtained 
value from sensor. Reference value is desired value which is 
determine where the line should be under of sensor.  It has 
importance for stability and getting accurate results in the 
designed system.  
PID control has three different 
terminologies. These are proportional, integral, and 
derivative. The effects of each on the system are different. 
A ready-made robot car kit was used in the design of the 
robot mechanically. This kit also includes two dc motors 
with gearbox and caster wheels. The rest of this article will 
discuss in detail how these dc motors are controlled. 
In the electronic part of the robot, a motor driver is used 
to drive dc motors. SMT32 processor is used to control the 
robot. In addition, battery and voltage booster are also used 
and the power needed by the robot is given.  
After the electronic mechanics and software were 
brought together, the tests of the robot were started, and the 
final stabilization settings were made. 
II. LINE FOLLOWER ROBOT 
A. Mechanic Part of Robot 
Fig 1. Line follower robot  
As can be seen in the figure 1, a ready-made robot kit is 
used in the mechanical part of the robot. This kit consists of 
two layers. In the bottom layer, the motors, the motor driver, 
the voltage amplifier, and the batteries that will power the 
motors are used. On the upper layer of the robot, there is a 
power bank to feed the processor, a processor, and a switch 
to turn the robot on and off.  Pertinax is placed at the front of 
the car to better adjust the distance and position of the 
sensor. 
The robot has three moving parts. These parts are the 
wheel to which the two large drive systems are connected 
and a caster wheel.  
As an improvement in the mechanical field, positioning 
the sensor at a distance from the robot has made line tracking 
of the robot easier. The protruding position of the wheels 
connected to the drive system facilitated the vehicle's turns. 
B. Electronic Part of Robot 
Fig 2. Line follower robot schematic 
Battery: The robot design has no electrical source which 
carry the charges with cable. So that, system has a battery. 
The robot has two different battery section, one of them is 
for supplying the motor driver and motors. Two lithium-ion 
rechargeable batteries. These are easy to implement on this 
design. For an improvement, MT3608 boost converter is 
used, because the battery voltage may be reduced, and the 
system is not able to get enough power for proper working. 
MT3608 has a potentiometer for setting the output voltage 
level and it is always constant 12V for supplying the motor 
driver. Also, the system has a power bank for supplying the 
STM32 because motor driver has not able to get always 
constant 5V. 
Motor Driver: L298N motor driver was used in the robot. 
The motor driver has been supplied constant 12V. It also has 
regulator circuit for supplying 5V constant. L298N is able to 
drive two dc or step motors at same time. It has two pins 
which are for enabling the PWM signals of motor.  In our 
design, Two PWM enable pins are connected to the 5V pins 
of motor driver. So that, motors can on input pins. The motor 
driver has 4 pins for directional control of motors. In the 
system we designed, the motors are required to rotate in one 
direction, so only two input pins are used for two motors. 
PWM signal is given to the input pins. In this way, the speed 
of the motors can be adjusted. In addition, this integrated 
works conveniently with the system in terms of fast and 
accurate response. 
QRT8 Reflectance Sensor: In the system, data from the 
environment is collected only through this sensor. The 
working principle of the sensor is as follows. The light is 
sent to the ground from the infrared LEDs in the sensor. The 
reflected light is detected again with the photodetector in the 
sensor. It is expected that the reflected light will change 
according to the colour of the surface it hits. There are eight 
different sensor groups working with this principle in the 
sensor. With eight different sensor groups, eight different 
values can be read from the sensor. These different values 
give information about the line to be followed. The 
capacitors are charged by first outputting the sensors from 
the processor. then the value is read from the sensor. The 
process continues in this way. 
The most important factor is the discharge time of the 
capacitors in different colours. This period gives us the 
colour information. 
Table 1 illustrated the discharge time of capacitor. According 
to the discharge times embedded system code was designed.  
Sensor Number 
White(ms) 
Black(ms) 
8 
1ms 
10.1ms 
7 
1ms 
9.8ms 
6 
1ms 
8 ms 
5 
1ms 
12.4 ms 
4 
1.8ms 
11 ms 
3 
1.2ms 
24.6 ms 
2 
2.2ms 
17 ms 
1 
2ms 
20.6ms 
Table 1. Discharge Time of Sensor’s Capacitors 
DC-motors : 6-12V DC motors are used in the system. 
These motors also have a gearbox to increase torque. In this 
way, the speed transmitted to the vehicle wheel will decrease 
and the torque will increase. 
STM32F103C8 : In this project, blue pill development 
board with stm32f103c8 processor was used. The functions 
of STM32 play the most important role in guiding the robot. 
Data is read from the sensor with STM32. After the data 
received from the sensor are interpreted, they are processed 
with the PID control algorithm. PWM signals are arranged 
with the value extracted by the PID control algorithm and 
sent to the motor driver. The biggest load in this system is 
done with STM32. All these processes are done with 
embedded system programming of STM32. The detailed 
work of the written algorithm and the system in the code 
direction will be explained in the next sections. 
III. 
PID CONTROLLER DESIGN 
The main structure of the created line follower robot is 
shown in figure 2. PID structure is used as the controller in 
the line follower robot. First, the values read from the sensor 
are taken. In the light of these data, the controller has 
obtained the location information about where the line is. 
Then the PID controller is fed to update the motor speed by 
calculating the inaccuracy data from the sensor data. 
Whenever the line follower robot receives an error, this data 
is updated on the PID controller, and the line follower 
vehicle is provided to follow the path. 
A. Calculation of Positional Error 
The positional error calculation is performed with the 
data read from the sensor. The data received from each 
sensor corresponds to a position information. While 
calculating the positional error, eight different reference 
values are assigned to each sensor. 
(1) 
E. PID Controller General Structure 
According to the error value read, if the error is negative, 
this indicates that the robot following the line tends to turn 
to the left. If the error is positive, it indicates that the robot 
deviates from the line to the right. Pursuant the error data 
read; the motor speeds of the robot are adjusted by feeding 
the PID controller. If the error is negative, the speed of the 
left motor is reduced so that the robot catches the path. The 
operating logic is the same in return configurations. When 
the robot is going to turn left, the speed of the left motor is 
reduced to make it turn. Otherwise, if the error is positive, 
the speed of the right motor is reduced so that it catches the 
line, or the robot turns to the right. 
B. Proportional Error Calculation 
The P term in PID controls stands for proportional. The 
proportional error control output is proportional to the error 
accumulated over time. Proportional control compares the 
desired or set point with the actual value or the feedback 
value. The result error is found by multiplying the 
proportional error with the determined Kp coefficient. The 
proportional controller has the effect of reducing the rise 
time and reducing the steady-state error, but not completely 
eliminating it. As the Kp coefficient is increased, the 
maximum overshoot is observed, and as the Kp coefficient 
decreases, the time to stabilize becomes longer. In this 
system, it has been used together with integral and 
derivative controllers with its effective features. The 
mathematical representation of the proportional error is 
given below.  
          
C. Integral Error Calculation 
(2) 
Steady-state error is always present due to the limited 
influence of the P controller. The I controller, on the other 
hand, provides the necessary action to eliminate the steady
state error. It integrates the error for a period of time until 
the error value reaches zero. Holds the value to the last 
controller where the error was zero. The integral controller 
has the effect of eliminating the steady state error, but the 
transient response effect is slower. The speed of the 
response is increased by decreasing the integral gain Ki. The 
equation below has a mathematical representation. 
       
(3) 
D. Derivative Error Calculation 
The I controller is not capable of predicting the future 
behavior of the fault, so it responds normally when the 
reference point is changed. Therefore, the future behavior of 
the error is predicted using the D controller. The output 
error is found by multiplying the read error value with the 
K𝑑 coefficient. Derivative control (𝑘𝑑) has the effect of 
increasing system stability, reducing overshoot and 
improving transient response. That's why it is used with 
proportional and integral controllers. Below is the 
mathematical representation of the D controller. 
 
(4) 
As seen in Equation 5, the general mathematical 
representation of the PID controller is specified. Maximum 
efficiency is achieved when P, I and D controllers work 
together. The proportional term is used to minimize the 
positional error, the integral term is used to eliminate the 
steady-state errors that will occur during line tracing, and 
the derivative term is used to minimize oscillation and 
improving the transient response. 
(5) 
F. PID Controller Coefficients Calculation 
In order to make the error calculations of the PID 
controller more precise and the robot to work stably, the 
controller coefficients have been calculated. Calculations 
were made by creating a closed loop system of PID 
controller and motor. Calculations were made with the 
measured data in Table 2. 
Parameter 
Value 
Back-EMF Constant - K 
Internal Resistance - R 
  [V*s/rad] 
6.5 Ω 
Internal Inductance – L  
Rotor moment of Inertia - J 
 [H] 
6.4846 * 
0.06 [kg*
 Friction - b 
] 
0.05 [N*m*s/rad] 
Table 2. Parameter Table for Motor System 
Fig 3. General System Representation of Motor 
After the measured values, reference points were 
established for the controller coefficients by determining the 
maximum overshoot, rise time and settling time in order for 
the system to respond faster and to reach stability earlier. 
Therefore, operations were performed by selecting the 
maximum overshoot below 1 percent, the set time under two 
seconds, and the rise time below 0.1 seconds.  
 
(6) 
(7) 
In the first part, the dynamic equations of the motor for 
the circuit and the rotor are derived. After these equations 
were obtained, they were converted to the frequency domain 
using the Laplace transform. Then, by combining these two 
equations, a system equation called Gm is obtained for the 
engine. 
 
(8) 
the control algorithm results duty cycle of PWM’s are 
 
 
(9) 
(10) 
(11) 
(12) 
(13)  
Then the system equation of the PID controller was 
extracted and combined with the Gm. After putting them 
together, the characteristic equation of the characteristic 
system was obtained from the system equation. The values 
obtained by using the maximum overshoot and rise time 
were put into the time response equation and the 
proportional, integral, and derivative coefficients of the 
controller were found. 
(14) 
(15) 
(16) 
While Kp, Ki and Kd values were found, the denominator 
of the Gchar system was equated to the time response 
equation and Kp and Kd values were found by not assuming 
the Ki value. Then, since the Ki value was desired to be 
small, the appropriate Ki value was selected by examining 
the step response graphics from the MATLAB application. 
Finally, these values were given to the general equation in 
order to get the output error, and the activity of the line 
follower robot was examined. 
IV. EMBEDDED SOFTWARE 
Clock Settings: STM32F103C8 has 8 MHz system clock. 
For more proper and easier implementation, the 8MHz clock 
should be divide. Therefore, PLL is used, and system clock 
divided by 9. Thus, 72MHz clock is ready to use. The 
importance of the using prescaler is that getting more 
accurate result in certain process which is using 
microseconds.  
Timers: TIM2 and TIM1 were used for different 
functions. TIM2 was used for implementing delay functions. 
There are two different delay functions in the system. 
Because the system needs delay functions in different time 
levels. One of delay function is in the microsecond level. It 
especially used in capacitor charging and discharging in 
QTR8 sensor.  
TIM1 is designed for generating PWM signals. PWM 
signals responses are controlling the motors. According to 
updating. TIM1 has different channels. So, the system is able 
generate two different PWM signals for using just one timer.  
Sensor Read 
PID Algorithm: PID control algorithm codes take part of 
main function of the project. Project codes are written in 
register level. The algorithm has calculated the Positional 
error, Proportional error, Integral error, Derivative error. 
Then calculated errors were used in the PID output 
calculation. Algorithm has also featured for determining in 
specific conditions such as when there is no line and when it 
coincides with angles sharper than a right angle. In this kind 
of situations, algorithm accepts previous error and multiplies 
by 12 instead of current position error. It is multiplied by 12 
because of the output should be saturated for finding line 
again.  
V. ANALYSIS 
Fig 4. Overall Structure of System 
The overall structure of the controller is shown in figure 
4. This structure consists of a PID controller and plant 
system. The specified plant system is the motors. The PID 
controller, which is fed with the obtained error value, 
adjusts the PWM signal and commands the motor for 
movements and turns. The general structure was created 
using the MATLAB Simulink program.  
Fig 5. Step Response of PID Controller 
Figure 5 shows the step response graph of the 
controller. As a result of the calculations, the maximum 
overshoot value was selected below 1%, the rise time was 
determined as 0.1 seconds, and finally, the calculations were 
made by targeting the settling time to be less than 2 seconds.  
been determined as the most suitable and stable value for the 
duty cycle robot. 
The total length of this test track is approximately 
475 cm. The designed robot completes the course in 7.9 
seconds when driven with an 85 percent duty cycle. The 
average speed of the car is 2.164 km per hour, which 
corresponds to a speed of approximately 0.60 meters per 
second. As a result, the designed car completes a difficult 
and long track very quickly and smoothly.  
VII. DISCUSSION 
Fig 6. PID Coefficients Behaviour Graph 
In another figure, it is observed how the calculated 
PID controller coefficients get closer to the reference signal 
by simulating. The x-axis shows the data points and the y
axis shows the behavior of the PID controller. Inferences 
were made by monitoring the behavior of the PID controller 
according to the reference signal. The values obtained after 
the hand calculations were simulated and experiments were 
carried out, and after the values were tested on the line 
following robot, it was noted that they met the desired 
specifications. Calculations were made without considering 
the PID controller system separately from the plant system, 
thus optimum values were determined in a short time. 
VI. RESULTS 
Fig 7. Test Runway 
The runway shown in the figure was taken from the 
internet for the test of the robot and prepared using black 
tape.  The robot, which was designed according to the 
results of the tests, is successfully stable in sharp bends and 
very sharp 135 degree turns. It travels in straight lines 
without wobble. These features show that the PID control 
algorithm works tremendously. In most of the tests, the duty 
cycle of the PWM signals was set to a maximum of 85 
percent. In cases where Duty Cycle is 100%, the error rate 
of the robot has increased slightly, and in some cases, it has 
progressed beyond the desired. Therefore, 85 percent has 
In this part of the article, the improvements and 
challenges of robot will be discussed. Many of 
improvements are related to the PID control cause that PID 
is the especially main part of the control system and 
algorithm. For coefficients of PID control, all of them has 
calculated by hands instead of trying random numbers. For 
calculations, correct measurements are very necessary such 
as EMF, inductance of motor and RPM value except inertia 
of rotor. These are measured practically.  Thus, PID control 
coefficient are calculated according to the measurement 
results.  
The robot has two wheels. While the robot is 
turning in the sharp bends, one of the wheels on the turning 
side stops. In this case the other wheel starts to spin at 
maximum speed, so one of the wheels starts to spin. For 
solving the problem, we have to increase the friction force 
between surface and wheels. That's why we put on two 
wheels. The problem was solved, and the robot started to go 
faster and more stable. 
VIII. CONCLUSION 
The purpose of the line follower robot created as a result 
of the project is to follow the path determined by the black 
strip by using the PID controller, which is the feedback 
mechanism. The line follower robot was created with 
mechanical and electronic approaches. The reflection sensor 
is integrated into the circuit for positional error calculation.  
Positional error feeds the PID by entering the PID 
controller algorithm used for the vehicle to move more stable 
on the line. The coefficient factors of the PID algorithm were 
calculated as a result of the measurements made from the 
plant system. The coefficients found are multiplied by their 
error values and then summed together to obtain a general 
error output data. Motor control is provided with this error 
output data. The line follower vehicle was tested on the 
designated track and improvements were made as a result of 
the deficiencies observed.  
As a result of the improvements, the line-following robot 
quickly completes sharp turns and wide-angle places 
comfortably and quickly. In conclusion, the line following 
vehicle with the PID algorithm created completes the road by 
performing at a high level. 
The codes and project files are shared via link below. 
https://drive.google.com/drive/folders/1AwHNO8b20JGeos4aQeDt6TS7Zk
 XT3NuW?usp=sharing 
 
// EMBEDDED SYSTEM AND FEEDBACK CONTROL CAPSULE 
// LINE-FOLLOWER ROBOT PROJECT 
/*AUTHORS : 
ÖMER BOZKURT 
MAHMUT HILMI KAYA 
  
 /////////ALL CODES WERE WRITTEN BY THE ABOVE NAMED AUTHORS////////// 
         
*/ 
#include "stm32f10x.h" 
#include <stdlib.h> 
#include <stdbool.h> 
#include <stdio.h> 
#include <string.h> 
 
void GPIO_clock_init(void); 
void PWM_generator(void); 
void TIM2config(void); 
int QTR_position_read(int *sensor); 
void delay_us(uint16_t us);  
void delay_ms(uint16_t ms); 
 
int error_vector[8] = { -20, -15, -10, -5, 5, 10, 15, 20 }; 
int proportionalerror = 0; 
double KP = 3.3989; 
double KD = 6.0484; 
double KI = 0.00015; 
 
#define MaximumPWM 85 
 
void PLLClockConfiguration() 
{ 
  if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */ 
  { 
   RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW); /* (2) */ 
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) /* (3) */ 
   { 
   /* For robust implementation, add here time-out management */ 
   } 
  } 
  RCC->CR &= (uint32_t)(~RCC_CR_PLLON);/* (4) */ 
  while((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */ 
  { 
   /* For robust implementation, add here time-out management */ 
  } 
   
  RCC->CFGR &=  ~(RCC_CFGR_PLLSRC) | RCC_CFGR_PLLSRC_HSI_Div2;  // PLL 
source is HSI/2 
   
   
  RCC->CFGR = RCC->CFGR & (RCC_CFGR_PLLMULL_2) | (RCC_CFGR_PLLMULL_1)| 
(RCC_CFGR_PLLMULL); /* (6)) */ 
 
  RCC->CR |= RCC_CR_PLLON; /* (7) */ 
  while((RCC->CR & RCC_CR_PLLRDY) == 0) /* (8) */ 
  { 
   /* For robust implementation, add here time-out management */ 
  } 
  RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); /* (9) */ 
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (10) */ 
  { 
   /* For robust implementation, add here time-out management */ 
  } 
} 
void TIM2config(void) { 
//enable timer clk/ 
 RCC->APB1ENR |= (1 << 0); 
 //ARR value/ 
 TIM2->ARR = 0xffff - 1; 
 //*SET UP PRESCALER */ 
 TIM2->PSC = 72 - 1; //--us/ 
 //enable timer/ 
 TIM2->CR1 |= (1 << 0); 
 //Enable update flag/ 
 while (!(TIM2->SR & (1 << 0))) 
  ; //wait to set/ 
} 
 
void delay_us(uint16_t us) { 
 TIM2->CNT = 0; //set counter to 0/ 
 while (TIM2->CNT < us) 
  ; 
 
} 
 
void delay_ms(uint16_t ms) { // each clock cycle is 1us therefore counter increase 1 for 
each 1us. 
 for (uint16_t i = 0; i < ms; i++) { 
  delay_us(1000); 
 } 
} 
void GPIO_clock_init(void) { 
 
 RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // enable A port clk  
 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //enable B port clk 
 
 
 GPIOB->CRL = 0x33333333; //IT IS KEPT ON TO RECEIVE HIGHVOLTAGE FROM ANY B-PIN. 
 
} 
 
int QTR_position_read(int *sensor) { 
 
 GPIOA->CRL |= 0x33333333; //All low pins are set output mode 50MHz, output push-pull 
 GPIOA->ODR |= 0xFFFFFFFF; // all pins set 1  
 delay_us(12); //wait 12us for charging the capacitors of sensors 
 
 GPIOA -> CRL &= 0x00000000; 
 GPIOA -> CRL |= 0x88888888; //All loow pins are set input pull-up/pull-down 
 delay_ms(6); // Capacitor voltages are read for 6 ms  
 
 int numberofSensors = 0; 
 int position = 0; 
 
 for (int i = 0; i < 8; i++) { 
 
  if ((GPIOA->IDR >> i) & 1) { 
   sensor[i] = 1; 
   position += error_vector[i]; 
  } else { 
   sensor[i] = 0; 
  } 
  numberofSensors = numberofSensors + sensor[i]; 
 } 
 return position / numberofSensors; 
} 
 
void PWM_generator(void) { 
 // for generationg PWM signals TIM1 was used  
 // TIM1 has more channels for getting PWM 
 // channel 1 and channel 4 was used 
 // default duty cycles were entered as 50% 
 // channel 1 output is A8 and channel4 output is A11 pins 
 RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN; 
 
 GPIOA->CRH |= GPIO_CRL_MODE0; 
 GPIOA->CRH |= (GPIO_CRL_CNF0_1); 
 GPIOA->CRH &= ~(GPIO_CRL_CNF0_0); 
 GPIOA->CRH |= GPIO_CRL_MODE3; 
 GPIOA->CRH |= (GPIO_CRL_CNF3_1); 
 GPIOA->CRH &= ~(GPIO_CRL_CNF3_0); 
 TIM1->PSC = 710; 
 TIM1->ARR = 100; 
 TIM1->CR1 |= TIM_CR1_CEN; 
 TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E; 
 TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
 TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; 
 TIM1->BDTR |= (1 << 15); 
 AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP; 
 TIM1->CCR1 = 50; 
 TIM1->CCR4 = 50; 
} 
 
void initClockHSI (void) 
{ 
    RCC->CR|= RCC_CR_HSION ; // enable internal HSI(RC) 
 
    while((RCC->CR & RCC_CR_HSIRDY)!=RCC_CR_HSIRDY);  // wait HSI to be ready 
 
    RCC->CFGR &= ~(RCC_CFGR_SW); //clear SW bits 
    RCC->CFGR |= RCC_CFGR_SW_HSI; //set HSI as system clock 
 
    while( (RCC-> CFGR & RCC_CFGR_SWS_HSI)!= RCC_CFGR_SWS_HSI); // wait HSI to be system 
clock 
} 
 
 
int main(void) { 
   
 PWM_generator(); 
 GPIO_clock_init(); 
 TIM2config(); 
 PLLClockConfiguration(); 
  
 int32_t currentPositionError = 0; 
 int32_t previousPositionError = 0; 
 int32_t proportionalPosition = 0; 
 int32_t derivativePosition = 0; 
 int64_t integralPosition = 0; 
 int32_t output = 0; 
 
 while (1) { 
   
  int sensor[8]; 
  int positionError = QTR_position_read(sensor); 
 
  bool empty = true; // In cases where no position information is read from the 
sensors, 
//  the pwm value is increased by taking the previous error value. 
  for (int i = 0; i < 8; i++) { 
   if (sensor[i] == 1) { 
    empty = false; 
    break; 
   } 
  } 
 
  if (empty) { 
   currentPositionError = previousPositionError * 12;  
   //The previous error value was multiplied by 12 and equalized to the 
current error value. 
      //The purpose of this process is to ensure that the robot goes at maximum speed in 
the direction  
   //of the last line it sees when position information is not received. 
    
    
   //Since the current position value will be multiplied by 12 in each 
loop, 
   //the current postion error is limited to 1500 at this stage. 
    
   if (currentPositionError < -1500) 
    currentPositionError = -1500; 
   else if (currentPositionError > 1500) 
    currentPositionError = 1500; 
  } else { 
   currentPositionError = positionError; 
  } 
 
  proportionalPosition = currentPositionError; 
  derivativePosition = currentPositionError - previousPositionError; 
  integralPosition += currentPositionError; 
  previousPositionError = currentPositionError; 
  output = proportionalPosition * KP + derivativePosition * KD + 
integralPosition * KI; 
 
  if (output > MaximumPWM) 
   output = MaximumPWM; 
  else if (output < -MaximumPWM) 
   output = -MaximumPWM; 
 
  if (output < 0) { 
   TIM1->CCR1 = MaximumPWM + output; 
   TIM1->CCR4 = MaximumPWM; 
  } else { 
   TIM1->CCR1 = MaximumPWM; 
   TIM1->CCR4 = MaximumPWM - output; 
  } 
 
 } 
} 
REFERENCES 
[1] M. Engin and D. Engin, "Path planning of line follower robot," 2012 
5th European DSP Education and Research Conference (EDERC), 
Amsterdam, Netherlands, 2012, pp. 1-5, doi: 
10.1109/EDERC.2012.6532213. 
[2] M. V. Gomes, L. A. Bássora, O. Morandin and K. C. T. Vivaldini, 
"PID control applied on a line-follower AGV using a RGB camera," 
2016 IEEE 19th International Conference on Intelligent 
Transportation Systems (ITSC), Rio de Janeiro, Brazil, 2016, pp. 194
198, doi: 10.1109/ITSC.2016.7795553. 
[3] E. H. Binugroho, D. Pratama, A. Z. R. Syahputra and D. 
Pramadihanto, "Control for balancing line follower robot using 
discrete cascaded PID algorithm on ADROIT V1 education robot," 
2015 International Electronics Symposium (IES), Surabaya, 
Indonesia, 2015, pp. 245-250, doi: 
10.1109/ELECSYM.2015.7380849. 
[4] Rantung, J., & Luntungan, H. (2020). DC MOTOR PID 
CONTROLLER WITH PWM FEEDBACK. Jurnal Tekno Mesin, 
6(1). 
[5] Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2020). Feedback 
control of Dynamic Systems. Pearson Education Limited.  
 
