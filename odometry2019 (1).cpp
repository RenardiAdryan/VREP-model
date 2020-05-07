



int posX,posY,posT;
int vx,vy,vt;
float posx,posy,post;

void odometry(){

const float dt  	= 0.001;	// 1ms sampling
const float PHI 	= 3.1415926535897932384626433832795028841971693993751058209749445923078164062;
const float PPR 	= 1440;		//quadrature
const float Dwheel  = 4.8f;     //cm
const float L 		= 13.5;     //cm


float rotary1_tick = (PHI * Dwheel)/PPR;    //Notation = cm/tick
float rotary2_tick = (PHI * Dwheel)/PPR;    //Notation = cm/tick
float rotary3_tick = (PHI * Dwheel)/PPR;    //Notation = cm/tick


float rotary1_getTick = 32767-readEncoder1(1);
float rotary2_getTick = 32767-readEncoder2(1);
float rotary3_getTick = 32767-readEncoder3(1);


TIM3->CNT = 32767;//RESET Encoder
TIM1->CNT = 32767;//RESET Encoder
TIM4->CNT = 32767;//RESET Encoder

float distance1 = rotary1_getTick * rotary1_tick; 
float distance2 = rotary2_getTick * rotary2_tick;
float distance3 = rotary3_getTick * rotary3_tick;

float V1  = distance1 / dt;	//Notation CM/s
float V2  = distance2 / dt;	//Notation CM/s
float V3  = distance3 / dt;	//Notation CM/s



// ===========LOCAL FRAME=== FORWARD KINEMATIC ======= AL-Khawarizmi====
float VX = V3 - V1*0.5 - V2*0.5;
float VY = V1*0.8666 - V2*0.8666;
float VT = V1/L + V2/L + V3/L;
	
//float VX = -V1 -V2;
//float VY = 0.576967*V1 -0.576967*V2; 


float th = VT * dt;
float dataIMU = dataYaw;
//============ WORLD FRAME ==========//

double delta_x = (VX * cos(dataIMU/57.2958) - VY * sin(dataIMU/57.2958)) * dt;		//Notation CM
double delta_y = (VX * sin(dataIMU/57.2958) + VY * cos(dataIMU/57.2958)) * dt;		//Notation CM
double delta_t =  VT * dt;

posx+=delta_x;
posy+=delta_y;


vx = delta_x / dt;
vy = delta_y / dt;
vt = delta_t / dt;

posX = posx;
posY = posy;
posT = dataIMU;


}