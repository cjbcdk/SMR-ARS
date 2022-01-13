/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#define increment 10000

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;

// Arrays to save reading data
double visionpar[10];
double laserpar[10];
double datalog[increment][3], odolog[increment][4];


void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

void line_calib(int *p);
double findline(void);
double line_cm(int color);

componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.06522	/* m */
#define WHEEL_SEPARATION 0.26	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	8000 //24902
#define sample_time 0.01
#define MAX_ACC 0.5*sample_time


typedef struct{ //input signals
		int left_enc, right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr, cl;   // meters per encodertick
	        
    //output signals
		double right_pos, left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		// Update odo varibles
		double dU, dtheta, dUr, dUl;
		// x, y and theta		
		double x, y, theta;
		} odotype;


void reset_odo(odotype *p);
void update_odo(odotype *p);




/********************************************
* Motion control
*/

typedef struct{//input
    int cmd;
		int curcmd;
		double speedcmd;
		double dist;
		double angle;
		double left_pos,right_pos;
		// parameters
		double w;
		//output
		double motorspeed_l, motorspeed_r; 
		int finished;
		// internal variables
		double startpos;
		double startangle;

	} motiontype;
	       
enum {mot_stop=1,mot_move,mot_follow,mot_turn, mot_follow_L, mot_follow_R};


// Function prototypes
void update_motcon(motiontype *p);	       

int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int follow(double dist, double speed, int time, int heading);



typedef struct{
    int state, oldstate;
		int time;
	      } smtype;

void sm_update(smtype *p);
// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

// Sensor calibrated values
double calib_linesensor[8];
double calib_offset[8] = {43,45,44,43,44,44,45,44};
double calib_coefficient[8] = {0.0417,0.05,0.04,0.04,0.0286,0.0323,0.04,0.05};

enum {ms_init,ms_fwd,ms_turn,ms_follow,ms_end};

int main()
{
  int running,n=0,arg,time=0,j=0;
  double dist=0,angle=0;

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
           
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
     // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

  }   
 
   
   
   
  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

  }   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w=0.256;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
  running=1; 
  mission.state=ms_init;
  mission.oldstate=-1;



  // Start of the program (Main loop)
  while (running) { 
    if (lmssrv.config && lmssrv.status && lmssrv.connected){
      while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
        xml_proca(xmllaser);
    }
        
    if (camsrv.config && camsrv.status && camsrv.connected){
      while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
        xml_proc(xmldata);
    }
        

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);


    // Save readings in the arrays. This  will be used later to log a data in an output life.
    datalog[j][0] = mission.time;
    datalog[j][1] = mot.motorspeed_l;
    datalog[j][2] = mot.motorspeed_r;

    odolog[j][0] = mission.time;
    odolog[j][1] = odo.x;
    odolog[j][2] = odo.y;
    odolog[j][3] = odo.theta;

    j++; // Increment after every run


    /****************************************
    / mission statemachine   
    */
    sm_update(&mission);
    switch (mission.state) {
      case ms_init:
        n = 4; dist = 1; angle = 90.0/180*M_PI;
        mission.state= ms_follow;
        break;

      case ms_fwd:
        if (fwd(dist,0.3,mission.time))  mission.state=ms_turn;
        break;

      case ms_turn:
        if (turn(angle,0.3,mission.time)){
          n=n-1;
        if (n==0) 
          mission.state=ms_end;
        else
          mission.state=ms_fwd;
        }
        break;    
      

      // First stage (distance measurement)
      case ms_follow:
        if (follow(dist,0.3, mission.time, 0)){
          mission.state=ms_end;
        }
        break;
    
      case ms_end:
        mot.cmd=mot_stop;
        running=0;
        break;
    }  
    /*  end of mission  */


    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot);
    speedl->data[0] = 100*mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100*mot.motorspeed_r;
    speedr->updated=1;


    if (time  % 100 == 0) time++;
    /* stop if keyboard is activated
    *
    */
    ioctl(0, FIONREAD, &arg);
    if (arg!=0) running=0;

  } /* end of main control loop */


  // After finishing main loop, data is logged to an output file.
  FILE *out;
  out=fopen("datalog.dat","w");
  // Check for errors
  if(out== NULL){
    fprintf(stderr, "error in opening .txt");
    exit(EXIT_FAILURE);
  }

  for(int i=0; i<j; i++){
    fprintf(out, "%.2f %.3f %.3f \n", datalog[i][0], datalog[i][1], datalog[i][2]);
  }
  fclose(out);

  // Trying with the same pointer to file
  // Opening second file
  out = fopen("odolog.dat","w");
  // Check for errors
  if(out == NULL){
    fprintf(stderr, "error in opening .txt");
    exit(EXIT_FAILURE);
  }
  for(int i=0; i<j; i++){
    fprintf(out, "%.2f %.3f %.3f %.3f \n", odolog[i][0], odolog[i][1], odolog[i][2], odolog[i][3]);
  }
  fclose(out);

  // Save speed data
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);

}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

// only used once to set initial values.
void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->dUr = 0.000;
  p->dUl = 0.000;
  p->dU = (p->dUr + p->dUl)/2;
  p->dtheta = (p->dUr - p->dUl)/WHEEL_SEPARATION;
  p->x = p->y = p->theta = 0.0;
}


void update_odo(odotype *p)
{
  int delta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  p->dUr = delta*p->cr;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  p->dUl= delta*p->cl;
   
  p->dU = (p->dUr + p->dUl)/2;
  p->dtheta = (p->dUr - p->dUl)/WHEEL_SEPARATION;
  p->theta += p->dtheta;
  p->x += p->dU*cos(p->theta);
  p->y += p->dU*sin(p->theta);
  
}


void update_motcon(motiontype *p) { 

  if (p->cmd != 0) {

  p->finished = 0;

  switch (p->cmd) {

    case mot_stop:
      p->curcmd=mot_stop;
      break;

    case mot_move:
      p->startpos = (p->left_pos + p->right_pos)/2;
      p->curcmd = mot_move;
      break;
       
    case mot_turn:
	    //p->startangle = fabs(odo.theta);
	    p->startangle = (odo.theta);
      if (p->angle > 0) 
	      p->startpos=p->right_pos;
	    else
	      p->startpos=p->left_pos;
        p->curcmd=mot_turn;
      break;

    // Line follow statements
    case mot_follow:
	    p->startpos=(p->left_pos+p->right_pos)/2;
      p->curcmd=mot_follow;
      break;

    case mot_follow_R:
	    p->startpos=(p->left_pos+p->right_pos)/2;
      p->curcmd=mot_follow_R;
      break;

    case mot_follow_L:
	    p->startpos=(p->left_pos+p->right_pos)/2;
      p->curcmd=mot_follow_L;
      break;
  }
     
  p->cmd = 0;
  }

  double v_max = sqrt(2*MAX_ACC/sample_time*(p->dist-((p->right_pos+p->left_pos)/2- p->startpos)));
  //double v_max_turn = sqrt(2*MAX_ACC/sample_time*(fabs(p->angle)*p->w - (fabs(odo.theta)-p->startangle)*p->w));
  double k = 0.5, des_speed, k_ls = 0.15, ind;
  double dV = k*(p->angle+p->startangle - odo.theta);
   
  switch (p->curcmd) {

    case mot_stop:
      p->motorspeed_l=0;
      p->motorspeed_r=0;
      break;

    case mot_move:
      if(v_max < p->speedcmd) {
        p->speedcmd = v_max;
      }

      if ((p->right_pos+p->left_pos)/2 - p->startpos >= p->dist) {
        p->finished=1;
        p->motorspeed_l=0;
        p->motorspeed_r=0;
      }

      if((p->motorspeed_l + MAX_ACC) < p->speedcmd) {
        p->motorspeed_l += MAX_ACC;
      }
      else {
        p->motorspeed_l = p->speedcmd;
      }

      if((p->motorspeed_r + MAX_ACC) < p->speedcmd) {
        p->motorspeed_r += MAX_ACC;
      }
      else {
        p->motorspeed_r = p->speedcmd;
      }
      break;
    

    //Follow line in the middle 
    case mot_follow:
      line_calib(linesensor->data);
      ind = line_cm(0);
      double dV_ls = k_ls*(ind-3.5);
      p->motorspeed_l = p->speedcmd - dV_ls;
      p->motorspeed_r = p->speedcmd + dV_ls;

      if((p->right_pos+p->left_pos) / 2- p->startpos >= p->dist){
        p->motorspeed_r=0;
        p->motorspeed_l=0;
        p->finished=1;
      }
      break;

    // Follow line to the right
    case mot_follow_R:
      line_calib(linesensor->data);
      // ind = line_cm(0); // 0 is to follow the black line 
      ind = findline();
      double dV_lsr = k_ls*(ind - 1); 
      p->motorspeed_l = p->speedcmd - dV_lsr;
      p->motorspeed_r = p->speedcmd + dV_lsr;
      
      if((p->right_pos+p->left_pos)/2- p->startpos >= p->dist) {
        p->motorspeed_r=0;
        p->motorspeed_l=0;
        p->finished=1;
	    }
      break;
     

    case mot_follow_L:
      line_calib(linesensor->data);
      ind = line_cm(0); // 0 is to follow the black line 
      double dV_lsl = k_ls*(ind - 6); 
      p->motorspeed_l = p->speedcmd - dV_lsl;
      p->motorspeed_r = p->speedcmd + dV_lsl;
      
      if((p->right_pos+p->left_pos)/2- p->startpos >= p->dist){
        p->motorspeed_r=0;
        p->motorspeed_l=0;
        p->finished=1;
	    }
      break;
    

    case mot_turn:
      if(fabs(dV) < k/100) {
        p->motorspeed_r=0;
        p->motorspeed_l=0;
              p->finished=1;
      }
      else {
        des_speed = dV + dV/fabs(dV)*k/10;

        if(p->speedcmd < fabs(dV)) {
          des_speed = (dV/fabs(dV))*p->speedcmd;
        }
        p->motorspeed_l = -des_speed;
        p->motorspeed_r = des_speed;
      }
    break;
   }   
}


int fwd(double dist, double speed,int time){
  if (time==0){ 
    mot.cmd=mot_move;
    mot.speedcmd=speed;
    mot.dist=dist;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed,int time){
  if (time==0){ 
    mot.cmd=mot_turn;
    mot.speedcmd=speed;
    mot.angle=angle;
    return 0;
  }
  else
    return mot.finished;
}


// Line follower 
int follow(double dist, double speed, int time,int heading){
	if (time==0){

    if (heading==1) {
      mot.cmd=mot_follow;
    }
    else if (heading==2) {
      mot.cmd=mot_follow_R;
    }
    else {
      mot.cmd=mot_follow_L;
    }
		// past mot.cmd
		mot.speedcmd=speed;
		mot.dist=dist;
		return 0;
	} else {
		return mot.finished;
	}

}

void sm_update(smtype *p) {
  if (p->state != p->oldstate){
    p->time = 0;
    p->oldstate = p->state;
  }
  else {
    p->time++;
  }
}

// Line sensor calibration
void line_calib(int *p){
  for(int i=0; i<8; i++){
    //calib_linesensor[i] = (p[i]-calib_offset[i])*calib_coefficient[i];
    calib_linesensor[i] = p[i]/128.0;
  }
}

// Finding line with minimum intensity algorithm
double findline(void){
  int min = 0;
  for(int i=1;i<7;i++){
    if(calib_linesensor[i] < calib_linesensor[min]) min = i;
  }
  return (double) min;
}

//Center of mass algorithm
// This function takes an integer as argument which represents the color of the line.
// 0 represents black and 1 represents white.
double line_cm(int color) {
  double weightedsum=0.0, sum=0.0;
  if (color) {
    for (int i=0;i<8;i++){
      weightedsum += calib_linesensor[i]*i;
      sum += calib_linesensor[i];
    }
  }else {
    for (int i=0;i<8;i++){
      weightedsum += (1-calib_linesensor[i])*i;
      sum += (1-calib_linesensor[i]);
    }
  }
  if (sum != 0.0) return weightedsum/sum;
  else return 0.0;
}

