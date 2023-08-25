#include <Arduino.h>
#include <Configuration.h>
#include "parameter.h"
#include <ArduinoEigen.h>
#ifndef CONTROL_H
#define CONTROL_H



float k_x_position= 1;
float k_x_velocity     = 0;

float k_y_position= 1;
float k_y_velocity  = 0;

float k_z_position    = 1;
float k_z_velocity = 0;


float k_roll        = 23.5;
float k_roll_rate   = 5.5;

// float k_roll        = 46.3*3/4;
// float k_roll_rate   = 11.0*3/4;


float k_pitch       = k_roll; 
float k_pitch_rate  = k_roll_rate;

float k_yaw         = 29.1f;
float k_yaw_rate    = 7.6f;

// float k_yaw         = 34.6f;
// float k_yaw_rate    = 10.0f;


float _roll_integrator      =0; 
float _pitch_integrator     =0;
float _yaw_integrator       =0;
float _k_roll_integrator    =0;
float _k_pitch_integrator   =0;
float _k_yaw_integrator     =0;
float _dt = dt;


float u1,u2,u3,u4, u5, u6;
float Force_x_input;
float Force_y_input;
float Force_z_input;

float yaw_error, yaw_rate_error;
int m1,m2,m3,m4;
float s1 = 0,s2= 0,s3= 0,s4= 0;
float omega2_m1,omega2_m2,omega2_m3,omega2_m4;

using Eigen::MatrixXd;
 
 void print_mtxf(MatrixXd K) {
  int i, j;
  for (i = 0; i < K.rows(); i++) {
    for (j = 0; j < K.cols(); j++) {
      Serial.print(K(i, j));
      Serial.print(" ");
    }
    Serial.println();
  }
}
MatrixXd At(6, 4);


void integrator(float *integrator, float *k_integrator, float error)
{
    #ifdef CONTROL_INTEGRATOR
    *integrator += *k_integrator * error * _dt;
    if(*integrator > INTEGRATOR_LIMIT)
    {
        *integrator = INTEGRATOR_LIMIT;
    }
    else if(*integrator < -INTEGRATOR_LIMIT)
    {
        *integrator = -INTEGRATOR_LIMIT;
    }
    #endif
}

// equation for the pwm to thrust
int thrust_to_pwm(float input_thrust)
{
    return input_thrust/abs(input_thrust)*(0.1704*pow(abs(input_thrust),0.6735))*10;
}

int omega_to_pwm(float input_omega)
{
    return (((0.0000006*(input_omega*input_omega))+(input_omega*0.005))*10);
}

int omega2_to_pwm(float input_omega2)
{       
        
        if ( input_omega2 <= 0 )
        {
            return 998;
        }
        return 998+(omega_to_pwm(sqrtf(input_omega2)));
}

void control_setup()
{
    #ifdef CONTROL_INTEGRATOR
    _k_roll_integrator    =k_roll_integrator ;
    _k_pitch_integrator   =k_pitch_integrator;
    _k_yaw_integrator     =k_yaw_integrator  ;
    _roll_integrator = 0;
    _pitch_integrator = 0;
    _yaw_integrator = 0;
    #endif
}

void force_to_pwm(radio_parameter *data_radio)
{
    // if (data_radio->armed)
    // {

    //     m1 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
    //         +(u5/2)/ARM_LEGHT + (u6/2) ) + (data_radio->ch_yaw-MOTOR_PWM_CENTER);
    //     m2 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
    //         -(u4/2)/ARM_LEGHT - (u6/2) ) - (data_radio->ch_yaw-MOTOR_PWM_CENTER);
    //     m3 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
    //         -(u5/2)/ARM_LEGHT + (u6/2) ) + (data_radio->ch_yaw-MOTOR_PWM_CENTER);
    //     m4 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
    //         +(u4/2)/ARM_LEGHT - (u6/2) ) - (data_radio->ch_yaw-MOTOR_PWM_CENTER);

    // }
    // else
    // {
    //     m1 = MOTOR_PWM_MIN;
    //     m2 = MOTOR_PWM_MIN;
    //     m3 = MOTOR_PWM_MIN;
    //     m4 = MOTOR_PWM_MIN;
    // }

    // calculate servo angle
    #ifdef Y_POSITION_CONTROLLER
        //calculate servo angle
    s1 = atan2(2*u2,u3)*RAD2DEG;
    s3 = atan2(2*u2,u3)*RAD2DEG;
    #else
    u2 = 0;
    s1 = 0;
    s3 = 0;

    #endif

    #ifdef X_POSITION_CONTROLLER
    s2 = atan2(2*u1,u3)*RAD2DEG;
    s4 = atan2(2*u1,u3)*RAD2DEG;
    #else
    u1 = 0;
    s2 = 0;
    s4 = 0;
    #endif

    #ifdef AUX_SETPOINT
    if (data_radio->aux1>1500)
    {
      u2 = 0;
      s1 = 0;
      s3 = 0;
    }
    else 
    {
      s1 = atan2(2*u2,u3)*RAD2DEG;
      s3 = atan2(2*u2,u3)*RAD2DEG;
    }
    
    if(data_radio->aux2>1500)
    {
      u1 = 0;
      s2 = 0;
      s4 = 0;
    }
    else
    {
    s2 = atan2(2*u1,u3)*RAD2DEG;
    s4 = atan2(2*u1,u3)*RAD2DEG;
    }
    
    #endif

 At << 0 , force_constant*sin(s2*DEG_TO_RAD) , 0 , force_constant*sin(s4*DEG_TO_RAD),
        force_constant*sin(s1*DEG_TO_RAD) , 0 , force_constant*sin(s3*DEG_TO_RAD) , 0,
        force_constant*cos(s1*DEG_TO_RAD) , force_constant*cos(s2*DEG_TO_RAD) ,
         force_constant*cos(s3*DEG_TO_RAD) , force_constant*cos(s4*DEG_TO_RAD),
        0 , -ARM_LEGHT*force_constant*cos(s2*DEG_TO_RAD)+torque_constant*sin(s2*DEG_TO_RAD) 
        , 0 , ARM_LEGHT*force_constant*cos(s4*DEG_TO_RAD)+torque_constant*sin(s4*DEG_TO_RAD),
        ARM_LEGHT*force_constant*cos(s1*DEG_TO_RAD)-torque_constant*sin(s1*DEG_TO_RAD) , 0 ,
         -ARM_LEGHT*force_constant*cos(s3*DEG_TO_RAD)-torque_constant*sin(s3*DEG_TO_RAD) , 0,
        torque_constant*cos(s1*DEG_TO_RAD)+ARM_LEGHT*force_constant*sin(s1*DEG_TO_RAD) ,
         -torque_constant*cos(s2*DEG_TO_RAD)-ARM_LEGHT*force_constant*sin(s2*DEG_TO_RAD) ,
          torque_constant*cos(s3*DEG_TO_RAD)-ARM_LEGHT*force_constant*sin(s3*DEG_TO_RAD) ,
           -torque_constant*cos(s4*DEG_TO_RAD)+ARM_LEGHT*force_constant*sin(s4*DEG_TO_RAD);

    MatrixXd pmAt=At.completeOrthogonalDecomposition().pseudoInverse();
    MatrixXd Force(6,1);
    Force << u1,u2,u3,u4,u5,u6;
    MatrixXd w = pmAt*Force;
    omega2_m1 = w(0,0);
    omega2_m2 = w(1,0);
    omega2_m3 = w(2,0);
    omega2_m4 = w(3,0);

    if (data_radio->armed)
    {

        m1 = omega2_to_pwm(omega2_m1);
        m2 = omega2_to_pwm(omega2_m2);
        m3 = omega2_to_pwm(omega2_m3);
        m4 = omega2_to_pwm(omega2_m4);

    }
    else
    {
        m1 = MOTOR_PWM_MIN;
        m2 = MOTOR_PWM_MIN;
        m3 = MOTOR_PWM_MIN;
        m4 = MOTOR_PWM_MIN;
    }

    
}

void control(attitude_parameter *attitude,attitude_parameter *attitude_ref 
            ,radio_parameter *data_radio)
{
    #ifdef TILT_QUADROTOR
    
    // calculate yaw error
    yaw_error = attitude->yaw - attitude_ref->yaw;
    if (yaw_error > 180)
    {
        yaw_error = yaw_error - 360;
    }
    else if (yaw_error < -180)
    {
        yaw_error = yaw_error + 360;
    }



    Force_x_input = (-k_x_position * (attitude->x_position-attitude_ref->x_position)) 
        +(-k_x_velocity * (attitude->x_velocity-attitude_ref->x_velocity));
    Force_y_input = (-k_y_position * (attitude->y_position-attitude_ref->y_position) ) 
        +(-k_y_velocity * (attitude->y_velocity-attitude_ref->y_velocity));
    Force_z_input = (-k_z_position * (attitude->z_position-attitude_ref->z_position)) 
        +(-k_z_velocity * (attitude->z_velocity-attitude_ref->z_velocity));


    u1 = Force_x_input;
    u2 = Force_y_input;
    u3 = Force_z_input;
    u4 = (-k_roll * (attitude->roll-attitude_ref->roll) 
        +(-k_roll_rate * (attitude->roll_rate-attitude_ref->roll_rate)));
    u5 = (-k_pitch * (attitude->pitch-attitude_ref->pitch) ) 
        +(-k_pitch_rate * (attitude->pitch_rate-attitude_ref->pitch_rate));
    u6 = (-k_yaw * (yaw_error)) 
        +(-k_yaw_rate * (attitude->yaw_rate-attitude_ref->yaw_rate));
    
    #endif

    #ifdef CONTROL_INTEGRATOR
    _roll_integrator  += -_k_roll_integrator*(attitude->roll-attitude_ref->roll)*_dt;
    _pitch_integrator += -_k_pitch_integrator*(attitude->pitch-attitude_ref->pitch)*_dt;
    _yaw_integrator   += -_k_yaw_integrator*(yaw_error)*_dt;
    _roll_integrator = constrain(roll_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
    _pitch_integrator = constrain(pitch_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);

    //integral reset system
    if(data_radio->ch_throttle<1200||!data_radio->armed)
    {
        _roll_integrator = 0;
        _pitch_integrator = 0;
        _yaw_integrator = 0;
    }
    if(attitude->roll-attitude_ref->roll < 0.15 && attitude->roll-attitude_ref->roll >-0.15)
    {
        _roll_integrator = 0;
    }
    if(attitude->pitch-attitude_ref->pitch <  0.15 && attitude->pitch-attitude_ref->pitch > 0.15)
    { 
        _pitch_integrator = 0;
    }
    if(attitude->yaw-attitude_ref->pitch <  0.2 && attitude->yaw-attitude_ref->pitch > 0.2)
    {
        _pitch_integrator = 0;
    }

    u4 = u4 + _roll_integrator;
    u5 = u5 + _pitch_integrator;
    u6 = u6 + _yaw_integrator;
    // u6 = 0;

    #endif

    force_to_pwm(data_radio);
    
}

void get_setpoint(attitude_parameter *attitude_ref,radio_parameter *data_radio,attitude_parameter *attitude)
{
    #ifdef ROLL_CONTROLLER
    if(data_radio->ch_roll>(RADIO_PWM_CENTER+15)||data_radio->ch_roll<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->roll =  (((float)data_radio->ch_roll-RADIO_PWM_CENTER)/500*LIMIT_ROLL);
      attitude_ref->roll_rate =0;

    }
    else
    {
      attitude_ref->roll =0;
    }
    #else
    attitude_ref->roll = 0;
    attitude_ref->roll_rate = 0;
    #endif

    #ifdef PITCH_CONTROLLER
    if(data_radio->ch_pitch>(RADIO_PWM_CENTER+15)||data_radio->ch_pitch<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->pitch =  ((((float)data_radio->ch_pitch-RADIO_PWM_CENTER)/500*-LIMIT_PITCH)) ;
      attitude_ref->pitch_rate =0;
    
    }
    else
    {
      attitude_ref->pitch =0;
    }
    #else
    attitude_ref->pitch = 0;
    attitude_ref->pitch_rate = 0;
    #endif


    #ifdef YAW_CONTROLLER
    if(data_radio->ch_yaw>(RADIO_PWM_CENTER+50)||data_radio->ch_yaw<(RADIO_PWM_CENTER-50))
    {
      attitude_ref->yaw = attitude->yaw+ ((((float)data_radio->ch_yaw-RADIO_PWM_CENTER)/500*LIMIT_YAW_RATE)) ;
    //   attitude_ref->yaw = attitude->yaw;
        if (attitude_ref->yaw > 180)
        {
            attitude_ref->yaw = attitude_ref->yaw - 360;
        }
        else if (attitude_ref->yaw < -180)
        {
            attitude_ref->yaw = attitude_ref->yaw + 360;
        }
    }
    else
    {
      attitude_ref->yaw_rate =0;
    }
    #else 
    attitude_ref->yaw_rate = 0 ;
    attitude_ref->yaw = 0;
    #endif

    #ifdef Z_POSITION_CONTROLLER
    attitude_ref->z_position =  (11620*4*(data_radio->ch_throttle-995)/1000)+1;;
    #endif

    #ifdef X_POSITION_CONTROLLER
    if(data_radio->ch_pitch>(RADIO_PWM_CENTER+15)||data_radio->ch_pitch<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->x_position =  tan(((float)data_radio->ch_pitch-1500)/500*SERVO_ANGLE_MAX*DEG2RAD)*(attitude_ref->z_position)/2;
        
    }
    else
    {
      attitude_ref->x_position =0;
    }

    #else 
      attitude_ref->x_position =0;

    #endif

    #ifdef Y_POSITION_CONTROLLER
    if(data_radio->ch_roll>(RADIO_PWM_CENTER+15)||data_radio->ch_roll<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->y_position = tan(((float)data_radio->ch_roll-1500)/500*SERVO_ANGLE_MAX*DEG2RAD)*(attitude_ref->z_position)/2;
    }
    else
    {
      attitude_ref->y_position =0;
    }

    #else
      attitude_ref->y_position =0;

    #endif
}


void get_auxsetpoint(attitude_parameter *attitude_ref,radio_parameter *data_radio,attitude_parameter *attitude)
{
       #ifdef YAW_CONTROLLER
    if(data_radio->ch_yaw>(RADIO_PWM_CENTER+50)||data_radio->ch_yaw<(RADIO_PWM_CENTER-50))
    {
      attitude_ref->yaw = attitude->yaw+ ((((float)data_radio->ch_yaw-RADIO_PWM_CENTER)/500*LIMIT_YAW_RATE)) ;
    //   attitude_ref->yaw = attitude->yaw;
        if (attitude_ref->yaw > 180)
        {
            attitude_ref->yaw = attitude_ref->yaw - 360;
        }
        else if (attitude_ref->yaw < -180)
        {
            attitude_ref->yaw = attitude_ref->yaw + 360;
        }
    }
    else
    {
      attitude_ref->yaw_rate =0;
    }
    #else 
    attitude_ref->yaw_rate = 0 ;
    attitude_ref->yaw = 0;
    #endif

    #ifdef Z_POSITION_CONTROLLER
    attitude_ref->z_position =  (11620*4*(data_radio->ch_throttle-995)/1000)+1;;
    #endif
   
   if(data_radio->aux1>1500)
   {
    if(data_radio->ch_roll>(RADIO_PWM_CENTER+15)||data_radio->ch_roll<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->roll =  (((float)data_radio->ch_roll-RADIO_PWM_CENTER)/500*LIMIT_ROLL);
      attitude_ref->roll_rate =0;

    }
    else
    {
      attitude_ref->roll =0;
    }
    attitude_ref->y_position =0;
   }
   else
   {
    attitude_ref->roll = 0;
    attitude_ref->roll_rate = 0;
    if(data_radio->ch_roll>(RADIO_PWM_CENTER+15)||data_radio->ch_roll<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->y_position = tan(((float)data_radio->ch_roll-1500)/500*SERVO_ANGLE_MAX*DEG2RAD)*(attitude_ref->z_position)/2;
    }
    else
    {
      attitude_ref->y_position =0;
    }
   }

  if(data_radio->aux2>1500)
  {
    if(data_radio->ch_pitch>(RADIO_PWM_CENTER+15)||data_radio->ch_pitch<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->pitch =  ((((float)data_radio->ch_pitch-RADIO_PWM_CENTER)/500*-LIMIT_PITCH)) ;
      attitude_ref->pitch_rate =0;
    
    }
    else
    {
      attitude_ref->pitch =0;
    }
      attitude_ref->x_position =0;
  }
  else
  {
    attitude_ref->pitch = 0;
    attitude_ref->pitch_rate = 0;
    if(data_radio->ch_pitch>(RADIO_PWM_CENTER+15)||data_radio->ch_pitch<(RADIO_PWM_CENTER-15))
    {
      attitude_ref->x_position =  tan(((float)data_radio->ch_pitch-1500)/500*SERVO_ANGLE_MAX*DEG2RAD)*(attitude_ref->z_position)/2;
        
    }
    else
    {
      attitude_ref->x_position =0;
    }
  }
}


#endif