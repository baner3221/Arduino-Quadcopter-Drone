#include <Wire.h>
#include <SoftWire.h>
#define Wire HWire;
#define STM32_board_LED PC13;
TwoWire HWire(2, I2C_FAST_MODE);


//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int timer12;

int32_t channel_1_start, channel_1_stop, channel_1;
int32_t channel_2_start, channel_2_stop, channel_2;
int32_t channel_3_start, channel_3_stop, channel_3;
int32_t channel_4_start, channel_4_stop, channel_4;

int level_calibration_on = 0;
boolean gyro_angle_set;
float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;         


int16_t motor_idle_speed = 1100;           //Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.


float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).

#define STM32_board_LED PC13               //Change PC13 if the LED on the STM32 is connected to another output.

//Tuning parameters/settings is explained in this video: https://youtu.be/ys-YpOaA2ME
#
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t check_byte, flip32, start;
uint8_t error, error_counter, error_led;
uint8_t channel_select_counter;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1, pid_roll_setpoint_base, channel_1_stop;
int32_t channel_2_start, channel_2, pid_pitch_setpoint_base, channel_2_stop;
int32_t channel_3_start, channel_3, channel_3_stop;
int32_t channel_4_start, channel_4, channel_4_stop;

int32_t measured_time, measured_time_start;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

int32_t acc_alt_integrated;

uint32_t loop_timer, error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;


//Adjust settings online
uint32_t setting_adjust_timer;
uint16_t setting_click_counter;
uint8_t previous_channel_6;
float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  pinMode(4, INPUT_ANALOG);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(STM32_board_LED, OUTPUT);
  digitalWrite(STM32_board_LED, HIGH);

  timer_setup();
  delay(50);

  HWire.begin();
  HWire.beginTransmission(gyro_address);
  HWire.endTransmission();

  gyro_setup();

  delay(4);

  calibrate_gyro();

  battery_voltage = (float)analogRead(4)/112.81;
  calibrate_receiver();
}

void loop() {

  loop_timer = micros();
  // put your main code here, to run repeatedly:
  timer12 = micros();
  gyro_signalen();

  
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);

  angle_pitch += (float)gyro_pitch * 0.0000611;
  angle_roll += (float)gyro_roll * 0.0000611;
  angle_yaw += (float)gyro_yaw * 0.0000611;

  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066); 

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  
  if(abs(acc_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }

  if(abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  if(channel_3 < 1050 && channel_4 < 1050) start = 1;

  if(start == 1 && channel_3 < 1050 && channel_4 > 1450) {
    start = 2;
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;

    gyro_angle_set = true;

    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  if(start == 2 && channel_3 < 1050 && channel_4 > 1950) start = 0;

  pid_roll_setpoint = 0;

  if(channel_2 > 1508) pid_roll_setpoint = channel_1 - 1508;
  else if(channel_1 < 1492) pid_roll_setpoint = channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;

  if(channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if(channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;

  if(channel_3 > 1050){
    if(channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508)/3.0;
    else if(channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492)/3.0;
  }

  calculate_pid();

  battery_voltage = battery_voltage * 0.92 + (analogRead(4) + 65) * 0.09853;

  throttle = channel_3;

  if (start == 2) {
    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    if (battery_voltage < 1240 && battery_voltage > 800) {
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);
    }
    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;
  }

  else{
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  TIMER4_BASE->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER4_BASE->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER4_BASE->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER4_BASE->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
  TIMER4_BASE->CNT = 5000;

  if (micros() - loop_timer > 4050)error = 2;                                      //Output an error if the loop time exceeds 4050us.

  Serial.print("Loop Time");
  Serial.println(micros() - looper_timer);
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  

}
