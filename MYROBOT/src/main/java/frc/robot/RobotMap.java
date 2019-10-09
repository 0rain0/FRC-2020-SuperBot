package frc.robot;

public class RobotMap {
  //// Chassis Setting ////
  public final static byte Motor_RA = 3;
  public final static byte Motor_RB = 4;
  public final static byte Motor_LA = 1;
  public final static byte Motor_LB = 2;
  public final static boolean Motor_RA_Invert = true;
  public final static boolean Motor_RB_Invert = true;
  public final static boolean Motor_LA_Invert = false;
  public final static boolean Motor_LB_Invert = false;
  public final static double ChassisPowerPercentage = 0.5;
  public final static double PIDEnable_Delay = 0.25;
  public final static double Chassis_Kp = 0.008;
  public final static double Chassis_Ki = 0.0;
  public final static double Chassis_Kd = 0.05;
  
  //// Cargo Setting ////
  public final static byte Intake_Motor_R = 11;
  public final static byte Intake_Motor_L = 12;
  public final static byte Intake_Motor_Pitch = 13;
  public final static boolean Intake_Motor_R_Invert = true;
  public final static boolean Intake_Motor_L_Invert = true;
  public final static boolean Intake_Motor_Pitch_invert = false;
  public final static double IntakePowerPercentage = 0.7;
  public final static double IntakePitchPowerPercentage = 0.35;
  public final static int Encoder_Pitch_DIOA = 9;
  public final static int Encoder_Pitch_DIOB = 8;
  public final static boolean Encoder_Pitch_Invert = false;

  //// Panel Setting ////
  public final static byte Panel_Motor = 0;
  public final static boolean Panel_Motor_Invert = true;
  public final static double PanelPowerPercentage = 0.65;
  public final static byte Encoder_Arm_DIOA = 0;
  public final static byte Encoder_Arm_DIOB = 1;
  public final static boolean Encoder_Arm_Invert = false;
  public final static int Pulse_Per_Rotation = 1304;
  public final static double Panel_Kp = 0.2;
  public final static double Panel_Ki = 0.000;
  public final static double Panel_Kd = 0;

  //// Controller Setting ////
  public final static byte Joystick_Port = 0;

  public final static double Joystick_DeadZone = 0.2;
  
  public final static boolean Joystick_LY_Invert = false;
  public final static double Joystick_LY_OutPutRate = 1;
  public final static double Joystick_LY_Exponential = 2;

  public final static boolean Joystick_LX_Invert = false;
  public final static double Joystick_LX_OutPutRate = 1;
  public final static double Joystick_LX_Exponential = 2;

  public final static boolean Joystick_RY_Invert = false;
  public final static double Joystick_RY_OutPutRate = 1;
  public final static double Joystick_RY_Exponential = 2;

  public final static boolean Joystick_RX_Invert = false;
  public final static double Joystick_RX_OutPutRate = 0.5;
  public final static double Joystick_RX_Exponential = 2;
  
  public final static byte Button_Down = 1; //A-YELLOW
  public final static byte Button_Right = 2;//B-RED
  public final static byte Button_Left = 3;//X-BLUE
  public final static byte Button_Up = 4;//Y-YELLOW
  public final static byte Button_Left_Back = 5;
  public final static byte Button_Right_Back = 6;
  public final static byte Button_Back = 7;
  public final static byte Button_Start = 8;
  public final static byte Button_Joystick_L = 9;
  public final static byte Button_Joysticl_R = 10;
  public final static byte Axis_LT = 2;
  public final static byte Axis_RT = 3;
  public final static byte Joystick_LX = 0;
  public final static byte Joystick_LY = 1;
  public final static byte Joystick_RX = 4;
  public final static byte Joystick_RY = 5;
}
