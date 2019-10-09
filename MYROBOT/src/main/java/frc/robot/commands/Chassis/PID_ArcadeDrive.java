package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utility;
import frc.robot.systems.PID_System;

//// Still Testing ////
public class PID_ArcadeDrive extends Command {
  public PID_System PID1 = new PID_System();
  public Timer PID1Enable_Timer = new Timer();

  double PID1Enable_PriviousTime = 0;
  boolean PID1Enable = true;

  double Gryo_HeadingAngle = 0;

  boolean Joystick_LY_Invert = false;
  double Joystick_LY_OutPutRate = 1;
  double Joystick_LY_Exponential = 2;

  boolean Joystick_RX_Invert = false;
  double Joystick_RX_OutPutRate = 1;
  double Joystick_RX_Exponential = 2;

  double Joystick_DeadZone = 0.2;

  public PID_ArcadeDrive() {
    requires(Robot.m_Chassis);
    PID1.Init();
    PID1.Enable_PID(true);
    PID1.Enable_AntiWindUp(true, 1);
    PID1.Enable_AutoStop(false, 0, 0);
    PID1.Enable_TimeOut(false, 0);

    PID1Enable_Timer.reset();
    PID1Enable_Timer.start();
  }

  @Override
  protected void initialize() {
    Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
  }
  
  @Override
  protected void execute() {
    double Joystick_LY = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_RX = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);
    
    boolean Joystick_LY_InDeadZone = false;
    boolean Joystick_RX_InDeadZone = false;

    if(Joystick_LY_Invert){
      Joystick_LY = Joystick_LY * -1;
    }
    if(Joystick_RX_Invert){
      Joystick_RX = Joystick_RX * -1;
    }

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_LY >= Joystick_DeadZone && Joystick_LY <= 1){
      Joystick_LY = Joystick_LY_OutPutRate * Math.pow(Math.abs(Joystick_LY),Joystick_LY_Exponential);
    }else if(Joystick_LY <= -Joystick_DeadZone && Joystick_LY >= -1){
      Joystick_LY = Joystick_LY_OutPutRate * -Math.pow(Math.abs(Joystick_LY),Joystick_LY_Exponential);
    }else{
      Joystick_LY = 0;
      Joystick_LY_InDeadZone = true;
    }
    if(Joystick_RX >= Joystick_DeadZone && Joystick_RX <= 1){
      Joystick_RX = Joystick_RX_OutPutRate * Math.pow(Math.abs(Joystick_RX),Joystick_RX_Exponential);
    }else if(Joystick_RX <= -Joystick_DeadZone && Joystick_RX >= -1){
      Joystick_RX = Joystick_RX_OutPutRate * -Math.pow(Math.abs(Joystick_RX),Joystick_RX_Exponential);
    }else{
      Joystick_RX = 0;
      Joystick_RX_InDeadZone = true;
    }

    double Rspd = 0;
    double Lspd = 0;
    if(Robot.m_Oi.GetButton(RobotMap.Button_Right)){
      Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
    }

    if(Joystick_RX_InDeadZone == true){
      if(PID1Enable){
        double Gryo = (((((Robot.m_Chassis.Get_Angle() + (360 - Gryo_HeadingAngle)) % 360) + 180) % 360) - 180);
        double Pid = PID1.PID(Gryo, RobotMap.Chassis_Kp, RobotMap.Chassis_Ki, RobotMap.Chassis_Kd);
        Rspd = Joystick_LY - Pid;
        Lspd = Joystick_LY + Pid;
      }else{
        if(PID1Enable_Timer.get() >= PID1Enable_PriviousTime + RobotMap.PIDEnable_Delay){
          PID1Enable = true;
          PID1.Init_Parameter();
          Gryo_HeadingAngle = Robot.m_Chassis.Get_Angle();
        }
        Rspd = Joystick_LY;
        Lspd = Joystick_LY;
      }
    }else{
      PID1Enable = false;
      PID1Enable_PriviousTime = PID1Enable_Timer.get();
      Rspd = Joystick_LY - Joystick_RX;
      Lspd = Joystick_LY + Joystick_RX;
    }

    Rspd = Utility.Constrain(Rspd, 1, -1);
    Lspd = Utility.Constrain(Lspd, 1, -1);

    Robot.m_Chassis.SetSpeed(Lspd,Rspd);
  }
  
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.m_Chassis.SetSpeed(0,0);
  }

  @Override
  protected void interrupted() {
    this.end();
  }
}
