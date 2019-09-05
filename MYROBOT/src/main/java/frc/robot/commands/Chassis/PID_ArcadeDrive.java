package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Useful;

public class PID_ArcadeDrive extends Command {

  public PID_ArcadeDrive() {
    requires(Robot.m_Chassis);
  }

  @Override
  protected void initialize() {
    Robot.m_Chassis.InitGryo();
  }
  
  @Override
  protected void execute() {
    double Joystick_Y = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_X = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);
    boolean Joystick_Y_InDeadZone = false;
    boolean Joystick_X_InDeadZone = false;

    if(RobotMap.Joystick_Y_Invert){
      Joystick_Y = Joystick_Y * -1;
    }
    if(RobotMap.Joystick_X_Invert){
      Joystick_X = Joystick_X * -1;
    }

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_Y > RobotMap.Joystick_DeadZone && Joystick_Y < 1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }else if(Joystick_Y < -RobotMap.Joystick_DeadZone && Joystick_Y > -1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * -Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }else{
      Joystick_Y = 0;
      Joystick_Y_InDeadZone = true;
    }
    if(Joystick_X > RobotMap.Joystick_DeadZone && Joystick_X < 1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }else if(Joystick_X < -RobotMap.Joystick_DeadZone && Joystick_X > -1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * -Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }else{
      Joystick_X = 0;
      Joystick_X_InDeadZone = true;
    }

    double Rspd = 0;
    double Lspd = 0;

    if(Joystick_X_InDeadZone == true){
      
    }else{
      Rspd = Joystick_Y - Joystick_X;
      Lspd = Joystick_Y + Joystick_X;
    }

    /*
    double Rspd = 0.0;
    double Lspd = 0.0;
    
    if(Math.abs(Joystick_Y) < RobotMap.Joystick_DeadZone){
      Joystick_Y = 0;
    }
    if(Math.abs(Joystick_X) < RobotMap.Joystick_DeadZone){
      Robot.m_oi.SetRumble(0);
      Robot.m_Chassis.EnablePID();
      Joystick_X = 0;
      double gryo = Robot.m_Chassis.ReadNowAngle(Robot.m_Chassis.ReadAngle());
      double pid = Robot.m_Chassis.PID(gryo, RobotMap.Chassis_Kp, RobotMap.Chassis_Ki,RobotMap.Chassis_Kd);
      Rspd = Joystick_Y + pid;
      Lspd = Joystick_Y - pid;
    }else{
      Robot.m_oi.SetRumble(Math.abs(Joystick_X));
      Rspd = Joystick_Y + Joystick_X;
      Lspd = Joystick_Y - Joystick_X;
      Robot.m_Chassis.DisablePID();
      Robot.m_Chassis.SetInitPIDVariable();
    }
    Rspd = Useful.Constrain(Rspd,1,-1);
    Lspd = Useful.Constrain(Lspd,1,-1);
    //System.out.println(Lspd + "  " + Rspd);
    if(Robot.m_oi.GetAxis(RobotMap.Axis_LT) > 0.75){
      Robot.m_Chassis.SetSpeed(-Lspd*0.5,Rspd*0.5);
    }else{
      Robot.m_Chassis.SetSpeed(-Lspd,Rspd);
    }
  */
    
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
