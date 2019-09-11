package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Basic_OmniDrive extends Command {
  public Basic_OmniDrive(){
    requires(Robot.m_Chassis);
  }

  @Override
  protected void initialize(){
  }

  @Override
  protected void execute() {
    double Joystick_Y = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_X = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);

    if(RobotMap.Joystick_Y_Invert){
      Joystick_Y = Joystick_Y * -1;
    }
    if(RobotMap.Joystick_X_Invert){
      Joystick_X = Joystick_X * -1;
    }

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_Y > 0 && Joystick_Y < 1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }else if(Joystick_Y < 0 && Joystick_Y > -1){
      Joystick_Y = RobotMap.Joystick_Y_OutPutRate * -Math.pow(Math.abs(Joystick_Y),RobotMap.Joystick_Y_Exponential);
    }
    if(Joystick_X > 0 && Joystick_X < 1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }else if(Joystick_X < 0 && Joystick_X > -1){
      Joystick_X = RobotMap.Joystick_X_OutPutRate * -Math.pow(Math.abs(Joystick_X),RobotMap.Joystick_X_Exponential);
    }

    Joystick_X = Joystick_X / Math.sqrt(2);
    Joystick_Y = Joystick_Y / Math.sqrt(2);

    double Vector = Math.sqrt(Math.pow(Joystick_Y,2) + Math.pow(Joystick_X, 2));
    double Angle = Math.atan2(Joystick_Y,Joystick_X);

    double RF = Math.sin(Angle - (Math.PI/4)) * Vector;
    double LF = Math.sin(Angle - (Math.PI*3/4)) * Vector;
    double LB = Math.sin(Angle - (Math.PI*5/4)) * Vector;
    double RB = Math.sin(Angle - (Math.PI*7/4)) * Vector;

    //System.out.println("LF:"+ LF + "  " + "RF:" + RF);
    //System.out.println("LB:"+ LB + "  " + "RB:" + RB);
    //System.out.println("////////////////////");

    Robot.m_Chassis.SetSeparateSpeed(RF, LF, LB, RB);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
