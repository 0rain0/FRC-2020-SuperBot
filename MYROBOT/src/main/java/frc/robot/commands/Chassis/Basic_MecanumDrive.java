package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utility;

//Type-O Mecanum 
public class Basic_MecanumDrive extends Command {
  boolean Joystick_LX_Invert = false;
  double Joystick_LX_OutPutRate = 1;
  double Joystick_LX_Exponential = 2;

  boolean Joystick_LY_Invert = false;
  double Joystick_LY_OutPutRate = 1;
  double Joystick_LY_Exponential = 2;

  boolean Joystick_RX_Invert = false;
  double Joystick_RX_OutPutRate = 1;
  double Joystick_RX_Exponential = 2;

  double Joystick_DeadZone = 0.2;

  public Basic_MecanumDrive() {
    requires(Robot.m_Chassis);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    double Joystick_LY = Robot.m_Oi.GetAxis(RobotMap.Joystick_LY);
    double Joystick_LX = Robot.m_Oi.GetAxis(RobotMap.Joystick_LX);
    double Joystick_RX = Robot.m_Oi.GetAxis(RobotMap.Joystick_RX);

    if(Joystick_LY_Invert){
      Joystick_LY = Joystick_LY * -1;
    }
    if(Joystick_LX_Invert){
      Joystick_LX = Joystick_LX * -1;
    }
    if(Joystick_RX_Invert){
      Joystick_RX = Joystick_RX * -1;
    }

    //https://www.desmos.com/calculator/epgkans3c0
    if(Joystick_LY > 0 && Joystick_LY <= 1){
      Joystick_LY = Joystick_LY_OutPutRate * Math.pow(Math.abs(Joystick_LY),Joystick_LY_Exponential);
    }else if(Joystick_LY < 0 && Joystick_LY >= -1){
      Joystick_LY = Joystick_LY_OutPutRate * -Math.pow(Math.abs(Joystick_LY),Joystick_LY_Exponential);
    }

    if(Joystick_LX > 0 && Joystick_LX <= 1){
      Joystick_LX = Joystick_LX_OutPutRate * Math.pow(Math.abs(Joystick_LX),Joystick_LX_Exponential);
    }else if(Joystick_LX < 0 && Joystick_LX >= -1){
      Joystick_LX = Joystick_LX_OutPutRate * -Math.pow(Math.abs(Joystick_LX),Joystick_LX_Exponential);
    }
    
    if(Joystick_RX > 0 && Joystick_RX <= 1){
      Joystick_RX = Joystick_RX_OutPutRate * Math.pow(Math.abs(Joystick_RX),Joystick_RX_Exponential);
    }else if(Joystick_RX < 0 && Joystick_RX >= -1){
      Joystick_RX = Joystick_RX_OutPutRate * -Math.pow(Math.abs(Joystick_RX),Joystick_RX_Exponential);
    }

    //https://editor.p5js.org/UnreaLin/sketches/tC4rB7MPT
    //Make sure every motor rotate direction before road test
    //When you face output shaft, Direction must be 
    //LF:CounterClockwise   RF:ClockWise
    //LB:CounterClockwise   RB:Clockwise
    //If not, Use RobotMap to adjust it

    double Vector = Math.sqrt(Math.pow(Joystick_LY,2) + Math.pow(Joystick_LX,2));
    double Angle = Math.atan2(Joystick_LY,Joystick_LX);

    double RF = Vector * Math.cos(Angle - (Math.PI * 1 / 4)) / Math.sqrt(2) - Joystick_RX;
    double LF = Vector * Math.cos(Angle - (Math.PI * 3 / 4)) / Math.sqrt(2) + Joystick_RX;
    double LB = Vector * Math.cos(Angle - (Math.PI * 5 / 4)) / -Math.sqrt(2) + Joystick_RX;
    double RB = Vector * Math.cos(Angle - (Math.PI * 7 / 4)) / -Math.sqrt(2) - Joystick_RX;

    RF = Utility.Constrain(RF, 1, -1);
    LF = Utility.Constrain(LF, 1, -1);
    LB = Utility.Constrain(LB, 1, -1);
    RF = Utility.Constrain(RF, 1, -1);

    Robot.m_Chassis.SetSeparateSpeed(RF, LF, LB, RB);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.m_Chassis.SetSeparateSpeed(0, 0, 0, 0);
  }

  @Override
  protected void interrupted() {
    this.end();
  }
}
