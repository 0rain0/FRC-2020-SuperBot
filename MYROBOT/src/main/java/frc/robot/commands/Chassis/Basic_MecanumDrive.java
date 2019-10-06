package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utility;

//Type-O Mecanum 
public class Basic_MecanumDrive extends Command {
  public Basic_MecanumDrive() {
    requires(Robot.m_Chassis);
  }

  @Override
  protected void initialize() {
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

    //https://editor.p5js.org/UnreaLin/sketches/tC4rB7MPT
    //Make sure every motor rotate direction before road test
    //When you face output shaft, Direction must be 
    //RF:CounterClockwise   LF:ClockWise
    //LB:CounterClockwise   RB:Clockwise
    //If not, Use RobotMap to adjust it

    double Vector = Math.sqrt(Math.pow(Joystick_Y,2) + Math.pow(Joystick_X,2));
    double Angle = Math.atan2(Joystick_Y,Joystick_X);

    double RF = Vector * Math.cos(Angle - (Math.PI * 1 / 4)) / Math.sqrt(2);
    double LF = Vector * Math.cos(Angle - (Math.PI * 3 / 4)) / Math.sqrt(2);
    double LB = Vector * Math.cos(Angle - (Math.PI * 5 / 4)) / -Math.sqrt(2);
    double RB = Vector * Math.cos(Angle - (Math.PI * 7 / 4)) / -Math.sqrt(2);

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
