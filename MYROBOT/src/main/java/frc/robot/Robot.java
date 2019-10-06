package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Chassis;


//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //Subsystem Declaration//
  //public static UsbCamera m_Camera;
  public static Chassis m_Chassis;
  public static OI m_Oi;
  
  Command m_autonomousCommand;

  //Sensor Declaration//

  
  
  //Command m_autonomousCommand;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_Chassis = new Chassis();
    m_Oi = new OI();
    //m_Camera = CameraServer.getInstance().startAutomaticCapture(0);
    //m_Camera.setResolution(160, 120);
    //m_Camera.setFPS(20);
    Robot.m_Chassis.Init_Gryo();
    
  }
  @Override
  public void robotPeriodic() {
    
  }

  
  @Override
  public void disabledInit() {

  }
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
    //Robot.m_Chassis.SetInitPIDVariable();/////////////////////////////////////
    //Robot.m_Chassis.InitGryo();//Robot.m_Chassis.SetInitAngle(Robot.m_Chassis.ReadAngle());//////////////////////////////

    //m_autonomousCommand = new AutonomousCommand();
    //m_autonomousCommand.start();

    //m_autonomousCommand = m_chooser.getSelected();
    //if (m_autonomousCommand != null) {
      //m_autonomousCommand.start();
    //}
    
  }
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }


  @Override
  public void teleopInit() {
    //Robot.m_Chassis.SetInitPIDVariable();//////////////////////////////////
    //Robot.m_Chassis.InitGryo();//Robot.m_Chassis.SetInitAngle(Robot.m_Chassis.ReadAngle());
    //if (m_autonomousCommand != null) {
    //  m_autonomousCommand.cancel();
    //}
  }
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
