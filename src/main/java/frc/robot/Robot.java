// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveForwardAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandSwerveDrivetrain m_drivetrain;
  public RobotContainer m_RobotContainer = new RobotContainer();

  public Robot() {
    m_drivetrain = m_RobotContainer.drivetrain;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }


  @Override
  public void disabledInit() {}
  

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_RobotContainer.setAutonomous();
    m_autonomousCommand = new DriveForwardAuto(m_drivetrain);

     if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
     }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_RobotContainer.setTeleop();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
