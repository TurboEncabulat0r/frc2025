// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveForwardAuto;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Timer timer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    timer = new Timer();
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
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_autonomousCommand = null;

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    if (timer.get() < 2) {
      m_robotContainer.drivetrain.applyRequest(() ->
      m_robotContainer.drive.withVelocityX(3) // Drive forward with negative Y (forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withRotationalRate(0));
    } else {
      m_robotContainer.drivetrain.applyRequest(() ->
      m_robotContainer.drive.withVelocityX(3) // Drive forward with negative Y (forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withRotationalRate(0));
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
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
