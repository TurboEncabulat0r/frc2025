// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandSwerveDrivetrain m_drivetrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    //  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.RobotCentric m_driverequest = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private Timer timer;

  public Robot() {
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
      m_drivetrain.applyRequest(() ->
      m_driverequest.withVelocityX(3) // Drive forward with negative Y (forward)
          .withVelocityY(0) // Drive left with negative X (left)
          .withRotationalRate(0));
    } else {
      m_drivetrain.applyRequest(() ->
      m_driverequest.withVelocityX(0) // Drive forward with negative Y (forward)
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
