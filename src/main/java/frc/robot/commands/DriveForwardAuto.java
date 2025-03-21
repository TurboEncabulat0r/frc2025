package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardAuto extends Command {
  private final CommandSwerveDrivetrain m_drive;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  
  private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private Boolean finished = false;
  private Timer t = new Timer();
  public DriveForwardAuto(CommandSwerveDrivetrain drivetrain) {
    m_drive = drivetrain;


  }

  @Override
  public void initialize() {
    t.restart();
  }

  @Override
  public void execute() {
    if (t.get() < 2){
        m_drive.setControl(m_request
        .withVelocityX(0.5) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0)); 
    }
    else{
        m_drive.setControl(m_request
        .withVelocityX(0) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0)); 
    }



    // Commands.sequence(Commands.waitSeconds(0.3), Commands.run(() -> m_drive.setControl(m_request
    // .withVelocityX(0) // Drive forward with negative Y(forward)
    // .withVelocityY(0) // Drive left with negative X (left)
    // .withRotationalRate(0))));
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setControl(m_request
    .withVelocityX(0) // Drive forward with negative Y(forward)
    .withVelocityY(0) // Drive left with negative X (left)
    .withRotationalRate(0)); 
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}