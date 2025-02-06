package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("unused")
public class ArmDownCommand extends Command {
  private final ArmSubsystem m_arm;

  private double startEncoderPos;

  public ArmDownCommand(ArmSubsystem arm) {
    m_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    

  }

  @Override
  public void execute() {
    m_arm.runArm(ArmConstants.ARM_SPEED_DOWN);
  }


  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(ArmConstants.ARM_HOLD_DOWN);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}