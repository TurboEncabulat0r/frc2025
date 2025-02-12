package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("unused")
public class ArmDownCommand extends Command {
  private final AlgaeArmSubsystem m_arm;

  private double startEncoderPos;

  public ArmDownCommand(AlgaeArmSubsystem arm) {
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