package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ArmUpCommand extends Command {
  private final AlgaeArmSubsystem m_arm;

  public ArmUpCommand(AlgaeArmSubsystem arm) {
    m_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.runArm(ArmConstants.ARM_SPEED_UP);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(ArmConstants.ARM_HOLD_UP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}