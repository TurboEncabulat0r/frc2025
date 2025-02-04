package frc.robot.commands;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class AlgaeInCommand extends Command {
  private final RollerSubsystem m_roller;

  public AlgaeInCommand(RollerSubsystem roller) {
    m_roller = roller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_roller.runRoller(RollerConstants.ROLLER_ALGAE_IN);
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    m_roller.runRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}