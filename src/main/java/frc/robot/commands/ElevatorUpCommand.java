package frc.robot.commands;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorUpCommand extends Command {
  private final ElevatorSubsystem elevator;
  private final double voltage;
  
  public ElevatorUpCommand(ElevatorSubsystem e, double v) {
    elevator = e;
    voltage = v;
    addRequirements(e);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    elevator.setVoltage(10.0);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}