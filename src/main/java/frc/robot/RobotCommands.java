package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.CoralSim.CoralSimLocation;
import frc.robot.Constants.ScoreLevel;
public class RobotCommands {
            

    public static ScoreLevel lastScore = ScoreLevel.None;

    public static Command prepareCoralScoreCommand(ScoreLevel level, ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command autoPrepareCoralScoreCommand(ScoreLevel level, ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        Commands.waitSeconds(0.5).andThen(arm.moveToPositionCommand(() -> armPosition)).asProxy(),
                        Commands.waitSeconds(0)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command scoreCoralCommand(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(
                        ScoreLevel.L1, Commands.parallel(
                                // moves robot bacwards ?? not needed??
                                //drivetrain.moveVoltageTimeCommand(4, 0.5),
                                elevator.movePositionDeltaCommand(() -> Constants.Elevator.SCORING_MOVEMENT)
                                        .asProxy())),
                Map.entry(
                        ScoreLevel.L2,
                        Commands.parallel(
                                arm.movePositionDeltaCommand(() -> Constants.CoralArmSubsystem.SCORING_MOVEMENT).asProxy())),
                Map.entry(
                        ScoreLevel.L3,
                        Commands.parallel(
                                arm.movePositionDeltaCommand(() -> Constants.CoralArmSubsystem.SCORING_MOVEMENT).asProxy())),
                Map.entry(
                        ScoreLevel.L4,
                        Commands.parallel(
                                arm.movePositionDeltaCommand(() -> Constants.CoralArmSubsystem.SCORING_MOVEMENT).asProxy(),
                                Commands.waitSeconds(0.5)
                                        .andThen(
                                                elevator.movePositionDeltaCommand(
                                                        () -> Constants.Elevator.SCORING_MOVEMENT))
                                        .asProxy())),
                Map.entry(
                        ScoreLevel.None,
                        Commands.none()));

        return Commands.select(commandMap, () -> lastScore);
    }

    public static Command prepareIntakeCoralCommand(ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()));
    }

    public static Command intakeCoralCommand(ElevatorSubsystem elevator, CoralArmSubsystem arm, CoralSim coralSim) {
        return Commands.sequence(
                prepareIntakeCoralCommand(elevator, arm),
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                elevator.movePositionDeltaCommand(() -> 0.31).asProxy().alongWith(
                        Commands.waitSeconds(0.1).andThen(coralSim.setLocationCommand(CoralSimLocation.CLAW))),
                Commands.parallel(
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy()),
                        arm.moveToPositionCommand(() -> ArmPosition.TOP).asProxy()));
    }

    public static Command intakeIntoScoreCommand(ScoreLevel level, ElevatorSubsystem elevator, CoralArmSubsystem arm, CoralSim coralSim) {
        return Commands.sequence(
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                autoPrepareCoralScoreCommand(level, elevator, arm).alongWith(
                        Commands.waitSeconds(0.1).andThen(coralSim.setLocationCommand(CoralSimLocation.CLAW))));
    }

    public static Command prepareAlgaeL2RemoveCommand(ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L2).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    public static Command prepareAlgaeL3RemoveCommand(ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L3).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    public static Command algaeRemoveCommand(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, CoralArmSubsystem arm) {
        return Commands.sequence(
                Commands.parallel(
                        // moves robot bacwards ?? not needed??
                        //drivetrain.moveVoltageTimeCommand(-2, 0.5),
                        elevator.movePositionDeltaCommand(() -> -0.06).asProxy()));
    }
}
