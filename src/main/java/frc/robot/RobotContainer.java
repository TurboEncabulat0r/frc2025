// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeInCommand;
import frc.robot.commands.AlgaeOutCommand;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.DriveForwardAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.ScoreLevel;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.util.Units;


@SuppressWarnings("unused")
public class RobotContainer {

    // subsystems
    private final RollerSubsystem roller = new RollerSubsystem();
    private final AlgaeArmSubsystem armsubsystem = new AlgaeArmSubsystem();
    private final CoralScorer scorer = new CoralScorer();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    
    // controls
    private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandGenericHID console = new CommandGenericHID(1);

    // swerve settings
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SpeedModifer = 0.3;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public RobotContainer() {
        configureBindings();
    }

    public void setTeleop(){
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * SpeedModifer) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * SpeedModifer) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * SpeedModifer) // Drive counterclockwise with negative X (left)
            )
        );
    }

    public void setAutonomous(){
        drivetrain.setDefaultCommand(null);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.povUp().onTrue(new DriveForwardAuto(drivetrain));


        joystick.rightTrigger().whileTrue(new AlgaeInCommand(roller));
        joystick.rightBumper().whileTrue(new AlgaeOutCommand(roller));

        joystick.leftTrigger().onFalse(Commands.runOnce(() -> SpeedModifer = 0.3));
        joystick.leftTrigger().onTrue(Commands.runOnce(() -> SpeedModifer = 0.85));

        // xbox controller arm control
        // joystick.povUp().whileTrue(new ArmUpCommand(armsubsystem));
        // joystick.povDown().whileTrue(new ArmDownCommand(armsubsystem));

        // console.button(9).whileTrue(new ArmUpCommand(armsubsystem));
        // console.button(10).whileTrue(new ArmDownCommand(armsubsystem));

        // console.button(11).whileTrue(new AlgaeInCommand(roller));
        // console.button(12).whileTrue(new AlgaeOutCommand(roller));

        console.button(13).onTrue(Commands.sequence( elevator.setPosition(ElevatorPosition.L_FOUR), Commands.waitSeconds(0.8), Commands.runOnce(() -> scorer.moveCoralScorerToPose(21))));
        console.button(14).onTrue(Commands.sequence( elevator.setPosition(ElevatorPosition.L_THREE), Commands.waitSeconds(0.8), Commands.runOnce(() -> scorer.moveCoralScorerToPose(21))));
        console.button(15).onTrue(Commands.sequence( elevator.setPosition(ElevatorPosition.L_TWO), Commands.waitSeconds(0.8), Commands.runOnce(() -> scorer.moveCoralScorerToPose(21))));
        console.button(16).onTrue(Commands.sequence(Commands.runOnce(() -> scorer.moveCoralScorerToPose(0)), Commands.waitSeconds(0.8),elevator.setPosition(ElevatorPosition.L_ONE)));

        console.button(17).onTrue(elevator.setRaw(0.3));
        console.button(17).onFalse(elevator.lockPosition());

        console.button(18).onTrue(elevator.setRaw(-0.3));
        console.button(18).onFalse(elevator.lockPosition());

        console.button(7).onTrue(Commands.runOnce(()-> scorer.setPower(0.3)));
        console.button(7).onFalse(scorer.lockPosition());

        console.button(8).onTrue(Commands.runOnce(()-> scorer.setPower(-0.3)));
        console.button(8).onFalse(scorer.lockPosition());

        //console.button(19).onTrue(new AlignToReefTagRelative(false, drivetrain));
        //console.button(20).onTrue(new AlignToReefTagRelative(true, drivetrain));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
