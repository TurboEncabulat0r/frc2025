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

    private Mechanism2d mechanisms = new Mechanism2d(5, 3);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

    @SuppressWarnings("unused")
    private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", Units.inchesToMeters(5.5), 180, 0,
                    new Color8Bit(Color.kWhite)));
    @SuppressWarnings("unused")
    private MechanismLigament2d elevatorBase = root
            .append(new MechanismLigament2d("elevatorBase", Units.inchesToMeters(36), 90, 2,
                    new Color8Bit(Color.kWhite)));
    private MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevatorStage", Units.inchesToMeters(10), 90,
                    4,
                    new Color8Bit(Color.kOrange)));
    private MechanismLigament2d armLigament = elevatorLigament
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270,
                    5,
                    new Color8Bit(Color.kRed)));

    PositionTracker positionTracker = new PositionTracker();

    ElevatorSubsystem elevator = new ElevatorSubsystem();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SpeedModifer = 0.85;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    //  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final RollerSubsystem roller = new RollerSubsystem();
    private final AlgaeArmSubsystem armsubsystem = new AlgaeArmSubsystem();


    //private final CoralArmSubsystem coral = new CoralArmSubsystem(positionTracker, armLigament);
    private final CoralScorer scorer = new CoralScorer();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    //private final CommandXboxController joystickpt2 = new CommandXboxController(1);
    private final CommandGenericHID console = new CommandGenericHID(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * SpeedModifer) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * SpeedModifer) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * SpeedModifer) // Drive counterclockwise with negative X (left)
            )
        );


        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // joystick.R1().whileTrue(new AlgaeInCommand(roller));
        // joystick.R2().whileTrue(new AlgaeOutCommand(roller));
            // 33.3 
        // joystick.L1().whileTrue(new ArmUpCommand(armsubsystem));
        // joystick.L2().whileTrue(new ArmDownCommand(armsubsystem));

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


        joystick.rightTrigger().whileTrue(new AlgaeInCommand(roller));
        joystick.rightBumper().whileTrue(new AlgaeOutCommand(roller));

        // joystick.povUp().whileTrue(new ArmUpCommand(armsubsystem));
        // joystick.povDown().whileTrue(new ArmDownCommand(armsubsystem));

        console.button(9).whileTrue(new ArmUpCommand(armsubsystem));
        console.button(10).whileTrue(new ArmDownCommand(armsubsystem));

        console.button(11).whileTrue(new AlgaeInCommand(roller));
        console.button(12).whileTrue(new AlgaeOutCommand(roller));

        console.button(13).onTrue(Commands.sequence( elevator.setPosition(ElevatorPosition.L_FOUR), Commands.waitSeconds(0.8), Commands.runOnce(() -> scorer.moveCoralScorerToPose(21))));
        console.button(14).onTrue(Commands.sequence( elevator.setPosition(ElevatorPosition.L_THREE), Commands.waitSeconds(0.8), Commands.runOnce(() -> scorer.moveCoralScorerToPose(21))));
        console.button(15).onTrue(Commands.sequence( elevator.setPosition(ElevatorPosition.L_TWO), Commands.waitSeconds(0.8), Commands.runOnce(() -> scorer.moveCoralScorerToPose(21))));
        console.button(16).onTrue(Commands.sequence(Commands.runOnce(() -> scorer.moveCoralScorerToPose(0)), Commands.waitSeconds(0.8),elevator.setPosition(ElevatorPosition.L_ONE)));

        joystick.leftTrigger().onTrue(Commands.runOnce(() -> SpeedModifer = 0.3));
        joystick.leftTrigger().onFalse(Commands.runOnce(() -> SpeedModifer = 0.85));

        console.button(17).onTrue(elevator.setRaw(0.3));
        console.button(17).onFalse(elevator.lockPosition());

        console.button(18).onTrue(elevator.setRaw(-0.3));
        console.button(18).onFalse(elevator.lockPosition());

        console.button(7).onTrue(Commands.runOnce(()-> scorer.setPower(0.3)));
        console.button(7).onFalse(scorer.lockPosition());

        console.button(8).onTrue(Commands.runOnce(()-> scorer.setPower(-0.3)));
        console.button(8).onFalse(scorer.lockPosition());
        // old coral score 
        // console.button(7).onTrue(coral.CMDSetVoltage(2));
        // console.button(7).onFalse(coral.CMDSetVoltage(-0.2));
        // console.button(8).onTrue(coral.CMDSetVoltage(-2));
        // console.button(8).onFalse(coral.CMDSetVoltage(-0.2));

        console.button(19).onTrue(new AlignToReefTagRelative(false, drivetrain));
        console.button(20).onTrue(new AlignToReefTagRelative(true, drivetrain));
        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
