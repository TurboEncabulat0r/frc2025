package frc.robot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final double DRIVE_MOTOR_VOLTAGE_COMP = 12;
    public static final double SLOW_MODE_MOVE = 0.5;
    public static final double SLOW_MODE_TURN = 0.6;
  }

  public enum ScoreLevel {
    L1, L2, L3, L4, None;
    }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_CORAL_OUT = -.4;
    public static final double ROLLER_ALGAE_IN = -1;
    public static final double ROLLER_ALGAE_OUT = 0.8;
    public static final double ROLLER_CORAL_STACK = -1;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 6;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.1;
    public static final double ARM_SPEED_UP = -0.1;
    public static final double ARM_HOLD_DOWN = 0.005;
    public static final double ARM_HOLD_UP = -0.07;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 7;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -0.5;
    public static final double CLIMBER_SPEED_UP = 0.5;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static final class CoralArmSubsystem {
    public static enum ArmPosition {
        BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
        HORIZONTAL(0),
        L1(0),
        L2(Units.degreesToRadians(55)), // reef angle
        L3(Units.degreesToRadians(55)),
        L4(1.033),
        TOP(Math.PI / 2.0);

        public final double value;

        private ArmPosition(double value) {
            this.value = value;
        }
    }

    public static final double MOTION_LIMIT = -0.7;
    public static final double SCORING_MOVEMENT = -0.8;

    public static final int MOTOR_ID = 12;
    public static final boolean MOTOR_INVERTED = true;

    public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
    public static final double GEARING = 40.0; // TODO
    public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
    public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
    public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
    public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

    public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
    public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

    public static final int CURRENT_LIMIT = 50;

    public static final double kP = 10; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO
    public static final double kS = 0.017964; // TODO
    public static final double kG = 0.321192; // TODO
    public static final double kV = 0.876084;// TODO
    public static final double kA = 0.206676;// TODO
    public static final double TOLERANCE = 0.02;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
}

public static final class Intake {
    public static final int MOTOR_ID = 13;
    public static final boolean MOTOR_INVERTED = true;
    public static final int CURRENT_LIMIT = 60;
}

  
  public static final class Climber {
    public static final int MOTOR_ID = 4;
    public static final boolean MOTOR_INVERTED = false;
    public static final int CURRENT_LIMIT = 60;

    public static final double MIN_POSITION_METERS = 0.0;
    public static final double MAX_POSITION_METERS = 1.0; // TODO

    public static final double GEARING = 64.0;
    public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
    public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
    public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
    public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;

    }


    public static final class Arm {
        public static enum ArmPosition {
            BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
            HORIZONTAL(0),
            L1(0),
            L2(Units.degreesToRadians(55)), // reef angle
            L3(Units.degreesToRadians(55)),
            L4(1.033),
            TOP(Math.PI / 2.0);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = -0.7;
        public static final double SCORING_MOVEMENT = -0.8;

        public static final int MOTOR_ID = 12;
        public static final boolean MOTOR_INVERTED = true;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 40.0; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

        public static final int CURRENT_LIMIT = 50;

        public static final double kP = 10; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.321192; // TODO
        public static final double kV = 0.876084;// TODO
        public static final double kA = 0.206676;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }


     public static class ElevatorConstants {
        // MOTOR CAN BUS IDS
        public static final int kLeaderID = 4;
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = .1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        // MAXMOTION CONSTANTS TODO: TUNE
        public static final double kMAXMotionP = 0;
        public static final double kMAXMotionI = 0;
        public static final double kMAXMotionD = 0;
        public static final double kMAXMotionFF = 0;
        public static final double kMAXMotionMaxAcceleration = 0;
        public static final double kMAXMotionMaxVelocity = 0;
        public static final double kMAXMotionAllowedError = 0;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.0; // TODO: TUNE
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -5; // TODO: SET
        public static final double kMaximumRotationLimit = 100; // TODO: SET
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 40;
        public static final int kFreeLimit = 40;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }
}