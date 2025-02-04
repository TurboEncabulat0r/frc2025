package frc.robot;
import edu.wpi.first.math.util.Units;

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

  
  public static final class Climber {
    public static final int MOTOR_ID = 6;
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
}