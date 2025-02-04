package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor;

    public ArmSubsystem () {

    // Set up the arm motor as a brushless motor
    armMotor = new SparkMax(12, MotorType.kBrushless);

    armMotor.setCANTimeout(250);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.voltageCompensation(10);
    armConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
    armConfig.idleMode(IdleMode.kBrake);
    armConfig.inverted(true);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void runArm(double speed){
        armMotor.set(speed);
    }
}