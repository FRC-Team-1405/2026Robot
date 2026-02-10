package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;   // Import the new global ResetMode
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret extends SubsystemBase{
    private FireControl controllerOfFire;
    private Transform3d robotToTurret;
    private double numRotations;

    private DigitalInput callibrationSwitch; 
    private DigitalInputsConfigs callibrationConfig;

    private SparkMax turretMotor;
    private RelativeEncoder turretEncoder;
    private SparkClosedLoopController turretMotorController;
    private SparkMaxConfig turretMotorConfig;

    public Turret(final int TURRET_MOTOR_ID, final int TURRET_ABS_ENCODER_ID, final int TURRET_CALLI_SWITCH_ID, Transform3d roboToTur) { //TODO
        robotToTurret = roboToTur;
        turretMotor = new SparkMax(TURRET_MOTOR_ID, MotorType.kBrushless);
        callibrationSwitch = new DigitalInput(TURRET_CALLI_SWITCH_ID);
        // callibrationConfig
            
        turretMotorConfig = new SparkMaxConfig();
        turretMotorConfig
            .idleMode(IdleMode.kBrake).voltageCompensation(Constants.Turret.VOLTAGE).smartCurrentLimit(Constants.Turret.CURRENT);
        turretMotorConfig.closedLoop.pid(0.01, 0, 0.001); //TODO    
        turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretEncoder = turretMotor.getEncoder();
        turretMotorController = turretMotor.getClosedLoopController();

    }

    public double getNumRotations() {
        return numRotations;
    }

    private void pointToTarget(Rotation2d targetAngle) {
        numRotations = turretEncoder.getPosition() - targetAngle.getRotations() * Constants.Turret.TURRET_GEAR_RATIO_IO;
        turretMotorController.setSetpoint(numRotations, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", turretEncoder.getPosition());
        pointToTarget(controllerOfFire.getCurrentTarget());
    }
}