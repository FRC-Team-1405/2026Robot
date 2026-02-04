package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.ResetMode;   // Import the new global ResetMode
import com.revrobotics.PersistMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

public class Turret extends SubsystemBase{
    private FireControl controllerOfFire;
    private DigitalInput callibrationSwitch; 
    private Transform3d robotToTurret;

    private SparkMax turretMotor;
    private RelativeEncoder turretEncoder;
    private AbsoluteEncoder turretAbsEncoder;
    private SparkClosedLoopController turretMotorController;
    private SparkMaxConfig turretMotorConfig;

    

    public Turret(final int TURRET_MOTOR_ID, final int TURRET_ABS_ENCODER_ID, final int TURRET_CALLI_SWITCH_ID, Transform3d roboToTur) { //TODO
        robotToTurret = roboToTur;
        turretMotor = new SparkMax(TURRET_MOTOR_ID, MotorType.kBrushless);
        turretMotorConfig = new SparkMaxConfig();
        turretMotorConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.Turret.VOLTAGE).smartCurrentLimit(Constants.Turret.CURRENT);
        turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretEncoder = turretMotor.getEncoder();
        turretMotorController = turretMotor.getClosedLoopController();
    }

    public void pointToTarget() { //TODO
        turretMotorController.setSetpoint(0.0, ControlType.kPosition);
    }
}
