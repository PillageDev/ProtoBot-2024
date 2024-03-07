package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.constants.SDSMK4L4Constants;

public class ModuleIOKrakenX60 implements ModuleIO {
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;

    private static final double DRIVE_GEAR_RATIO = SDSMK4L4Constants.driveGearRatio;
    private static final double TURN_GEAR_RATIO = SDSMK4L4Constants.angleGearRatio;

    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    private static final double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
    private static final String CANBUS_ID = "Canivore";

    private final VoltageOut voltageOutCommand = new VoltageOut(0.0);
    private static VelocityVoltage velocityVoltageCommand = new VelocityVoltage(0.0).withSlot(0);

    private double absoluteEncoderOffset = 0.0;

    public ModuleIOKrakenX60(int index) {
        switch (index) { //TODO: Ensure CAN ids are correct
            case 0: // FL
                driveTalon = new TalonFX(0, CANBUS_ID);
                turnTalon = new TalonFX(1, CANBUS_ID);
                cancoder = new CANcoder(2, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: FIND THIS VALUE
                break;
            case 1: // FR
                driveTalon = new TalonFX(3, CANBUS_ID);
                turnTalon = new TalonFX(4, CANBUS_ID);
                cancoder = new CANcoder(5, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: FIND THIS VALUE
                break;
            case 2: // BL
                driveTalon = new TalonFX(6, CANBUS_ID);
                turnTalon = new TalonFX(7, CANBUS_ID);
                cancoder = new CANcoder(8, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: FIND THIS VALUE
                break;
            case 3: // BR
                driveTalon = new TalonFX(9, CANBUS_ID);
                turnTalon = new TalonFX(10, CANBUS_ID);
                cancoder = new CANcoder(11, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: FIND THIS VALUE
                break;
            default:
                throw new IllegalArgumentException("Invalid module index: " + index);
        }

        driveTalon.getConfigurator().apply(getDriveConfig());
        driveTalon.clearStickyFaults();
        setDriveBrakeMode(true);

        turnTalon.getConfigurator().apply(getTurnConfig());
        setTurnBrakeMode(true);

        cancoder.getConfigurator().apply(getCancoderConfig());

        drivePosition = driveTalon.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnTalon.setPosition(turnAbsolutePosition.getValueAsDouble());
        turnPosition = turnTalon.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            drivePosition, turnPosition, driveVelocity, turnVelocity, driveAppliedVolts, turnAppliedVolts, driveCurrent, turnCurrent);
        driveTalon.optimizeBusUtilization();
        turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(drivePosition, turnPosition, driveVelocity, turnVelocity, driveAppliedVolts, turnAppliedVolts, driveCurrent, turnCurrent);

        inputs.canCoderRotations = cancoder.getAbsolutePosition().getValue();
        inputs.canCoderAngle = Units.rotationsToDegrees(inputs.canCoderRotations);
        
        inputs.drivePositionRotations = drivePosition.getValueAsDouble();
        inputs.driveVelocityRotationsPerSec = driveVelocity.getValueAsDouble();
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnPositionAngle = inputs.turnPosition.getDegrees();
        inputs.turnVelocityRadPerSec = Units.degreesToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
    }

    private TalonFXConfiguration getDriveConfig() {
        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = SDSMK4L4Constants.driveMotorInvert;

        driveConfig.Slot0.kP = SDSMK4L4Constants.driveKP;
        driveConfig.Slot0.kI = SDSMK4L4Constants.driveKI;
        driveConfig.Slot0.kD = SDSMK4L4Constants.driveKD;

        driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        return driveConfig;
    }

    private TalonFXConfiguration getTurnConfig() {
        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.MotorOutput.Inverted = SDSMK4L4Constants.angleMotorInvert;

        turnConfig.Slot0.kP = SDSMK4L4Constants.angleKP;
        turnConfig.Slot0.kI = SDSMK4L4Constants.angleKI;
        turnConfig.Slot0.kD = SDSMK4L4Constants.angleKD;

        turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;

        return turnConfig;
    }

    private CANcoderConfiguration getCancoderConfig() {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SDSMK4L4Constants.canCoderSensorDirection;
        canCoderConfig.MagnetSensor.MagnetOffset = -absoluteEncoderOffset; // TODO: Make sure this is true

        return canCoderConfig;
    }
}
