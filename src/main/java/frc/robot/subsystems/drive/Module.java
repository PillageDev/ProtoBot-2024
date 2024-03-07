package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0); //TODO: change
    private static final double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
    public static final double ODOMETRY_FREQUENCY = 250.0;

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null; //Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // In meters / sec
    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
    private double lastPositionMeters = 0.0; // used in delta calcs
    private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        turnFeedback = new PIDController(0.0, 0.0, 0.0);
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void periodic() {
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        if (angleSetpoint != null) {
            io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            if (speedSetpoint != null) {
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());
                double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
                double velRotationsPerSec = velocityRadPerSec;
                io.setDriveVelocityRPS(velRotationsPerSec);
            }
        }

        positionDeltas = new SwerveModulePosition[1];
        for (int i = 0; i < 1; i++) {
            double positionMeters = inputs.drivePositionRotations * WHEEL_CIRCUMFERENCE;
            Rotation2d angle = inputs.turnPosition.plus(turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
            positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
            lastPositionMeters = positionMeters;
        }
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());
        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    public void runCharacterization(double volts) {
        angleSetpoint = new Rotation2d();

        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        angleSetpoint = null;
        speedSetpoint = null;
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    public void setDriveBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }

    public double getPositionMeters() {
        return inputs.drivePositionRotations * WHEEL_CIRCUMFERENCE;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRotationsPerSec * WHEEL_CIRCUMFERENCE;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModulePosition[] getPositionDeltas() {
        return positionDeltas;
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRotationsPerSec;
    }
}
