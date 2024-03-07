package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRotations = 0.0;
        public double driveVelocityRotationsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnPositionAngle = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};

        public double canCoderRotations = 0.0;
        public double canCoderAngle = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(double volts) {}

    public default void setDriveVelocityRPS(double velocity) {}

    public default void setTurnVoltage(double volts) {}

    public default void setDriveBrakeMode(boolean enabled) {}

    public default void setTurnBrakeMode(boolean enabled) {}
}
