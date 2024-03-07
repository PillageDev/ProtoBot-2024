package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d realYawPosition = new Rotation2d();
        public Rotation2d yawPosition = new Rotation2d();
        public Rotation2d yawOffset = new Rotation2d();
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public Rotation3d pose;
        public double yawVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}
