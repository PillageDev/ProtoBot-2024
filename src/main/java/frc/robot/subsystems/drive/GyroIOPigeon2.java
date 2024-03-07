package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.GyroConstants;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class GyroIOPigeon2 implements GyroIO {
    public final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(GyroConstants.kGyroCANid); //TODO: Get can bus id
    public static final boolean INVERTED = true; //TODO: check

    public GyroIOPigeon2() {
        pigeon.reset();
    }

    public void resetYaw() {
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);

        inputs.connected = true;
        inputs.realYawPosition = Rotation2d.fromDegrees(ypr[0] * (INVERTED ? -1.0 : 1.0)); // ypr[0] = yaw
        inputs.yawPosition = inputs.realYawPosition.minus(inputs.yawOffset);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getRate());
        inputs.pose = new Rotation3d(Math.toRadians(ypr[0]), Math.toRadians(ypr[1]), Math.toRadians(ypr[2]));
    }
}
