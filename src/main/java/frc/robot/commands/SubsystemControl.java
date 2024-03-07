package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class SubsystemControl {
    private SubsystemControl() {}

    public static Command joystickDrive(
        Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier
    ) {
        return Commands.run(
            () -> {
                double linearMagnitude = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                double omega = omegaSupplier.getAsDouble();

                linearMagnitude = linearMagnitude * linearMagnitude;
                omega = Math.copySign(omega * omega, omega);

                Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();
                
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMeterPerSecond(), 
                        linearVelocity.getY() * drive.getMaxLinearSpeedMeterPerSecond(),
                        omega * drive.getMaxAngularSpeedRadiansPerSecond(),
                        drive.getRotation()));
        }, drive);
    }
}
