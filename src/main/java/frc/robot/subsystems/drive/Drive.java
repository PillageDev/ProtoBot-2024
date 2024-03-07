package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LocalADStarAK;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5); //TODO: Find this value
    private static final double TRACK_WIDTH_X = Units.inchesToMeters(21.73); //TODO: Find this value
    private static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.73); //TODO: Find this value
    private static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // Fl, Fr, Bl, Br

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Pose2d pose = new Pose2d();
    private Rotation2d lastGyroRotation = new Rotation2d();

    public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(br, 0);
        modules[1] = new Module(bl, 1);
        modules[2] = new Module(fr, 2);
        modules[3] = new Module(fl, 3);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(
                        MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
                () ->
                        DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        for (var module : modules) {
            module.updateInputs();
        }

        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) { // Prevent from moving when disabled
            for (var module : modules) {
                module.stop();
            }

            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState() {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState() {});
        }

        int deltaCount = Integer.MAX_VALUE;

        for (var module : modules) {
            deltaCount = Math.min(deltaCount, module.getPositionDeltas().length);
        }

        for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
            SwerveModulePosition[] wheelDeltaas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                wheelDeltaas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
            }

            var twist = kinematics.toTwist2d(wheelDeltaas);
            if (gyroInputs.connected) {
                Rotation2d gyroRotation = gyroInputs.yawPosition;
                twist = new Twist2d(
                        twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians() / deltaCount
                );
            }

            pose = pose.exp(twist);
        }
        lastGyroRotation = gyroInputs.yawPosition;

    }

    public void resetRotation() {
        gyroInputs.yawOffset = gyroInputs.realYawPosition;
        var currentPose = getPose();
        setPose(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        Logger.recordOutput("Drive/Speeds", speeds);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    public void runFrontWheelDrive(double speedMetersPerSecond, double angle, boolean brake, double maxTurningAngle) {
        SwerveModuleState frontWheels = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(0).minus(Rotation2d.fromDegrees(angle * maxTurningAngle)));
        SwerveModuleState backWheels = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(0));
        SwerveModuleState[] setpointStates = {
                SwerveModuleState.optimize(frontWheels, new Rotation2d()),
                SwerveModuleState.optimize(frontWheels, new Rotation2d()),
                SwerveModuleState.optimize(backWheels, new Rotation2d()),
                SwerveModuleState.optimize(backWheels, new Rotation2d())
        };
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);

            if (brake) {
                modules[i].setDriveBrakeMode(true);
                continue;
            }

            modules[i].setDriveBrakeMode(false);
        }

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
    }

    // Stops and turns modules into X formation. They return to normal when runVelocity is called (with non-zero speeds)
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void runCharacterizationVolts(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    public double getCharacterizationVelocity() {
        double driveVelocityAvg = 0;
        for (var module : modules) {
            driveVelocityAvg += module.getCharacterizationVelocity();
        }
        return driveVelocityAvg / 4;
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return pose;
    }

    public Rotation2d getRotation() {
        return gyroInputs.yawPosition;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public double getMaxLinearSpeedMeterPerSecond() {
        return MAX_LINEAR_SPEED;
    }

    public double getMaxAngularSpeedRadiansPerSecond() {
        return MAX_ANGULAR_SPEED;
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
        };
    }
}
