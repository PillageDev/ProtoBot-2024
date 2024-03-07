package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SubsystemControl;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOKrakenX60;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  /* Subsystems */
  private final Drive drive;

  /* Controller init */
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(DriverConstants.kDriverControllerPort);

  public RobotContainer() {
    drive = new Drive(new GyroIOPigeon2(), new ModuleIOKrakenX60(0), new ModuleIOKrakenX60(1), new ModuleIOKrakenX60(2), new ModuleIOKrakenX60(3));

    configureBindings();
  }

  private void configureBindings() {
    /* Drive command */
    drive.setDefaultCommand(
      SubsystemControl.joystickDrive(
        drive, 
        () -> -m_driverController.getRightX(), 
        () -> -m_driverController.getRightY(), 
        () -> m_driverController.getLeftX()));

    /* Brake commands */
    m_driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    /* Reset gyro */
    m_driverController.b().onTrue(Commands.runOnce(drive::resetRotation, drive).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
