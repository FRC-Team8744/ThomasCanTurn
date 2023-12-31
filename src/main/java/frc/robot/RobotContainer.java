// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.util.Map;

// !!!alh import static edu.wpi.first.wpilibj.PS4Controller.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// !!!alh import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.DummyCmd;
import frc.robot.commands.TurnAngle;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // ShuffleboardLayout layoutPID = Shuffleboard.getTab("PID")
  // .getLayout("Commands", BuiltInLayouts.kList)
  // // .withSize(2, 2)
  // .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

  // The driver's controller
// !!!alh  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  public final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    // !!!alh -m_driverController.getLeftY(), -m_driverController.getRightX()),
                    -m_driverController.getY(), -m_driverController.getX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    // new JoystickButton(m_driverController, Button.kR1.value) !!!alh
    new JoystickButton(m_driverController, OIConstants.kButtonRightBumper)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.1)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.2)));

    // Stabilize robot to drive straight with gyro when left bumper is held
    // new JoystickButton(m_driverController, Button.kL1.value) !!!alh
    new JoystickButton(m_driverController, OIConstants.kButtonLeftBumper)
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizationP,
                    DriveConstants.kStabilizationI,
                    DriveConstants.kStabilizationD),
                // Close the loop on the turn rate
                m_robotDrive::getTurnRate,
                // Setpoint is 0
                0,
                // Pipe the output to the turning controls
                // !!!alh output -> m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), output),
                output -> m_robotDrive.arcadeDrive(-m_driverController.getY(), output),
                // Require the robot drive
                m_robotDrive));

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kCross.value) !!!alh
    new JoystickButton(m_driverController, OIConstants.kButtonX)
        .onTrue(new TurnToAngle(90, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kCircle.value) !!!alh
    new JoystickButton(m_driverController, OIConstants.kButtonY)
        .onTrue(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(5));

    // Turn to degrees when button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, OIConstants.kButtonA)
        .onTrue(new TurnAngle(-30, m_robotDrive).withTimeout(5));

    // Turn to degrees when button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, OIConstants.kButtonB)
        .onTrue(new TurnAngle(30, m_robotDrive).withTimeout(5));
    
    // Add command buttons for PID debug
    // layoutPID.add(new TurnToAngle(45, m_robotDrive).withTimeout(5));
    // layoutPID.add(new TurnToAngle(-45, m_robotDrive).withTimeout(5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // no auto
    return new InstantCommand();
    // return new DummyCmd();
  }
}
