// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnAngle extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final PIDController m_turnControl;
  private double m_setpoint;
  private double m_measurement;
  private double m_output;

  // Debug with shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("Angle");
  private GenericEntry showMeas = tab.add("Measurement", 0).getEntry();

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnAngle(double targetAngleDegrees, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drive;
    addRequirements(m_drivetrain);
    m_setpoint = targetAngleDegrees;
    
    m_turnControl = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
    m_turnControl.enableContinuousInput(-180, 180);
    m_turnControl.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

    // Send PID controller data to dashboard
    // ShuffleboardTab tab = Shuffleboard.getTab("Angle");
    // Shuffleboard.selectTab("Angle");
    // NetworkTableEntry dashMeasurement = tab.add("Measurement", 0).getEntry();
    // SmartDashboard.putData("Angle PID", m_turnControl);
    // Shuffleboard
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnControl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_measurement = m_drivetrain.getHeading();
    m_output = MathUtil.clamp(m_turnControl.calculate(m_measurement, m_setpoint), -1.0, 1.0);
    m_drivetrain.arcadeDrive(0, -m_output);

    SmartDashboard.putNumber("Setpoint", m_setpoint);
    SmartDashboard.putNumber("Measurement", m_measurement);
    showMeas.setDouble(m_measurement);
    SmartDashboard.putNumber("Output", m_output);
    SmartDashboard.putNumber("Angle Error", m_turnControl.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when goal is reached
    return m_turnControl.atSetpoint();
  }
}
