// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private double m_maxSpeed;

  // Debug with shuffleboard
  // private ShuffleboardTab tab = Shuffleboard.getTab("Angle");
  // private GenericEntry showMeas = tab.add("m_Measure", 0).getEntry();
  // private GenericEntry showSetPt = tab.add("Setpoint", 0).getEntry();
  // private GenericEntry showOutput = tab.add("Output", 0).getEntry();
  // private GenericEntry userSpeed = tab.add("Robot Speed", 0.2)
  //   .withWidget(BuiltInWidgets.kNumberSlider)
  //   .withProperties(Map.of("min", 0, "max", 1)).getEntry();
  // private GenericEntry userKp = tab.add("Kp", 1.0).getEntry();
  // private GenericEntry userKi = tab.add("Ki", 0.0).getEntry();
  // private GenericEntry userKd = tab.add("Kd", 0.0).getEntry();

  // Vision sensor interface
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

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
    SmartDashboard.putData("Angle PID", m_turnControl);
    SmartDashboard.putNumber("MaxSpeed", 0.1);
    // Shuffleboard

    // Shuffleboard.getTab("PID").add("Turn Ctrl", m_turnControl);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnControl.reset();
    
    // double speed = userSpeed.getDouble(0.1);
    // double Kp = userKp.getDouble(1.0);
    // double Ki = userKi.getDouble(0.0);
    // double Kd = userKd.getDouble(0.0);

    m_maxSpeed = SmartDashboard.getNumber("MaxSpeed", 0.1);
    m_drivetrain.setMaxOutput(m_maxSpeed);

    // Check for a visual target (April tag)
    // Is there a valid target?
    // if (tv.getBoolean(false)) {
    //   // Check tag number?
    //   // Assign PID setpoint to tag 
    //   m_setpoint = tx.getDouble(0.0);
    // } else {
    //   m_setpoint = 0; // Assign zero (no turn) to setpoint
    // }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_measurement = m_drivetrain.getHeading();
    m_output = MathUtil.clamp(m_turnControl.calculate(m_measurement, m_setpoint), -1.0, 1.0);
    m_drivetrain.arcadeDrive(0, -m_output);

    SmartDashboard.putNumber("Setpoint", m_setpoint);
    SmartDashboard.putNumber("Measurement", m_measurement);
    SmartDashboard.putNumber("Output", m_output);
    SmartDashboard.putNumber("Angle Error", m_turnControl.getPositionError());

    // showMeas.setDouble(m_measurement);
    // showSetPt.setDouble(m_setpoint);
    // showOutput.setDouble(m_output);
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
    // return false;
  }
}
