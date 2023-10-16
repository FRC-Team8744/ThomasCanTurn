// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DummyCmd extends CommandBase {
  /** Creates a new DummyCmd. */
  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry showMeas = tab.add("Measurement", 0).getEntry();
  private GenericEntry showSetPt = tab.add("Setpoint", 0).getEntry();
  private GenericEntry showOutput = tab.add("Output", 0).getEntry();
  private GenericEntry userSpeed = tab.add("Robot Speed", 0.2)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)).getEntry();
  private GenericEntry userKp = tab.add("Kp", 1.0).getEntry();
  private GenericEntry userKi = tab.add("Ki", 0.0).getEntry();
  private GenericEntry userKd = tab.add("Kd", 0.0).getEntry();
  // private GenericEntry userPID = tab.add("PID").withWidget(BuiltInWidgets.kPIDController).getEntry();

  // Vision sensor interface
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  public DummyCmd() {
    // Use addRequirements() here to declare subsystem dependencies.

    // Create slider for max robot speed
    // Shuffleboard.getTab("Drive")
    // .add("Max Speed", 1)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    // .getEntry();

    // Shuffleboard.getTab("PID").add("Turn Ctrl", mySendable);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = userSpeed.getDouble(0.1);
    double Kp = userKp.getDouble(1.0);
    double Ki = userKi.getDouble(0.0);
    double Kd = userKd.getDouble(0.0);

    // Check for a visual target (April tag)
    // Is there a valid target?
    if (tv.getBoolean(false)) {
      // Check tag number?
      // Assign PID setpoint to tag 
      double x = tx.getDouble(0.0);
    } else {
      // Assign zero (no turn) to setpoint
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    showMeas.setDouble(0);
    showSetPt.setDouble(0);
    showOutput.setDouble(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
