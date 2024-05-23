// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private NetworkTableEntry txEntry;
  private double tx = 0;
  private NetworkTableEntry taEntry;
  private double ta = 0;
  /** Creates a new Vision. */
  public Vision() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  txEntry = table.getEntry("tx");
  taEntry = table.getEntry("ta");
  }

  public double getTx() {
    return tx;
  }

  public double getTa() {
    return ta;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tx = txEntry.getDouble(0);
    ta = taEntry.getDouble(0);
  }
}
