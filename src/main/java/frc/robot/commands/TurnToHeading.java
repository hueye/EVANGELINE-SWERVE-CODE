// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurnToHeading extends Command {
  Drivetrain drivetrain;
  Rotation2d rotation;
  PIDController controller;
  /** Creates a new TurnToHeading. */
  public TurnToHeading(Drivetrain drivetrain, Rotation2d rotation) {
    this.drivetrain = drivetrain;
    this.rotation = rotation;
    controller = new PIDController(0.1, 0, 0);
    controller.enableContinuousInput(0, 360);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  private double clamp(double min, double desiredClamp, double max) {
    return Math.min(Math.max(min, desiredClamp), max);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double L = controller.calculate(drivetrain.getRotation2d().getDegrees(), rotation.getDegrees());
    drivetrain.drive(0, 0, clamp(-Math.PI, L, Math.PI), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
