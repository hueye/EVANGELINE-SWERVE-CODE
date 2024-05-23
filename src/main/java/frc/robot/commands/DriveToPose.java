// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends Command {
  Drivetrain drivetrain;
  Pose2d endPose;
  PIDController rotationController;
  PIDController xController;
  PIDController yController;
  /** Creates a new DriveToPose. */
  public DriveToPose(Drivetrain drivetrain, Pose2d endPose) {
    this.drivetrain = drivetrain;
    this.endPose = endPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    rotationController = new PIDController(0.1, 0, 0);
    xController = new PIDController(1, 0, 0);
    yController = new PIDController(1, 0, 0);

    rotationController.enableContinuousInput(0, 360);
  }

  private double clamp(double min, double desiredClamp, double max) {
    return Math.min(Math.max(min, desiredClamp), max);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset();
    xController.reset();
    yController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = drivetrain.getPose().getTranslation();
    
    double omega = rotationController.calculate(drivetrain.getRotation2d().getDegrees(), endPose.getRotation().getDegrees());
    double xSpeed = xController.calculate(translation.getX(), endPose.getX());
    double ySpeed = yController.calculate(translation.getY(), endPose.getY());

    drivetrain.drive(clamp(-2, xSpeed, 2), clamp(-2, ySpeed, 2), clamp(-Math.PI, omega, Math.PI), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Translation2d currentPose = drivetrain.getPose().getTranslation();
    Rotation2d currentRot = drivetrain.getRotation2d();

    double distance = Math.abs(currentPose.getDistance(endPose.getTranslation()));

    double rot = Math.abs(currentRot.minus(endPose.getRotation()).getDegrees());

    if(distance <= .1 && rot <= 2) {
      return true;
    } else {
      return false;
    }
  }
}
