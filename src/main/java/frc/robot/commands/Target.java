// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class Target extends Command {
  Drivetrain drivetrain;
  Vision vision;
  PIDController controller;
  PIDController distController;
  /** Creates a new Target. */
  public Target(Drivetrain drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    controller = new PIDController(0.1, 0, 0);
    distController = new PIDController(0.2, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  private double clamp(double min, double desiredClamp, double max) {
    return Math.min(Math.max(min, desiredClamp), max);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double L = controller.calculate(vision.getTx(), 0);
    double pidOutput = distController.calculate(vision.getTa(), 1);

    if(pidOutput < 0) {
      drivetrain.drive(clamp(-1, pidOutput*2, 1), 0, clamp(-Math.PI, L, Math.PI), false);
    } else if (pidOutput > 0) {
      drivetrain.drive(clamp(-1, pidOutput*4, 1), 0, clamp(-Math.PI, L, Math.PI), false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
