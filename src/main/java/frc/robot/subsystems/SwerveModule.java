// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  CANSparkMax driveSparkFlex;
  CANSparkMax turningSparkMax;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turningEncoder;

  SparkPIDController drivePIDController;
  SparkPIDController turningPIDController;

  double chassisAngularOffset = 0;
  SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, double chassisAngularOffset) {

  driveSparkFlex = new CANSparkMax(driveMotorID, MotorType.kBrushless);
  turningSparkMax = new CANSparkMax(turnMotorID, MotorType.kBrushless);
  
  driveSparkFlex.restoreFactoryDefaults(); 
  turningSparkMax.restoreFactoryDefaults(); 

  driveEncoder = driveSparkFlex.getEncoder();
  turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

  drivePIDController = driveSparkFlex.getPIDController();
  turningPIDController = turningSparkMax.getPIDController();

  drivePIDController.setFeedbackDevice(driveEncoder);
  turningPIDController.setFeedbackDevice(turningEncoder);

  driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
  driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR); 
  turningEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  turningEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

  turningEncoder.setInverted(ModuleConstants.invertTurnEncoder);

  turningPIDController.setPositionPIDWrappingEnabled(true);
  turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  
  //set drive motor PID values
  drivePIDController.setP(ModuleConstants.DRIVE_P);
  drivePIDController.setI(ModuleConstants.DRIVE_I);
  drivePIDController.setD(ModuleConstants.DRIVE_D);
  drivePIDController.setFF(ModuleConstants.DRIVE_FF);
  drivePIDController.setOutputRange(ModuleConstants.DRIVE_MIN_OUTPUT, ModuleConstants.DRIVE_MAX_OUTPUT);

  //Set turn motor PID values 
  turningPIDController.setP(ModuleConstants.TURN_P);
  turningPIDController.setI(ModuleConstants.TURN_I);
  turningPIDController.setD(ModuleConstants.TURN_D);
  turningPIDController.setFF(ModuleConstants.TURN_FF);
  turningPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  
//limits and brakes
driveSparkFlex.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
turningSparkMax.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
driveSparkFlex.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

//saves configs of modules in case of brownout
driveSparkFlex.burnFlash();
turningSparkMax.burnFlash();

this.chassisAngularOffset = chassisAngularOffset;
desiredState.angle = new Rotation2d(turningEncoder.getPosition());
driveEncoder.setPosition(0);
  }

  public void setDesiredState(SwerveModuleState desired) {
    desired.angle = desired.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    desiredState = desired;
    desiredState = SwerveModuleState.optimize(desired, Rotation2d.fromRadians(turningEncoder.getPosition()));
    drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    turningPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
  }

  public double getDesiredSpeed() {
    return desiredState.speedMetersPerSecond;
  }

  public double getActualSpeed() {
    return driveEncoder.getVelocity();
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRadians(turningEncoder.getPosition()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
