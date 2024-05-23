// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
// Controllers
  public static class OIConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static class GlobalConstants {
    // Pneumatics
    public static final int pneumaticsID = 14;
    // Gyro ID
    public static final int pigeonID = 9;
  }

  // DriveTrain
  public static final class DriveConstants {

    // Drive parameters
    public static final double maxSpeedMPS = 3.0;
    public static final double maxAngularSpeed = 2 * Math.PI; // Radians per sec
    public static final double slowDriveScalingConstant = .5; // Constant drive speed is multiplied by during slow drive
    public static final double wheelBaseRadius = 0.43;


    // chassis config
    public static final double trackWidth = Units.inchesToMeters(26); // Distance between centers of right and left
                                                                      // wheels on robot
    public static final double wheelBase = Units.inchesToMeters(26); // Distance between front and back wheels on robot

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));

    // angular offsets of modules from chassis
    public static final double flChassisOffset = -Math.PI / 2;
    public static final double frChassisOffset = 0.0;
    public static final double blChassisOffset = Math.PI;
    public static final double brChassisOffset = Math.PI / 2;

    // SPARK CAN IDS
    /* driving motor ids */
    public static final int flDriveID = 1;
    public static final int blDriveID = 7;
    public static final int frDriveID = 3;
    public static final int brDriveID = 5;

    /* turning motors ids */
    public static final int flTurnID = 2;
    public static final int blTurnID = 8;
    public static final int frTurnID = 4;
    public static final int brTurnID = 6;

    public static final boolean reverseGyro = false;

    // slew stuff constants
    public static final double directionSlewRate = 1.2; // radians per second
    public static final double magnitudeSlewRate = 3; //1.8; // percent per second (1 = 100%)
    public static final double rotationalSlewRate = 2.0; // percent per second (1 = 100%)
  }

  public static final class NeoMotorConstants {
    public static final double NEOFreeSpeed = 5676;
  }

  public static final class ModuleConstants {
    /* pinion gear teeth */
    public static final int driveMotorTeeth = 14;

    public static final boolean invertTurnEncoder = true;

    // Calculations for drive motor conversion factors and feed forwards
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEOFreeSpeed / 60;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3); //2.925 on well worn tread 2.935 worked new tread // new test 3.3435311
    public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (driveMotorTeeth * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFRENCE_METERS)
        / DRIVE_MOTOR_REDUCTION;

    public static final double DRIVE_ENCODER_POS_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVE_MOTOR_REDUCTION; // meters
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURN_ENCODER_POS_FACTOR = (2 * Math.PI); // radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    public static final double TURN_ENCODER_POS_MIN_INPUT = 0; // radians
    public static final double TURN_ENCODER_POS_MAX_INPUT = TURN_ENCODER_POS_FACTOR; // radians

    // drive motor PID
    public static final double DRIVE_P = 0.04;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
    public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS; // 1/6.31562329844;
    public static final double DRIVE_MIN_OUTPUT = -1;
    public static final double DRIVE_MAX_OUTPUT = 1;

    // turn motor PID
    public static final double TURN_P = 1;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
    public static final double TURN_FF = 0;
    public static final double TURN_MIN_OUTPUT = -1;
    public static final double TURN_MAX_OUTPUT = 1;

    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // amps
    public static final int TURN_MOTOR_CURRENT_LIMIT = 20; // amps
  }

}
