// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public static final class USB {

        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xBoxController = 2;
        public static final int testController = 4;
        
      }

      public static final class CAN {

        public static final int pigeon = 9;
    
        public static final int frontLeftCanCoder = 10;
        public static final int frontRightCanCoder = 11;
        public static final int backLeftCanCoder = 12;
        public static final int backRightCanCoder = 13;
    
        public static final int frontLeftDriveMotor = 20;
        public static final int frontLeftTurnMotor = 21;
        public static final int frontRightDriveMotor = 22;
        public static final int frontRightTurnMotor = 23;
        public static final int backLeftDriveMotor = 24;
        public static final int backLeftTurnMotor = 25;
        public static final int backRightDriveMotor = 26;
        public static final int backRightTurnMotor = 27;
      }

      public static final class SwerveDrive {
        public static final double kTrackWidth = Units.inchesToMeters(30);
        public static final double kWheelBase = Units.inchesToMeters(30);

        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
          };

          public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(16);
        public static final double kMaxRotationRadiansPerSecond = Math.PI * 1.5;
        public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 1.5;
    
        public static final double kP_X = 0.2;
        public static final double kD_X = 0;
        public static final double kP_Y = 0.2;
        public static final double kD_Y = 0;
        public static final double kP_Theta = 8;
        public static final double kD_Theta = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }

  public static final class SwerveModule {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 12.8;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
    public static final int kNeoCPR = 42;
    public static final int kCANCoderCPR = 4096; // Figure this out for Neo Motors.

    public static final DCMotor kDriveGearbox = DCMotor.getNEO(1);
    public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);

    public static final double kDriveRevToMeters =
            ((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio);
    public static final double kDriveRpmToMetersPerSecond =
            kDriveRevToMeters / 60.0;
    public static final double kTurnRotationsToDegrees =
            360.0 / kTurningMotorGearRatio;
    public static final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }
}





/*public static int shooterL = 6;
public static int shooterR = 3;
public static int shooterB = 8;
public static int intake = 7;
public static int climberL = 9;
public static int climberR = 10;
public static int controller = 6;
public static int operator = 5;
*/