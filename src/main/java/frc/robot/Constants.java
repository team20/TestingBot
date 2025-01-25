package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
		public static final double kTriggerDeadzone = .05;

		public static final class Button {
			/** Left middle button */
			public static final int kSquare = 1;
			/** Bottom button */
			public static final int kX = 2;
			/** Right middle button */
			public static final int kCircle = 3;
			/** Top button */
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kLeftTrigger = 7;
			public static final int kRightTrigger = 8;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}
	}

	public static final class DriveConstants {
		// CAN IDs (updated)
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;

		// Make sure these are tuned (can do with SysId)
		public static final double kP = 0.05;
		public static final double kI = 0.0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kV = 0.11;
		public static final double kA = 0.009;

		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kGearRatio = 6.12;
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kGearRatio;

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

		public static final int kEncoderDepth = 4;
		public static final int kEncoderMeasurementPeriod = 16;
		public static final int kSteerSmartCurrentLimit = 60;
		public static final int kSteerPeakCurrentLimit = kSteerSmartCurrentLimit + 15;
		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1;

		// DriveCommand.java Constants
		public static final double kDriveP = 4; // up to 1.0?
		// public static final double kDriveP = 0.4; // up to 1.0?
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxVelocity = 3; // up to 5?
		public static final double kDriveMaxAcceleration = 3; // up to 10?

		public static final double kTurnP = 0.2; // was 0.005 upto 0.2?
		// public static final double kTurnP = 0.02; // was 0.005 upto 0.2?
		public static final double kTurnI = 0; // was 0.003
		public static final double kTurnD = 0; // 0.0
		public static final double kTurnMaxVelocity = 120; // up to 240?
		public static final double kTurnMaxAcceleration = 240; // up to 360?

	}

	/**
	 * The {@code AprilTagFieldLayout}.
	 */
	public static AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	public static final class RobotConstants {

		/**
		 * The {@code Transform3d} expressing the pose of the camera relative to the
		 * pose of the robot.
		 */
		public static Transform3d kRobotToCamera1 = new Transform3d(new Translation3d(0.0, -0.1, 0.2),
				new Rotation3d(0, Units.degreesToRadians(-20), 0));
	}
}