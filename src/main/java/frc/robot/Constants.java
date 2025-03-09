package frc.robot;

import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.04;
		public static final double kI = 0.0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kV = 0.11;
		public static final double kA = 0.009;

		public static final double kRotationP = 5; // TODO: tune it
		public static final double kRotationI = 0.0;
		public static final double kRotationD = 0.1; // TODO: tune it
		public static final double kRotationS = 0;
		public static final double kRotationV = 1.9;
		public static final double kRotationA = 0.009;

		public static final double kDriveMaxVoltage = 12;
		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kDriveGearRatio = 6.12;
		public static final double kSteerGearRatio = 150.0 / 7;
		// public static final double kWheelDiameter = Units.inchesToMeters(3.67);
		public static final double kWheelDiameter = Units.inchesToMeters(3.74);
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio;

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

		public static final int kEncoderDepth = 4;
		public static final int kEncoderMeasurementPeriod = 16;
		public static final int kDriveSmartCurrentLimit = 20;
		public static final int kDrivePeakCurrentLimit = kDriveSmartCurrentLimit + 15;
		public static final int kSteerSmartCurrentLimit = 20;
		public static final int kSteerPeakCurrentLimit = kSteerSmartCurrentLimit + 15;
		public static final int kSteerSecondaryCurrentLimit = kSteerSmartCurrentLimit + 15;
		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1;

		public static final double kTeleopDriveMaxSpeed = 5.0; // 5 meters per second
		public static final double kTeleopTurnMaxAngularSpeed = Math.toRadians(360); // 1 rotation per second

		public static final double kDriveMaxSpeed = 5.0; // 5 meters per second
		public static final double kDriveMinSpeed = 0.2; // 0.2 meters per second
		public static final double kTurnMaxAngularSpeed = Math.toRadians(360); // 1 rotation per second
		public static final double kTurnMinAngularSpeed = Math.toRadians(0); // 0 degree per second

		// DriveCommand.java Constants
		public static final double kDriveP = 5;
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxAcceleration = 0.75 * kDriveMaxSpeed; // kDriveMaxSpeed in 1.5 sec

		public static final double kTurnP = 5;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0.1;
		public static final double kTurnMaxAcceleration = 2 * kTurnMaxAngularSpeed; // kTurnMaxAngularSpeed in 0.5
	}

	public static final class ElevatorConstants {
		public static final int kElevatorMotorPort = 26;
		public static final int kSmartCurrentLimit = 60; // TODO: 45
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kP = 6.0; // 1.1
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kS = 0.05631;
		public static final double kG = 0.43; // 0.22876
		public static final double kV = 5.3794;
		public static final double kA = 0.74041;
		public static final double kGearRatio = 10;

		/**
		 * 24 teeth, 5 mm pitch, one rotation moves 120 mm, 2 stage cascading elevator
		 * means total height change is 240 mm.
		 */
		public static final double kMetersPerPulleyRotation = (24.0 * 5 * 2 / 1000);
		/**
		 * <pre>
		 * 				   1 pulley rotation	 pulley circumference
		 * 1 motor rot * --------------------- * --------------------
		 *               kGearRatio motor rots    1 pulley rotation
		 * </pre>
		 */
		public static final double kMetersPerMotorRotation = (1 / kGearRatio)
				* kMetersPerPulleyRotation;
		public static final double kMaxVelocity = 2.75;
		public static final double kMaxAccel = 2.5; // 2.5
		// public static final double kTolerance = 1;
		public static final double kTolerance = 0.01;
		// TODO: During testing make sure these are right
		public static final double kLevelOneHeight = Units.inchesToMeters(3);
		public static final double kLevelTwoHeight = Units.inchesToMeters(8); // TODO: 31.88 from carpet
		public static final double kLevelThreeHeight = Units.inchesToMeters(29); // TODO: 47.63 from carpet (5.5 off?)
		// TODO: does this uhh do anything...? since the max height is supposedly lower?
		// public static final double kLevelFourHeight = Units.inchesToMeters(50);
		public static final double kLevelFourHeight = Units.inchesToMeters(48 + 0.75 + 0.5); // TODO: 72 from carpet
		public static final double kMaxExtension = Units.inchesToMeters(49.5 + 0.75); // TODO: Likely needs to be upped:
		// safety
		// TODO: The amount that the elevator decreases in order to score
		public static final double kClearanceHeight = Units.inchesToMeters(3.5);
		public static final double kToScoreHeightDecrease = Units.inchesToMeters(0);
		public static final double kCoralStationHeight = Units.inchesToMeters(17 + 2); // TODO: Change

		public static final double kAlgaeLevelThreeHeight = Units.inchesToMeters(0.25);
		public static final double kAlgaeLevelTwoHeight = Units.inchesToMeters(16);
	}

	public static final class WristConstants {
		public static final int kWristMotorPort = 27;
		public static final int kSmartCurrentLimit = 20;
		public static final int kSecondaryCurrentLimit = 20;
		public static final int kGrabberAngleLevelFour = 223;
		public static final int kGrabberAngleOthers = 221; // 5 degrees steeper from previous value (215)
		public static final int kGrabberAngleLevelThree = 235;
		public static final double kAlgaeWristHeight = 170;

		public static final double kWristForwardSoftLimit = 274; // Wrist facing down
		public static final double kWristReverseSoftLimit = 90; // Wrist facing up
		public static final double kWristOffset = 0.75;

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.015; // TODO: Optimize
		public static final double kI = 0.0;
		public static final double kD = 0.001; // TODO: Optimize

		public static final double kTolerance = 4; // TODO: Change this

		public static final double kMinElevatorExtension = 0.0;
	}

	public static final class AutoAlignConstants {
		/**
		 * The {@code AprilTagFieldLayout}.
		 */
		public static AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

		/**
		 * The {@code Transform3d} expressing the pose of the first camera relative to
		 * the pose of the robot.
		 */
		public static Transform3d kRobotToCamera1 = new Transform3d(new Translation3d(0.3, 0.0, 0.2),
				new Rotation3d(0, Units.degreesToRadians(-20), 0));

		/**
		 * The {@code Transform3d} expressing the pose of the second camera relative to
		 * the pose of the robot.
		 */
		public static Transform3d kRobotToCamera2 = new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
				new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180)));

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTags = { transform(1.1, 0.0, 180),
				transform(0.60, 0.0, 180) };

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the left of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsLeft = { transform(1.1, 0, 180),
				transform(0.60, -0.165, 180) };

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the right of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsRight = { transform(1.1, 0, 180),
				transform(0.60, 0.215, 180) };

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the left of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsLeftReady = { transform(1.1, -0.5, 180),
				transform(0.60, 0.0, 180) };

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the right of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsRightReady = { transform(1.1, 0.5, 180),
				transform(0.60, 0.0, 180) };

		/**
		 * A {@code Map} storing the distance to travel to score at each scoring level.
		 */
		static Map<Integer, Double> kOffsets = Map.of(1, 0.13, 2, 0.13, 3, 0.13, 4, 0.05);

	}

}
