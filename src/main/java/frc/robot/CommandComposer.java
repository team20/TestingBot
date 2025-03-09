package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PathDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static PoseEstimationSubsystem m_poseEstimationSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_poseEstimationSubsystem = poseEstimationSubsystem;
	}

	public static Command get3ScoreNorth() {
		return select(get3ScoreNorthRed(4), get3ScoreNorthBlue(4));
	}

	public static Command get3ScoreSouth() {
		return select(get3ScoreSouthRed(4), get3ScoreSouthBlue(4));
	}

	private static Command get3ScoreNorthBlue(int level) {
		return get3ScoreOptimized(
				toTag(20, level, kRobotToTagsRight), toTag(19, level, kRobotToTagsRight),
				toTag(19, level, kRobotToTagsLeft),
				13, kRobotToTagsRightReady);
	}

	private static Command get3ScoreNorthRed(int level) {
		return get3ScoreOptimized(
				toTag(9, level, kRobotToTagsLeft), toTag(8, level, kRobotToTagsLeft),
				toTag(8, level, kRobotToTagsRight),
				2, kRobotToTagsLeftReady);
	}

	private static Command get3ScoreSouthBlue(int level) {
		return get3ScoreOptimized(
				toTag(22, level, kRobotToTagsLeft), toTag(17, level, kRobotToTagsLeft),
				toTag(17, level, kRobotToTagsRight),
				12, kRobotToTagsLeftReady);
	}

	private static Command get3ScoreSouthRed(int level) {
		return get3ScoreOptimized(
				toTag(11, level, kRobotToTagsRight), toTag(6, level, kRobotToTagsRightReady),
				toTag(6, level, kRobotToTagsLeft),
				1, kRobotToTagsRight);
	}

	private static Command get3ScoreOptimized(Command align1, Command align2, Command align3, int stationTagID,
			Transform2d... robotToTags) {
		return sequence(
				scoreOptimized(align1, 4),
				toStation(stationTagID, robotToTags),
				scoreOptimized(align2, goToBase(), 4),
				toStation(stationTagID, robotToTags),
				scoreOptimized(align3, goToBase(), 4));
	}

	public static Command scoreOptimized(Command align, int level) {
		return scoreOptimized(align, runOnce(() -> {
		}), level);
	}

	public static Command scoreOptimized(Command align, Command pickup, int level) {
		switch (level) {
			case 4:
				return score(
						align, pickup, kLevelFourHeight, kGrabberAngleLevelFour);
			case 3:
				return score(align, pickup, kLevelThreeHeight, kGrabberAngleLevelThree);
			case 2:
				return score(align, pickup, kLevelTwoHeight, kGrabberAngleOthers);
			case 1:
				return score(align, pickup, kLevelOneHeight, kGrabberAngleOthers);
		}
		return runOnce(() -> {
		});
	}

	private static Command score(Command align, Command pickup, double level, double wristAngle) {
		return score(align, pickup, level, wristAngle, runOnce(() -> {
		}));
	}

	private static Command score(Command align, Command pickup, double level, double wristAngle,
			Command followup) {
		return sequence(
				prepareToScore(align, pickup, level, wristAngle),
				score(.2, followup));// TODO: Optimize
	}

	static Command prepareToScore(Command align, double level, double wristAngle) {
		return prepareToScore(align, runOnce(() -> {
		}), level, wristAngle);
	}

	static Command prepareToScore(Command align, Command pickup, double level, double wristAngle) {
		return parallel(
				align,
				sequence(pickup));
	}

	public static Command score(double releaseDuration, Command followup) {
		return sequence(followup);
	}

	private static Command toStation(int tagID, Transform2d... robotToTags) {
		return parallel(
				toTag(tagID, robotToTags), sequence());
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag}.
	 *
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag}
	 */
	public static Command toClosestTag(Transform2d... robotToTags) {
		return toClosestTag(3, 0.01, 1, 0.08, 8, robotToTags);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag}.
	 *
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag}
	 */
	public static Command toClosestTag(double distanceThresholdInMeters, double distanceTolerance,
			double angleToleranceInDegrees,
			double intermedateDistanceTolerance, double intermediateAngleToleranceInDegrees,
			Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleToleranceInDegrees,
				intermedateDistanceTolerance, intermediateAngleToleranceInDegrees,
				posesToClosestTag(distanceThresholdInMeters, robotToTags));
	}

	/**
	 * Creates a list of {@code Pose2d}s to automatically align the robot to the
	 * closest {@code AprilTag}.
	 *
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a list of {@code Pose2d}s to automatically align the robot to the
	 *         closest {@code AprilTag}
	 */
	public static List<Supplier<Pose2d>> posesToClosestTag(double distanceThresholdInMeters,
			Transform2d... robotToTags) {
		return Arrays.stream(robotToTags).map(r -> (Supplier<Pose2d>) (() -> {
			Pose2d closestTagPose = m_poseEstimationSubsystem.closestTagPose(180, distanceThresholdInMeters);
			if (closestTagPose == null)
				return m_driveSubsystem.getPose();
			return m_poseEstimationSubsystem.odometryCentricPose(closestTagPose.plus(r));
		})).toList();
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag} while driving the robot with joystick input
	 */
	public static Command driveWithLeftAlignment(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation) {
		return driveWithAlignment(forwardSpeed, strafeSpeed, rotation, kRobotToTagsLeft);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag} while driving the robot with joystick input
	 */
	public static Command driveWithRightAlignment(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation) {
		return driveWithAlignment(forwardSpeed, strafeSpeed, rotation, kRobotToTagsRight);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag} while driving the robot with joystick input
	 */
	public static Command driveWithAlignment(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation, Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1, 0.08, 8,
				posesToClosestTag(3, robotToTags)) {

			@Override
			public ChassisSpeeds chassisSpeeds() {
				ChassisSpeeds speeds = DriveSubsystem.chassisSpeeds(forwardSpeed, strafeSpeed, rotation);
				return speeds.plus(super.chassisSpeeds());
			}

		};
	}

	/**
	 * Returns a {@code Command} for moving forward and then backward.
	 * 
	 * @param distanceInFeet the distance in feet
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * 
	 * @return a {@code Command} for moving forward and then backward.
	 */
	public static Command moveForwardBackward(double distanceInFeet, double distanceTolerance,
			double angleTolerance) {
		return sequence(
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						pose(feetToMeters(distanceInFeet), 0, 0)),
				Commands.waitSeconds(2),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				Commands.waitSeconds(1),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)));
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move
	 * the robot forward or backward.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param displacement the displacement (positive: forward, negative: backward)
	 *        of the movement
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 */
	public static Command moveStraight(double displacement, double distanceTolerance,
			double angleToleranceInDegrees) {
		return new DriveCommand(m_driveSubsystem, distanceTolerance, angleToleranceInDegrees, () -> {
			return m_driveSubsystem.getPose().plus(transform(displacement, 0, 0));
		});
	}

	/**
	 * Returns a {@code Command} for moving the robot on a square.
	 * 
	 * @param sideLength the side length of the square in meters
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param timeout the maximum amount of the time given to the {@code Command}
	 * 
	 * @return a {@code Command} for moving the robot on a circle
	 */
	public static Command moveOnSquare(double sideLength, double distanceTolerance,
			double angleTolerance, double timeout) {
		return sequence(
				new DriveCommand(m_driveSubsystem,
						distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(sideLength, 0, 90)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						pose(sideLength, sideLength, 180)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, sideLength, 270)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0.0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)));
	}

	/**
	 * Returns a {@code Command} for aligning the robot to the specified
	 * {@code AprilTag}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param intermediateDistanceTolerance the distance error in meters which is
	 *        tolerable for intermeidate target {@code Pose2d}s
	 * @param intermediateAngleToleranceInDegrees the angle error in degrees which
	 *        is tolerable for intermeidate target {@code Pose2d}s
	 * @param robotToTagTransforms the {@code Pose2d}s of the {@code AprilTag}
	 *        relative to the center of the robot when the robit is aligned to the
	 *        ready and alignment poses
	 * @param robotToTagBackup the {@code Pose2d} of the {@code AprilTag} relative
	 *        to the center of the robot when the robit is aligned to the backup
	 *        pose
	 * @param tagIDs the IDs of the {@code AprilTag}s
	 * 
	 * @return a {@code Command} for aligning the robot to the specified
	 *         {@code AprilTag}s
	 */
	public static Command alignToTags(double distanceTolerance, double angleTolerance,
			double intermediateDistanceTolerance, double intermediateAngleToleranceInDegrees,
			List<Transform2d> robotToTagTransforms, Transform2d robotToTagBackup, int... tagIDs) {
		Pose2d previous = null;
		var commands = new LinkedList<Command>();
		for (int tagID : tagIDs) {
			var tagPose = kFieldLayout.getTagPose(tagID).get().toPose2d();
			var l = new LinkedList<Pose2d>();
			if (previous != null)
				l.add(previous);
			for (var r : robotToTagTransforms)
				l.add(tagPose.plus(r));
			previous = tagPose.plus(robotToTagBackup);
			var command = new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
					intermediateDistanceTolerance, intermediateAngleToleranceInDegrees, l.stream()
							.map(p -> (Supplier<Pose2d>) (() -> m_poseEstimationSubsystem.odometryCentricPose(p)))
							.toList());
			commands.add(command.andThen(new WaitCommand(.5)));
		}
		return sequence(commands.toArray(new Command[0]));
	}

	private static Command select(Command commandRedAlliance, Command commandBlueAlliance) {
		return new SelectCommand<Object>(Map
				.of(Alliance.Red, commandRedAlliance, Alliance.Blue, commandBlueAlliance),
				() -> {
					Alliance alliance = DriverStation.getAlliance().get();
					var middle = kFieldLayout.getFieldLength() / 2;
					try {
						var confidence = m_poseEstimationSubsystem.confidence();
						if (confidence < 0.3)
							return alert("Pose Confidence (" + confidence + ") Too Low!");
						var x = m_poseEstimationSubsystem.getEstimatedPose().getX();
						if ((alliance == DriverStation.Alliance.Blue && x > middle)
								|| (alliance == DriverStation.Alliance.Red && x < middle)) {
							return alert("Strange Robot Position (" + alliance + " Alliance)!");
						}
					} catch (Exception e) {
					}
					return alliance;
				});
	}

	private static Alert alert(String text) {
		var a = new Alert(text, AlertType.kError);
		a.set(true);
		return a;
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the scoring level
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	public static Command toTag(int tagID, int level, Transform2d... robotToTags) {
		return toTag(tagID, adjust(level, robotToTags));
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	public static Command toTag(int tagID, Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1,
				0.05, 5, // TODO: Optimize
				posesToTag(tagID, robotToTags));
	}

	/**
	 * Creates a list of {@code Pose2d}s to automatically align the robot to the
	 * target {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a list of {@code Pose2d}s to automatically align the robot to the
	 *         target {@code AprilTag}
	 */
	public static List<Supplier<Pose2d>> posesToTag(int tagID,
			Transform2d... robotToTags) {
		return Arrays.stream(robotToTags).map(r -> (Supplier<Pose2d>) (() -> {
			Pose2d pose = pose(tagID);
			if (pose == null)
				return m_driveSubsystem.getPose();
			return m_poseEstimationSubsystem.odometryCentricPose(pose.plus(r));
		})).toList();
	}

	/**
	 * Adjusts the specified the {@code Tranform2d}s by incorporating the offset
	 * needed to score at the specified level.
	 * 
	 * @param level the scoring level
	 * @param robotToTags {@code Tranform2d}s
	 * @return the adjusted {@code Tranform2d}s
	 */
	private static Transform2d[] adjust(int level, Transform2d... robotToTags) {
		return Arrays.stream(robotToTags)
				.map(t -> new Transform2d(t.getX() - kOffsets.get(level), t.getY(), t.getRotation())).toList()
				.toArray(new Transform2d[0]);
	}

	public static Command goToBase() {
		return sequence(
				parallel());
	}

}