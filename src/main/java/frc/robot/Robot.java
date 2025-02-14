// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonCameraSimulator;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSimulator;

public class Robot extends TimedRobot {
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final SendableChooser<Command> m_testSelector = new SendableChooser<Command>();

	private Command m_autonomousCommand;
	private Command m_testCommand;
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS4Controller m_driverController = new CommandPS4Controller(
			ControllerConstants.kDriverControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem,
			pose(kFieldLayout.getFieldLength() / 2 + 2.5, 1.91 + .3, 180), 0.01);
	private final PhotonCamera m_camera1 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera1", kRobotToCamera1, m_visionSimulator, 3, 0.1)
			: new PhotonCamera("Cool camera");
	private final PhotonCamera m_camera2 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera2", kRobotToCamera2, m_visionSimulator, 3, 0.1)
			: new PhotonCamera("Cool camera2");
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_driveSubsystem)
			.addCamera(m_camera1, kRobotToCamera1)
			.addCamera(m_camera2, kRobotToCamera2);

	public Robot() {
		CommandComposer.setSubsystems(m_driveSubsystem, m_poseEstimationSubsystem);

		m_autoSelector.addOption("Test DriveSubsystem", m_driveSubsystem.testCommand());
		SmartDashboard.putData("Auto Selector", m_autoSelector);

		double distanceTolerance = 0.03;
		double angleToleranceInDegrees = 2.0;

		m_testSelector
				.addOption(
						"Check kDriveGearRatio and kWheelDiameter (F/B 6 feet)",
						CommandComposer.moveForwardBackward2Controllers(6, distanceTolerance, angleToleranceInDegrees));
		m_testSelector
				.addOption(
						"Check PID Constants for Driving (Oval with Max Radius of 1m)",
						CommandComposer.moveOnOval(
								1, 8, 16, distanceTolerance, angleToleranceInDegrees, 16, 360 / 12 * 3 / 2));
		m_testSelector
				.addOption(
						"Quickly Align to AprilTags 7 and 8",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, 16, transform(0.8, 0, 180),
								transform(1.3, 0, 180), 7, 8, 7, 8, 7));
		m_testSelector.addOption("Check All Subsystems in Pitt", CommandComposer.testAllSubsystems());
		m_testSelector.addOption("Check All Subsystems on Field", CommandComposer.testAllSubsystems());
		m_testSelector.addOption("Check DriveSubsystem (Robot-Oriented F/B/L/R/LR/RR)", m_driveSubsystem.testCommand());
		m_testSelector
				.addOption(
						"Check DriveSubsystem (Field-Relative F/B with LR/RR)",
						CommandComposer.testDriveSubsystemFieldRelative());
		m_testSelector
				.addOption(
						"Check PID Constants for Driving (5'x5' Square)",
						CommandComposer
								.moveOnSquare(Units.feetToMeters(5), distanceTolerance, angleToleranceInDegrees, 16));
		m_testSelector.addOption("Test Rotation", CommandComposer.testRotation());
		m_testSelector.addOption("Turn toward Tag 1", CommandComposer.turnTowardTag(1));

		SmartDashboard.putData("Test Selector", m_testSelector);

		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(
				Map.of(
						10, "FR Drive", 11, "FR Turn", 20, "BR Drive", 21, "BR Turn", 30, "BL Drive", 31, "BL Turn",
						40, "FL Drive", 41, "FL Turn"));
		DriverStation.startDataLog(DataLogManager.getLog());
		bindDriveControls();
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						() -> !m_driverController.getHID().getSquareButton()));

		m_driverController.button(Button.kSquare)
				.whileTrue(
						driveWithAlignmentCommand(
								() -> -m_driverController.getLeftY(),
								() -> -m_driverController.getLeftX(),
								() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
								new Transform2d(0.5, 0, Rotation2d.fromDegrees(180)), 2, 0.03, 3));
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
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @return a {@code Command} to automatically align the robot to the closest tag
	 *         while driving the robot with joystick input
	 */
	Command driveWithAlignmentCommand(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation, Transform2d robotToTag, double distanceThresholdInMeters, double distanceTolerance,
			double angleToleranceInDegrees) {
		return new DriveCommand(m_driveSubsystem, distanceTolerance, angleToleranceInDegrees, () -> {
			Pose2d closestTagPose = m_poseEstimationSubsystem.closestTagPose(180, distanceThresholdInMeters);
			if (closestTagPose == null)
				return m_driveSubsystem.getPose();
			return m_poseEstimationSubsystem.odometryCentricPose(closestTagPose.plus(robotToTag));
		}) {

			@Override
			public ChassisSpeeds chassisSpeeds() {
				ChassisSpeeds speeds = DriveSubsystem.chassisSpeeds(forwardSpeed, strafeSpeed, rotation);
				return speeds.plus(super.chassisSpeeds());
			}

		};
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_autoSelector.getSelected();
		if (m_autonomousCommand != null)
			m_autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null)
			m_autonomousCommand.cancel();
		if (m_testCommand != null)
			m_testCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		m_testCommand = m_testSelector.getSelected();
		if (m_testCommand != null)
			m_testCommand.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}