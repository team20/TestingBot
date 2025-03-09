// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.PathDriveCommand;
import frc.robot.simulation.VisionSimulator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

	private final SendableChooser<Command> m_testingChooser = new SendableChooser<>();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(kOperatorControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	private final VisionSimulator m_visionSimulator = RobotBase.isReal() ? null
			: new VisionSimulator(m_driveSubsystem,
					pose(kFieldLayout.getFieldLength() / 2, kFieldLayout.getFieldWidth() / 2, 0),
					0.05); // movement overestimation by 5%
	SimCameraProperties cameraProp = new SimCameraProperties() {
		{
			setCalibration(640, 480, Rotation2d.fromDegrees(100));
			// Approximate detection noise with average and standard deviation error in
			// pixels.
			setCalibError(0.25, 0.08);
			// Set the camera image capture framerate (Note: this is limited by robot loop
			// rate).
			setFPS(20);
			// The average and standard deviation in milliseconds of image data latency.
			setAvgLatencyMs(35);
			setLatencyStdDevMs(5);

		}
	};
	private final PhotonCamera m_camera1 = RobotBase.isSimulation()
			? cameraSim("Camera1", kRobotToCamera1, m_visionSimulator, cameraProp)
			: new PhotonCamera("Cool camera");
	private final PhotonCamera m_camera2 = RobotBase.isSimulation()
			? cameraSim("Camera2", kRobotToCamera2, m_visionSimulator, cameraProp)
			: new PhotonCamera("Cool camera2");
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_driveSubsystem)
			.addCamera(m_camera1, kRobotToCamera1)
			.addCamera(m_camera2, kRobotToCamera2);

	public Robot() {
		CommandComposer.setSubsystems(m_driveSubsystem, m_poseEstimationSubsystem);
		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(
				Map.of(
						11, "FR Turn", 21, "BR Turn", 31, "BL Turn", 41, "FL Turn"));
		DriverStation.startDataLog(DataLogManager.getLog());
		addAutoCommands();
		addTestingCommands();
		addProgrammingCommands();
		bindDriveControls();
		SmartDashboard.putData("Auto Selector", m_autoSelector);
		SmartDashboard.putData("Testing Chooser", m_testingChooser);
		m_driverController.options().and(m_driverController.create()).and(() -> !DriverStation.isFMSAttached())
				.onTrue(Commands.deferredProxy(m_testingChooser::getSelected));
	}

	public void addAutoCommands() {
		m_autoSelector
				.addOption(
						"3 Score North", CommandComposer.get3ScoreNorth());
		m_autoSelector
				.addOption(
						"3 Score South", CommandComposer.get3ScoreSouth());
	}

	public void addTestingCommands() {
		double distanceTolerance = 0.01;
		double angleToleranceInDegrees = 1;
		double intermediateDistanceTolerance = 0.08;
		double intermediateAngleToleranceInDegrees = 8.0;
		m_testingChooser
				.addOption(
						"Quickly Align to AprilTags 22, 12, 17, 12, 17",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 22,
								12, 17, 12, 17));
		m_testingChooser
				.addOption(
						"Check Subsystems in Pitt",
						parallel(m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0)));
		m_testingChooser
				.addOption(
						"Check DriveSubsystem (F/B/L/R/LR/RR and F/B while rotating)",
						m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0));
		m_testingChooser
				.addOption(
						"Check PID Constants for Driving (5'x5' Square)",
						CommandComposer
								.moveOnSquare(Units.feetToMeters(5), distanceTolerance, angleToleranceInDegrees, 16));
		m_testingChooser
				.addOption(
						"Quickly Align to AprilTags 12, 13, 17, 18, and 19",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 18,
								17, 12, 17, 18, 19, 13, 19, 18));
		m_testingChooser
				.addOption(
						"Quickly Align AprilTags 17, 18, 19, 20, 21, and 22",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTagsLeft),
								kRobotToTagsLeft[0], 17, 18, 19, 20, 21, 22, 17));
		m_testingChooser
				.addOption(
						"Quickly Align to AprilTags 1, 2, 6, 7, and 8",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 7, 6,
								1, 6, 7, 8, 2, 8, 7));
		m_testingChooser
				.addOption(
						"Check kDriveGearRatio and kWheelDiameter (F/B 6 feet)",
						CommandComposer.moveForwardBackward(6, distanceTolerance, angleToleranceInDegrees));
		m_testingChooser
				.addOption(
						"Slowest Movement Test (F/B/L/R/LR/RR and F/B while rotating)",
						m_driveSubsystem.testCommand(kDriveMinSpeed, kTurnMinAngularSpeed, 1.0));
		m_testingChooser
				.addOption(
						"Fastest Forward/Backward Movement Test (5m)",
						sequence(
								CommandComposer.moveStraight(5, 0.1, 10),
								CommandComposer.moveStraight(-5, 0.1, 10)));
		m_testingChooser
				.addOption(
						"Fastest Rotation Test (5 rotations)",
						new PathDriveCommand(m_driveSubsystem, 1, 10,
								1, 100,
								IntStream.range(1, 1 + 3 * 5)
										.mapToObj(
												i -> (Supplier<Pose2d>) (() -> {
													var pose = m_driveSubsystem.getPose();
													return pose(pose.getX(), pose.getY(), 120 * i);
												}))
										.toList()));
	}

	public void addProgrammingCommands() {
		m_testingChooser
				.addOption("SysId Drive Quasistatic Forward", m_driveSubsystem.sysidQuasistatic(Direction.kForward));
		m_testingChooser
				.addOption("SysId Drive Quasistatic Reverse", m_driveSubsystem.sysidQuasistatic(Direction.kReverse));
		m_testingChooser.addOption("SysId Drive Dynamic Forward", m_driveSubsystem.sysidDynamic(Direction.kForward));
		m_testingChooser.addOption("SysId Drive Dynamic Reverse", m_driveSubsystem.sysidDynamic(Direction.kReverse));
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						// (m_driverController.axisMagnitudeGreaterThan(1, 5) == 0) ?
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getSquareButton)); // makes the robot robot-oriented
		// m_driveSubsystem.setDefaultCommand(
		// m_driveSubsystem.driveCommand(
		// () -> -m_driverController.getLeftY(),
		// () -> -m_driverController.getLeftX(),
		// () -> -m_driverController.getRightY(),
		// () -> -m_driverController.getRightX(),
		// m_driverController.getHID()::getSquareButton)); // makes the robot
		// robot-oriented

		m_driverController.options().onTrue(m_driveSubsystem.resetHeading());

		m_driverController.square().whileTrue(CommandComposer.toClosestTag(kRobotToTagsLeft));
		m_driverController.cross().whileTrue(CommandComposer.toClosestTag(kRobotToTagsRight));

		// m_operatorController.povLeft().whileTrue(
		// CommandComposer.driveWithLeftAlignment(
		// () -> -m_driverController.getLeftY(),
		// () -> -m_driverController.getLeftX(),
		// () -> m_driverController.getL2Axis() - m_driverController.getR2Axis()));

		// m_operatorController.povRight().whileTrue(
		// CommandComposer.driveWithRightAlignment(
		// () -> -m_driverController.getLeftY(),
		// () -> -m_driverController.getLeftX(),
		// () -> m_driverController.getL2Axis() - m_driverController.getR2Axis()));

	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
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
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
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
		var testCommand = m_testingChooser.getSelected();
		if (testCommand != null)
			testCommand.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	/**
	 * Constructs a {@code PhotonCamera} that provides simulation.
	 * 
	 * @param cameraName the name of the {@code PhotonCamera}
	 * @param robotToCamera the {@code Pose2d} of the {@code PhotonCamera} relative
	 *        to the center of the robot
	 * @param m_visionSimulator the {@code VisionSimulator} to use
	 * @param cameraProp the {@code SimCameraProperties} to use
	 * @return the constructed {@code PhotonCamera}
	 */
	PhotonCamera cameraSim(String cameraName, Transform3d robotToCamera, VisionSimulator m_visionSimulator,
			SimCameraProperties cameraProp) {
		PhotonCamera camera = new PhotonCamera(cameraName);
		PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
		cameraSim.enableProcessedStream(true);
		cameraSim.enableDrawWireframe(true);
		m_visionSimulator.addCamera(cameraSim, robotToCamera);
		return camera;
	}

	@Override
	public void simulationInit() {
		repositionSimulatedRobot(DriverStation.Alliance.Red, 2);
	}

	/**
	 * Repositions the robot in simulation according to the alliance station.
	 */
	void repositionSimulatedRobot() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent())
			repositionSimulatedRobot(alliance.get(), DriverStation.getLocation().getAsInt());
	}

	/**
	 * Repositions the robot in simulation according to the specified alliance
	 * station.
	 * 
	 * @param alliance the {@code Alliance}
	 * @param location the location of the team's driver station
	 */
	void repositionSimulatedRobot(DriverStation.Alliance alliance, int location) {
		var redAlliance = alliance == DriverStation.Alliance.Red;
		Map<Integer, Double> yCoordinates = Map.of(
				1, kFieldLayout.getFieldWidth() * 3 / 4, 2, kFieldLayout.getFieldWidth() * 2 / 4, 3,
				kFieldLayout.getFieldWidth() * 1 / 4);
		m_visionSimulator.setRobotPose(
				pose(
						kFieldLayout.getFieldLength() / 2 + 2 * (redAlliance ? 1 : -1),
						yCoordinates.get(location), redAlliance ? 0 : 180));
	}
}
