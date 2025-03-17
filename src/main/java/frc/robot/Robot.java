// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.CommandComposer.*;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
	private final Mechanism2d m_mechanism = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(100));
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(kOperatorControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	private final VisionSimulator m_visionSimulator = RobotBase.isReal() ? null
			: new VisionSimulator(m_driveSubsystem,
					pose(kFieldLayout.getFieldLength() / 2, kFieldLayout.getFieldWidth() / 2, 0),
					0.05); // movement overestimation by 5%
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_driveSubsystem);
	SimCameraProperties m_cameraProp = new SimCameraProperties() {
		{
			setCalibration(640, 480, Rotation2d.fromDegrees(100));
			// Approximate detection noise with average and standard deviation error in
			// pixels.
			setCalibError(0.25, 0.15);
			// Set the camera image capture framerate (Note: this is limited by robot loop
			// rate).
			setFPS(20);
			// The average and standard deviation in milliseconds of image data latency.
			setAvgLatencyMs(35);
			setLatencyStdDevMs(5);

		}
	};

	public Robot() {
		// TODO: Please configure cameras correctly and then enable BackCamera.
		addCamera("BackCamera", kRobotToCamera1); // TODO: check camera names.
		// addCamera("FrontCamera", kRobotToCamera2);
		SignalLogger.start();
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
		CommandComposer.setSubsystems(m_driveSubsystem, m_poseEstimationSubsystem);
		var dropChute = new MechanismLigament2d("bottom", Units.inchesToMeters(5), 0, 5, new Color8Bit(Color.kBeige));
		dropChute.append(new MechanismLigament2d("side", Units.inchesToMeters(12), 90, 5, new Color8Bit(Color.kWhite)));
		m_mechanism.getRoot("dropChute", Units.inchesToMeters(28), Units.inchesToMeters(9)).append(dropChute);
		SmartDashboard.putData("Superstructure", m_mechanism);
		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(
				Map.of(
						11, "FR Turn", 21, "BR Turn", 31, "BL Turn", 41, "FL Turn", kElevatorMotorPort, "Elevator",
						kClimberMotorPort, "Climber Motor", kWristMotorPort, "Wrist Motor", kFlywheelMotorPort,
						"Algae Flywheel Motor", kGrabberAnglePort, "Algae Pivot Motor"));
		DriverStation.startDataLog(DataLogManager.getLog());
		addAutoCommands();
		addTestingCommands();
		addProgrammingCommands();
		bindClimberControls();
		bindDriveControls();
		bindElevatorControls();
		bindWristControls();
		bindAlgaeControls();
		bindCheeseStickControls();
		bindAlert(
				new Alert("Driver Joystick Disconnected!", AlertType.kError), () -> !m_driverController.isConnected());
		bindAlert(
				new Alert("Operator Joystick Disconnected!", AlertType.kError),
				() -> !m_operatorController.isConnected());
		DriverStation.silenceJoystickConnectionWarning(true);
		SmartDashboard.putData("Testing Chooser", m_testingChooser);
		SmartDashboard.putData("Auto Selector", m_autoSelector);
		m_driverController.options().and(m_driverController.create()).and(() -> !DriverStation.isFMSAttached())
				.onTrue(Commands.deferredProxy(m_testingChooser::getSelected));
	}

	public void addAutoCommands() {
		m_autoSelector
				.addOption(
						"Middle Score and Algae", CommandComposer.getMiddleScoreAndAlgae());
		m_autoSelector.addOption("Leave", CommandComposer.leave());
		m_autoSelector
				.addOption(
						"3 Score North (Level 3)",
						// TODO: Optimize
						CommandComposer.get3ScoreNorth(3, 0.5, 1.0, 0.1));
		m_autoSelector
				.addOption(
						"3 Score South (Level 3)",
						// TODO: Optimize
						CommandComposer.get3ScoreSouth(3, 0.5, 1.0, 0.1));
	}

	public void addTestingCommands() {
		m_testingChooser
				.addOption(
						"Pick Up and Score at Level 3 (Left)",
						score(toClosestTag(kLevelOffset.get(3), 0, kRobotToTagsLeft), goToBase(), 3));
		m_testingChooser
				.addOption(
						"Pick Up and Score at Level 3 (Right)",
						score(toClosestTag(kLevelOffset.get(3), 0, kRobotToTagsRight), goToBase(), 3));
		m_testingChooser
				.addOption(
						"Pick Up and Score at Level 4 (Left)",
						score(toClosestTag(kLevelOffset.get(4), 0, kRobotToTagsLeft), goToBase(), 4));
		m_testingChooser
				.addOption(
						"Pick Up and Score at Level 4 (Right)",
						score(toClosestTag(kLevelOffset.get(4), 0, kRobotToTagsRight), goToBase(), 4));
		m_testingChooser
				.addOption(
						"Scoring Test North (Level 3)",
						getScoringTestNorth(3, 0.5, 1.0, 0.1));
		m_testingChooser
				.addOption(
						"Scoring Test South (Level 3)",
						getScoringTestSouth(3, 0.5, 1.0, 0.1));
		m_testingChooser
				.addOption(
						"Reposition the Robot in Simulation",
						runOnce(() -> repositionSimulatedRobot()));
		m_testingChooser
				.addOption(
						"Pick Up and Score Left at Level 3 (6, 7, 8, 9, 10, 11)",
						testLeftAlignment(
								3, 0.1, 1.5, 3.0,
								6, 7, 8, 9, 10, 11));
		m_testingChooser
				.addOption(
						"Pick Up and Score at Right Level 3 (6, 7, 8, 9, 10, 11)",
						testRightAlignment(
								3, 0.1, 1.5, 3.0,
								6, 7, 8, 9, 10, 11));
		m_testingChooser
				.addOption(
						"Pick Up and Score Left at Level 3 (17, 18, 19, 20, 21, 22)",
						testLeftAlignment(
								3, 0.1, 1.5, 3.0,
								17, 18, 19, 20, 21, 22));
		m_testingChooser
				.addOption(
						"Pick Up and Score Right at Level 3 (17, 18, 19, 20, 21, 22)",
						testRightAlignment(
								3, 0.1, 1.5, 3.0,
								17, 18, 19, 20, 21, 22));
		m_testingChooser
				.addOption(
						"Prepare to Score at Level 3 (Left)",
						prepareToScore(
								toClosestTag(kLevelOffset.get(3), 0, kRobotToTagsLeft), kLevelThreeHeight,
								kGrabberAngleLevelThree));
		m_testingChooser
				.addOption(
						"Prepare to Score at Level 4 (Left)",
						prepareToScore(
								toClosestTag(kLevelOffset.get(4), 0, kRobotToTagsLeft), kLevelFourHeight,
								kGrabberAngleLevelFour));
		m_testingChooser
				.addOption(
						"Check All Subsystems",
						parallel(
								sequence(/*
											 * m_elevatorSubsystem.testCommand(2.0),
											 */
										parallel(/*
													 * m_cheeseStickSubsystem.testCommand(2.0),
													 * m_wristSubsystem.testCommand(2.0)
													 */)),
								m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0)));
		m_testingChooser
				.addOption(
						"Check DriveSubsystem (F/B/L/R/LR/RR and F/B while rotating)",
						m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0));
		m_testingChooser
				.addOption(
						"Check Absolute Orientation",
						testAbsoluteOrientation(2));
		double distanceTolerance = 0.01;
		double angleToleranceInDegrees = 1;
		double intermediateDistanceTolerance = 0.08;
		double intermediateAngleToleranceInDegrees = 8.0;
		m_testingChooser
				.addOption(
						"Align to AprilTags 17, 18, 19, 20, 21, and 22",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTagsLeft),
								kRobotToTagsLeft[0], 17, 18, 19, 20, 21, 22, 17));
		m_testingChooser
				.addOption(
						"Align to AprilTags 6, 7, 8, 9, 10, and 11",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTagsLeft),
								kRobotToTagsLeft[0], 6, 7, 8, 9, 10, 11, 6));
		m_testingChooser
				.addOption(
						"Align to AprilTags 12, 13, 17, 18, and 19",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 18,
								17, 12, 17, 18, 19, 13, 19, 18));
		m_testingChooser
				.addOption(
						"Align to AprilTags 1, 2, 6, 7, and 8",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 7, 6,
								1, 6, 7, 8, 2, 8, 7));
		m_testingChooser
				.addOption(
						"Check PID Constants for Driving (5'x5' Square)",
						CommandComposer
								.moveOnSquare(Units.feetToMeters(5), distanceTolerance, angleToleranceInDegrees, 16));
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
						"Fastest Forward/Backward Movement Test (5m)", forwardBackwardSpeedTest(5, 5, 0.01, 1));
		m_testingChooser
				.addOption(
						"Fastest Rotation Test (5 rotations)",
						new PathDriveCommand(m_driveSubsystem, 1, 10,
								1, 100,
								IntStream.range(1, 1 + 3 * 5)
										.mapToObj(
												i -> (Supplier<Pose2d>) (() -> {
													var pose = m_driveSubsystem.getPose();
													return new Pose2d(pose.getX(), pose.getY(),
															Rotation2d.fromDegrees(120 * i));
												}))
										.toList()));
	}

	public void bindAlert(Alert alert, BooleanSupplier event) {
		CommandScheduler.getInstance().getActiveButtonLoop().bind(() -> alert.set(event.getAsBoolean()));
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
						() -> -m_driverController.getRightY(),
						() -> -m_driverController.getRightX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getCreateButton)); // makes the robot robot-oriented

		m_driverController.L1().whileTrue(
				toClosestTag(kRobotToTagsLeft).withName("toClosestTag(kRobotToTagsLeft)"));
		m_driverController.R1().whileTrue(
				toClosestTag(kRobotToTagsRight).withName("toClosestTag(kRobotToTagsRight)"));
		m_driverController.options().onTrue(m_driveSubsystem.resetHeading());

		m_driverController.square().onTrue(m_driveSubsystem.toggleCoastMode());
	}

	public void bindElevatorControls() {
		m_operatorController.circle().onTrue(CommandComposer.scoreLevelOneInTeleop());
		m_operatorController.L1().and(m_operatorController.triangle()).onTrue(CommandComposer.removeAlgaeLevelThree());
		m_operatorController.L1().and(m_operatorController.square()).onTrue(CommandComposer.removeAlgaeLevelTwo());
		m_operatorController.L1().and(m_operatorController.circle()).onTrue(CommandComposer.prepareForCoralPickup());
		m_operatorController.L1().and(m_operatorController.cross()).onTrue(CommandComposer.goToBase());
	}

	public void bindAlgaeControls() {
	}

	public void bindWristControls() {
	}

	public void bindCheeseStickControls() {
	}

	public void bindClimberControls() {
	}

	public void bindLEDControls() {
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

	private void addCamera(String cameraName, Transform3d robotToCamera) {
		PhotonCamera camera = RobotBase.isSimulation()
				? cameraSim(cameraName, robotToCamera, m_visionSimulator, m_cameraProp)
				: new PhotonCamera(cameraName);
		m_poseEstimationSubsystem.addCamera(camera, robotToCamera);
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
						kFieldLayout.getFieldLength() / 2 + 1.5 * (redAlliance ? 1 : -1),
						yCoordinates.get(location), redAlliance ? 0 : 180));
	}

}
