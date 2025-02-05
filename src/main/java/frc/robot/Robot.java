// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommand2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonCameraSimulator;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSimulator;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS4Controller m_driverController = new CommandPS4Controller(
			ControllerConstants.kDriverControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem,
			pose(kFieldLayout.getFieldLength() / 2, 1.91, 0), 0.01);
	private final PhotonCamera m_camera1 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera1", kRobotToCamera1, m_visionSimulator, 3, 0.1)
			: new PhotonCamera("Cool camera");
	private final PhotonCamera m_camera2 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera2", kRobotToCamera2, m_visionSimulator, 3, 0.1)
			: new PhotonCamera("Cool camera2");
	private final PoseEstimationSubsystem m_poseEstimationSubystem = new PoseEstimationSubsystem(m_driveSubsystem)
			.addCamera(m_camera1, kRobotToCamera1)
			.addCamera(m_camera2, kRobotToCamera2);

	public Robot() {
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
						() -> m_driverController.getR2Axis() - m_driverController.getL2Axis(),
						m_driverController.getHID()::getSquareButton));

		double distanceTolerance = 0.03;
		double angleTolerance = 3;
		m_driverController.button(Button.kSquare) // home
				.whileTrue(
						new DriveCommand(
								m_driveSubsystem, pose(0, 0, 0), distanceTolerance, angleTolerance));
		m_driverController.button(Button.kX) // 3 feet forward
				.whileTrue(
						DriveCommand.moveForward(m_driveSubsystem, .9, distanceTolerance, angleTolerance));
		m_driverController.button(Button.kCircle) // 6 feet forward and then backward
				.whileTrue(
						sequence(
								DriveCommand.moveForward(m_driveSubsystem, 1.8, distanceTolerance, angleTolerance),
								DriveCommand.moveForward(m_driveSubsystem, -1.8, distanceTolerance, angleTolerance)));

		Transform2d robotToTarget = new Transform2d(1.8, 0, Rotation2d.fromDegrees(180));
		double angleOfCoverageInDegrees = 90;
		double distanceThresholdInMeters = 4;
		// m_driverController.button(Button.kTriangle)
		// .whileTrue(
		// alignTest(1, robotToTarget, distanceTolerance, angleTolerance));

		m_driverController.button(Button.kTriangle)
				.whileTrue(
						AlignCommand.turnToClosestTag(
								m_driveSubsystem, m_poseEstimationSubystem, angleOfCoverageInDegrees,
								distanceThresholdInMeters,
								distanceTolerance, angleTolerance));

		m_driverController.button(Button.kLeftBumper)
				.whileTrue(
						driveToClosestTag(new Transform2d(0.5, 0, Rotation2d.fromDegrees(180)), 2));

		// m_driverController.button(Button.kLeftBumper)
		// .whileTrue(
		// AlignCommand.moveToClosestTag(
		// m_driveSubsystem, m_poseEstimationSubystem, angleOfCoverageInDegrees,
		// distanceThresholdInMeters, robotToTarget,
		// distanceTolerance, angleTolerance));

		m_driverController.button(Button.kRightBumper)
				.whileTrue(
						tourCommandOptimized(
								m_driveSubsystem, m_poseEstimationSubystem, distanceTolerance,
								angleTolerance,
								robotToTarget, 1, 6, 7, 8, 2, 8, 7, 6, 1));

		// m_driverController.button(Button.kRightBumper)
		// .whileTrue(
		// tourCommand(
		// m_driveSubsystem, m_poseEstimationSubystem, distanceTolerance,
		// angleTolerance,
		// robotToTarget, 1, 6, 7, 8, 2, 8, 7, 6, 1));

		// m_driverController.button(Button.kRightBumper)
		// .whileTrue(
		// tourCommand(
		// m_driveSubsystem, m_poseEstimationSubystem, distanceTolerance,
		// angleTolerance,
		// robotToTarget, 12, 15, 14, 16, 17, 18, 19, 20, 21, 22));
	}

	Command driveToClosestTag(Transform2d robotToTarget, double distanceThresholdInMeters) {
		@SuppressWarnings("resource")
		PIDController controllerXY = new PIDController(kDriveP, kDriveI, kDriveD);
		controllerXY.setSetpoint(0);

		@SuppressWarnings("resource")
		PIDController controllerYaw = new PIDController(kTurnP, kTurnI, kTurnD);
		controllerYaw.enableContinuousInput(-180, 180);
		controllerYaw.setSetpoint(0);

		return run(() -> {

			double fwdSpeed = MathUtil
					.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.kDeadzone);
			fwdSpeed = Math.signum(fwdSpeed) * Math.pow(fwdSpeed, 2) * kDriveMaxSpeed;

			double strSpeed = MathUtil
					.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.kDeadzone);
			strSpeed = Math.signum(strSpeed) * Math.pow(strSpeed, 2) * kDriveMaxSpeed;

			double rotSpeed = MathUtil.applyDeadband(
					m_driverController.getR2Axis() - m_driverController.getL2Axis(), ControllerConstants.kDeadzone);
			rotSpeed = Math.signum(rotSpeed) * Math.pow(rotSpeed, 2) * kTurnMaxAngularSpeed;

			var pose = m_poseEstimationSubystem.getEstimatedPose();

			var targetPose = m_poseEstimationSubystem
					.closestTagPose(180, distanceThresholdInMeters);
			if (targetPose != null) {
				targetPose = targetPose.plus(robotToTarget);
				Translation2d t = targetPose.getTranslation().minus(pose.getTranslation());
				double speed = controllerXY.calculate(t.getNorm());
				t = t.getNorm() > 0 ? t.times(-speed / t.getNorm()) : t; // translation toward target
				fwdSpeed += t.getX();
				strSpeed += t.getY();
				rotSpeed += -controllerYaw
						.calculate(targetPose.getRotation().minus(pose.getRotation()).getDegrees());
			}
			m_driveSubsystem.drive(fwdSpeed, strSpeed, rotSpeed, true);
		});
	}

	Command alignTest(int tagID, Transform2d robotToTarget, double distanceTolerance, double angleTolerance) {
		return sequence(
				AlignCommand.moveTo(
						m_driveSubsystem, m_poseEstimationSubystem,
						kFieldLayout.getTagPose(tagID).get().toPose2d().plus(robotToTarget),
						distanceTolerance, angleTolerance),
				AlignCommand.moveTo(
						m_driveSubsystem, m_poseEstimationSubystem,
						() -> m_poseEstimationSubystem.getEstimatedPose()
								.plus(pose(-2, 0, 0).minus(Pose2d.kZero)),
						distanceTolerance, angleTolerance));
	}

	Command tourCommand(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubystem,
			double distanceTolerance, double angleTolerance, Transform2d robotToTarget, int... tagIDs) {
		List<DriveCommand2> commands = Arrays.stream(tagIDs).mapToObj(i -> kFieldLayout.getTagPose(i))
				.filter(p -> p.isPresent())
				.map(p -> p.get())
				.map(p -> p.toPose2d().plus(robotToTarget)).map(
						p -> AlignCommand
								.moveTo(
										driveSubsystem, poseEstimationSubystem, p, distanceTolerance,
										angleTolerance))
				.toList();
		return sequence(commands.toArray(new Command[0]));
	}

	Command tourCommandOptimized(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubystem,
			double distanceTolerance, double angleTolerance, Transform2d robotToTarget, int... tagIDs) {
		var l = Arrays.stream(tagIDs).mapToObj(i -> kFieldLayout.getTagPose(i))
				.filter(p -> p.isPresent())
				.map(p -> p.get())
				.map(p -> p.toPose2d().plus(robotToTarget)).toList();
		List<Command> commands = new LinkedList<Command>();
		DriveCommand2 previous = null;
		for (var p : l) {
			boolean last = p == l.get(l.size() - 1);
			DriveCommand2 c = previous == null ? AlignCommand.moveTo(
					driveSubsystem, poseEstimationSubystem, p, last ? distanceTolerance : 3 * distanceTolerance,
					last ? angleTolerance : 3 * angleTolerance)
					: AlignCommand.moveTo(
							driveSubsystem, poseEstimationSubystem, p, last ? distanceTolerance : 3 * distanceTolerance,
							last ? angleTolerance : 3 * angleTolerance, previous);
			commands.add(c);
			previous = c;
		}
		return sequence(commands.toArray(new Command[0]));
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
		m_autonomousCommand = null;

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
		sequence(
				m_driveSubsystem.testCommand(), // F, B, SL, SR, RL, RR
				DriveCommand.testCommand(m_driveSubsystem).withTimeout(2),
				DriveCommand2.testCommand(m_driveSubsystem).withTimeout(2))
						.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}