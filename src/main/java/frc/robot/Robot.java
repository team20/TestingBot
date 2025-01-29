// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.Map;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem, pose(1, 1, 0), 0.01);
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

		double distanceTolerance = 0.05;
		double angleTolerance = 5;
		m_driverController.button(Button.kSquare) // home
				.whileTrue(
						new DriveCommand(
								m_driveSubsystem, pose(0, 0, 0), distanceTolerance, angleTolerance));
		m_driverController.button(Button.kX) // 1m away
				.whileTrue(
						DriveCommand.moveForward(m_driveSubsystem, 1, distanceTolerance, angleTolerance));
		m_driverController.button(Button.kCircle) // 2m away
				.whileTrue(
						sequence(
								DriveCommand.moveForward(m_driveSubsystem, 2, distanceTolerance, angleTolerance),
								DriveCommand.moveForward(m_driveSubsystem, -2, distanceTolerance, angleTolerance)));

		double angleOfCoverageInDegrees = 90;
		double distanceThresholdInMeters = 4;
		m_driverController.button(Button.kTriangle)
				.whileTrue(
						AlignCommand.turnToClosestTag(
								m_driveSubsystem, m_poseEstimationSubystem, angleOfCoverageInDegrees,
								distanceThresholdInMeters,
								distanceTolerance, angleTolerance));

		Transform2d robotToTarget = new Transform2d(1.5, 0, Rotation2d.fromDegrees(180));
		m_driverController.button(Button.kLeftBumper)
				.whileTrue(
						AlignCommand.moveToClosestTag(
								m_driveSubsystem, m_poseEstimationSubystem, angleOfCoverageInDegrees,
								distanceThresholdInMeters, robotToTarget,
								distanceTolerance, angleTolerance));

		m_driverController.button(Button.kRightBumper)
				.whileTrue(
						tourCommand(
								m_driveSubsystem, m_poseEstimationSubystem, distanceTolerance, angleTolerance,
								robotToTarget, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24));

	}

	Command tourCommand(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubystem,
			double distanceTolerance, double angleTolerance, Transform2d robotToTarget, int... tagIDs) {
		var s = Arrays.stream(tagIDs).mapToObj(i -> kFieldLayout.getTagPose(i)).filter(p -> p.isPresent())
				.map(p -> p.get())
				.map(p -> p.toPose2d().plus(robotToTarget)).map(
						p -> AlignCommand
								.moveTo(
										driveSubsystem, poseEstimationSubystem, p, distanceTolerance,
										angleTolerance));
		return sequence(s.toList().toArray(new Command[0]));
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
				DriveCommand.testCommand(m_driveSubsystem)).schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}