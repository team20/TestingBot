// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.RobotConstants.*;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonCameraSimulator;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS4Controller m_driverController = new CommandPS4Controller(
			ControllerConstants.kDriverControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	PhotonCamera m_camera1 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera1", kRobotToCamera1, m_driveSubsystem,
					new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
					0.01, 3,
					0.1)
			: new PhotonCamera("Cool camera"); // TODO: Check the camera name
	private final PoseEstimationSubsystem m_poseEstimationSubystem = new PoseEstimationSubsystem(m_driveSubsystem,
			m_camera1);

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
		Transform2d robotToTarget = new Transform2d(1.5, 0, Rotation2d.fromDegrees(180));
		m_driverController.button(Button.kSquare)
				.whileTrue(
						AlignCommand.moveToClosestTag(
								m_driveSubsystem, m_poseEstimationSubystem, 90, 3, robotToTarget,
								.2, 10));
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
		new WaitCommand(0)
				.andThen(m_driveSubsystem.testCommand()) // F, B, SL, SR, RL, RR
				.andThen(DriveCommand.testCommand(m_driveSubsystem))
				.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}