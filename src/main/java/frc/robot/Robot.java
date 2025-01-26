// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
	private final CommandComposer m_composer;
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS4Controller m_driverController = new CommandPS4Controller(
			ControllerConstants.kDriverControllerPort);
	private AutoChooser m_autoChooser = new AutoChooser();

	public Robot() {
		m_composer = new CommandComposer(m_driveSubsystem);

		m_autoChooser.addRoutine("LeaveRoutine", m_composer::leave);
		m_autoChooser.addRoutine("RoundTripRoutine", m_composer::roundTrip);
		m_autoChooser.addRoutine("MiddleBallRoutine", m_composer::middleBall);
		m_autoChooser.addCmd("ZigZag", m_composer.zigZag(0.3, 5));
		m_autoChooser.addCmd("InverseZigZag", m_composer.zigZag(-0.3, 5));
		SmartDashboard.putData("Autos", m_autoChooser);

		RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());

		bindDriveControls();
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						() -> m_driverController.getR2Axis() - m_driverController.getL2Axis(),
						m_driverController.getHID()::getSquareButton));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
}
