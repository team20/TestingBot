// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_steerController = new PIDController(kP, kI, kD);
	protected final CANcoder m_CANCoder;
	protected final SparkMax m_driveMotor;
	protected final SparkMax m_steerMotor;

	public SwerveModule(int canId, int drivePort, int steerPort) {
		m_CANCoder = new CANcoder(canId);
		m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);

		var config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).voltageCompensation(12);
		config.openLoopRampRate(kRampRate).closedLoopRampRate(kRampRate);
		// Helps with encoder precision (not set in stone)
		config.encoder.uvwAverageDepth(kEncoderDepth).uvwMeasurementPeriod(kEncoderMeasurementPeriod);
		config.smartCurrentLimit(kDriveSmartCurrentLimit).secondaryCurrentLimit(kDrivePeakCurrentLimit);
		m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).voltageCompensation(12);
		config.openLoopRampRate(kRampRate).closedLoopRampRate(kRampRate);
		// Helps with encoder precision (not set in stone)
		config.encoder.uvwAverageDepth(kEncoderDepth).uvwMeasurementPeriod(kEncoderMeasurementPeriod);
		config.smartCurrentLimit(kSteerSmartCurrentLimit).secondaryCurrentLimit(kSteerPeakCurrentLimit);
		m_steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_steerController.enableContinuousInput(0, 360);
	}

	/**
	 * Returns drive encoder distance in meters traveled.
	 * 
	 * @return The position in meters.
	 */
	public double getDriveEncoderPosition() {
		return m_driveMotor.getEncoder().getPosition() * kMetersPerMotorRotation;
	}

	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	public double getDriveCurrent() {
		return m_driveMotor.getOutputCurrent();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveMotor.getEncoder().setPosition(0);
	}

	/**
	 * Gets the current drive motor voltage.
	 * 
	 * @return The motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_driveMotor.getAppliedOutput() * kDriveMaxVoltage;
	}

	/**
	 * Gets the current drive motor temperature.
	 * 
	 * @return The temperature in degrees Celsius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getMotorTemperature();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return The module angle
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Returns the module position.
	 * 
	 * @return The module position
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Gets the module speed and angle.
	 * 
	 * @return The module state
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state. Note that the speedMetersPerSecond field has
	 *        been repurposed to contain volts, not velocity.
	 */
	public void setModuleState(SwerveModuleState state) {
		m_driveMotor.setVoltage(state.speedMetersPerSecond);
		double turnPower = m_steerController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_steerMotor.setVoltage(turnPower);
	}

	/**
	 * Sets the module angle.
	 * 
	 * @param angle the target angle
	 */
	public void setAngle(double angle) {
		m_steerMotor.setVoltage(m_steerController.calculate(getModuleAngle(), angle));
	}

}