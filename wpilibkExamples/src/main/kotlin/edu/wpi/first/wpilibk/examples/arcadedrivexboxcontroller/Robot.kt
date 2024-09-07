// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.arcadedrivexboxcontroller

import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with split
 * arcade steering and an Xbox controller.
 */
object Robot : TimedRobot() {
    private val m_leftMotor = PWMSparkMax(0)
    private val m_rightMotor = PWMSparkMax(1)
    private val m_robotDrive = DifferentialDrive(
        { speed: Double -> m_leftMotor.set(speed) },
        { speed: Double -> m_rightMotor.set(speed) })
    private val m_driverController = XboxController(0)

    /** Called once at the beginning of the robot program.  */
    init {
        SendableRegistry.addChild(m_robotDrive, m_leftMotor)
        SendableRegistry.addChild(m_robotDrive, m_rightMotor)

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotor.inverted = true
    }

    override fun teleopPeriodic() {
        // Drive with split arcade drive.
        // That means that the Y axis of the left stick moves forward
        // and backward, and the X of the right stick turns left and right.
        m_robotDrive.arcadeDrive(-m_driverController.leftY, -m_driverController.rightX)
    }
}