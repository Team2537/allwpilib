// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.armbotoffboard.subsystems

import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibk.examples.armbotoffboard.Constants.DriveConstants
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier

class DriveSubsystem : SubsystemBase() {
    // The motors on the left side of the drive.
    private val m_leftLeader = PWMSparkMax(DriveConstants.kLeftMotor1Port)
    private val m_leftFollower = PWMSparkMax(DriveConstants.kLeftMotor2Port)

    // The motors on the right side of the drive.
    private val m_rightLeader = PWMSparkMax(DriveConstants.kRightMotor1Port)
    private val m_rightFollower = PWMSparkMax(DriveConstants.kRightMotor2Port)

    // The robot's drive
    private val m_drive = DifferentialDrive(
        { speed: Double -> m_leftLeader.set(speed) },
        { speed: Double -> m_rightLeader.set(speed) })

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    // The left-side drive encoder
    val leftEncoder: Encoder = Encoder(
        DriveConstants.kLeftEncoderPorts.get(0),
        DriveConstants.kLeftEncoderPorts.get(1),
        DriveConstants.kLeftEncoderReversed
    )

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    // The right-side drive encoder
    val rightEncoder: Encoder = Encoder(
        DriveConstants.kRightEncoderPorts.get(0),
        DriveConstants.kRightEncoderPorts.get(1),
        DriveConstants.kRightEncoderReversed
    )

    /** Creates a new DriveSubsystem.  */
    init {
        SendableRegistry.addChild(m_drive, m_leftLeader)
        SendableRegistry.addChild(m_drive, m_rightLeader)

        // Sets the distance per pulse for the encoders
        leftEncoder.distancePerPulse = DriveConstants.kEncoderDistancePerPulse
        rightEncoder.distancePerPulse = DriveConstants.kEncoderDistancePerPulse

        m_leftLeader.addFollower(m_leftFollower)
        m_rightLeader.addFollower(m_rightFollower)

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightLeader.inverted = true
    }

    /**
     * A split-stick arcade command, with forward/backward controlled by the left hand, and turning
     * controlled by the right.
     *
     * @param fwd supplier for the commanded forward movement
     * @param rot supplier for the commanded rotation
     */
    fun arcadeDriveCommand(fwd: DoubleSupplier, rot: DoubleSupplier): Command {
        return Commands.run({
            m_drive.arcadeDrive(
                fwd.asDouble,
                rot.asDouble
            )
        }, this)
    }

    /** Resets the drive encoders to currently read a position of 0.  */
    fun resetEncoders() {
        leftEncoder.reset()
        rightEncoder.reset()
    }

    val averageEncoderDistance: Double
        /**
         * Gets the average distance of the two encoders.
         *
         * @return the average of the two encoder readings
         */
        get() = (leftEncoder.distance + rightEncoder.distance) / 2.0

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    fun limitOutputCommand(maxOutput: Double): Command {
        return Commands.runOnce({ m_drive.setMaxOutput(maxOutput) })
    }
}