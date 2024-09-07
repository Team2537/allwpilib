// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.armbot.subsystems

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibk.examples.armbot.Constants.ArmConstants
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem

/** A robot arm subsystem that moves with a motion profile.  */
class ArmSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        ArmConstants.kP,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond,
            ArmConstants.kMaxAccelerationRadPerSecSquared
        )
    ),
    0.0
) {
    private val m_motor = PWMSparkMax(ArmConstants.kMotorPort)
    private val m_encoder: Encoder = Encoder(ArmConstants.kEncoderPorts.get(0), ArmConstants.kEncoderPorts.get(1))
    private val m_feedforward = ArmFeedforward(
        ArmConstants.kSVolts, ArmConstants.kGVolts,
        ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad
    )

    /** Create a new ArmSubsystem.  */
    init {
        m_encoder.distancePerPulse = ArmConstants.kEncoderDistancePerPulse
        // Start arm at rest in neutral position
        setGoal(ArmConstants.kArmOffsetRads)
    }

    public override fun useOutput(output: Double, setpoint: TrapezoidProfile.State) {
        // Calculate the feedforward from the sepoint
        val feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity)
        // Add the feedforward to the PID output to get the motor output
        m_motor.setVoltage(output + feedforward)
    }

    public override fun getMeasurement(): Double {
        return m_encoder.distance + ArmConstants.kArmOffsetRads
    }
}