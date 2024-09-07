// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.armbotoffboard.subsystems

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibk.examples.armbotoffboard.Constants.ArmConstants
import edu.wpi.first.wpilibk.examples.armbotoffboard.ExampleSmartMotorController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem

/** A robot arm subsystem that moves with a motion profile.  */
class ArmSubsystem : TrapezoidProfileSubsystem(
    TrapezoidProfile.Constraints(
        ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared
    ),
    ArmConstants.kArmOffsetRads
) {
    private val m_motor: ExampleSmartMotorController = ExampleSmartMotorController(ArmConstants.kMotorPort)
    private val m_feedforward = ArmFeedforward(
        ArmConstants.kSVolts, ArmConstants.kGVolts,
        ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad
    )

    /** Create a new ArmSubsystem.  */
    init {
        m_motor.setPID(ArmConstants.kP, 0.0, 0.0)
    }

    public override fun useState(setpoint: TrapezoidProfile.State) {
        // Calculate the feedforward from the sepoint
        val feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity)
        // Add the feedforward to the PID output to get the motor output
        m_motor.setSetpoint(
            ExampleSmartMotorController.PIDMode.kPosition, setpoint.position, feedforward / 12.0
        )
    }

    fun setArmGoalCommand(kArmOffsetRads: Double): Command {
        return Commands.runOnce({ setGoal(kArmOffsetRads) }, this)
    }
}