// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.armbotoffboard

/**
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 *
 * Has no actual functionality.
 * 
 * @param port The port number of the motor controller.
 */
class ExampleSmartMotorController(private val port: Int) {
    
    enum class PIDMode {
        kPosition,
        kVelocity,
        kMovementWitchcraft
    }

    /**
     * Example method for setting the PID gains of the smart controller.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     */
    fun setPID(kp: Double, ki: Double, kd: Double) {}

    /**
     * Example method for setting the setpoint of the smart controller in PID mode.
     *
     * @param mode The mode of the PID controller.
     * @param setpoint The controller setpoint.
     * @param arbFeedforward An arbitrary feedforward output (from -1 to 1).
     */
    fun setSetpoint(mode: PIDMode?, setpoint: Double, arbFeedforward: Double) {}

    /**
     * Places this motor controller in follower mode.
     *
     * @param leader The leader to follow.
     */
    fun follow(leader: ExampleSmartMotorController?) {}

    val encoderDistance: Double
        /**
         * Returns the encoder distance.
         *
         * @return The current encoder distance.
         */
        get() = 0.0

    val encoderRate: Double
        /**
         * Returns the encoder rate.
         *
         * @return The current encoder rate.
         */
        get() = 0.0

    /** Resets the encoder to zero distance.  */
    fun resetEncoder() {}

    fun set(speed: Double) {}

    fun get(): Double {
        return 0.0
    }

    var inverted: Boolean
        get() = false
        set(isInverted) {}

    fun disable() {}

    fun stopMotor() {}
}