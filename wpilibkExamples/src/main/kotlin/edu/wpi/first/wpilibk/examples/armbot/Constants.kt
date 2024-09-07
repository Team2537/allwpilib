// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.armbot

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    object DriveConstants {
        const val kLeftMotor1Port: Int = 0
        const val kLeftMotor2Port: Int = 1
        const val kRightMotor1Port: Int = 2
        const val kRightMotor2Port: Int = 3

        val kLeftEncoderPorts: IntArray = intArrayOf(0, 1)
        val kRightEncoderPorts: IntArray = intArrayOf(2, 3)
        const val kLeftEncoderReversed: Boolean = false
        const val kRightEncoderReversed: Boolean = true

        const val kEncoderCPR: Int = 1024
        const val kWheelDiameterInches: Double = 6.0
        const val kEncoderDistancePerPulse: Double =  // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / kEncoderCPR
    }

    object ArmConstants {
        const val kMotorPort: Int = 4

        const val kP: Double = 1.0

        // These are fake gains; in actuality these must be determined individually for each robot
        const val kSVolts: Double = 1.0
        const val kGVolts: Double = 1.0
        const val kVVoltSecondPerRad: Double = 0.5
        const val kAVoltSecondSquaredPerRad: Double = 0.1

        const val kMaxVelocityRadPerSecond: Double = 3.0
        const val kMaxAccelerationRadPerSecSquared: Double = 10.0

        val kEncoderPorts: IntArray = intArrayOf(4, 5)
        const val kEncoderPPR: Int = 256
        const val kEncoderDistancePerPulse: Double = 2.0 * Math.PI / kEncoderPPR

        // The offset of the arm from the horizontal in its neutral position,
        // measured from the horizontal
        const val kArmOffsetRads: Double = 0.5
    }

    object AutoConstants {
        const val kAutoTimeoutSeconds: Double = 12.0
        const val kAutoShootTimeSeconds: Double = 7.0
    }

    object OIConstants {
        const val kDriverControllerPort: Int = 0
    }
}