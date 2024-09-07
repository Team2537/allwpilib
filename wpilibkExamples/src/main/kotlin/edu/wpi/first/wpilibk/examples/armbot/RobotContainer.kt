// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package edu.wpi.first.wpilibk.examples.armbot

import edu.wpi.first.wpilibk.examples.armbot.Constants.OIConstants
import edu.wpi.first.wpilibk.examples.armbot.subsystems.ArmSubsystem
import edu.wpi.first.wpilibk.examples.armbot.subsystems.DriveSubsystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems
    private val m_robotDrive: DriveSubsystem = DriveSubsystem()
    private val m_robotArm: ArmSubsystem = ArmSubsystem()

    // The driver's controller
    var m_driverController: CommandXboxController = CommandXboxController(OIConstants.kDriverControllerPort)

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand( // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            Commands.run(
                {
                    m_robotDrive.arcadeDrive(
                        -m_driverController.leftY, -m_driverController.rightX
                    )
                },
                m_robotDrive
            )
        )
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [edu.wpi.first.wpilibj.GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        // Move the arm to 2 radians above horizontal when the 'A' button is pressed.
        m_driverController
            .a()
            .onTrue(
                Commands.runOnce(
                    {
                        m_robotArm.setGoal(2.0)
                        m_robotArm.enable()
                    },
                    m_robotArm
                )
            )

        // Move the arm to neutral position when the 'B' button is pressed.
        m_driverController
            .b()
            .onTrue(
                Commands.runOnce(
                    {
                        m_robotArm.setGoal(Constants.ArmConstants.kArmOffsetRads)
                        m_robotArm.enable()
                    },
                    m_robotArm
                )
            )

        // Disable the arm controller when Y is pressed.
        m_driverController.y().onTrue(Commands.runOnce(m_robotArm::disable))

        // Drive at half speed when the bumper is held
        m_driverController
            .rightBumper()
            .onTrue(Commands.runOnce({ m_robotDrive.setMaxOutput(0.5) }))
            .onFalse(Commands.runOnce({ m_robotDrive.setMaxOutput(1.0) }))
    }

    /**
     * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
     * disable to prevent integral windup.
     */
    fun disablePIDSubsystems() {
        m_robotArm.disable()
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = Commands.none()
}