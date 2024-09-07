// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibk.examples.addressableled

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond

object Robot : TimedRobot() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    private val m_led: AddressableLED = AddressableLED(9)
    
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    private val m_ledBuffer: AddressableLEDBuffer = AddressableLEDBuffer(60)

    // Create an LED pattern that will display a rainbow across
    // all hues at maximum saturation and half brightness
    private val m_rainbow = LEDPattern.rainbow(255, 128)

    // Our LED strip has a density of 120 LEDs per meter
    private val kLedSpacing: Measure<Distance> = Meters.of(1 / 120.0)


    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.
    private val m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), kLedSpacing)

    /** Called once at the beginning of the robot program. */
    init {
        m_led.setLength(m_ledBuffer.length)

        // Set the data
        m_led.setData(m_ledBuffer)
        m_led.start()
    }

    override fun robotPeriodic() {
        // Update the buffer with the rainbow animation
        m_scrollingRainbow.applyTo(m_ledBuffer)
        // Set the LEDs
        m_led.setData(m_ledBuffer)
    }
}

