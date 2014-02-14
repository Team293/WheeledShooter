/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 * @author Peter
 */
public class Ports {

    public static final int //joysticks
            leftJoystick = 1,
            rightJoystick = 2,
            gamepad = 3,
            //buttons
            //___right joystick
            fire = 1,
            toggleDriveDirection = 3,
            //___gamepad
            toggleFeeder = 1,
            pass = 2,
            //axes
            gamepadLeftXAxis = 1,
            gamepadLeftYAxis = 2,
            gamepadRightXAxis = 3,
            gamepadRightYAxis = 4,
            //motors
            leftDrive = 1,
            rightDrive = 2,
            feeder = 1,
            trigger = 2,
            cageRelease = 3,
            shooterLow = 3,
            shooterMiddle = 4,
            shooterHigh = 5,
            cameraServo = 10,
            //encoders
            shooter1EncA = 2,
            shooter1EncB = 3,
            shooter2EncA = 4,
            shooter2EncB = 5,
            shooter3EncA = 6,
            shooter3EncB = 7,
            //gyros
            gyro = 1,
            //Ultrasonics
            rightUltrasonic = 2,
            leftUltrasonic = 3,
            //Digital Outputs
            ultrasonicSignal = 9,
            led1 = 10,
            led2 = 11,
            led3 = 12,
            led4 = 13,
            //optical limtis
            ballLimit = 1;
}
