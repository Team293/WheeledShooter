/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.buttons.SpikeButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    Talon leftMotor = new Talon(Ports.leftDrive),
            rightMotor = new Talon(Ports.rightDrive);

    Talon shooter1 = new Talon(Ports.shooter1),
            shooter2 = new Talon(Ports.shooter2),
            shooter3 = new Talon(Ports.shooter3);

    Relay trigger = new Relay(Ports.trigger),
            feederMotor = new Relay(Ports.feeder);
    RobotDrive drive = new RobotDrive(leftMotor, rightMotor);

    Joystick leftJoystick = new Joystick(1),
            rightJoystick = new Joystick(2),
            gamepad = new Joystick(3);

    SpikeButton pass = new SpikeButton(gamepad, 2),
            toggleFeeder = new SpikeButton(gamepad, 1),
            shootButton = new SpikeButton(rightJoystick, 1);

    DigitalInput ballLimit = new DigitalInput(Ports.ballLimit);
    boolean shooting = false;

    Encoder enc1 = new Encoder(Ports.shooter1EncA, Ports.shooter1EncB, true, CounterBase.EncodingType.k4X);
    Encoder enc2 = new Encoder(Ports.shooter2EncA, Ports.shooter2EncB, true, CounterBase.EncodingType.k4X);
    Encoder enc3 = new Encoder(Ports.shooter3EncA, Ports.shooter3EncB, true, CounterBase.EncodingType.k4X);

    public void robotInit() {
        //Test.addComponents();
        SmartDashboard.putNumber("1", 0.0);
        SmartDashboard.putNumber("2", 0.0);
        SmartDashboard.putNumber("3", 0.0);
        enc1.start();
        enc2.start();
        enc3.start();
        enc1.setDistancePerPulse(0.0128);
        enc2.setDistancePerPulse(0.0128);
        enc3.setDistancePerPulse(0.0128);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        double speed1 = SmartDashboard.getNumber("1", 0.0);
        double speed2 = SmartDashboard.getNumber("2", 0.0);
        double speed3 = SmartDashboard.getNumber("3", 0.0);

        drive.tankDrive(leftJoystick.getY(), rightJoystick.getY());

        SmartDashboard.putBoolean("feeder state", toggleFeeder.getState());
        SmartDashboard.putBoolean("ball limit", ballLimit.get());
        SmartDashboard.putBoolean("pass", pass.get());
        SmartDashboard.putBoolean("shooting", shooting);

        /* speed values range from 0 to 1 & 1=100% */
        shooter1.set(speed1);
        shooter2.set(speed2);
        shooter3.set(speed3);

        if (shootButton.getClick()) {
            shooting = true;
        }

        Relay.Value feederValue;

        if (!shooting) {
            if (pass.get()) {
                feederValue = Relay.Value.kReverse;
            } else if (toggleFeeder.getState()) {
                if (!ballLimit.get()) {
                    feederValue = Relay.Value.kForward;
                } else {
                    feederValue = Relay.Value.kOff;
                }
            } else {
                feederValue = Relay.Value.kOff;
            }
        } else {
            trigger.set(Relay.Value.kOff);
            feederValue = Relay.Value.kForward;
            if (!ballLimit.get()) {
                shooting = false;
                trigger.set(Relay.Value.kForward);
            }
        }

        feederMotor.set(feederValue);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
