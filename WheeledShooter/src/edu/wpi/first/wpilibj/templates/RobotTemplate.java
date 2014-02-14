/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* tFhe project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
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

    Talon shooterLow = new Talon(Ports.shooterLow),
            shooterMiddle = new Talon(Ports.shooterMiddle),
            shooterHigh = new Talon(Ports.shooterHigh);

    Relay trigger = new Relay(Ports.trigger),
            feederMotor = new Relay(Ports.feeder);
    RobotDrive drive = new RobotDrive(leftMotor, rightMotor);

    Joystick leftJoystick = new Joystick(Ports.leftJoystick),
            rightJoystick = new Joystick(Ports.rightJoystick),
            gamepad = new Joystick(Ports.gamepad);

//    SpikeEncoder enc1 = new SpikeEncoder(Ports.shooterLowEncA, Ports.shooterLowEncB),
//            enc2 = new SpikeEncoder(Ports.shooterMiddleEncA, Ports.shooterMiddleEncB),
//            enc3 = new SpikeEncoder(Ports.shooterHighEncA, Ports.shooterHighEncB);
    SpikeButton pass = new SpikeButton(gamepad, Ports.pass),
            toggleFeeder = new SpikeButton(gamepad, Ports.toggleFeeder),
            fire = new SpikeButton(rightJoystick, Ports.fire),
            toggleDriveDirection = new SpikeButton(rightJoystick, Ports.toggleDriveDirection);

    DigitalInput ballLimit = new DigitalInput(Ports.ballLimit);
    boolean shooting = false;

    public void robotInit() {
        addComponents();
        //enc1.start();
        //enc2.start();
        //enc3.start();
    }

    public void teleopInit() {
        SmartDashboard.putNumber("1", 0.4);
        SmartDashboard.putNumber("2", 0.4);
        SmartDashboard.putNumber("3", 0.4);
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
        SmartDashboard.putBoolean("feeder state", toggleFeeder.getState());
        SmartDashboard.putBoolean("ball limit", ballLimit.get());
        SmartDashboard.putBoolean("pass", pass.get());
        SmartDashboard.putBoolean("shooting", shooting);
        double speed1 = SmartDashboard.getNumber("1", 0.0);
        double speed2 = SmartDashboard.getNumber("2", 0.0);
        double speed3 = SmartDashboard.getNumber("3", 0.0);
//        SmartDashboard.putNumber("enc1 RPM", enc1.getRPM());
//        SmartDashboard.putNumber("enc2 RPM", enc2.getRPM());
//        SmartDashboard.putNumber("enc3 RPM", enc3.getRPM());

        if (toggleDriveDirection.getState()) {
            drive.tankDrive(leftJoystick.getY(), rightJoystick.getY());
        } else {
            drive.tankDrive(-rightJoystick.getY(), -leftJoystick.getY());
        }

        shooterLow.set(speed1);
        shooterHigh.set(speed3);
        shooterMiddle.set(speed2);

        if (fire.getClick()) {
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

    public void addComponents() {
        LiveWindow.addActuator("shooter rack", "shooter 1", shooterLow);
        LiveWindow.addActuator("shooter rack", "shooter 2", shooterMiddle);
        LiveWindow.addActuator("shooter rack", "shooter 3", shooterHigh);
//        LiveWindow.addSensor("shooter rack", "shooter enc 1", enc1);
//        LiveWindow.addSensor("shooter rack", "shooter enc 2", enc2);
//        LiveWindow.addSensor("shooter rack", "shooter enc 3", enc3);
        LiveWindow.addActuator("trigger", "trigger", trigger);
        LiveWindow.addSensor("trigger", "ball limit", ballLimit);
        LiveWindow.addActuator("feeder", "feeder", feederMotor);
    }
}
