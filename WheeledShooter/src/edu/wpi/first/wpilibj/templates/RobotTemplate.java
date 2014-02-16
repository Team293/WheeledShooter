/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* tFhe project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpikeEncoder;
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
    RobotDrive drive = new RobotDrive(leftMotor, rightMotor);

    Talon shooterLow = new Talon(Ports.shooterLow),
            shooterMiddle = new Talon(Ports.shooterMiddle),
            shooterHigh = new Talon(Ports.shooterHigh);

    Relay trigger = new Relay(Ports.trigger),
            feederMotor = new Relay(Ports.feeder),
            feederMotor2 = new Relay(Ports.feeder2);

    Joystick leftJoystick = new Joystick(Ports.leftJoystick),
            rightJoystick = new Joystick(Ports.rightJoystick),
            gamepad = new Joystick(Ports.gamepad);

    SpikeEncoder enc1 = new SpikeEncoder(Ports.shooterLowEncA, Ports.shooterLowEncB, SpikeEncoder.BLACK),
            enc2 = new SpikeEncoder(Ports.shooterMiddleEncA, Ports.shooterMiddleEncB, SpikeEncoder.BLACK),
            enc3 = new SpikeEncoder(Ports.shooterHighEncA, Ports.shooterHighEncB, SpikeEncoder.BLACK);

    SpikeButton pass = new SpikeButton(gamepad, Ports.pass),
            toggleFeeder = new SpikeButton(gamepad, Ports.toggleFeeder),
            fire = new SpikeButton(rightJoystick, Ports.fire),
            autoDistance = new SpikeButton(leftJoystick, 1),
            toggleDriveDirection = new SpikeButton(rightJoystick, Ports.toggleDriveDirection);

    Servo cageRelease = new Servo(Ports.cageRelease);

    DigitalInput ballLimit = new DigitalInput(Ports.ballLimit);

    Gyro gyro = new Gyro(Ports.gyro);

    AnalogChannel leftUltrasonic = new AnalogChannel(Ports.leftUltrasonic);
    AnalogChannel rightUltrasonic = new AnalogChannel(Ports.rightUltrasonic);
    DigitalOutput ultrasonicSignal = new DigitalOutput(Ports.ultrasonicSignal);
    boolean shooting = false;
    int ping = 0;
    double leftDistance = 0, rightDistance;

    public void robotInit() {
        addComponents();
        enc1.start();
        enc2.start();
        enc3.start();
        cageRelease.set(0);
    }

    public void teleopInit() {
        SmartDashboard.putNumber("1", -0.4);
        SmartDashboard.putNumber("2", 0.4);
        SmartDashboard.putNumber("3", 0.4);
        trigger.set(Relay.Value.kForward);
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
        //ping ultrasonic sensors
        ping++;
        if (ping % 5 == 0) {
            ultrasonicSignal.pulse(0.0001);
        }
        leftDistance = convertToDistance(leftUltrasonic.getAverageVoltage());
        rightDistance = convertToDistance(rightUltrasonic.getAverageVoltage());

        SmartDashboard.putNumber("leftD", leftDistance);
        SmartDashboard.putNumber("right", rightDistance);
        SmartDashboard.putBoolean("feeder state", toggleFeeder.getState());
        SmartDashboard.putBoolean("ball limit", ballLimit.get());
        SmartDashboard.putBoolean("pass", pass.get());
        SmartDashboard.putBoolean("shooting", shooting);
        double speed1 = SmartDashboard.getNumber("1", 0.0);
        double speed2 = SmartDashboard.getNumber("2", 0.0);
        double speed3 = SmartDashboard.getNumber("3", 0.0);
        SmartDashboard.putNumber("enc1 RPM", enc1.getRPM());
        SmartDashboard.putNumber("enc2 RPM", enc2.getRPM());
        SmartDashboard.putNumber("enc3 RPM", enc3.getRPM());

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
                SmartDashboard.putString("feeder value", "passing!");
            } else if (toggleFeeder.getState()) {
                //ball limit is wired in reverse!~~~
                if (ballLimit.get()) {
                    feederValue = Relay.Value.kForward;
                    SmartDashboard.putString("feeder value", "feeding!");
                } else {
                    feederValue = Relay.Value.kOff;
                    SmartDashboard.putString("feeder value", "stopped!");
                }
            } else {
                feederValue = Relay.Value.kOff;
                SmartDashboard.putString("feeder value", "off state!");
            }
        } else {
            trigger.set(Relay.Value.kOff);
            feederValue = Relay.Value.kForward;
            if (ballLimit.get()) {
                shooting = false;
                trigger.set(Relay.Value.kForward);
            }
        }
        SmartDashboard.putString("final feederValue", feederValue + "");
        feederMotor.set(feederValue);
        feederMotor2.set(feederValue);
        isAligned();
        if (isAtDistance() && autoDistance.get()) {
            moveToDistance();
        } else {
            if (toggleDriveDirection.getState()) {
                drive.tankDrive(leftJoystick.getY(), rightJoystick.getY());
            } else {
                drive.tankDrive(-rightJoystick.getY(), -leftJoystick.getY());
            }

        }
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
        LiveWindow.addSensor("shooter rack", "shooter enc 1", enc1);
        LiveWindow.addSensor("shooter rack", "shooter enc 2", enc2);
        LiveWindow.addSensor("shooter rack", "shooter enc 3", enc3);
        LiveWindow.addActuator("trigger", "trigger", trigger);
        LiveWindow.addSensor("trigger", "ball limit", ballLimit);
        LiveWindow.addSensor("gyro", "gyro", gyro);
        LiveWindow.addActuator("feeder", "feeder", feederMotor);
    }

    public double convertToDistance(double rawVoltage) {
        return (rawVoltage + 0.0056) / 0.12;
    }

    public boolean isAligned() {
        double difference = leftDistance - rightDistance;
        SmartDashboard.putNumber("aligned", difference);
        SmartDashboard.putBoolean("aligned", difference < 0.4);
        if (difference < 0.4) {
            return true;
        }
        return false;
    }

    public boolean isAtDistance() {
        double difference = leftDistance - rightDistance;
        double average = (leftDistance + rightDistance) / 2.0;
        SmartDashboard.putBoolean("distanced", Math.abs(average - 12) < 1);
        if (difference < 5 && Math.abs(average - 12) < 1) {
            return true;
        }
        return false;
    }

    public void moveToDistance() {
        double difference = leftDistance - rightDistance;
        double average = (leftDistance + rightDistance) / 2.0;
        SmartDashboard.putNumber("average dstiance", average);
        if (difference < 5) {
            drive.tankDrive(0.4, 0.4);
        } else {
            drive.tankDrive(0, 0);
        }
    }
}
