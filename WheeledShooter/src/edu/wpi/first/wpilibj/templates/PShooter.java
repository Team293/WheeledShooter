/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SpikeEncoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Robotics Laptop B
 */
public class PShooter{

    static int ID = 0;
    public Talon motor;
    public SpikeEncoder encoder;
    int id;
    double kP, setpoint, rpm;
    boolean reversed, enabled;

    /**
     * 
     * @param motorPort
     * @param encoderA
     * @param encoderB
     * @param kP
     * @param setpoint
     * @param reversed 
     */
    PShooter(int motorPort, int encoderA, int encoderB, double kP, double setpoint, boolean reversed) {
        motor = new Talon(motorPort);
        encoder = new SpikeEncoder(encoderA, encoderB);
        this.kP = 0.0015;
        this.setpoint = setpoint;
        this.id = ID;
        this.reversed = reversed;
        this.enabled = false;
        ID++;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enable() {
        enabled = true;
        encoder.start();
        encoder.reset();
        SmartDashboard.putNumber("kP " + id, kP);
        SmartDashboard.putNumber("setpoint " + id, setpoint);
    }

    public void run() {
        double output = 0, error = 0;
        if (enabled) {
            kP = SmartDashboard.getNumber("kP " + id, kP);
            setpoint = SmartDashboard.getNumber("setpoint " + id, setpoint);
            rpm = encoder.getRPM();
            error = setpoint - rpm;
            output = kP * error;
            motor.set(output);
        }
        SmartDashboard.putNumber("_kP " + id, kP);
        SmartDashboard.putNumber("_error" + id, error);
        SmartDashboard.putNumber("_output " + id, output);
        SmartDashboard.putNumber("_rpm " + id, rpm);
    }

    public void disable() {
        enabled = false;
    }

    public void setConstant(double kP) {
        this.kP = kP;
    }

    public boolean onTarget() {
        return Math.abs(setpoint - rpm) < 40;
    }

    public void stop() {
        motor.set(0);
    }
}
