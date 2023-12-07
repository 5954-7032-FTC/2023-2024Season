package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorDevice implements SubSystem {
    DistanceSensor device;

    public DistanceSensorDevice(DistanceSensor device) {
        this.device = device;
    }

    public boolean checkDistanceMM(double distance) {
        return (device.getDistance(DistanceUnit.MM) < distance);
    }

    public boolean checkDistanceIN(double distance) {
        return (device.getDistance(DistanceUnit.INCH) < distance);
    }

    public double getDistanceMM() {
        return device.getDistance(DistanceUnit.MM);
    }
}
