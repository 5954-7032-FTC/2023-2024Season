package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorDeviceImpl;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

@Autonomous(name = "Auto-Right-BLUE")
public class AutoRightBlue extends AutoLinearBase {
    @Override
    public int side() {
        return RIGHT_SIDE;
    }

    @Override
    public double turnDirection() {
        return this.RIGHT;
    }

    @Override
    public ColorSensorDeviceImpl getColorSensorDevice() {
        return colorSensorDeviceLeft;
    }

    @Override
    public void strafeDirection(double distance) {
        driveLeft(distance);
    }

    @Override
    public void strafeAntiDirection(double distance) {
        driveRight(distance);
    }

    @Override
    public void lightOn() {
        light.blueOn();
    }
    @Override
    public Lights getLight() {
        return new Lights(hardwareMap.dcMotor.get("LIGHTS")){public void on() {light.blueOn();}};
    }
}