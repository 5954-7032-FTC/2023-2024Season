package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.util.Color;

public class ColorSensorDeviceImpl implements SubSystem, ColorSensorDevice {
    private final ColorSensor _colorSensor;

    public ColorSensorDeviceImpl(ColorSensor colorSensor) {
        _colorSensor = colorSensor;
    }


    public Color findMaxColor() {
        int red = _colorSensor.red();
        int blue = _colorSensor.blue();
        int green = _colorSensor.green();
        if ((red > blue) &&  (red > green)) return Color.RED;
        else if (blue > green) return Color.BLUE;
        else return Color.GREEN;
    }
}
