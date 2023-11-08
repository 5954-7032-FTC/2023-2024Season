package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumDriveGyroImpl extends MecanumDriveImpl {
    protected BNO055IMU _imu;

    protected Telemetry.Item T_IMU;
    public MecanumDriveGyroImpl(MecanumDriveParameters params) {
        super(params);
        _imu = params.imu;
        T_IMU = _telemetry.addData("IMU", "(0,0,0)");
    }

    @Override
    public void outputTelemetry(TelemetryTypes type) {
        switch (type) {
            case IMU:
                Orientation orientation = _imu.getAngularOrientation();
                T_IMU.setValue("("+orientation.firstAngle+","+orientation.secondAngle+","+orientation.thirdAngle+")");
                break;
            default:
                super.outputTelemetry(type);
        }
    }

    @Override
    public void movePolar(double power, double angle, double rotate) {
        super.movePolar(power, angle, rotate);
        outputTelemetry(TelemetryTypes.IMU);
    }
}
