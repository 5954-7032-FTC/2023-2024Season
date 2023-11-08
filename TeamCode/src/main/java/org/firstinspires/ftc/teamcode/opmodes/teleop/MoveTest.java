package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.util.RobotDevices;
import org.firstinspires.ftc.teamcode.util.TweakableDouble;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;

@TeleOp(name = "LinearMoveTest")
@Disabled
public class MoveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDevices robotDevices=RobotDevices.getDevices(hardwareMap);

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.telemetry = telemetry;
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0,1,2,3};
        driveParameters.FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        MecanumDrive drive = new MecanumDriveImpl(driveParameters);


        motorRampProfile _Joy1X, _Joy1Y, _Joy2X;
        TweakableDouble _ramp_rate_J1X =  new TweakableDouble("RampRateJ1X", 0.02, 1.5);
        TweakableDouble _ramp_rate_J1Y =  new TweakableDouble("RampRateJ1Y", 0.02, 1.5);
        TweakableDouble _ramp_rate_J2X =  new TweakableDouble("RampRateJ2X", 0.02, 1.5);
        TweakableDouble _fine_control = new TweakableDouble("FineControl", 0.05,0.55);
        TweakableDouble _zone_lateral = new TweakableDouble("LateralZone",0.02, 0.2);
        TweakableDouble _zone_forward =new TweakableDouble("ForwardZone", 0.02, 0.2 );
        TweakableDouble _zone_rotation =new TweakableDouble("RotateZone", 0.02, 0.1);

        _Joy1Y = new motorRampProfile(_ramp_rate_J1X.value);
        _Joy1X = new motorRampProfile(_ramp_rate_J1Y.value);
        _Joy2X = new motorRampProfile(_ramp_rate_J2X.value);


        waitForStart();

        while(opModeIsActive()) {
            telemetry.update();
            if (gamepad1.right_trigger > 0.2) {
                drive.moveRect(
                        _Joy1Y.ramp(deadzone(gamepad1.left_stick_y * _fine_control.value, _zone_forward.value)),
                        _Joy1X.ramp(deadzone(gamepad1.left_stick_x * _fine_control.value, _zone_lateral.value)),
                        _Joy2X.ramp(deadzone(gamepad1.right_stick_x * _fine_control.value, _zone_rotation.value))
                );
            } else {
                drive.moveRect(
                        _Joy1Y.ramp(deadzone(gamepad1.left_stick_y, _zone_forward.value)),
                        _Joy1X.ramp(deadzone(gamepad1.left_stick_x, _zone_lateral.value)),
                        _Joy2X.ramp(deadzone(gamepad1.right_stick_x, _zone_rotation.value))
                );
            }
        }

    }

    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }
}
