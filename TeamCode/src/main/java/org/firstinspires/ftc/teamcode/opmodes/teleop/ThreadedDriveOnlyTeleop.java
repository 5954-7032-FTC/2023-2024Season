package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.threads.MovementThread;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

@TeleOp(name = "TeleOp-MoveOnly")
@Disabled
public class ThreadedDriveOnlyTeleop extends OpMode {

    MovementThread _move;

    Telemetry.Item _threadCount;//,_bot_cone;
    RobotDevices robotDevices;


    @Override
    public void init() {
        robotDevices = RobotDevices.getDevices(hardwareMap);
        IMU imu = robotDevices.imunew;
        _move = new MovementThread(gamepad1,robotDevices.wheels,telemetry, imu,robotDevices.droneRelease, robotDevices.sensorServos,robotDevices.wallSensor);

        _threadCount = telemetry.addData("Threads", Thread.activeCount());

    }

    @Override
    public void start() {
        _move.start();
    }

    @Override
    public void loop() {
        _threadCount.setValue(Thread.activeCount());
        telemetry.update();

    }

    @Override
    public void stop() {

        _move.cancel();
    }

}
