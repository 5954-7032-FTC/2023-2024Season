package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.threads.ArmControlThread;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.RobotDevices;
import org.firstinspires.ftc.teamcode.threads.TweakableMovementThread;


//threaded tele op controller......
public abstract class ThreadedTeleOp extends OpMode {

    TweakableMovementThread _move;
    ArmControlThread _arm;
    Lights _light;

    Telemetry.Item _threadCount;//,_bot_cone;
    private BNO055IMU imu         = null;
    RobotDevices robotDevices;

    @Override
    public void init() {
        telemetry.setAutoClear(false);

        robotDevices =  RobotDevices.getDevices(hardwareMap);

        imu = robotDevices.imu;

        _move = new TweakableMovementThread(gamepad1, robotDevices.wheels, telemetry, imu, 500, false);
        _arm = new ArmControlThread(gamepad2,
                telemetry,
                robotDevices.intakeServos,
                robotDevices.lowBelt,
                robotDevices.highBelt,
                robotDevices.armLift,
                robotDevices.upperArmLimit,
                robotDevices.lowerArmLimit
        );

        _light = getLights();

        _threadCount = telemetry.addData("Threads", Thread.activeCount());

    }



    public abstract Lights getLights();

    @Override
    public void start() {
        _move.start();
        _arm.start();
    }

    @Override
    public void loop() {
        _light.on();
        _threadCount.setValue(Thread.activeCount());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        _light.off();
        _move.cancel();
        _arm.cancel();
    }


}
