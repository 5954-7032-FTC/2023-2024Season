package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.threads.ArmControlThread;
import org.firstinspires.ftc.teamcode.threads.TweakableMovementThread;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

@TeleOp(name = "TeleOp-ArmOnly")
public class ThreadedArmOnlyTeleop extends OpMode {

    ArmControlThread _arm;

    Telemetry.Item _threadCount;//,_bot_cone;
    RobotDevices robotDevices;


    @Override
    public void init() {
        robotDevices = RobotDevices.getDevices(hardwareMap);
        _arm = new ArmControlThread(gamepad2,
                telemetry,
                robotDevices.intakeServos,
                robotDevices.lowBelt,
                robotDevices.highBelt,
                robotDevices.armLift,
                robotDevices.upperArmLimit,
                robotDevices.lowerArmLimit
        );

        _threadCount = telemetry.addData("Threads", Thread.activeCount());

    }

    @Override
    public void start() {
        super.start();
        _arm.start();
    }

    @Override
    public void loop() {
        _threadCount.setValue(Thread.activeCount());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
        //_light.off();
        _arm.cancel();
    }

}
