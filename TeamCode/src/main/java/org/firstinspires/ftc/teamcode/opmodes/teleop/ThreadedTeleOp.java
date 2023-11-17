package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.threads.ArmControlThread;
import org.firstinspires.ftc.teamcode.threads.MovementThread;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

@TeleOp(name = "TeleOp")
public class ThreadedTeleOp extends OpMode {

    ArmControlThread _arm;
    MovementThread _move;

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
                robotDevices.lowerArmLimit,
                robotDevices.pixelHold
        );


        _move = new MovementThread(gamepad1,robotDevices.wheels,telemetry,robotDevices.imunew,robotDevices.droneRelease,robotDevices.sensorServos,robotDevices.wallSensor);

        _threadCount = telemetry.addData("Threads", Thread.activeCount());


    }

    @Override
    public void start() {
        super.start();
        _arm.start();
        _move.start();
        robotDevices.leftPixelFlip.setPosition(0.0);
        robotDevices.rightPixelFlip.setPosition(1.0);
        try {
            Thread.sleep(Constants.pixelDropPositions.LEFT_RESET.ms>Constants.pixelDropPositions.RIGHT_RESET.ms?Constants.pixelDropPositions.LEFT_RESET.ms:Constants.pixelDropPositions.RIGHT_RESET.ms );
            }
            catch (Exception nope) {
            }
        robotDevices.leftPixelArm.setPosition(Constants.pixelDropPositions.LEFT_RESET.pos);
        robotDevices.rightPixelArm.setPosition(Constants.pixelDropPositions.RIGHT_RESET.pos);
    }

    @Override
    public void loop() {
        _threadCount.setValue(Thread.activeCount());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
        _arm.cancel();
        _move.cancel();

    }

}
