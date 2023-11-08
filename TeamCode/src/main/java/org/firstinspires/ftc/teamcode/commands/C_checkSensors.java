package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public class C_checkSensors implements Command {
    DistanceSensor Front, Back;


    enum Place {
        FRONT, BACK, SIDE,UNKNOWN
    }
    Place place=Place.UNKNOWN;

    public C_checkSensors(DistanceSensor Front, DistanceSensor Back) {
        this.Front = Front;
        this.Back = Back;
    }

    public Place getPlace() {
        return place;
    }

    @Override
    public void Execute() {
        if (Front.getDistance(DistanceUnit.MM) < 12) {
            place = Place.FRONT;
        }
        else if (Back.getDistance(DistanceUnit.MM) < 12) {
            place = Place.BACK;
        }
        else {
            place = Place.SIDE;
        }
    }

    @Override
    public SubSystem getHardwareDevice() {
        return null;
    }
}
