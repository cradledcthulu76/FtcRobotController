package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.colesMap;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

@Autonomous
public class colesCode extends OpMode {
    MecanumDrive MecanumDriveObj = new MecanumDrive();
    colesMap r = new colesMap();

    @Override
    public void init() {
        MecanumDriveObj.init(hardwareMap);
    }
    @Override
    public void start(){resetRuntime();}

    @Override
    public void loop() {
        while (getRuntime() >= 4){
            r.coleStraight();}
        while (getRuntime() >=8) {
            MecanumDriveObj.TurnRight();
        }
        while ( getRuntime() >= 10){
            MecanumDriveObj.Backwards();
        }
    }
}
