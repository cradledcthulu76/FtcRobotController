package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.util.MecanumDrive;

@Autonomous
public class TryAtEncoders extends LinearOpMode {

    MecanumDrive MecanumDriveObj = new MecanumDrive();
    public double TicksPerRev = 383.6;
    public void getpos(){
        telemetry.addData("positions", MecanumDriveObj.brm.getCurrentPosition());
        telemetry.addData("position", MecanumDriveObj.blm.getCurrentPosition());
        telemetry.addData("position", MecanumDriveObj.frm.getCurrentPosition());
        telemetry.addData("position", MecanumDriveObj.flm.getCurrentPosition());
    }

    public void runOpMode() {
        MecanumDriveObj.init(hardwareMap);
        waitForStart();
        MecanumDriveObj.flm.getCurrentPosition();
        MecanumDriveObj.frm.getCurrentPosition();
        MecanumDriveObj.blm.getCurrentPosition();
        MecanumDriveObj.brm.getCurrentPosition();

        MecanumDriveObj.DriveForwardEncoder(0.1, 16);
        getpos();
        MecanumDriveObj.StrafeEncoder(0.1,32, "Left");

    }
}
