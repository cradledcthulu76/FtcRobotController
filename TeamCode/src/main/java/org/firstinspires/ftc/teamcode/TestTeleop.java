package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.MecanumDrive;

@TeleOp
public class TestTeleop extends OpMode {
    MecanumDrive MecanumDriveObj = new MecanumDrive();
    @Override
    public void init(){
        MecanumDriveObj.init(hardwareMap);
    }
    @Override
    public void loop(){
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        MecanumDriveObj.drive(forward,right,rotate);
    }
}
