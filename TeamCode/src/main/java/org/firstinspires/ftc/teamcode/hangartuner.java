package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.MecanumDrive;
@TeleOp(name="The hangman")
    public class hangartuner extends OpMode {
        MecanumDrive MecanumDriveObj = new MecanumDrive();
        @Override
        public void init() {
            MecanumDriveObj.init(hardwareMap);
            MecanumDriveObj.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MecanumDriveObj.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        @Override
        public void loop() {
            MecanumDriveObj.hang.setPower(-gamepad1.left_stick_y);
            telemetry.addData("hang position",MecanumDriveObj.hang.getCurrentPosition());
            telemetry.update();
        }
    }
