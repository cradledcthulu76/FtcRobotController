package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDrive {
    public DcMotor frm;
    public DcMotor flm;
    public DcMotor brm;
    public DcMotor blm;
    public DcMotor arm;
    public IMU imu;

    public void init(HardwareMap hardwareMap) {
        flm = hardwareMap.get(DcMotor.class, "front-left-motor");
        frm = hardwareMap.get(DcMotor.class, "front-right-motor");
        brm = hardwareMap.get(DcMotor.class, "back-right-motor");
        blm = hardwareMap.get(DcMotor.class, "back-left-motor");
        arm = hardwareMap.get(DcMotor.class, "arm-base");
        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.FORWARD);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//back left
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //back right

        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));


    }

    public void FLM() {
        flm.setPower(0.5);
    }

    public void FRM() {
        frm.setPower(0.5);
    }

    public void BLM() {
        blm.setPower(0.5);
    }

    public void BRM() {
        brm.setPower(0.5);
    }

    public void Straight() {

        flm.setPower(0.5);
        frm.setPower(0.5); //back left
        blm.setPower(0.5);
        brm.setPower(0.5); //back right
    }

    public void StraightRight() {
        flm.setPower(0.5);
        frm.setPower(0); //back left
        blm.setPower(0);
        brm.setPower(0.5); //back right//
    }

    public void TurnRight() {
        flm.setPower(0.5);
        frm.setPower(-0.5);
        blm.setPower(0.5);
        brm.setPower(-0.5);
    }

    public void Stopper() {
        flm.setPower(0);
        frm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);
    }

    public void armmove() {
        arm.setPower(0.3);

    }
    public void Backwards(){
        flm.setPower(-0.5);
        frm.setPower(-0.5);
        blm.setPower(-0.5);
        brm.setPower(-0.5);
    }
    private void setPowers(double flp, double frp, double blp, double brp){
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed,Math.abs(flp));
        maxSpeed = Math.max(maxSpeed,Math.abs(frp));
        maxSpeed = Math.max(maxSpeed,Math.abs(blp));
        maxSpeed = Math.max(maxSpeed,Math.abs(brp));

        flp /= maxSpeed;
        frp /= maxSpeed;
        blp /= maxSpeed;
        brp /= maxSpeed;

        flm.setPower(flp);
        frm.setPower(frp);
        blm.setPower(blp);
        brm.setPower(brp);

    }
    public void drive(double forward, double right, double rotate){
        double flp =forward + right + rotate;
        double frp =forward - right - rotate;
        double blp =forward - right + rotate;
        double brp =forward + right - rotate;
        setPowers(flp,frp,blp,brp);
    }
}
