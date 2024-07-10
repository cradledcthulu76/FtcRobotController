package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private DcMotor frm;
    private DcMotor flm;
    private DcMotor brm;
    private DcMotor blm;

    public void init(HardwareMap hardwareMap){
        flm = hardwareMap.get(DcMotor.class, "front-left-motor");
        frm = hardwareMap.get(DcMotor.class, "front-right-motor");
        brm = hardwareMap.get(DcMotor.class, "back-right-motor");
        blm = hardwareMap.get(DcMotor.class, "back-left-motor");
        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.FORWARD);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

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





    }

    public void FLM(){
        flm.setPower(0.5);
    }

    public void FRM(){
        frm.setPower(0.5);
    }

    public void BLM(){
        blm.setPower(0.5);
    }

    public void BRM(){
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
    public void TurnRight(){
        flm.setPower(0.5);
        frm.setPower(-0.5);
        blm.setPower(0.5);
        brm.setPower(-0.5);
    }
    public void Stopper(){
        flm.setPower(0);
        frm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);
    }

}

