package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class MecanumDrive {
    public DcMotor frm;
    public DcMotor flm;
    public DcMotor brm;
    public DcMotor blm;
    public Servo arm;
    public Servo claw;
    public IMU imu;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    double deflator = 0.5;
    public void init(HardwareMap hardwareMap) {
        flm = hardwareMap.get(DcMotor.class, "front-left-motor");
        frm = hardwareMap.get(DcMotor.class, "front-right-motor");
        brm = hardwareMap.get(DcMotor.class, "back-right-motor");
        blm = hardwareMap.get(DcMotor.class, "back-left-motor");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");

        flm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.FORWARD);
        brm.setDirection(DcMotorSimple.Direction.FORWARD);

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

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


    }
    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public double Pitch(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getPitch(angleUnit);
    }

    public double Roll(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getRoll(angleUnit);
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

    public void Backwards() {
        flm.setPower(-0.5);
        frm.setPower(-0.5);
        blm.setPower(-0.5);
        brm.setPower(-0.5);
    }
    public void Forward(double power){
        flm.setPower(power);
        frm.setPower(power);
        blm.setPower(power);
        brm.setPower(power);
    }
    public void ArmOut(){
        arm.setPosition(0.7);
    }
    public void ClawHold(){
        claw.setPosition(1);
    }
    public void ClawRelease(){
        claw.setPosition(0);
    }
    public void ArmBack(){
        arm.setPosition(0);
    }

    public void strafe() {
        flm.setPower(-0.5);
        blm.setPower(0.5);
        frm.setPower(0.5);
        brm.setPower(-0.5);
    }
    public int getAmountRed(){
        return colorSensor.red();
    }

    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }

    public double getlength(DistanceUnit du) {
        return du.fromCm(10);
    }

    public void DriveForwardEncoder(double power, int distance){
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setTargetPosition(distance);
        flm.setTargetPosition(distance);
        brm.setTargetPosition(distance);
        blm.setTargetPosition(distance);
        flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Forward(power);

        while (flm.isBusy() && frm.isBusy()){
        }
        Stopper();
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void StrafeLeftEncoder(double power,int distance){
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setTargetPosition(-distance);
        flm.setTargetPosition(distance);
        brm.setTargetPosition(distance);
        blm.setTargetPosition(-distance);
        flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Forward(power);

        while (flm.isBusy() && frm.isBusy()){
        }
        Stopper();
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void setPowers(double flp, double frp, double blp, double brp, double sensitivity) {
        double maxSpeed = 1.0;
        //double sensitivity = 1.0; //must be grater than 1
        maxSpeed = Math.max(maxSpeed, Math.abs(flp));
        maxSpeed = Math.max(maxSpeed, Math.abs(frp));
        maxSpeed = Math.max(maxSpeed, Math.abs(blp));
        maxSpeed = Math.max(maxSpeed, Math.abs(brp));

        flp /= maxSpeed;
        frp /= maxSpeed;
        blp /= maxSpeed;
        brp /= maxSpeed;

        if (flp < 0) {
            flp = -(Math.pow(sensitivity, Math.abs(flp)) - 1) / (sensitivity - 1);
        } else {
            flp = (Math.pow(sensitivity, flp) - 1) / (sensitivity - 1);
        }

        if (frp < 0) {
            frp = -(Math.pow(sensitivity, Math.abs(frp)) - 1) / (sensitivity - 1);
        } else {
            frp = (Math.pow(sensitivity, frp) - 1) / (sensitivity - 1);
        }

        if (blp < 0) {
            blp = -(Math.pow(sensitivity, Math.abs(blp)) - 1) / (sensitivity - 1);
        } else {
            blp = (Math.pow(sensitivity, blp) - 1) / (sensitivity - 1);
        }

        if (brp < 0) {
            brp = -(Math.pow(sensitivity, Math.abs(brp)) - 1) / (sensitivity - 1);
        } else {
            brp = (Math.pow(sensitivity, brp) - 1) / (sensitivity - 1);
        }

        flm.setPower(flp);
        frm.setPower(frp);
        blm.setPower(blp);
        brm.setPower(brp);

    }

    public void drive(double forward, double right, double rotate, double sensitivity) {
        double flp = forward + right + rotate;
        double frp = forward - right - rotate;
        double blp = forward - right + rotate;
        double brp = forward + right - rotate;
        setPowers(flp*deflator, frp*deflator, blp*deflator, brp*deflator, sensitivity);
    }
}
