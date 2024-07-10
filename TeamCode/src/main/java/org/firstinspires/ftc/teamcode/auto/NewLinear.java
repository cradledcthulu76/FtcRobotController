package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.HardwareMap;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

@Autonomous
public class NewLinear extends OpMode {
   MecanumDrive MecanumDriveObj = new MecanumDrive();
   /* public DcMotor frm;
    public DcMotor flm;
    public DcMotor blm;
    public DcMotor brm;
    */

    @Override
    public void init() {
     MecanumDriveObj.init(hardwareMap);

    }
    public void loop() {
    while (getRuntime() <=2) {
     MecanumDriveObj.FLM();
    }
    MecanumDriveObj.Stopper();
    while (getRuntime() <=4) {
     MecanumDriveObj.FRM();
    }
    MecanumDriveObj.Stopper();
    while (getRuntime() <=8) {
     MecanumDriveObj.BLM();
    }
     MecanumDriveObj.Stopper();
    while (getRuntime() <=10) {
     MecanumDriveObj.BRM();
    }
     MecanumDriveObj.Stopper();

    }
    /*public void Straight() {

        MecanumDriveObj.flm.setPower(0.5);
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

     */


        /*flm = hardwareMap.get(DcMotor.class, "front-left-motor");
        frm = hardwareMap.get(DcMotor.class, "front-right-motor");
        brm = hardwareMap.get(DcMotor.class, "back-right-motor");
        blm = hardwareMap.get(DcMotor.class, "back-left-motor");
        brm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.FORWARD);
        flm.setDirection(DcMotorSimple.Direction.FORWARD);

        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//back left
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //back right
*/
       /*
        MecanumDriveObj.initialize(hardwareMap);

        waitForStart();

        MecanumDriveObj.Straight();
        sleep(2000);
        MecanumDriveObj.TurnRight();
        sleep(2000);
        MecanumDriveObj.Straight();
        sleep(3000);
        MecanumDriveObj.StraightRight();
        sleep(2000);
        stop();
        */
    }
