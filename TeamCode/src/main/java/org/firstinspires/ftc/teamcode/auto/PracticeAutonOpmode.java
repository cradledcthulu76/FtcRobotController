package org.firstinspires.ftc.teamcode.auto;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class PracticeAutonOpmode extends OpMode {
    protected DcMotor frm;
    protected DcMotor flm;
    protected DcMotor blm;
    protected DcMotor brm;
    @Override
    public void init(){
        flm = hardwareMap.get(DcMotor.class, "front-left-motor");
        frm = hardwareMap.get(DcMotor.class, "front-right-motor");
        brm = hardwareMap.get(DcMotor.class, "back-right-motor");
        blm = hardwareMap.get(DcMotor.class, "back-left-motor");
        flm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        while (getRuntime() <=1){
            flm.setPower(1);
            frm.setPower(1);
            blm.setPower(1);
            brm.setPower(1);
        }
        while(getRuntime()<=2){
            flm.setPower(0);
            frm.setPower(0);
            blm.setPower(0);
            brm.setPower(0);
        }
        while (getRuntime()<=3){
            flm.setPower(1);
            frm.setPower(1);
        }
        flm.setPower(0);
        frm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);

    }
}
