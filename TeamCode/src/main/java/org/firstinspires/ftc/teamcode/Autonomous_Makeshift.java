package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by zhitao on 11/5/2016.
 */
@Autonomous(name = "Auto", group = "Teamcode")
public class Autonomous_Makeshift extends LinearOpMode{
    DcMotor drivemotorL;
    DcMotor drivemotorR;
    DcMotor flywheelL;
    DcMotor flywheelR;
    DcMotor intake;
    DcMotor lift;


    public void driveForwardTime(double power, long time) throws InterruptedException{
        drivemotorL.setPower(power);
        drivemotorR.setPower(power);
        Thread.sleep(time);
        drivemotorL.setPower(0);
        drivemotorR.setPower(0);
    }
    public void turnLeftTime(double power, long time) throws InterruptedException{
        drivemotorL.setPower(-power);
        drivemotorR.setPower(power);
        Thread.sleep(time);
        drivemotorL.setPower(0);
        drivemotorR.setPower(0);
    }
    public void turnRightTime(double power, long time) throws InterruptedException{
        drivemotorL.setPower(power);
        drivemotorR.setPower(-power);
        Thread.sleep(time);
        drivemotorL.setPower(0);
        drivemotorR.setPower(0);
    }
    public void shootIt(double power) {
        flywheelL.setPower(power);
        flywheelR.setPower(power);
    }
    public void liftAndLaunch(double power,long time) throws InterruptedException {
        lift.setPower(power);
        shootIt(power);
        Thread.sleep(time);
        lift.setPower(0);
        shootIt(0);
    }

    public void runOpMode() throws InterruptedException{
        drivemotorL = hardwareMap.dcMotor.get("left_drive");
        drivemotorR = hardwareMap.dcMotor.get("right_drive");
        flywheelL = hardwareMap.dcMotor.get("left_fly");
        flywheelR = hardwareMap.dcMotor.get("right_fly");
        lift = hardwareMap.dcMotor.get("pulley");
        intake = hardwareMap.dcMotor.get("intake");
        drivemotorL.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        //Start dat robot
        waitForStart();
        liftAndLaunch(.5, 4);


    }
}