package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by zhitao on 11/5/2016.
 */
@Autonomous(name = "Auto", group = "Teamcode")
public class Autonomous_Beacons extends LinearOpMode{
    DcMotor drivemotorL = null;
    DcMotor drivemotorR = null;
    DcMotor flywheelL;
    DcMotor flywheelR;
    DcMotor intake;
    DcMotor lift;
    ColorSensor color;



    public void driveForwardTime(double power, long time) throws InterruptedException{
        drivemotorL.setPower(power);
        drivemotorR.setPower(power);
        Thread.sleep(time);
        drivemotorL.setPower(0);
        drivemotorR.setPower(0);
    }
    public void turnRightTime(double power, long time) throws InterruptedException{
        drivemotorL.setPower(-power);
        drivemotorR.setPower(power);
        Thread.sleep(time);
        drivemotorL.setPower(0);
        drivemotorR.setPower(0);
    }
    public void turnLeftTime(double power, long time) throws InterruptedException{
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
        intake.setPower(-1);
        lift.setPower(.5);
        shootIt(power);
        Thread.sleep(time);
        intake.setPower(0);
        lift.setPower(0);
        shootIt(0);
    }
    public void DriveForward(double power)
    {
        drivemotorL.setPower(power);
        drivemotorR.setPower(power);
    }
    public void StopDriving()
    {
        DriveForward(0);
    }
    public void DriveForwardDistance(double power, int distance)
    {
        // Reset encoders
        drivemotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivemotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target positions
        drivemotorL.setTargetPosition(distance);
        drivemotorR.setTargetPosition(distance);

        //Set to RUN_TO_POSITION mode
        drivemotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivemotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Drive forward
        DriveForward(power);

        while(drivemotorL.isBusy() && drivemotorR.isBusy())
        {

        }

        //Stop and change modes back to normal
        StopDriving();
        drivemotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivemotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() throws InterruptedException{
        drivemotorL = hardwareMap.dcMotor.get("left_drive");
        drivemotorR = hardwareMap.dcMotor.get("right_drive");
        flywheelL = hardwareMap.dcMotor.get("left_fly");
        flywheelR = hardwareMap.dcMotor.get("right_fly");
        lift = hardwareMap.dcMotor.get("pulley");
        intake = hardwareMap.dcMotor.get("intake");
        color = hardwareMap.colorSensor.get("color");
        drivemotorL.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        drivemotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivemotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Start dat robot
        waitForStart();
        //int ANDYMARK_TYICKS_PER_REV = 1120;
        liftAndLaunch(1, 4000);
        Thread.sleep(2000);
        driveForwardTime(.5, 750);
        Thread.sleep(2000);
        turnRightTime(.5, 600);
        Thread.sleep(2000);
        driveForwardTime(.5, 2000);
        Thread.sleep(2000);
        turnLeftTime(.5, 1150);
        Thread.sleep(2000);
        driveForwardTime(-.5, 900);





    }
}