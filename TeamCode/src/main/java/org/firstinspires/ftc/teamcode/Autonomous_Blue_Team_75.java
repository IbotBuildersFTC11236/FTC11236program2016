package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by zhitao on 11/5/2016.
 */
@Autonomous(name = "AutoB", group = "Teamcode")
public class Autonomous_Blue_Team_75 extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    static final char     RIGHT                   = 'R';
    static final char     LEFT                    = 'L';
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DIST_WHEELS             = 14.0;
    static final double     PI                      = 3.1415926535897932;
    static final double     INCH_PER_DEG             = ((PI * DIST_WHEELS) / 360); //Two-wheel turn
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    DcMotor drivemotorL = null;
    DcMotor drivemotorR = null;
    DcMotor flywheelL = null;
    DcMotor flywheelR = null;
    DcMotor intake = null;
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
        shootIt(power);
        Thread.sleep(500);
        intake.setPower(1);
        Thread.sleep(time);
        intake.setPower(0);
        shootIt(0);
    }
    public void DriveForward(double power)
    {
        drivemotorL.setPower(power);
        drivemotorR.setPower(power);
    }
    public void encoderDrivebyDistance2(double speed,double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        double leftCount = leftInches * 72.0;
        double rightCount = rightInches * 72.0;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                if (speed < 0) {
                    newLeftTarget = drivemotorL.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                    newRightTarget = drivemotorR.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
                } else {
                    newLeftTarget = drivemotorL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                    newRightTarget = drivemotorR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                }
                drivemotorL.setTargetPosition(newLeftTarget);
                drivemotorR.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                drivemotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drivemotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                drivemotorL.setPower(speed);
                drivemotorR.setPower(speed);
                if (speed < 0) {
                    while (opModeIsActive() &&
                            (drivemotorL.getCurrentPosition() > newLeftTarget) &&
                            (drivemotorL.isBusy() && drivemotorR.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                drivemotorL.getCurrentPosition(),
                                drivemotorR.getCurrentPosition());
                        telemetry.update();
                    }
                }

                // keep looping while we are still active, and there is time left, and both motors are running.
                else if (leftInches >= rightInches) {
                    while (opModeIsActive() &&
                            (drivemotorL.getCurrentPosition() < newLeftTarget) &&
                            (drivemotorL.isBusy() && drivemotorR.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                drivemotorL.getCurrentPosition(),
                                drivemotorR.getCurrentPosition());
                        telemetry.update();
                    }
                } else if (rightInches > leftInches) {
                    while (opModeIsActive() &&
                            (drivemotorL.getCurrentPosition() < newRightTarget) &&
                            (drivemotorL.isBusy() && drivemotorR.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                drivemotorL.getCurrentPosition(),
                                drivemotorR.getCurrentPosition());
                        telemetry.update();
                    }
                }

                // Stop all motion;
                drivemotorL.setPower(0);
                drivemotorR.setPower(0);

                // Turn off RUN_TO_POSITION
                drivemotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drivemotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
    }

    public void encoderDrivebyDistance(double speed,
                                       double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        int mvLeftDir, mvRightDir;

        if( speed * leftInches > 0){
            mvLeftDir = 1;
        }
        else{
            mvLeftDir = 0;
        }

        if( speed * rightInches > 0){
            mvRightDir = 1;
        }
        else{
            mvRightDir = 0;
        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (speed < 0) {
                newLeftTarget = drivemotorL.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = drivemotorR.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            } else {
                newLeftTarget = drivemotorL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = drivemotorR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            }
            drivemotorL.setTargetPosition(newLeftTarget);
            drivemotorR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            drivemotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drivemotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            drivemotorL.setPower(speed);
            drivemotorR.setPower(speed);


            while (opModeIsActive() &&
            //        (drivemotorL.getCurrentPosition() > newLeftTarget) &&
                    (drivemotorL.isBusy() && drivemotorR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        drivemotorL.getCurrentPosition(),
                        drivemotorR.getCurrentPosition());
                telemetry.update();
                //left to the position
                if( mvLeftDir == 1 ){
                    if( drivemotorL.getCurrentPosition() >= newLeftTarget ){
                        drivemotorL.setPower(0);
                    }
                }
                else{
                    if( drivemotorL.getCurrentPosition() <= newLeftTarget ) {
                        drivemotorL.setPower(0);
                    }
                }
                //right to the position
                if( mvRightDir == 1 ){
                    if( drivemotorR.getCurrentPosition() >= newRightTarget ){
                        drivemotorR.setPower(0);
                    }
                }
                else {
                    if (drivemotorR.getCurrentPosition() <= newRightTarget) {
                        drivemotorR.setPower(0);
                    }
                }
            }

            // Stop all motion;
            drivemotorL.setPower(0);
            drivemotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            drivemotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drivemotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderTurnByDistance(double speed, double degree, char direction)
    {
        double numOfInches;
        numOfInches = (degree * INCH_PER_DEG);
        if(direction == LEFT)
        {
            drivemotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drivemotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderDrivebyDistance(speed, numOfInches, -numOfInches);
        }
        else if(direction == RIGHT)
        {
            drivemotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drivemotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderDrivebyDistance(speed, -numOfInches, numOfInches);
        }
        else
        {

        }

    }
    public void StopDriving()
    {
        DriveForward(0);
    }
    /*public void DriveForwardDistance(double power, int distance)
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
    }*/

    public void runOpMode() throws InterruptedException {
        // hsvValues is an array that will hold the hue, saturation, and value information.


        // bPrevState and bCurrState represent the previous and current state of the button.


        // bLedOn represents the state of the LED.
        float hsvValues[] = {0F, 0F, 0F};
        boolean bLedOn = true;
        drivemotorL = hardwareMap.dcMotor.get("left_drive");
        drivemotorR = hardwareMap.dcMotor.get("right_drive");
        flywheelL = hardwareMap.dcMotor.get("left_fly");
        flywheelR = hardwareMap.dcMotor.get("right_fly");
        intake = hardwareMap.dcMotor.get("intake");
        color = hardwareMap.colorSensor.get("color");
        drivemotorL.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        drivemotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivemotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        color.enableLed(bLedOn);
        //Start dat robot

        // Set the LED in the beginning

        waitForStart();


        encoderDrivebyDistance(.5, 48, 48);

        encoderTurnByDistance(.1, 90, LEFT);

        encoderDrivebyDistance(-.4, 20, 20);
        Thread.sleep(500);
        liftAndLaunch(.8, 3000);

        encoderDrivebyDistance(-.2, 8, 8);


        encoderDrivebyDistance(.3, 3, 3);


        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);


        if(color.red() >= 1)
        {
            Thread.sleep(4000);
            encoderDrivebyDistance(-.3, 5, 5);
            encoderDrivebyDistance(.2, 12,12);

        }
        else if(color.blue() >= 1)
        {
            Thread.sleep(100);
            encoderDrivebyDistance(.2, 12,12);

        }
        encoderTurnByDistance(.2, 90, LEFT);

        encoderDrivebyDistance(-1, 40, 40);

        encoderTurnByDistance(-.3, 90, LEFT);
        Thread.sleep(500);
        encoderDrivebyDistance(-.3,11,11);
        encoderDrivebyDistance(.6,7,7);
        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.

        if(color.red() >= 1)
        {
            Thread.sleep(4000);
            encoderDrivebyDistance(-.3, 7, 7);
            encoderDrivebyDistance(.2, 2,2);

        }
        encoderTurnByDistance(.8, 50, LEFT);
        encoderDrivebyDistance(1, 80, 80);



    }
}