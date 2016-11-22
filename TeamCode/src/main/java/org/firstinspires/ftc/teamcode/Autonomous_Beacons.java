package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.drm.DrmStore;
import android.graphics.Color;
import android.view.View;

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
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by zhitao on 11/5/2016.
 */
@Autonomous(name = "Auto", group = "Teamcode")
public class Autonomous_Beacons extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    static final char     RIGHT                   = 'R';
    static final char     LEFT                    = 'L';
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DIST_WHEELS             = 15.0;
    static final double     PI                      = 3.1415926535897932;
    static final double     INCH_PER_DEG             = (PI * 2 * (DIST_WHEELS/2) / 360); //Two-wheel turn
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
    DcMotor lift = null;
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
    public void encoderDrivebyDistance(double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        double leftCount = leftInches * 72.0;
        double rightCount = rightInches * 72.0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if(speed < 0)
            {
                newLeftTarget = drivemotorL.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = drivemotorR.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            }
            else
            {
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
            if(speed < 0)
            {
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
             else if(leftInches >= rightInches) {
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
            }
            else if(rightInches > leftInches)
            {
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
    public void encoderTurnByDistance(double speed, double degree, char direction)
    {
        double numOfInches;
        numOfInches = (degree * INCH_PER_DEG);
        if(direction == LEFT)
        {
            encoderDrivebyDistance(speed, numOfInches, -numOfInches);
        }
        else if(direction == RIGHT)
        {
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
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;
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
        color.enableLed(bLedOn);
        //Start dat robot

        // Set the LED in the beginning

        waitForStart();

        //int ANDYMARK_TICKS_PER_REV = 1120;
        liftAndLaunch(1, 4000);
        Thread.sleep(1000);
        // Go forward
        //driveForwardTime(.5, 850);
        encoderDrivebyDistance(.2, 15, 15);
        Thread.sleep(1000);
        //Make turn to beacon
        //turnRightTime(.5, 600);
        encoderTurnByDistance(.2, 60, RIGHT);
        Thread.sleep(1000);
        //Go forward on slant
        //driveForwardTime(.5, 2075);
        encoderDrivebyDistance(.4, 45, 45);
        Thread.sleep(1000);
        //Turn to face beacon
        //turnLeftTime(.5, 1300);
        encoderTurnByDistance(.2, 150, LEFT);
        Thread.sleep(1000);
        // Go to beacon
        driveForwardTime(-.5, 670);

        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue ", color.blue());
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        telemetry.update();
        Thread.sleep(1000);

        if(color.red() >= 1)
        {
            telemetry.addData("REEED", color.red());
            telemetry.update();
            Thread.sleep(1000);
            encoderDrivebyDistance(.5, 8, 8);
        }
        else if(color.blue() >= 1)
        {
            telemetry.addData("BLUEEEE", color.blue());
            telemetry.update();
            Thread.sleep(1000);
            encoderDrivebyDistance(.5, 24, 24);
        }




    }
}