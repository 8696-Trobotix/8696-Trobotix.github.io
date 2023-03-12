[< Back](../index.md)
# Rover Ruckus

| <image src="../images/RoverRuckus/RoverRuckusImg.png" style="max-width: 100%; border: none; box-shadow: none;" /> |
| :---: |
| Too many CAD drawings, not enough photos of the actual robot. |

The following source code was retrieved from an archived document.  
This [former Trobotix member](https://github.com/Schierkes?tab=repositories) may have potentially more up-to-date versions as well as hardware classes. Any communications or questions should be directed at the team organization linked on the [main page](../index.md).  
League Tournament: Think Award  
Super Qualifiers: Design Award  
We did not advance past State.

___

## Non-OpMode Classes
Please see description above.
<details><summary>OpMode8696</summary>

```java
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//DogeCV Imports (Important) ((Haha, get it?))

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;

//Gyro Imports
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
* Superclass used by all of team 8696's opModes.
* Contains all the methods and functionality that
* any generic robot might have.
*/
public abstract class OpMode8696 extends LinearOpMode8696 {

   //Robot motors/servos
   protected DcMotor lift;
   protected DcMotor leftFront;
   protected DcMotor leftBack;
   protected DcMotor rightFront;
   protected DcMotor rightBack;
   protected DcMotor rightPivot;
   protected DcMotor leftPivot;
   protected DcMotor extender;
   protected CRServo dumper;

   //Sensors
   protected SamplingOrderDetector detector;
   protected BNO055IMU imu;
   BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

   protected DcMotor[] motors = new DcMotor[4];

   protected void initGyro(){

       parameters.mode                = BNO055IMU.SensorMode.IMU;
       parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
       parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       parameters.loggingEnabled      = false;

       // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
       // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
       // and named "imu".
       imu = hardwareMap.get(BNO055IMU.class, "imu");

       imu.initialize(parameters);

       telemetry.addData("Mode", "calibrating...");
       telemetry.update();

       // make sure the imu gyro is calibrated before continuing.
       while (!isStopRequested() && !imu.isGyroCalibrated())
       {
           sleep(50);
           idle();
       }
   }

   protected void initDogeCV(){

       telemetry.addData("DogeBoi TM", "Sampling dectector is good to go!");

       // Setup detector
       detector = new SamplingOrderDetector(); // Create the detector
       detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
       detector.useDefaults(); // Set detector to use default settings

       detector.downscale = 0.4; // How much to downscale the input frames

       // Optional tuning
       detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
       //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
       detector.maxAreaScorer.weight = 0.001;

       detector.ratioScorer.weight = 15;
       detector.ratioScorer.perfectRatio = 1.0;

       detector.enable(); // Start detector
   }

   protected void initRobot() {
       initDriveTrain();
       initOtherStuff();
   }

   protected void initDriveTrain() {

       rightBack = hardwareMap.dcMotor.get("right_back");
       leftBack = hardwareMap.dcMotor.get("left_back");
       rightFront = hardwareMap.dcMotor.get("right_front");
       leftFront = hardwareMap.dcMotor.get("left_front");

       motors[0] = leftBack;
       motors[1] = rightBack;
       motors[2] = leftFront;
       motors[3] = rightFront;
   }

   protected void initOtherStuff() {
       lift = hardwareMap.get(DcMotor.class, "lift");
       rightPivot = hardwareMap.get(DcMotor.class, "rightPivot");
       leftPivot = hardwareMap.get(DcMotor.class, "leftPivot");
       extender = hardwareMap.get(DcMotor.class, "extender");
       dumper = hardwareMap.get(CRServo.class,  "scoop");
       imu = hardwareMap.get(BNO055IMU.class, "imu");


       leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       leftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       imu.initialize(parameters);
       lift.setDirection(DcMotor.Direction.REVERSE);
   }

}
```
</details>

___

## Autonomous
<details><summary>FullDepo</summary>

```java
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
* Created by USER on 2/24/2018.
*/

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "FullDepo", group = "test")
public class FullDepo extends OpMode8696 {

   @Override
   public void runOpMode(){

       initRobot();
       initDogeCV();

       waitForStart();

       /*lift.setPower(1);
       sleep(1250);

       lift.setPower(0);
       sleep(1000); */

       leftFront.setPower(0.2);
       leftBack.setPower(0.2);
       rightFront.setPower(-0.2);
       rightBack.setPower(-0.2);

       sleep(600);

       leftFront.setPower(0);
       leftBack.setPower(0);
       rightFront.setPower(0);
       rightBack.setPower(0);

       sleep(1000);

       rightFront.setPower(-0.5);
       rightBack.setPower(0.5);
       leftFront.setPower(-0.5);
       leftBack.setPower(0.5);

       sleep(1200);

       leftFront.setPower(0);
       leftBack.setPower(0);
       rightFront.setPower(0);
       rightBack.setPower(0);

       sleep(500);
      
       detector.disable();

   }

}
```
</details>
<details><summary>Crater</summary>

```java
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
* Created by USER on 2/24/2018.
*/

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "Crater", group = "test")
public class Crater extends OpMode8696 {

   @Override
   public void runOpMode(){

       initRobot();
       initDogeCV();

       waitForStart();

       lift.setPower(1);
       sleep(1250);

       lift.setPower(0);
       sleep(1000);

       leftFront.setPower(0.2);
       leftBack.setPower(0.2);
       rightFront.setPower(-0.2);
       rightBack.setPower(-0.2);

       sleep(600);

       leftFront.setPower(0);
       leftBack.setPower(0);
       rightFront.setPower(0);
       rightBack.setPower(0);

       sleep(1000);

   }

}
```
</details>
<details><summary>Depo</summary>

```java
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
* Created by USER on 2/24/2018.
*/

/* From lander to drop-zone 4.5
From drop zone to crater 8.5*/
@Autonomous(name = "Depo", group = "test")
public class Depo extends OpMode8696 {

   @Override
   public void runOpMode(){

       initRobot();
       initDogeCV();

       waitForStart();

       lift.setPower(1);
       sleep(1250);

       lift.setPower(0);
       sleep(1000);

       leftFront.setPower(0.2);
       leftBack.setPower(0.2);
       rightFront.setPower(-0.2);
       rightBack.setPower(-0.2);

       sleep(600);

       leftFront.setPower(0);
       leftBack.setPower(0);
       rightFront.setPower(0);
       rightBack.setPower(0);

       sleep(1000);

       rightFront.setPower(-0.5);
       rightBack.setPower(0.5);
       leftFront.setPower(-0.5);
       leftBack.setPower(0.5);

       sleep(2200);

       leftFront.setPower(0);
       leftBack.setPower(0);
       rightFront.setPower(0);
       rightBack.setPower(0);

       sleep(500);

       dumper.setPower(1);
       sleep(2000);

       dumper.setPower(0);
       sleep(1000);

   }

}
```
</details>

___

## TeleOp
<details><summary>TroTeleOp</summary>

```java
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TroTeleOP", group="Linear Opmode")
public class TroTeleOp extends OpMode8696 {

   private int encoderval = 0;

   @Override
   public void runOpMode() {
       telemetry.addData("Status of Mr. Roboto", "Good to go! ...I think. Yeah. We're good.");
       telemetry.update();

       //Initializes [the robots parts, including wheels and lifting mechanism.
       initRobot();

       waitForStart();
       //Run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {

           //Sets power for the tank drive mecanum wheels. We use the left and right gamepad 1 sticks in order to do this.
           leftBack.setPower(gamepad1.left_stick_y);
           leftFront.setPower(gamepad1.left_stick_y);
           rightBack.setPower(-gamepad1.right_stick_y);
           rightFront.setPower(-gamepad1.right_stick_y);

           //Strafing right for the right bumper and left for the left bumper.
           //Helps with aligning our robot to the lander correctly.
           if (gamepad1.right_bumper){
               leftBack.setPower(1);
               leftFront.setPower(-1);
               rightBack.setPower(1);
               rightFront.setPower(-1);
           }
           if (gamepad1.left_bumper){
               leftBack.setPower(-1);
               leftFront.setPower(1);
               rightBack.setPower(-1);
               rightFront.setPower(1);
           }

           //All other controls besides movement reside on the second controller. It makes the driver's life easier.

           //Lift controls are set on the Dpad. Pivoting mechanisms also rely somewhat on the Dpad.
           if (gamepad2.dpad_down) lift.setPower(-1);
           if (gamepad2.dpad_up) lift.setPower(1);
           if (gamepad2.dpad_left){
               encoderval += 1;
               leftPivot.setTargetPosition(encoderval);
               rightPivot.setTargetPosition(encoderval);
               leftPivot.setPower(1);
               rightPivot.setPower(1);
               extender.setPower(-0.3);
           }
           if (gamepad2.dpad_right){
               encoderval -= 1;
               leftPivot.setTargetPosition(encoderval);
               rightPivot.setTargetPosition(encoderval);
               rightPivot.setPower(1);
               leftPivot.setPower(1);
           }
           if (!gamepad2.dpad_down && !gamepad2.dpad_up) lift.setPower(0);


           //Extending our robot arm
           if (-gamepad2.right_stick_y > 0){
               extender.setPower(-1);
           }
           if (-gamepad2.right_stick_y < 0){
               extender.setPower(1);
           }
           if (-gamepad2.right_stick_y == 0 && !gamepad2.y && !gamepad2.a && !gamepad2.dpad_right && !gamepad2.dpad_left) {
               extender.setPower(0);
           }

           //Pivots to a specific point for the robot.
           if (gamepad2.a){
               leftPivot.setTargetPosition(-75);
               rightPivot.setTargetPosition(-75);
               leftPivot.setPower(1);
               rightPivot.setPower(1);
               leftPivot.setTargetPosition(-120);
               rightPivot.setTargetPosition(-120);
               leftPivot.setPower(0.30);
               rightPivot.setPower(0.30);
               extender.setPower(-0.3);
           }
           if (gamepad2.y){
               leftPivot.setTargetPosition(-50);
               rightPivot.setTargetPosition(-50);
               leftPivot.setPower(0.1);
               rightPivot.setPower(0.1);
               leftPivot.setTargetPosition(0);
               rightPivot.setTargetPosition(0);
               rightPivot.setPower(0.25);
               leftPivot.setPower(0.25);
           }

	   //Controls our servo that pushes out minerals.
           if (gamepad2.x){
               dumper.setPower(1);
           }
           if (gamepad2.b){
               dumper.setPower(-1);
           }
           if (!gamepad2.x && !gamepad2.b){
               dumper.setPower(0);
           }
       }
   }
}
```
</details>
