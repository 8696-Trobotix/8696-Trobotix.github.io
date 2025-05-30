---
title: "Skystone 2019 - 2020"
permalink: /Skystone
---

[< Back](../index.md) | [< General Info](./index.md)

# Skystone

| <image src="../images/Skystone/SkystoneImg.png" style="max-width: 100%; border: none; box-shadow: none;" /> |
| :---: |
| This season looked fun. |

The following source code was retrieved from a branch other than the main branch because the main branch was effectively empty.  
League Tournament: Think Award  
Super Qualifiers: Think Award  
Iowa Championship: Promote Award

___

## Non-OpMode Classes
This season used a in-development version of the Trobot package highlighted in [Ultimate Goal](./UltimateGoal.md) along with a confusing bundle of other items.

___

## Autonomous
<details><summary>FoundationBlue</summary>

```java
package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Foundation Blue", group="Autonomous")
public class FoundationBlue extends LinearOpMode {

    Robot_OmniDrive robot    = new Robot_OmniDrive();
    private ElapsedTime runtime = new ElapsedTime();

    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override
    public void runOpMode() {

        final long START = 1000;
        final long END = 10000;
        robot.initDrive(this);
        robot.setServo(0.5);
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        waitForStart();
        sleep(START);
        robot.encoderDrive(.06, 10, 10, -10, -10, 1);           //Strafe Right
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(0.06, -10, 10, -10, 10, 1.3);       //Drive Forward
        robot.stopMotor();
        sleep(1000);

        rightServo.setPosition(1);                                                                              //Lower Servos
        leftServo.setPosition(1);
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(.06, 10, -10, 10, -10, 1.9);           //Drive Backwards
        robot.stopMotor();
        sleep(1000);

//        robot.encoderDrive(0.06, -10, -10, -10, -10, 1);        //Turn Left
//        robot.stopMotor();
//        sleep(1000);

        rightServo.setPosition(0);                                                                              //Lower Servos
        leftServo.setPosition(0);
        robot.stopMotor();
        sleep(1000);
        sleep(END);
        robot.encoderDrive(.06, -10, -10, 10, 10, 2.2);           //Strafe Left
//        robot.stopMotor();
//        sleep(1000);
//        robot.encoderDrive(.06, 10, -10, 10, -10, 2);           //Drive Backwards
//        robot.stopMotor();
        sleep(1500);
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(120);  //Turn left
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(110);  //Turn left
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(90);  //Turn left
        robot.encoderDrive(0.06, 10, -10, 10, -10, .5);
        sleep(1000);
    }


}
```
</details>
<details><summary>FoundationRed</summary>

```java
package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Foundation Red", group="Autonomous")
public class FoundationRed extends LinearOpMode {

    Robot_OmniDrive robot    = new Robot_OmniDrive();
    private ElapsedTime runtime = new ElapsedTime();

    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override
    public void runOpMode() {
        final long START = 1000;
        final long END = 10000;
        robot.initDrive(this);
        robot.setServo(0.5);
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        waitForStart();
        sleep(START);
        robot.encoderDrive(.06, -10, -10, 10, 10, 1);           //Strafe Right
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(0.06, -10, 10, -10, 10, 1.3);       //Drive Forward
        robot.stopMotor();
        sleep(1000);

        rightServo.setPosition(1);                                                                              //Lower Servos
        leftServo.setPosition(1);
        robot.stopMotor();
        sleep(1000);

        robot.encoderDrive(.06, 10, -10, 10, -10, 1.9);           //Drive Backwards
        robot.stopMotor();
        sleep(1000);

//        robot.encoderDrive(0.06, -10, -10, -10, -10, 1);        //Turn Left
//        robot.stopMotor();
//        sleep(1000);

        rightServo.setPosition(0);                                                                              //Lower Servos
        leftServo.setPosition(0);
        robot.stopMotor();
        sleep(1000);
        sleep(END);
        robot.encoderDrive(.06, 10, 10, -10, -10, 2.2);           //Strafe Left
//        robot.stopMotor();
//        sleep(1000);
//        robot.encoderDrive(.06, 10, -10, 10, -10, 2);           //Drive Backwards
//        robot.stopMotor();
        sleep(1500);
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(120);  //Turn left
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(110);  //Turn left
//        telemetry.addData("Add", robot.getAngle());
//        telemetry.addData("Last", robot.lastAngles.firstAngle);
//        telemetry.update();
//        robot.turnLeft(90);  //Turn left
        robot.encoderDrive(0.06, 10, -10, 10, -10, .5);
        sleep(1000);
    }


}
```
</details>

___

## TeleOp
<details><summary>TrobotixTeleOp</summary>

```java
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear Opmode")

public class TrobotixTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;

    private DcMotor beltMotor = null;
//    private DcMotor topIntake = null;
//    private DcMotor botIntake = null;

    private DcMotor shooterMotor = null;
//    private DcMotor leftShooter = null;
//    private DcMotor rightShooter = null;


    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear right");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear left");

        beltMotor = hardwareMap.get(DcMotor.class, "belt");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {





            //To shoot press X
            //To move belt forward press A
            //To reverse belt press B





            double leftStickY = -gamepad1.left_stick_y;
            double rightStickY = gamepad1.right_stick_y;
            double leftStickX = -gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;

            boolean beltInA = gamepad1.a;
            boolean beltOutB = gamepad1.b;

            boolean shooterX = gamepad1.x;


            //double stick drive
            //this drive will be like the old one
            if(Math.abs(leftStickY) >= 0.1 || Math.abs(rightStickY) >= 0.1){
                frontLeftDrive.setPower(leftStickY);
                frontRightDrive.setPower(rightStickY);
                rearLeftDrive.setPower(leftStickY);
                rearRightDrive.setPower(rightStickY);
            }
            else{
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                rearLeftDrive.setPower(0);
                rearRightDrive.setPower(0);
            }



            //INCOMPLETE CODE
            //CANNOT BOTH TURN AND MOVE
            //WILL NEED MATH

            //new drive mode
            //left stick will be to move the robot
            //right stick will be to turn the robot
            //orientation of the robot will have the front right be the "front" of the robot
//            if(Math.abs(leftStickY) >= 0.1 || Math.abs(leftStickX) >= 0.1){
//                frontLeftDrive.setPower(leftStickY);
//                frontRightDrive.setPower(leftStickX);
//                rearLeftDrive.setPower(leftStickX);
//                rearRightDrive.setPower(leftStickY);
//            }
//            else if(Math.abs(rightStickX) >= 0.1){        //this is how the robot will turn
//                frontLeftDrive.setPower(-rightStickX);
//                frontRightDrive.setPower(rightStickX);
//                rearLeftDrive.setPower(-rightStickX);
//                rearRightDrive.setPower(rightStickY);
//            }
//            else{
//                frontLeftDrive.setPower(0);
//                frontRightDrive.setPower(0);
//                rearLeftDrive.setPower(0);
//                rearRightDrive.setPower(0);
//            }

            if(beltInA == true){
                beltMotor.setPower(1);
            }
            else if(beltOutB == true){
                beltMotor.setPower(-1);
            }
            else{
                beltMotor.setPower(0);
            }

            if(shooterX == true){
                shooterMotor.setPower(1);
            }
            else{
                shooterMotor.setPower(0);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f), rear right (%.2f)", leftStickY, rightStickY, leftStickY, rightStickY);
            telemetry.update();

        }
    }
}
```
</details>
