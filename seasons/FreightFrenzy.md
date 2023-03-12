---
title: "Freight Frenzy 2021 - 2022"
---

[< Back](../index.md) | [< General Info](./index.md)

# Freight Frenzy

| <image src="../images/FreightFrenzy/FreightFrenzyImg.png" style="max-width: 100%; border: none; box-shadow: none;" /> |
| :---: |
| Robot picks up an orange. This image was originally cropped at the corners because a shoe was in view. |

Nearly all the members were new this year, with former members being occupied.  
Regardless, it was a great learning experience!  
We did not advance past the League Tournament.

___

## Autonomous
<details><summary>backupAutonomous</summary>

```java
/*
This is the backup autonomous.
Underneath the line of equal signs is where you put functions calls.
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="backupAutonomous 3.12", group="Linear Opmode")

public class backupAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearLeft, rearRight, carousel;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        runtime.reset();
        //=============================================================
        move(-0.5, 1.7);
    }
    public void move(double speed, double duration) {
        rearLeft.setPower(speed);
        rearRight.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < duration);
    }
    public void turn(double quantity, double duration) {
        rearLeft.setPower(quantity);
        rearRight.setPower(-quantity);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < duration);
    }
    public void carouselTurn(double speed, double duration) {
        carousel.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < duration);
    }
}
```
</details>

___

## TeleOp
<details><summary>TeleOpII</summary>

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpII 3.12", group="Linear Opmode")
public class TeleOpII extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearLeft, rearRight, carousel, clawMotor;
    private Servo clawServo;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        clawMotor = hardwareMap.get(DcMotor.class, "clawMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        clawMotor.setDirection(DcMotor.Direction.FORWARD);

        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        // cp = carousel power, cm = claw motor, cs = claw servo
        double drive, turn, sprint, cp, cm, cs = 0, l, r, over;
        final double clawMotorSpeed = 0.3, reduction = 0.5, open = 1, close = 0, maxTurn = 0.75;

        while (opModeIsActive()) {
            // DRIVING
            turn = maxTurn * -gamepad1.right_stick_x;
            sprint = gamepad1.left_trigger * reduction;
            drive = gamepad1.left_stick_y / (1 + reduction - sprint);
            l = drive + turn;
            r = drive - turn;
            over = Math.max(Math.abs(l), Math.abs(r));
            if (over > 1.0) {
                l /= over;
                r /= over;
            }
            // CAROUSEL
            if (gamepad1.x) carousel.setDirection(DcMotor.Direction.FORWARD);
            else if (gamepad1.y) carousel.setDirection(DcMotor.Direction.REVERSE);
            cp = gamepad1.right_trigger;
            // CLAW MOTOR
            if (gamepad1.left_bumper) cm = -clawMotorSpeed;
            else if (gamepad1.right_bumper) cm = clawMotorSpeed;
            else cm = 0;
            // CLAW SERVO
            if (gamepad1.a) cs = close;
            else if (gamepad1.b) cs = open;
            // SET ATTRIBUTES
            rearLeft.setPower(l);
            rearRight.setPower(r);
            carousel.setPower(cp);
            clawMotor.setPower(cm);
            clawServo.setPosition(cs);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
```
</details>
