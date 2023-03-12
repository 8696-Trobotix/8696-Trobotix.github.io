[< Back](../index.md)
# Ultimate Goal

| <image src="../images/UltimateGoal/UltimateGoalImg.png" style="max-width: 100%; border: none; box-shadow: none;" /> |
| :---: |
| This image of the robot may not be reflective of its final form. |

COVID year.  
We have limited information on this year and prior years.  
League Tournament: 2nd Place Innovate (did not advance)

___

## Non-OpMode Classes
<details><summary>Component</summary>

```java
/* Copyright (c) 2020, All Rights Reserved
 *
 * This file is intended for FTC Team #8696 Trobotix only. Redistribution, duplication, or use in
 * source and binary forms, with or without modification, is not permitted without explicit
 * permission from the creator or authorized moderator.
 *
 * Written by Timothy (Tikki) Cui
 */

package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 'Component' controls the robot's features, mostly mechanisms designed to score points. It
 * controls robot features such as latches, claws, elevators, intake motors, etc.
 *
 * Key features include:
 * - intake motors
 * - foundation latches
 * - elevator to raise block
 *
 * 'Component' needs a lot of rework each season. A different game theme will have a major impact on
 * a robot's components since each component is usually designed to score points
 *
 * @author Tikki
 * @version 3.6.0
 * @since release
 *
 * Version Log: (Obfuscated by site maintainer.)
 */

public class Component {
    private HardwareMap hardwareMap;

    private DcMotor wobbleArm;
    private Servo armLatch;

    public Component(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        wobbleArm = hardwareMap.get(DcMotor.class, "wobble");
        armLatch = hardwareMap.get(Servo.class, "latch");
        armLatch.setDirection(Servo.Direction.REVERSE);
    }

    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public DcMotor getWobbleArm() {return wobbleArm;}

    public Servo getArmLatch() {return armLatch;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setWobbleArm(DcMotor wobbleArm) {this.wobbleArm = wobbleArm;}


    // Utilities
    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // TODO: mesaure actual values with LatchTelem.java
    public void armLatch() {
        armLatch.setPosition(0.35);
    }

    public void armUnlatch() {
        armLatch.setPosition(0.65);
    }

    public void openWobbleArm() {
        wobbleArm.setPower(0.5);
    }

    public void openWobbleArm(int time) {
        openWobbleArm();
        sleep(time);
        stopWobbleArm();
    }

    public void closeWobbleArm() {
        wobbleArm.setPower(-0.5);
    }

    public void closeWobbleArm(int time) {
        closeWobbleArm();
        sleep(time);
        stopWobbleArm();
    }

    public void stopWobbleArm() {
        wobbleArm.setPower(0);
    }
}
```
</details>
<details><summary>Drivetrain</summary>

```java
/* Copyright (c) 2020, All Rights Reserved
 *
 * This file is intended for FTC Team #8696 Trobotix only. Redistribution, duplication, or use in
 * source and binary forms, with or without modification, is not permitted without explicit
 * permission from the creator or authorized moderator.
 *
 * Written by Timothy (Tikki) Cui
 */

package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * 'Drivetrain' controls the motion motors of 'Trobot'. It contains the drive motor variables as
 * well as containing many utility methods.
 *
 * Key features include:
 * - driving with power inputs
 * - strafing
 * - auto-drive using encoders
 * - speed reduction
 * - checking motor statuses
 *
 * 'Drivetrain' remains relatively constant through each season since a different game theme doesn't
 * affect a robot's basic movement abilities.
 *
 * @author Tikki
 * @version 4.8.3
 * @since release
 *
 * Version Log: (Obfuscated by site maintainer.)
 */

public class Drivetrain {
    private HardwareMap hardwareMap;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;

    private DistanceSensor frontSensor;
    private DistanceSensor bottomSensor;
    private DistanceSensor topSensor;

    // start with full speed
    private boolean isSpeedReduced = false;
    private String speedStatus = "Normal";

    // imu variables
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation lastAngles;
    private double globalAngle;
    private double correction;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear left");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear right");

        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Distance sensors
        frontSensor = hardwareMap.get(DistanceSensor.class, "front");
        topSensor = hardwareMap.get(DistanceSensor.class, "top");
        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottom");

        // imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit= BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        lastAngles = new Orientation();

        while (!imu.isGyroCalibrated()) {}
    }

    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public DcMotor getFrontLeftDrive() {return frontLeftDrive;}
    public DcMotor getFrontRightDrive() {return frontRightDrive;}
    public DcMotor getRearLeftDrive() {return rearLeftDrive;}
    public DcMotor getRearRightDrive() {return rearRightDrive;}
    public boolean isSpeedReduced() {return isSpeedReduced;}
    public String getSpeedStatus() {return speedStatus;}
    public DistanceSensor getFrontSensor() {return frontSensor;}
    public DistanceSensor getTopSensor() {return topSensor;}
    public DistanceSensor getBottomSensor() {return bottomSensor;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setFrontLeftDrive(DcMotor frontLeftDrive) {this.frontLeftDrive = frontLeftDrive;}
    public void setFrontRightDrive(DcMotor frontRightDrive) {this.frontRightDrive = frontRightDrive;}
    public void setRearLeftDrive(DcMotor rearLeftDrive) {this.rearLeftDrive = rearLeftDrive;}
    public void setRearRightDrive(DcMotor rearRightDrive) {this.rearRightDrive = rearRightDrive;}
    public void setSpeedReduced(boolean speedReduced) {isSpeedReduced = speedReduced;}
    public void setSpeedStatus(String speedStatus) {this.speedStatus = speedStatus;}
    public void setFrontSensor(DistanceSensor sensor) {frontSensor = sensor;}
    public void setTopSensor(DistanceSensor sensor) {topSensor = sensor;}
    public void setBottomSensor(DistanceSensor sensor) {bottomSensor = sensor;}

    // Utility
    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public int detectRings() {
        double bottomSensorDistance = bottomSensor.getDistance(DistanceUnit.INCH);
        double topSensorDistance = topSensor.getDistance(DistanceUnit.INCH);

        if (topSensorDistance <= 2.5) {
            return 4;
        } else if (bottomSensorDistance <= 2.5) {
            return 1;
        } else {
            return 0;
        }
    }
    
    public void drive(double power) {
        if (!isSpeedReduced) {
            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            rearLeftDrive.setPower(power);
            rearRightDrive.setPower(power);
        } else {
            frontLeftDrive.setPower(power * 0.65);
            frontRightDrive.setPower(power * 0.65);
            rearLeftDrive.setPower(power * 0.65);
            rearRightDrive.setPower(power * 0.65);
        }
    }

    public void drive(double leftPower, double rightPower) {
        if (!isSpeedReduced) {
            frontLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearLeftDrive.setPower(leftPower);
            rearRightDrive.setPower(rightPower);
        } else {
            frontLeftDrive.setPower(leftPower * 0.65);
            frontRightDrive.setPower(rightPower * 0.65);
            rearLeftDrive.setPower(leftPower * 0.65);
            rearRightDrive.setPower(rightPower * 0.65);
        }
    }

    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void switchSpeed() {
        isSpeedReduced = !isSpeedReduced;

        if (isSpeedReduced) {
            speedStatus = "Reduced";
        } else {
            speedStatus = "Normal";
        }
    }

    public void strafeRight(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        rearLeftDrive.setPower(-power);
        rearRightDrive.setPower(power);
    }

    public void strafeLeft(double power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(-power);
    }

    public void autoDriveTime(double power, long milliseconds) {
        drive(power);

        sleep(milliseconds);

        stop();
    }

//    public void autoDriveDistance(double power, double distance) {
//        // TODO: Confirm measurements
//        double threadsPerCentimeter = ((1120 * 2) / (10 * Math.PI));
//
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeftDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
//        frontRightDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
//        rearLeftDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
//        rearRightDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
//
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        drive(power);
//
//        while (frontLeftDrive.isBusy() || rearRightDrive.isBusy()) {}
//
//        stop();
//    }

    public void autoDriveDistance(double power, double stopDistance) {

        double distance = frontSensor.getDistance(DistanceUnit.INCH);
        while(distance >= stopDistance)
        {
            distance = frontSensor.getDistance(DistanceUnit.INCH);
            driveIMU(power);
        }
        stop();
        sleep(1000);
    }


    public double getPosition() {
        return (frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition()
                + rearLeftDrive.getCurrentPosition() + rearRightDrive.getCurrentPosition()) / 4.0;
    }

    public void resetEncoder() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy() {
        return frontLeftDrive.isBusy() || frontRightDrive.isBusy() || rearLeftDrive.isBusy() || rearRightDrive.isBusy();
    }

    // IMU
    public void autoDriveIMU(double power, long time) {
        long startTime = System.currentTimeMillis(); //fetch starting time

        while((System.currentTimeMillis() - startTime) < time) {
            driveIMU(power);
        }

        stop();
        sleep(1000);
    }

    public void driveIMU(double power) {
        // Use gyro to drive in a straight line.
        correction = checkDirection();

        if (correction > power / 2) {
            correction = power / 2;
        }

        drive(power - correction, power + correction);
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        // - quote from FTC SDK

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.10;

        angle = getAngle();

        if (angle <= 1.5 && angle >= -1.5) {
            correction = 0;             // no adjustment.
        } else {
            correction = -angle;        // reverse sign of angle for correction.
        }

        correction = correction * gain;

        return correction;
    }

    private void rotate(int degrees, double power) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else {
            return;
        }

        // set power to rotate.
        drive(leftPower, rightPower);

        // rotate until turn is completed.
        if (degrees < 0) { // On right turn we have to get off zero first.
            while (getAngle() == 0) {}
            while (getAngle() > degrees) {}
        } else { // left turn.
            while (getAngle() < degrees) {}
        }

        // turn the motors off.
        stop();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
```
</details>
<details><summary>Trobot</summary>

```java
/* Copyright (c) 2020, All Rights Reserved
 *
 * This file is intended for FTC Team #8696 Trobotix only. Redistribution, duplication, or use in
 * source and binary forms, with or without modification, is not permitted without explicit
 * permission from the creator or authorized moderator.
 *
 * Written by Timothy (Tikki) Cui
 */

package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Trobot is a composition of 'Drivetrain' and 'Component'. It is the master file that each OpMode,
 * whether TeleOp or autonomous, will implement. It also tracks runtime. While it doesn't have any
 * utility methods, it improves code concision by combining different parts into a single object
 * that the user can import and implement.
 *
 * @author Timothy (Tikki) Cui and others
 * @version 2.5.0
 * @since release
 *
 * Version Log: (Obfuscated by site maintainer.)
 */

public class Trobot {
    private HardwareMap hardwareMap;

    private org.firstinspires.ftc.teamcode.SourceFiles.Drivetrain drivetrain;
    private org.firstinspires.ftc.teamcode.SourceFiles.Component component;

    private ElapsedTime runtime;

    // Constructor must utilize a Hardware Map from the source. However, Java always automatically
    // creates a default constructor, so custom error message must be made to catch error
    public Trobot() {
        throw new NullPointerException("Must pass hardwareMap into constructor");
    }

    public Trobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new org.firstinspires.ftc.teamcode.SourceFiles.Drivetrain(hardwareMap);
        component = new org.firstinspires.ftc.teamcode.SourceFiles.Component(hardwareMap);

        runtime = new ElapsedTime();
    }

    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public org.firstinspires.ftc.teamcode.SourceFiles.Drivetrain getDrivetrain() {return drivetrain;}
    public org.firstinspires.ftc.teamcode.SourceFiles.Component getComponent() {return component;}
    public ElapsedTime getRuntime() {return runtime;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setDrivetrain(org.firstinspires.ftc.teamcode.SourceFiles.Drivetrain drivetrain) {this.drivetrain = drivetrain;}
    public void setComponent(org.firstinspires.ftc.teamcode.SourceFiles.Component component) {this.component = component;}
    public void setElapsedTime(ElapsedTime runtime) {this.runtime = runtime;}

    public void idle() {
        while(true) {}
    }
}
```
</details>

___

## Autonomous
<details><summary>AutoZones</summary>

```java
package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in
 * either the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "AutoZones", group = "Auto")
public class AutoZones extends LinearOpMode {
    private Trobot trobot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        trobot = new Trobot(hardwareMap);

        sleep(1000);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        trobot.getRuntime().reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Autonomous program begun");
            telemetry.update();

            driveToRings();
            final int RINGS = trobot.getDrivetrain().detectRings();

            telemetry.addData("Rings", RINGS);
            telemetry.update();
//
//            if (RINGS == 0) {
//                driveToZoneA();
//            } else if (RINGS == 1) {
//                driveToZoneB();
//            } else if (RINGS == 4) {
//                driveToZoneC();
//            }

            sleep(30000);
        }
    }

    public void driveToRings() {
        telemetry.addData("Status", "Driving to rings");
        telemetry.update();

        // strafe left
        trobot.getDrivetrain().strafeLeft(0.3);
        sleep(500);
        trobot.getDrivetrain().stop();
        sleep(1000);

        // open arm
        trobot.getComponent().openWobbleArm(1500);
        sleep(1000);

        // get the wobble, pausing briefly to secure it
        trobot.getDrivetrain().autoDriveIMU(0.2, 1500);

        // close the latch
        trobot.getComponent().armLatch();
        sleep(1000);

        // strafe right a little
        trobot.getDrivetrain().strafeRight(0.3);
        sleep(1000);
        trobot.getDrivetrain().stop();
        sleep(1000);

        // drive forward
        trobot.getDrivetrain().autoDriveIMU(0.2, 1100);
        
        sleep(2000);
    }

    public void driveToZoneA() {
        telemetry.addData("Status", "Driving to Zone A");
        telemetry.update();
        trobot.getDrivetrain().autoDriveIMU(0.2, 3000);

        trobot.getComponent().armUnlatch();

        sleep(1000);

        trobot.getDrivetrain().drive(-0.2);
        sleep(500);

        trobot.getDrivetrain().stop();

        trobot.getDrivetrain().strafeLeft(0.2);
        sleep(1500);

        trobot.getDrivetrain().stop();

    }

    public void driveToZoneB() {
        telemetry.addData("Status", "Driving to Zone B");
        telemetry.update();

        trobot.getDrivetrain().autoDriveDistance(0.2, 35);

        trobot.getDrivetrain().strafeLeft(.3);
        sleep(2000);

        trobot.getDrivetrain().stop();
        sleep(500);

        trobot.getComponent().armUnlatch();

        sleep(1000);

        trobot.getDrivetrain().drive(-0.2);
        sleep(500);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().strafeLeft(.3);

        sleep(1500);
        trobot.getDrivetrain().stop();

        trobot.getDrivetrain().drive(-0.2);
        sleep(1500);

        trobot.getDrivetrain().stop();
    }

    public void driveToZoneC() {
        telemetry.addData("Status", "Driving to Zone C");
        telemetry.update();

        trobot.getDrivetrain().autoDriveDistance(0.2, 15);

        trobot.getComponent().armUnlatch();

        sleep(1000);

        trobot.getDrivetrain().drive(-0.2);
        sleep(500);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().strafeLeft(.3);

        sleep(1500);
        trobot.getDrivetrain().stop();

        trobot.getDrivetrain().drive(-0.2);
        sleep(2300);

        trobot.getDrivetrain().stop();
    }
}
```
</details>

___

## TeleOp
<details><summary>TeleOp_POV</summary>

```java
package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic POV Teleop for a four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp_POV extends LinearOpMode {
    public Trobot trobot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        trobot = new Trobot(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        trobot.getRuntime().reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double leftPower = Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -1.0, 1.0);
            double rightPower = Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -1.0, 1.0);

            // Send calculated power to wheels
            trobot.getDrivetrain().drive(leftPower*0.3, rightPower*0.3);

            // Set a-button for speed reduction
            if (gamepad1.y) {
                trobot.getDrivetrain().switchSpeed();
            }

            // Set D-Pad for strafing -> not used for Joe 2019-2020
            if (gamepad1.dpad_left) {
                trobot.getDrivetrain().strafeLeft(0.7);
            } else if (gamepad1.dpad_right) {
                trobot.getDrivetrain().strafeRight(0.7);
            } else if (gamepad1.dpad_up) {
                trobot.getDrivetrain().drive(0.7);
            } else if (gamepad1.dpad_down) {
                trobot.getDrivetrain().drive(-0.7);
            }

            // Wobble arm
            if (gamepad1.a) {
                trobot.getComponent().openWobbleArm();
            } else if (gamepad1.b) {
                trobot.getComponent().closeWobbleArm();
            } else {
                trobot.getComponent().stopWobbleArm();
            }

            // Latch
            if (gamepad1.x) {
                trobot.getComponent().armLatch();
            } else if (gamepad1.y) {
                trobot.getComponent().armUnlatch();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + trobot.getRuntime().toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            telemetry.addData("Speed", trobot.getDrivetrain().getSpeedStatus());
            telemetry.update();
        }
    }
}
```
</details>
