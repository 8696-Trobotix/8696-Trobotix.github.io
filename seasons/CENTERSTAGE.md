---
title: "CENTERSTAGE 2023 - 2024"
permalink: /CENTERSTAGE
---

[< Back](../index.md) | [< General Info](./index.md)

# CENTERSTAGE

| <image src="../images/CENTERSTAGE/CenterstageImg.jpg" style="max-width: 100%; border: none; box-shadow: none;" /> | <image src="../images/CENTERSTAGE/Glow.jpg" style="max-width: 100%; border: none; box-shadow: none;" /> | <image src="../images/CENTERSTAGE/Field.jpg" style="max-width: 100%; border: none; box-shadow: none;" /> |
| :---: | :---: | :---: |
| Final robot. | Return of the LED lights. | Robot on field. |

| <image src="../images/CENTERSTAGE/Trobotix2.png" style="width: 100%; max-width: 100%; border: none; box-shadow: none;" /> | <image src="../images/CENTERSTAGE/Vision.png" style="max-width: 100%; border: none; box-shadow: none;" /> | <video src="../images/CENTERSTAGE/HangTest.webm" style="max-width: 100%; border: none; box-shadow: none;" preload="auto" muted controls></video> |
| :---: | :---: | :---: |
| Trobotix #2. | Contour-based object detection with EOCV. | First successful hanging test. |

This season's robot name is "<span style="font-style: italic;">Troballin</span>", but is referred to as <span style="font-style: italic;">Squid</span> in the source code.  
The [mollusc](https://github.com/8696-Trobotix/mollusc) library was jointly developed and used extensively this season.  
League Tournament: Winning Alliance Captain and 1st Place Inspire Award  
Iowa Championship: Rank #7 in Black Division (did not advance)

___

## mollusc
See [6e446fa](https://github.com/8696-Trobotix/mollusc/tree/6e446fa5511e04d43cd89c9d4b7c47a7520a5fdf) (basically pre-release v0.1.0).

## Image Processing Pipelines

<details><summary>TotemPipeline</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta.pipelines;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;
import org.firstinspires.ftc.teamcode.mollusc.vision.*;

import java.util.List;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Mat;

public class TotemPipeline extends OpenCvPipeline {

    // Note: Red hues wrap around the hue color wheel.
    public Scalar lowerRed = new Scalar(0, 95, 95);
    public Scalar upperRed = new Scalar(10, 255, 255);
    public Scalar lowerRed2 = new Scalar(160, 95, 95);
    public Scalar upperRed2 = new Scalar(180, 255, 255);
    public Scalar lowerBlue = new Scalar(100, 80, 80);
    public Scalar upperBlue = new Scalar(130, 255, 255);

    private ColorRange redColorRange = new ColorRange(lowerRed, upperRed);
    private ColorRange redColorRange2 = new ColorRange(lowerRed2, upperRed2);
    private ColorRange blueColorRange = new ColorRange(lowerBlue, upperBlue);

    private Point p1 = new Point(0, 0), p2 = new Point(110, 240);
    private Point p3 = new Point(110, 0), p4 = new Point(320, 240);
    private Rect r1 = new Rect (p1, p2);
    private Rect r2 = new Rect (p3, p4);

    private ColorRange[] focusRange;
    private Alliance alliance;
    private List<VisionObject> objs = null;

    public double threshold = 0.025;
    public boolean calibrate = true;

    public TotemPipeline(Alliance alliance) {
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            focusRange = new ColorRange[] {redColorRange, redColorRange2};
        } else {
            focusRange = new ColorRange[] {blueColorRange};
        }
    }

    public void setBound(Scalar bound, int idx, double value) {
        bound.val[idx] = value;
    }

    @Override
    public void init(Mat firstFrame) {
    }

    @Override
    public Mat processFrame(Mat inputFrame) {
        objs = ObjectDetector.coloredObjectCoordinates(
            inputFrame, 
            threshold, 
            0.0, 
            calibrate, 
            focusRange
        );
        if (calibrate) {
            ObjectDetector.rectangle(
                inputFrame, 
                (int)p1.x, 
                (int)p1.y, 
                (int)p2.x, 
                (int)p2.y, 
                ObjectDetector.BLUE_COLOR, 2
            );
        }
        return inputFrame;
    }

    public TotemZone getZone() {
        if (objs == null || objs.isEmpty()) {
            return TotemZone.RIGHT;
        } else { // This may help mitigate the exceedingly rare edge case where the image processing thread clears `objs` after the prior check.
            VisionObject largest = objs.get(0);
            for (VisionObject obj : objs) {
                if (obj.pixelTotality > largest.pixelTotality) {
                    largest = obj;
                }
            }
            if (largest.x > p2.x) {
                return TotemZone.CENTER;
            }
            return TotemZone.LEFT;
        }
    }

    public enum Alliance {
        RED, BLUE
    }
    public enum TotemZone {
        LEFT, CENTER, RIGHT
    }
}
```
</details>

___

## Configuration
<details><summary>SquidWare</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta;

import org.firstinspires.ftc.teamcode.squid.delta.subsystems.*;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.*;
import org.firstinspires.ftc.teamcode.mollusc.drivetrain.*;
import org.firstinspires.ftc.teamcode.mollusc.wrapper.*;
import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

// import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class SquidWare {

    public Configuration config;

    public DrivetrainBaseFourWheel base;

    // Deadwheels.
    public DeadWheels deadWheels;

    public Intake intake = new Intake();
    public Conveyor conveyor = new Conveyor();
    public Scoring scoring = new Scoring();
    public Launcher launcher = new Launcher();
    public Lift lift = new Lift();

    public IMU imu;

    public SquidWare(Configuration config, boolean resetIMU) throws Exception {
        this.config = config;

        intake.UP_1 = intake.intakePos1 = config.getDouble("intake servo up 1");
        intake.UP_2 = intake.intakePos2 = config.getDouble("intake servo up 2");
        intake.DOWN_1 = config.getDouble("intake servo down 1");
        intake.DOWN_2 = config.getDouble("intake servo down 2");
        intake.MAX_DOWN_1 = config.getDouble("intake servo 1 max down");
        intake.MAX_DOWN_2 = config.getDouble("intake servo 2 max down");
        intake.DELTA = config.getDouble("intake servo delta");
        intake.PRESET_POWER = config.getDouble("intake power");

        conveyor.PRESET_POWER = config.getDouble("conveyor power");
        conveyor.DISTANCE_THRESHOLD = config.getDouble("pixel distance threshold cm");
        conveyor.PIXEL_TIME = config.getDouble("pixel time seconds");
        conveyor.CARRY_TIME = config.getDouble("pixel carry time seconds");
        
        scoring.SPINNER_POWER = config.getDouble("spinning servo powers");
        scoring.SLIDE_MIN = config.getInteger("slide min");
        scoring.SLIDE_MAX = config.getInteger("slide max");
        scoring.RELEASE_PIXEL = config.getDouble("release go");
        scoring.RELEASE_RESET = config.getDouble("release reset");
        scoring.ANGLER_DELTA = config.getDouble("angling servo delta");

        launcher.FIRE = config.getDouble("launcher fire");

        lift.POWER = config.getDouble("lift power");

        Make make = new Make();

        imu = make.imu(
            "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
            resetIMU
        );

        base = new DrivetrainBaseFourWheel(
            make.motor("frontLeft", getStringConfigDirection("frontLeft direction").motor), 
            make.motor("frontRight", getStringConfigDirection("frontRight direction").motor), 
            make.motor("rearLeft", getStringConfigDirection("rearLeft direction").motor), 
            make.motor("rearRight", getStringConfigDirection("rearRight direction").motor)
        );

        intake.servo1 = make.servo("intakeServo1", getStringConfigDirection("intake servo 1 direction").servo);
        intake.servo2 = make.servo("intakeServo2", getStringConfigDirection("intake servo 2 direction").servo);
        intake.motor = make.motor("intake", getStringConfigDirection("intake motor direction").motor);
        
        conveyor.motor = make.motor("conveyor", getStringConfigDirection("conveyor motor direction").motor);
        conveyor.sensor = Mollusc.opMode.hardwareMap.get(DistanceSensor.class, "pixelSensor");

        scoring.angler = make.servo("angler", getStringConfigDirection("angling servo direction").servo);
        scoring.slide = make.motor("slide", getStringConfigDirection("slide motor direction").motor);
        scoring.slideEncoder = new Encoder(scoring.slide, 1, 1, 1);
        // scoring.limitSwitch = Mollusc.opMode.hardwareMap.get(TouchSensor.class, "limitSwitch");
        scoring.spinner1 = make.crservo("spinner1", getStringConfigDirection("spinner servo 1 direction").crservo);
        scoring.spinner2 = make.crservo("spinner2", getStringConfigDirection("spinner servo 2 direction").crservo);
        scoring.stopper = make.servo("release", getStringConfigDirection("release servo direction").servo);

        launcher.servo = make.crservo("launcher", CRServo.Direction.FORWARD);

        int TPR = config.getInteger("TPR");
        DcMotorEx leftEncoderMotor = make.motor("leftEncoder", DcMotorEx.Direction.FORWARD);
        Encoder leftEncoder = new Encoder(leftEncoderMotor, -1.0, TPR, 35);
        Encoder rightEncoder = new Encoder(intake.motor, 1.0, TPR, 35);
        Encoder centerEncoder = new Encoder(conveyor.motor, 1.0, TPR, 35);
        deadWheels = new DeadWheels(
            new Pose(0, 0, Math.PI),
            leftEncoder, rightEncoder, centerEncoder, 
            -352.435, 30
        );

        // lift.motor = make.motor("lift", DcMotorEx.Direction.FORWARD);
        lift.motor = leftEncoderMotor;
    }

    public DirectionGroup getStringConfigDirection(String key) throws Exception {
        String direction = config.getString(key);
        return new DirectionGroup(direction);
    }

    public class DirectionGroup {
        public DcMotorEx.Direction motor;
        public Servo.Direction servo;
        public CRServo.Direction crservo;

        public DirectionGroup(String direction) throws Exception {
            if (!direction.equals("forward") && !direction.equals("reverse")) {
                throw new Exception("Invalid motor direction value: " + direction);
            }
            if (direction.equals("forward")) {
                this.motor = DcMotorEx.Direction.FORWARD;
                this.servo = Servo.Direction.FORWARD;
                this.crservo = CRServo.Direction.FORWARD;
            } else {
                this.motor = DcMotorEx.Direction.REVERSE;
                this.servo = Servo.Direction.REVERSE;
                this.crservo = CRServo.Direction.REVERSE;
            }
        }
    }
}
```
</details>
<details><summary>delta [TXT]</summary>

```
// Configuration File

field centric on: true

autonomous drive power: 0.5

drive power max: 0.87
strafe power max: 0.97
turn power max: 0.87
drive power max reduced: 0.6
strafe power max reduced: 0.7
turn power max reduced: 0.6

frontLeft direction:  reverse
frontRight direction: forward
rearLeft direction:   reverse
rearRight direction:  forward

TPR: 8192

intake servo 1 direction: forward
intake servo 2 direction: forward
intake motor direction: forward

conveyor motor direction: reverse
pixel distance threshold cm: 5
pixel time seconds: 0.89
pixel carry time seconds: 0.2

angling servo direction: forward
slide motor direction: reverse
spinner servo 1 direction: forward
spinner servo 2 direction: forward
release servo direction: forward

intake servo up 1: 0.52
intake servo up 2: 0.70
intake servo down 1: 0.67
intake servo down 2: 0.55
intake servo 1 max down: 0.69
intake servo 2 max down: 0.53
intake servo delta: 0.01
intake power: 0.68

conveyor power: 1
spinning servo powers: 1
slide min: 10
slide max: 2050
angling servo delta: 0.015
release go: 0.35
release reset: 0.63

launcher fire: 1

lift power: -1
```
</details>

___

## Subsystems
<details><summary>Conveyor</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

public class Conveyor {

    public DcMotorEx motor;
    public DistanceSensor sensor;
    public ElapsedTime timer = new ElapsedTime();

    public double PRESET_POWER;
    // PIXEL_TIME --> time until sensor begins searching for pixels again.
    // CARRY_TIME --> time until the conveyor is switched to manual / stop mode after detecting the second pixel.
    public double DISTANCE_THRESHOLD, PIXEL_TIME, CARRY_TIME;

    public DetectionState detectionState = DetectionState.NONE;

    public void power(double power) {
        motor.setPower(power);
    }

    public boolean pixelOverflow() {
        boolean sensorState = sensor.getDistance(DistanceUnit.CM) <= DISTANCE_THRESHOLD;

        switch (detectionState) {
            case NONE:
                if (sensorState && timer.seconds() >= PIXEL_TIME) {
                    detectionState = DetectionState.FIRST_PIXEL;
                    timer.reset();
                }
                break;
            case FIRST_PIXEL:
                if (sensorState && timer.seconds() >= PIXEL_TIME) {
                    detectionState = DetectionState.SECOND_PIXEL;
                    timer.reset();
                }
                break;
            case SECOND_PIXEL:
                if (timer.seconds() >= CARRY_TIME) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void resetState() {
        detectionState = DetectionState.NONE;
        timer.reset();
    }

    public enum DetectionState {
        NONE, FIRST_PIXEL, SECOND_PIXEL
    }
}
```
</details>
<details><summary>Intake</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public Servo servo1, servo2;
    public DcMotorEx motor;

    public double UP_1, UP_2;
    public double DOWN_1, DOWN_2;
    public double DELTA;
    public double MAX_DOWN_1, MAX_DOWN_2;
    public double PRESET_POWER;
    public double intakePos1, intakePos2;

    public void presetUp() {
        position(UP_1, UP_2);
    }

    public void presetDown() {
        position(DOWN_1, DOWN_2);
    }

    public void up() {
        position(intakePos1 - DELTA, intakePos2 + DELTA);
    }

    public void down() {
        position(intakePos1 + DELTA, intakePos2 - DELTA);
    }

    public void position(double pos1, double pos2) {
        if (pos1 >= UP_1 && pos1 <= MAX_DOWN_1) {
            intakePos1 = pos1;
            servo1.setPosition(intakePos1);
        }
        if (pos2 <= UP_2 && pos2 >= MAX_DOWN_2) {
            intakePos2 = pos2;
            servo2.setPosition(intakePos2);
        }
    }

    public void power(double power) {
        motor.setPower(power);
    }
}
```
</details>
<details><summary>Launcher</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

public class Launcher {

    public CRServo servo;

    public double FIRE;

    public void fire() {
        servo.setPower(FIRE);
    }
    public void stop() {
        servo.setPower(0);
    }
}
```
</details>
<details><summary>Lift</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {

    public DcMotorEx motor;

    public double POWER;

    public void power(double power) {
        motor.setPower(power);
    }
}
```
</details>
<details><summary>Scoring</summary>

```java
package org.firstinspires.ftc.teamcode.squid.delta.subsystems;

import org.firstinspires.ftc.teamcode.mollusc.wrapper.Encoder;

// import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {

    public Servo angler;
    public DcMotorEx slide;
    public Encoder slideEncoder;
    // public TouchSensor limitSwitch;
    public CRServo spinner1, spinner2;
    public Servo stopper;

    public double SPINNER_POWER;
    public double RELEASE_PIXEL, RELEASE_RESET;
    public int SLIDE_MIN, SLIDE_MAX;
    public double ANGLER_DELTA;

    public double anglerPos = 1.0;
    
    public void slidePower(double power) {
        if (slide.isOverCurrent()
            || (power > 0 && slideEncoder.getTicks() > SLIDE_MAX)
            || (power < 0 && slideEncoder.getTicks() < SLIDE_MIN)
            /*|| (power < 0 && limitSwitch.isPressed())*/) {
            slide.setPower(0);
            return;
        }
        slide.setPower(power);
    }

    public void spinningOn() {
        spinner1.setPower(-SPINNER_POWER);
        spinner2.setPower(-SPINNER_POWER);
    }

    public void spinningOff() {
        spinner1.setPower(0);
        spinner2.setPower(0);
    }

    public void spinningPanic() {
        spinner1.setPower(SPINNER_POWER);
        spinner2.setPower(SPINNER_POWER);
    }

    public void anglerUp() {
        anglerPosition(anglerPos + ANGLER_DELTA);
    }
    public void anglerDown() {
        anglerPosition(anglerPos - ANGLER_DELTA);
    }
    public void anglerPosition(double pos) {
        if (0 <= pos && pos <= 1) {
            anglerPos = pos;
            angler.setPosition(anglerPos);
        }
    }

    public void releasePixel() {
        stopper.setPosition(RELEASE_PIXEL);
    }

    public void releaseReset() {
        stopper.setPosition(RELEASE_RESET);
    }
}
```
</details>

___

## Autonomous
<details><summary>AutoSquidDelta</summary>

```java
/*
AutoSquidDelta

Main robot.
*/

package org.firstinspires.ftc.teamcode.squid.delta;

import org.firstinspires.ftc.teamcode.squid.delta.pipelines.TotemPipeline;

import org.firstinspires.ftc.teamcode.mollusc.auto.interpreter.*;
import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.wrapper.*;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;
import org.firstinspires.ftc.teamcode.mollusc.auto.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="AutoSquidDelta III-I", group="Squid")

public class AutoSquidDelta extends MolluscLinearOpMode {

    private Configuration config;

    private Auto auto;
    private SquidWare squidWare;
    private Interpreter interpreter;
    private TotemPipeline totemPipeline;

    private boolean alliance = true, route = true;

    @Override
    public void molluscRunOpMode() {
        telemetry.setAutoClear(false);

        Configuration.handle(() -> {
            config = new Configuration("delta.txt");
            alliance = Configuration.inputBoolean("Alliance", "Red", "Blue", alliance);
            route = Configuration.inputBoolean("Route", "Short", "Long", route);
            squidWare = new SquidWare(config, true);
            interpreter = new Interpreter(new Asset("script_delta.txt"));

            register(); // Important.

            double maxDrivePower = config.getDouble("autonomous drive power");
            auto = new MecanumAutoII(
                squidWare.base, 
                squidWare.deadWheels, 
                interpreter, 
                new PIDF(0.005, 0, 0, 0.15, 0.25, 1),
                new PIDF(0.009, 0, 0, 0.2, 0.25, 1),
                new PIDF(1.4, 0.00, 0.00, 0.15, 0.25, 1),
                maxDrivePower, 
                30,
                6
            );
            auto.register();
        });

        telemetry.setAutoClear(true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        totemPipeline = new TotemPipeline(alliance ? TotemPipeline.Alliance.RED : TotemPipeline.Alliance.BLUE);
        webcam.setPipeline(totemPipeline);

        PIDF.bulkMode();

        // Pre-start manual image processing calibrations.
        while (!gamepad1.dpad_up && !isStopRequested()) {
            telemetry.addLine("Press dpad up to confirm.");
            telemetry.addLine("(A) --> Configure threshold.");
            telemetry.addLine("(X) --> Configure Red 1.");
            telemetry.addLine("(Y) --> Configure Red 2.");
            telemetry.addLine("(B) --> Configure Blue.");
            displayTotemZone();
            telemetry.update();

            if (gamepad1.a) {
                configDoubleNonBlocking(totemPipeline.threshold, 0.001, (double value) -> {
                    totemPipeline.threshold = value;
                    telemetry.addData("Threshold", value);
                });
            } else if (gamepad1.x) {
                Configuration.handle(() -> {
                    boolean ub = Configuration.inputBoolean("Bound", "Upper", "Lower", true);
                    int idx = Configuration.inputInteger("Value", 0, 0.25, 1, new String[] {"Hue", "Saturation", "Brightness"});
                    if (ub) {
                        configDoubleNonBlocking(totemPipeline.upperRed.val[idx], 1, (double value) -> {
                            totemPipeline.setBound(totemPipeline.upperRed, idx, value);
                            telemetry.addData("Upper Red Color Bound", totemPipeline.upperRed);
                        });
                    } else {
                        configDoubleNonBlocking(totemPipeline.lowerRed.val[idx], 1, (double value) -> {
                            totemPipeline.setBound(totemPipeline.lowerRed, idx, value);
                            telemetry.addData("Lower Red Color Bound", totemPipeline.lowerRed);
                        });
                    }
                });
            } else if (gamepad1.y) {
                Configuration.handle(() -> {
                    boolean ub = Configuration.inputBoolean("Bound", "Upper", "Lower", true);
                    int idx = Configuration.inputInteger("Value", 0, 0.25, 1, new String[] {"Hue", "Saturation", "Brightness"});
                    if (ub) {
                        configDoubleNonBlocking(totemPipeline.upperRed2.val[idx], 1, (double value) -> {
                            totemPipeline.setBound(totemPipeline.upperRed2, idx, value);
                            telemetry.addData("Upper Red Color Bound 2", totemPipeline.upperRed2);
                        });
                    } else {
                        configDoubleNonBlocking(totemPipeline.lowerRed2.val[idx], 1, (double value) -> {
                            totemPipeline.setBound(totemPipeline.lowerRed2, idx, value);
                            telemetry.addData("Lower Red Color Bound 2", totemPipeline.lowerRed2);
                        });
                    }
                });
            } else if (gamepad1.b) {
                Configuration.handle(() -> {
                    boolean ub = Configuration.inputBoolean("Bound", "Upper", "Lower", true);
                    int idx = Configuration.inputInteger("Value", 0, 0.25, 1, new String[] {"Hue", "Saturation", "Brightness"});
                    if (ub) {
                        configDoubleNonBlocking(totemPipeline.upperBlue.val[idx], 1, (double value) -> {
                            totemPipeline.setBound(totemPipeline.upperBlue, idx, value);
                            telemetry.addData("Upper Blue Color Bound", totemPipeline.upperBlue);
                        });
                    } else {
                        configDoubleNonBlocking(totemPipeline.lowerBlue.val[idx], 1, (double value) -> {
                            totemPipeline.setBound(totemPipeline.lowerBlue, idx, value);
                            telemetry.addData("Lower Blue Color Bound", totemPipeline.lowerBlue);
                        });
                    }
                });
            }

            idle();
        }

        totemPipeline.calibrate = false;
        telemetry.addLine("Waiting for start.");
        telemetry.update();

        waitForStart();

        webcam.pauseViewport();

        Configuration.handle(() -> {
            interpreter.run(true);
        });
    }

    private void register() {
        interpreter.register("GoToTotemSection", (Object[] args) -> {
            String delimiter = alliance ? "Red" : "Blue";
            switch (totemPipeline.getZone()) {
                case CENTER:
                    interpreter.advanceTo("SectionTotemCenter" + delimiter);
                    break;
                case LEFT:
                    interpreter.advanceTo("SectionTotemLeft" + delimiter);
                    break;
                case RIGHT:
                    interpreter.advanceTo("SectionTotemRight" + delimiter);
                    break;
            }
        });
        interpreter.register("SectionTotemCenterRed", Interpreter.noneAction);
        interpreter.register("SectionTotemLeftRed", Interpreter.noneAction);
        interpreter.register("SectionTotemRightRed", Interpreter.noneAction);
        interpreter.register("SectionTotemCenterBlue", Interpreter.noneAction);
        interpreter.register("SectionTotemLeftBlue", Interpreter.noneAction);
        interpreter.register("SectionTotemRightBlue", Interpreter.noneAction);

        interpreter.register("GoToShortOrLongRoute", (Object[] args) -> {
            if (route) {
                interpreter.advanceTo("ShortRoute");
            } else {
                interpreter.advanceTo("LongRoute");
            }
        });
        interpreter.register("ShortRoute", Interpreter.noneAction);
        interpreter.register("LongRoute", Interpreter.noneAction);

        interpreter.register("intake", (Object[] args) -> {
            switch ((String)args[0]) {
                case "up":
                    squidWare.intake.presetUp();
                    break;
                case "down":
                    squidWare.intake.presetDown();
                    break;
                case "on":
                    squidWare.intake.power(squidWare.intake.PRESET_POWER);
                    break;
                case "reverse":
                    squidWare.intake.power(-squidWare.intake.PRESET_POWER);
                    break;
                case "off":
                    squidWare.intake.power(0);
                    break;
            }
        }, String.class);
        
        interpreter.register("itsPower", (Object[] args) -> {
            squidWare.conveyor.power((Double)args[0]);
        }, Double.class);
        interpreter.register("slidePower", (Object[] args) -> {
            squidWare.scoring.slidePower((Double)args[0]);
            interpreter.getActions().get("wait_seconds-f").execute(new Object[] {args[1]});
            squidWare.scoring.slidePower(0);
        }, Double.class, Double.class);

        interpreter.register("setAnglerPos", (Object[] args) -> {
            squidWare.scoring.anglerPosition((Double)args[0]);
        }, Double.class);

        interpreter.register("scoring", (Object[] args) -> {
            switch ((String)args[0]) {
                case "release":
                    squidWare.scoring.releasePixel();
                    break;
                case "close":
                    squidWare.scoring.releaseReset();
                    break;
            }
        }, String.class);

        interpreter.register("pause", (Object[] args) -> {
            while (!gamepad1.a && !isStopRequested()) {
                idle();
            }
        });

        interpreter.register("GoToEnd", (Object[] args) -> {
            interpreter.advanceTo("END");
        });
        interpreter.register("END", Interpreter.noneAction);
    }

    public void displayTotemZone() {
        telemetry.addData("Totem Zone", totemPipeline.getZone());
    }

    private void configDoubleNonBlocking(double value, double delta, Synchronous synchronous) {
        while (gamepad1.a && !isStopRequested()) {
            idle();
        }
        while (!gamepad1.a && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                value += delta;
                sleep(250);
            } else if (gamepad1.dpad_down) {
                value -= delta;
                sleep(250);
            } else if (gamepad1.dpad_left) {
                delta /= 10;
                sleep(250);
            } else if (gamepad1.dpad_right) {
                delta *= 10;
                sleep(250);
            }
            synchronous.process(value);
            displayTotemZone();
            telemetry.addLine("Press (A) to confirm.");
            telemetry.addData("Value", value);
            telemetry.addData("Delta", delta);
            telemetry.update();
            idle();
        }
        while (gamepad1.a && !isStopRequested()) {
            idle();
        }
    }

    @FunctionalInterface
    private interface Synchronous {
        void process(double value);
    }
}
```
</details>
<details><summary>script_delta [TXT]</summary>

```
// Timeout seconds: seconds before a move times out and proceeds to the next instruction.
// Static timeout milliseconds: milliseconds during which, if the algorithms determines that the robot
//                              is in the right pose based on tolerances, it will end the move.
set_timeout_seconds 5.0
set_static_timeout_milliseconds 50

// Drive command: x / forward axis, y / horizontal axis, heading in degrees
// Example: drive 1000 0 180 --> x = 100, y = 0, z = 180 deg.

intake down
setAnglerPos 1.0
wait_seconds 1.5

// BEGIN ISOLATED TESTING

// Note: use the command "pause" to pause the autonomous at that point until gamepad1.a.

// Run some isolated tests here.
//GoToEnd
// END ISOLATED TESTING

GoToShortOrLongRoute

ShortRoute
    GoToTotemSection

    SectionTotemCenterRed
        drive 800	-8.72	178.7628321
        intake up
        itsPower -1.0
        wait_seconds 0.5
        itsPower 0.0
        drive 543.91	-4.69	178.7628321
        drive 656.57	641.65	171.3143807
        drive 625.13	674.22	-89.95437384
        drive 626.45	956.53	-89.95437384
        wait_seconds 0.3
        setAnglerPos 0.0
        wait_seconds 0.9
        scoring release
        wait_seconds 2.0
        drive 626 890 -89.95
        drive 60	850	-89.95437384
        GoToEnd

    SectionTotemLeftRed
        drive 397.34	6.72	177.6169165
        drive 405.06	32.17	-146.6771956
        drive 686.08	-151.03	-128.3425461
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 556.9	172.35	-136.3639552
        drive 630.12	165.71	-91.67324722
        drive 750.7	980.25	-94.5380362
        wait_seconds 0.3
        setAnglerPos 0.0
        wait_seconds 0.9
        scoring release
        wait_seconds 2.0
        drive 751	890.68	-94.50282517
        drive -3.3	871.68	-97.40282517
        GoToEnd

    SectionTotemRightRed
        drive 325.3	269.5	179.3357899
        drive 656.65	272.33	178.7628321
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 301.27	263.74	178.1898743
        drive 364.58	294.51	-92.24620502
        drive 484.5	1006.79	-91.67324722
        wait_seconds 0.3
        setAnglerPos 0.0
        wait_seconds 0.9
        scoring release
        wait_seconds 2.0
        drive 484.58 950 -91.67
        drive 48.98	910.33	-92.24620502
        GoToEnd

    SectionTotemCenterBlue
        drive 807.01	-3.07	-179.3357899
        intake up
        itsPower -1.0
        wait_seconds 0.5
        itsPower 0.0
        drive 420.1	2.18	-178.7628321
        drive 688.16	-1014	92.24620502
        wait_seconds 0.3
        setAnglerPos 0.0
        wait_seconds 0.9
        scoring release
        wait_seconds 2.0
        drive 688.16 -950 92.25
        drive 43.38	-903.21	93.9650784
        GoToEnd

    SectionTotemLeftBlue
        drive 402.16	-261.46	-177.6169165
        drive 661.23	-273.34	-178.1898743
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 309.23	-258.59	-177.6169165
        drive 481.18	-985.84	92.81916281
        wait_seconds 0.3
        setAnglerPos 0.0
        wait_seconds 0.9
        scoring release
        wait_seconds 2.0
        drive 481.18 -950 92.82
        drive 34.27	-910	93.9650784
        GoToEnd

    SectionTotemRightBlue
        drive 475.96	6.47	-179.3357899
        drive 467.6	27.7	148.3960689
        drive 686.7	173.08	139.801702
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 472.14	-132.4	140.9476176
        drive 812.57	-1001.28	93.9650784
        wait_seconds 0.3
        setAnglerPos 0.0
        wait_seconds 0.9
        scoring release
        wait_seconds 2.0
        drive 812.57 -950 93.965
        drive 43.06	-914.46	90.52733163
        GoToEnd

LongRoute
    GoToTotemSection

    SectionTotemCenterRed
        drive 792.02	-27.63	-178.7628321
        intake up
        itsPower -1.0
        wait_seconds 0.5
        itsPower 0.0
        drive 525.96	-0.98	-178.7628321
        drive 514.4	-392.49	179.9087477
        drive 1363.54	-390.8	-178.7628321
        drive 1341.46	-481.21	90.52733163
        setAnglerPos 0.0
        wait_seconds 0.5
        drive 1404.45	1807.39	85.37071147
        drive 760.09	1800.26	91.67324722
        drive 732.16	1885.96	-90.52733163
        drive 722.5	2189.4	-91.67324722
        scoring release
        wait_seconds 2.0
        drive 722.5 2100 -91.67
        drive 1321.6	2179.86	-93.9650784
        GoToEnd

    SectionTotemLeftRed
        drive 341.75	-239.31	-177.6169165
        drive 661.65	-243.3	-178.1898743
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 367.2	-251.73	-177.6169165
        drive 371.79	26.82	-178.7628321
        drive 1255.44	3.24	-179.3357899
        drive 1298.4	-28.39	92.24620502
        setAnglerPos 0.0
        wait_seconds 0.5
        drive 1382.77	1901.32	90.52733163
        drive 753.79	1984.12	90.52733163
        drive 880.15	2135.31	-91.10028943
        drive 880	2216.99	-91.67324722
        scoring release
        wait_seconds 2.0
        drive 880 2180 -91.67
        drive 1394.92	2123.56	-92.81916281
        GoToEnd

    SectionTotemRightRed
        drive 740.51	-322.67	91.100
        drive 760.79 92.98 91.673
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 714.64 -378.83 91.100
        drive 1391.31 -253.43 91.08958
        setAnglerPos 0.0
        wait_seconds 0.5
        drive 1366.08 1924.05 88.235500
        drive 790 1977.66 -95.684
        drive 690 2150.96 -95.68
        scoring release
        wait_seconds 2.0
        drive 690 2090 -95.68
        drive 1372.83 2070.38 -97.976
        GoToEnd

    SectionTotemCenterBlue
        drive 825.53 16.56 -178.19
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 417.69 38.18 -176.47
        drive 411.11 440.22 -177.04
        drive 1284.78 408.61 179.91
        drive 1348.43 330.31 -91.100
        setAnglerPos 0.0
        wait_seconds 0.5
        drive 1367.78 -1951.86 -89.38
        drive 797.3 -2022.37 86.52
        drive 817.99 -2185.11 85.944
        scoring release
        wait_seconds 2.0
        drive 817.99 -2100 85.944
        drive 1370.94 -2083.26 87.09
        GoToEnd

    SectionTotemLeftBlue
        drive 756.56 254.07 -91.67
        drive 777.56 -88.87 -91.67
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 752.78 378.4 -89.95
        drive 1329.63 390.62 -93.965
        setAnglerPos 0.0
        wait_seconds 0.5
        drive 1414.41 -1905.87 -92.246
        drive 743.44 -1963.94 84.798
        drive 758.97 -2182.25 84.978
        scoring release
        wait_seconds 2.0
        drive 758.97 -2100 84.978
        drive 1424.54 -2039.83 83.651
        GoToEnd

    SectionTotemRightBlue
        drive 443.72 282.42 179.336
        drive 688.39 277.46 178.19
        intake up
        itsPower -1.0
        wait_seconds 1.0
        itsPower 0.0
        drive 398.47 274.36 178.76
        drive 387.53 -34.25 179.33
        drive 1257.17 0.89 178.76
        drive 1254.31 30.11 -92.2462
        setAnglerPos 0.0
        wait_seconds 0.5
        drive 1300.43 -1892.43 -91.67
        drive 930.69 -1922.98 84.80
        drive 950.18 -2204.24 85.371
        scoring release
        wait_seconds 2.0
        drive 950.18 -2150 85.371
        drive 1390.62 -2045.26 86.5166
        GoToEnd

END
```
</details>

___

## TeleOp
<details><summary>SquidDelta</summary>

```java
/*
SquidDelta

Main robot.
*/

package org.firstinspires.ftc.teamcode.squid.delta;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.*;
import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.wrapper.*;
import org.firstinspires.ftc.teamcode.squid.delta.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SquidDelta III-I", group="Squid")

public class SquidDelta extends MolluscLinearOpMode {

    private Configuration config;

    private Drivetrain drivetrain;
    private SquidWare squidWare;

    private MecanumFieldCentric fieldCentricDrive;
    private MecanumRobotCentric robotCentricDrive;
    private double maxDrive, maxStrafe, maxTurn;
    private double maxDriveReduced, maxStrafeReduced, maxTurnReduced;
    private double drive, strafe, turn;

    private UpDownState intakeState = UpDownState.UP;
    private SystemState intakeMotorState = SystemState.OFF;
    private SystemState conveyorMotorState = SystemState.OFF;
    private SystemState spinningState = SystemState.OFF;
    private UpDownState launchState = UpDownState.DOWN;
    private SystemState gState = SystemState.OFF;
    private SystemState reducedDrivePowerState = SystemState.OFF;

    @Override
    public void molluscRunOpMode() {
        telemetry.setAutoClear(false);

        Configuration.handle(() -> {
            config = new Configuration("delta.txt");
            boolean fieldCentric = config.getBoolean("field centric on");
            maxDrive = config.getDouble("drive power max");
            maxStrafe = config.getDouble("strafe power max");
            maxTurn = config.getDouble("turn power max");
            maxDriveReduced = config.getDouble("drive power max reduced");
            maxStrafeReduced = config.getDouble("strafe power max reduced");
            maxTurnReduced = config.getDouble("turn power max reduced");

            squidWare = new SquidWare(config, false);

            fieldCentricDrive = new MecanumFieldCentric(squidWare.base, squidWare.imu);
            fieldCentricDrive.setDriveParams(maxDrive, maxStrafe, maxTurn, Math.PI);
            robotCentricDrive = new MecanumRobotCentric(squidWare.base) {
                @Override
                public void drive(double drive, double strafe, double turn) {
                    super.drive(-drive, -strafe, turn); // Because it starts backwards.
                }
            };
            robotCentricDrive.setDriveParams(maxDrive, maxStrafe, maxTurn);
            if (fieldCentric) {
                drivetrain = fieldCentricDrive;
                telemetry.speak("Driving field centric.");
            } else {
                drivetrain = robotCentricDrive;
                telemetry.speak("Driving robot centric.");
            }
        });

        telemetry.setAutoClear(true);

        waitForStart();

        squidWare.intake.presetUp();
        squidWare.scoring.anglerPosition(squidWare.scoring.anglerPos);

        while (opModeIsActive()) {

            // Quadratic controller sensitivity.
            drive  = Controls.quadratic(-gamepad1.left_stick_y);
            strafe = Controls.quadratic(gamepad1.left_stick_x);
            turn   = Controls.quadratic(gamepad1.right_stick_x);

            drivetrain.drive(drive, strafe, turn);

            if (Controls.singlePress("toggle drive", gamepad1.x)) {
                if (drivetrain instanceof MecanumFieldCentric) {
                    drivetrain = robotCentricDrive;
                } else {
                    drivetrain = fieldCentricDrive;
                }
            }
            if (Controls.singlePress("toggle reduce", gamepad1.b)) {
                if (reducedDrivePowerState == SystemState.OFF) {
                    reducedDrivePowerState = SystemState.ON;
                    fieldCentricDrive.setDriveParams(maxDriveReduced, maxStrafeReduced, maxTurnReduced, Math.PI);
                    robotCentricDrive.setDriveParams(maxDriveReduced, maxStrafeReduced, maxTurnReduced);
                } else {
                    reducedDrivePowerState = SystemState.OFF;
                    fieldCentricDrive.setDriveParams(maxDrive, maxStrafe, maxTurn, Math.PI);
                    robotCentricDrive.setDriveParams(maxDrive, maxStrafe, maxTurn);
                }
            }

            // Intake system.
            if (Controls.singlePress("intake servo", gamepad2.b)) {
                if (intakeState == UpDownState.UP) {
                    intakeState = UpDownState.DOWN;
                    squidWare.intake.presetDown();
                } else {
                    intakeState = UpDownState.UP;
                    squidWare.intake.presetUp();
                }
            } else if (Controls.spacedHold("intake servo up", gamepad2.dpad_up, 0.05)) {
                squidWare.intake.up();
            } else if (Controls.spacedHold("intake servo down", gamepad2.dpad_down, 0.05)) {
                squidWare.intake.down();
            }
            if (Controls.singlePress("intake motor", gamepad2.y)) {
                if (intakeMotorState == SystemState.OFF) {
                    intakeMotorState = SystemState.ON;
                    squidWare.intake.power(squidWare.intake.PRESET_POWER);
                } else {
                    intakeMotorState = SystemState.OFF;
                    squidWare.intake.power(0);
                }
            } else if (intakeMotorState == SystemState.OFF || Math.abs(gamepad2.right_stick_y) > 0) {
                intakeMotorState = SystemState.OFF;
                squidWare.intake.power(-gamepad2.right_stick_y);
            }

            // Internal transfer system.
            if (Controls.singlePress("conveyor motor out", gamepad2.dpad_left)) {
                if (conveyorMotorState == SystemState.OFF) {
                    conveyorMotorState = SystemState.ON;
                    squidWare.conveyor.power(-squidWare.conveyor.PRESET_POWER);
                } else {
                    conveyorMotorState = SystemState.OFF;
                    squidWare.conveyor.power(0);
                }
            } else if (Controls.singlePress("conveyor motor in", gamepad2.dpad_right)) {
                if (conveyorMotorState == SystemState.OFF) {
                    conveyorMotorState = SystemState.ON;
                    squidWare.conveyor.power(squidWare.conveyor.PRESET_POWER);
                } else {
                    conveyorMotorState = SystemState.OFF;
                    squidWare.conveyor.power(0);
                }
            }
            boolean pixelOverflowNow = false;
            if (
                conveyorMotorState == SystemState.OFF 
                || Math.abs(gamepad2.right_stick_x) > 0
                || (pixelOverflowNow = squidWare.conveyor.pixelOverflow())
            ) {
                if (pixelOverflowNow) {
                    intakeState = UpDownState.UP;
                    intakeMotorState = SystemState.OFF;
                    squidWare.intake.power(0);
                    squidWare.intake.presetUp();
                }
                conveyorMotorState = SystemState.OFF;
                squidWare.conveyor.resetState();
                squidWare.conveyor.power(gamepad2.right_stick_x);
            }

            // Scoring system.
            // Spinning servos.
            if (Controls.singlePress("spinning", gamepad2.x)) {
                if (spinningState == SystemState.ON) {
                    spinningState = SystemState.OFF;
                    squidWare.scoring.spinningOff();
                } else {
                    spinningState = SystemState.ON;
                    squidWare.scoring.spinningOn();
                }
            }
            if (Controls.singlePress("spinning panic", gamepad2.left_bumper)) {
                spinningState = SystemState.ON;
                squidWare.scoring.spinningPanic();
            }
            // Linear slide.
            squidWare.scoring.slidePower(-gamepad2.left_stick_y);
            // Angler.
            if (Controls.spacedHold("angler servo up", gamepad2.right_trigger > 0, 0.01)) {
                squidWare.scoring.anglerUp();
            } else if (Controls.spacedHold("angler servo down", gamepad2.left_trigger > 0, 0.01)) {
                squidWare.scoring.anglerDown();
            }
            // Release.
            if (gamepad1.left_bumper) {
                squidWare.scoring.releaseReset();
            } else if (gamepad1.right_bumper) {
                squidWare.scoring.releasePixel();
            }
            // Launch.
            if (Controls.singlePress("launcher", gamepad1.y)) {
                if (launchState == UpDownState.DOWN) {
                    launchState = UpDownState.UP;
                    squidWare.launcher.fire();
                } else {
                    launchState = UpDownState.DOWN;
                    squidWare.launcher.stop();
                }
            }

            // Lift.
            if (gamepad1.dpad_up) {
                squidWare.lift.power(squidWare.lift.POWER);
            } else if (gamepad1.dpad_down) {
                squidWare.lift.power(-squidWare.lift.POWER);
            } else {
                squidWare.lift.power(0);
            }

            // Combo.
            if (Controls.singlePress("g state", gamepad2.a)) {
                if (gState == SystemState.OFF) {
                    gState = SystemState.ON;
                    intakeState = UpDownState.DOWN;
                    intakeMotorState = SystemState.ON;
                    conveyorMotorState = SystemState.ON;
                    squidWare.intake.presetDown();
                    squidWare.intake.power(squidWare.intake.PRESET_POWER);
                    squidWare.conveyor.power(squidWare.conveyor.PRESET_POWER);
                } else {
                    gState = SystemState.OFF;
                    intakeState = UpDownState.UP;
                    intakeMotorState = SystemState.OFF;
                    conveyorMotorState = SystemState.OFF;
                    squidWare.intake.presetUp();
                    squidWare.intake.power(0);
                    squidWare.conveyor.power(0);
                }
            }
        }
    }

    private enum UpDownState {
        UP, DOWN
    }

    private enum SystemState {
        ON, OFF
    }
}
```
</details>
