---
title: "Power Play 2022 - 2023"
---

[< Back](../index.md) | [< General Info](./index.md)

# Power Play
> Or however else it's titled

| <image src="../images/PowerPlay/PowerPlayImg.png" style="max-width: 100%; border: none; box-shadow: none;" /> | <image src="../images/PowerPlay/Robot.png" style="max-width: 100%; border: none; box-shadow: none;" /> | <image src="../images/PowerPlay/Spot.png" style="max-width: 100%; border: none; box-shadow: none;" />
| :---: | :---: | :---: |
| Final robot. | Robot on field. | Spot. |

This season's robot name is <span style="font-style: italic;">Octo</span>.  
Development was continuously running during the later half of the season, so some decisions may seem odd.  
League Tournament: Winning Alliance Captain  
Super Qualifiers: 2nd Place Think Award  
State: Rank #5 (did not advance)

> Note: We bumped up the Android API level to 24 in the Gradle common build file.

___

## Non-OpMode Classes
<details><summary>Action</summary>

```java
package org.firstinspires.ftc.teamcode.octo;

public interface Action {
    int execute(int [] arguments);
}
```
</details>
<details><summary>Instruction</summary>

```java
package org.firstinspires.ftc.teamcode.octo;

public class Instruction {
    public String action;
    public int [] arguments;
    public int    line;
    public Instruction(String a, int [] args, int l) {
        action = a;
        arguments = args;
        line = l;
    }
}
```
</details>
<details><summary>OctoConfig</summary>

```java
/*
OctoConfig

This not an Autonomous nor TeleOp.
Utilities class for obtaining configuration values, usually prior to game start.
Current Status:
    Supports configuration for double and boolean values.
    TUI interface in driver station telemetry.
    Obtains battery power percentage (not level).

    !!! Interpreter for pseudo-code like script used to control autonomous movements. !!!

Potential Additions:
    Format text as HTML for fancy formatting.
    Complicated UI to allow for modifying past inputs.
    Input from second controller? Customized controls?

*/

package org.firstinspires.ftc.teamcode.octo;

import java.util.stream.Collectors;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;

import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.IOException;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// vII-XXI

public abstract class OctoConfig extends LinearOpMode {

    // Configuration

    // public static final long BOOL_DELAY = 100;
    // public static final long DOUBLE_DELAY = 250;
    // public static final long FULL_CHARGE_VOLTAGE = 14;
    public long BOOL_DELAY = 100;
    public long DOUBLE_DELAY = 250;
    public long FULL_CHARGE_VOLTAGE = 14;

    public boolean bool(boolean defaultValue, String caption, String label0, String label1) {
        boolean autoClear = telemetry.isAutoClear();
        if (autoClear) telemetry.setAutoClear(false); // Can't clear items now.
        boolean b = defaultValue; // Not strictly necessary, but good for being explicit.
        Telemetry.Item telItem = telemetry.addData(caption, b ? label1 : label0);
        while (!gamepad1.a && !isStopRequested()) {
            telItem.setValue(b ? label1 : label0);
            telemetry.update();
            if (gamepad1.dpad_up) b = false;
            else if (gamepad1.dpad_down) b = true;
            sleep(BOOL_DELAY);
        }
        while (gamepad1.a); // In case they hold A for too long.
        telItem.setCaption(caption.toUpperCase());
        telemetry.update();
        if (autoClear) telemetry.setAutoClear(true); // Reset to original state.
        return b;
    }
    public double float64(double defaultValue, double var, int precision, String caption) {
        boolean autoClear = telemetry.isAutoClear();
        if (autoClear) telemetry.setAutoClear(false);
        double num = defaultValue;
        Telemetry.Item telItem = telemetry.addData(caption, (long) (num * Math.pow(10, precision)) / Math.pow(10, precision));
        while (!gamepad1.a && !isStopRequested()) {
            telItem.setValue((long) (num * Math.pow(10, precision)) / Math.pow(10, precision));
            telemetry.update();
            if (gamepad1.dpad_up) num += var;
            else if (gamepad1.dpad_down) num -= var;
            sleep(DOUBLE_DELAY);
        }
        while (gamepad1.a);
        telItem.setCaption(caption.toUpperCase());
        telemetry.update();
        if (autoClear) telemetry.setAutoClear(true);
        return num;
    }
    public double batteryPower() { // Returns infinity upon error. This is not a level, but the power from 0 to 1.
        double p = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) p = Math.min(p, voltage);
        }
        return (p == Double.POSITIVE_INFINITY ? p : p / FULL_CHARGE_VOLTAGE);
    }
    public void success() {
        telemetry.log().add("CONFIGURATION COMPLETE");
        telemetry.update();
        telemetry.speak("Configuration Complete");
    }

    // Script Parser and Executor / Interpreter
    /*
    Script:
        Each line contains exactly one command and its parameters.
        The command consists of a name without spaces.
        It is then followed by a space and its arguments (space separated).
            "Command 0 9 8 7 6"
        The arguments must be integers, for now.
        Lines starting with "//" are comments, and will be ignored.
        An instruction is a term describing a command followed by its arguments.
    Parser:
        Parsing of the entire script is done prior to execution.
        Leading and trailing whitespace for each line is removed.
        The parser will notify of an error if:
            Extra spaces are found within a command.
            The number of arguments do not match the parameter count determined by command registration.
            The command is not registered (see below).
            An argument is invalid.
    Run:
        Call to parse the script.
        Returns false upon parse failure or if runState is set to false by other Java, and true upon success.
    
    Registering Commands (advanced):
        Any commands used by the script must first be registered in the HashMap `actions`.
            This includes if-else logic and loops.
        A command is registered by adding its name and parameter count, separated by a '-', to the HashMap.
            Along with a lambda expression that expects an integer array dictating what it does.
            The lambda must return an integer exit code, which may be used by other registered commands. This value can be anything.
            "
            actions.put("Command-2", (int [] args) -> {
                telemetry.log().add("Executing Command " + args[0] + ' ' + args[1]);
                return 0;
            });
            "
    Misc:
        A function called `jumpTo` which accepts a single integer will move script execution to that instruction.
            All subsequent executions will be from that instruction.
            Use this function with care, as it can easily create infinite loops and go out of indexing bounds.
            Note: Instructions are indexed from zero.
        Note: Disables telemetry auto clear. Does not disable upon exit.
        Note: If you change the Instruction ArrayList you must update the instructionCount variable!
    */

    public HashMap <String, Action> actions = new HashMap <> ();
    public ArrayList <Instruction> instructions = null;
    public int instructionCount = -1, instructionNum = 0, returnCode = 0;
    public boolean runState = true;
    public boolean run(String script) {
        telemetry.setAutoClear(false);
        instructions = parse(script);
        if (instructions == null) return false;
        instructionCount = instructions.size();
        for (; runState && instructionNum < instructionCount && instructionNum > -1; ++instructionNum) {
            Instruction i = instructions.get(instructionNum);
            telemetry.log().add(i.line + ": " + 
                                i.action.substring(0, i.action.length() - 2) + 
                                ' ' + Arrays.toString(i.arguments));
            returnCode = actions.get(i.action).execute(i.arguments);
            telemetry.update();
        }
        return runState;
    }
    public ArrayList <Instruction> parse(String script) {
        ArrayList <Instruction> instructions = new ArrayList <> ();
        String [] lines = script.split("\n");
        int lineNum = 0;
        for (String line : lines) {
            ++lineNum;
            line = line.trim();
            if (line.isEmpty() || line.startsWith("//")) continue;
            String [] v = line.split(" ");
            for (String i : v) if (i.isEmpty()) {
                error(lineNum, "Extra spaces detected.");
                return null;
            }
            v[0] += "-" + (v.length - 1);
            if (!actions.containsKey(v[0])) {
                error(lineNum, "Unidentified instruction name / not declared:" + v[0]);
                return null;
            }
            int [] arguments = new int [v.length - 1];
            for (int i = 1; i < v.length; ++i) {
                int arg;
                try {
                    arg = Integer.parseInt(v[i]);
                }
                catch (Exception e) {
                    error(lineNum, "Invalid argument passed as parameter. Must be an integer.");
                    return null;
                }
                arguments[i - 1] = arg;
            }
            instructions.add(new Instruction (v[0], arguments, lineNum));
        }
        return instructions;
    }
    public void jumpTo(int lineNum) {
        instructionNum = lineNum - 2;
    }
    public void error(int lineNum, String msg) {
        telemetry.log().add("Script error on line: " + lineNum + " --> " + msg);
        telemetry.update();
    }

    // For loading script stored in src/main/assets.
    public String loadScript(String path) {
        
        // Reference: https://stackoverflow.com/questions/309424/how-do-i-read-convert-an-inputstream-into-a-string-in-java
        // Note: The app's context is required to get assets. Conveniently, the SDK provides a medium for this through the hardwareMap.

        try {
            return new BufferedReader(
                new InputStreamReader(hardwareMap.appContext.getAssets().open(path))).lines().collect(Collectors.joining("\n")
            );
        }
        catch (IOException e) {
            runState = false;
            error(0, "Could not open script file.");
            return "";
        }
    }
}
```
</details>
<details><summary>OctoWare</summary>

```java
/*
OctoWare

This not an Autonomous nor TeleOp.

Temporary hardware class to avoid boilerplate and repetition.

*/

package org.firstinspires.ftc.teamcode.octo;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

// vII-XVIII

public abstract class OctoWare extends OctoConfig {
    public DcMotor     frontLeft, frontRight, rearLeft, rearRight;
    public TouchSensor clawRest, magnet;
    public ColorSensor colorSensor;
    public DcMotorEx   slideUp;
    public Servo       claw;
    public IMU         imu;

    public void setHardware(boolean encoders) {
        // Connect Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft   = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight  = hardwareMap.get(DcMotor.class, "rearRight");
        slideUp    = hardwareMap.get(DcMotorEx.class, "slideUp");

        // Connect Servos
        claw = hardwareMap.get(Servo.class, "claw");

        // Connect Color Sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Connect IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Connect Sensors
        clawRest = hardwareMap.get(TouchSensor.class, "clawRest");
        magnet   = hardwareMap.get(TouchSensor.class, "magnet");

        // Set Motor Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        slideUp.setDirection(DcMotorEx.Direction.REVERSE);

        // Set Motors to Brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset Encoder Counts
        if (encoders) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Configure IMU
        // Copied from SDK example.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}
```
</details>

___

## Autonomous
<details><summary>AutoOctoGamma</summary>

```java
/*
AutoOctoGamma

Current Status: Uses a pseudo code like script for easy adjustments.
                Utilizes wheel encoders and an IMU for odometry.
                The slide uses BOTH encoders and magnetic limit switches.

*/

package org.firstinspires.ftc.teamcode.octo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="AutoOcto III-III", group="Octo")

public class AutoOctoGamma extends OctoWare {

    // Runtimes
    ElapsedTime runtime = new ElapsedTime ();
    ElapsedTime runtime2 = new ElapsedTime ();

    // Constants
    final int ACTION_PAUSE = 50; // Pause after driving or moving the claw (milliseconds).
    final int SLIDE_TIMEOUT = 3;
    final double DEFAULT_POWER = 0.5; // Driving
    final double CLAW_OPEN = 0.3, CLAW_CLOSE = 0.65;
    final double AUTO_SLIDE_POWER = 0.7;

    // Mutable Variables
    boolean direction = true; // false --> left, true --> right
    boolean searching = false;
    boolean rest, last, now = true;
    boolean debug = false;
    int curLevel = 0, toLevel = 0;
    int globalOffset = 0;
    double al = 0, as = 0, bl = 0, bs = 0;
    double akp = 0.9, aki = 0, akd = 0;
    double bkp = 0.005, bki = 0, bkd = 0;

    // For data interchange between commands.
    boolean ifState = false;
    int parkZone = 3;

    @Override
    public void runOpMode() {
        setHardware(true);
        setCommands();

        // Critical Warning
        if (!clawRest.isPressed()) {
            String warning = "Warning. The linear slide is not in ground position. Press A to continue.";
            telemetry.speak(warning);
            telemetry.log().add(warning);
            telemetry.update();
            while (!gamepad1.a) idle();
        }

        telemetry.addData("CONFIGURATION", "Use dpad up / down to adjust values. Hold A to confirm.");

        direction = bool(direction, "Field Side", "Left", "Right");
        globalOffset = (int)float64(globalOffset, 25, 0, "Global Encoder Offset");
        debug = bool(debug, "Debug Mode (turn off for competition)", "Off", "On");

        success();

        // Wait
        waitForStart();

        // Ensure yaw is zeroed.
        imu.resetYaw();

        boolean success = run(loadScript("AutoOctoScriptGamma.txt"));
        telemetry.log().add("Script completed with exit status: " + success);
        telemetry.update();
        while (debug && opModeIsActive()) idle();
    }
    public int averageCount() {
        return (frontLeft.getCurrentPosition() +
                frontRight.getCurrentPosition() +
                rearLeft.getCurrentPosition() +
                rearRight.getCurrentPosition()) / 4;
    }
    public double PIDMP(double error, double Kp, double Ki, double Kd, Double lastError, Double intSum/*, double threshold*/) {
        intSum += error * runtime.seconds();
        double deriv = (error - lastError) / runtime.seconds();
        lastError = error;
        runtime.reset();
        return error * Kp + intSum * Ki + deriv * Kd;
    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
    public void setCommands() {
        // Command Registration for Script
        actions.put("Drive-2", (int [] ticks) -> {
            bl = bs = 0.0;
            al = as = 0.0;
            runtime.reset();
            runtime2.reset();
            double target = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // int start = averageCount();
            while (opModeIsActive() && runtime2.milliseconds() < ticks[1]) {
                double drivePower = Math.min(DEFAULT_POWER, Math.max(-DEFAULT_POWER, PIDMP(ticks[0]/* + start*/ + globalOffset - averageCount(), bkp, bki, bkd, bl, bs)));
                double turnPower = PIDMP(angleWrap(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - target), akp, aki, akd, al, as);
                double max = Math.max(Math.abs(drivePower) + Math.abs(turnPower), 1);
                double fl = (drivePower + turnPower) / max;
                double fr = (drivePower - turnPower) / max;
                double rl = (drivePower + turnPower) / max;
                double rr = (drivePower - turnPower) / max;
                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                rearLeft.setPower(rl);
                rearRight.setPower(rr);
            }
            stopAll();

            actions.get("WaitSeconds-1").execute(new int [] {ACTION_PAUSE});

            return 0;
        });
        actions.put("Strafe-2", (int [] ticks) -> {
            bl = bs = 0.0;
            al = as = 0.0;
            runtime.reset();
            runtime2.reset();
            double target = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // int start = (frontLeft.getCurrentPosition() + rearRight.getCurrentPosition()) / 2;
            while (opModeIsActive() && runtime2.milliseconds() < ticks[1]) {
                double drivePower = PIDMP(ticks[0]/* + start*/ + globalOffset - (frontLeft.getCurrentPosition() + rearRight.getCurrentPosition()) / 2,
                                          bkp, bki, bkd, bl, bs);
                double turnPower = PIDMP(angleWrap(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - target), akp, aki, akd, al, as);
                double max = Math.max(Math.abs(drivePower) + Math.abs(turnPower), 1);
                double fl = (drivePower + turnPower) / max;
                double fr = (-drivePower - turnPower) / max;
                double rl = (-drivePower + turnPower) / max;
                double rr = (drivePower - turnPower) / max;
                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                rearLeft.setPower(rl);
                rearRight.setPower(rr);
            }
            stopAll();

            actions.get("WaitSeconds-1").execute(new int [] {ACTION_PAUSE});

            return 0;
        });
        actions.put("TurnTo-2", (int [] deg) -> {
            al = as = 0.0;
            runtime.reset();
            runtime2.reset();
            while (opModeIsActive() && runtime2.milliseconds() < deg[1]) {
                double power = PIDMP(angleWrap(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.toRadians(deg[0])),
                                     akp, aki, akd, al, as);
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                rearLeft.setPower(power);
                rearRight.setPower(-power);
            }
            stopAll();

            actions.get("WaitSeconds-1").execute(new int [] {ACTION_PAUSE});

            return 0;
        });
        actions.put("ReadSleeve-0", (int [] none) -> {
            // Calculate color reading percentages.
            double red = colorSensor.red(), green = colorSensor.green(), blue = colorSensor.blue();
            double total = red + green + blue;
            double pred = red / total, pgreen = green / total, pblue = blue / total; // Calculate percentage of each.

            // Determine zone to move to.
            if (pblue > pred && pblue > pgreen) parkZone = 1;
            else if (pred > pblue && pred > pgreen) parkZone = 2;
            else if (pgreen > pred && pgreen > pblue) parkZone = 3;

            telemetry.addData("Zone Read", parkZone);

            return 0;
        });
        actions.put("SlidePosition-1", (int [] pos) -> {
            ElapsedTime t = new ElapsedTime ();
            toLevel = pos[0];
            while (true) {
                // Sensors
                rest = clawRest.isPressed();
                last = now;
                now = magnet.isPressed() || rest;

                if (!searching && curLevel == toLevel) {
                    slideUp.setPower(0);
                    break;
                } else if (!searching && curLevel < toLevel) {
                    searching = true;
                    slideUp.setPower(AUTO_SLIDE_POWER);
                } else if (!searching && curLevel > toLevel) {
                    searching = true;
                    slideUp.setPower(-AUTO_SLIDE_POWER / Math.sqrt(curLevel)); // Slower if higher up.
                } else if (searching && curLevel < toLevel && !last && now) {
                    searching = false;
                    ++curLevel;
                } else if (searching && curLevel > toLevel && !last && now) {
                    searching = false;
                    --curLevel;
                }

                // Failsafes
                if (searching && curLevel > toLevel && rest) {
                    searching = false;
                    curLevel = toLevel = 0;
                }
                if (t.seconds() > SLIDE_TIMEOUT) {
                    slideUp.setPower(0);
                    actions.get("STOP-0").execute(new int [] {});
                    break;
                }
            }

            actions.get("WaitSeconds-1").execute(new int [] {ACTION_PAUSE});

            return 0;
        });
        actions.put("SlidePositionE-1", (int [] pos) -> {
            slideUp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideUp.setTargetPosition(slideUp.getCurrentPosition() + pos[0]);
            slideUp.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slideUp.setPower(AUTO_SLIDE_POWER);

            while (slideUp.isBusy() && !slideUp.isOverCurrent()) idle();

            slideUp.setPower(0);
            slideUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            actions.get("WaitSeconds-1").execute(new int [] {ACTION_PAUSE});

            return 0;
        });
        actions.put("LowerSlideNB-0", (int [] none) -> {
            Runnable asyncStuff = () -> {
                slideUp.setPower(-AUTO_SLIDE_POWER);
                while (!clawRest.isPressed()) idle();
                slideUp.setPower(0);
            };
            Thread runStuff = new Thread (asyncStuff);
            runStuff.start();

            return 0;
        });
        actions.put("ClawState-1", (int [] pos) -> {
            if (pos[0] == 0) claw.setPosition(CLAW_OPEN);
            else if (pos[0] == 1) claw.setPosition(CLAW_CLOSE);

            actions.get("WaitSeconds-1").execute(new int [] {ACTION_PAUSE});

            return 0;
        });
        actions.put("WaitSeconds-1", (int [] msec) -> {
            sleep((long)msec[0]);

            return 0;
        });
        // Expand if-else statement logic later.
        actions.put("If-1", (int [] v) -> {
            ifState = false;

            if (v[0] == returnCode) return returnCode;

            int nest = 1;
            for (instructionNum += 1; instructionNum < instructionCount; ++instructionNum) {
                String act = instructions.get(instructionNum).action;
                if (act.equals("If-1") || act.equals("Elseif-1") || act.equals("Else-0")) ++nest;
                else if (act.equals("End-0")) {
                    --nest;
                    if (nest == 0) break;
                }
            }

            return returnCode;
        });
        actions.put("End-0", (int [] none) -> {
            if (instructionNum == 0) {
                error(instructions.get(instructionNum).line, "Cannot be at start.");
                runState = false;
                return 0;
            }

            ifState = true;

            return returnCode;
        });
        actions.put("Else-0", (int [] none) -> {
            if (instructionNum == 0) {
                error(instructions.get(instructionNum).line, "Cannot be at start.");
                runState = false;
                return 0;
            } else if (!instructions.get(instructionNum - 1).action.equals("End-0")) {
                error(instructions.get(instructionNum - 1).line, "Requires previous action to be End.");
                runState = false;
                return 0;
            }

            if (!ifState) return returnCode;

            int nest = 1;
            for (instructionNum += 1; instructionNum < instructionCount; ++instructionNum) {
                String act = instructions.get(instructionNum).action;
                if (act.equals("If-1") || act.equals("Elseif-1") || act.equals("Else-0")) ++nest;
                else if (act.equals("End-0")) {
                    --nest;
                    if (nest == 0) break;
                }
            }

            return returnCode;
        });
        actions.put("Elseif-1", (int [] v) -> {
            int preInsNum = instructionNum;
            actions.get("Else-0").execute(new int [] {});
            if (preInsNum == instructionNum) actions.get("If-1").execute(v);

            return returnCode;
        });
        actions.put("GetZone-0", (int [] none) -> {
            return parkZone;
        });
        actions.put("GetSide-0", (int [] none) -> {
            return direction ? 1 : 0;
        });
        actions.put("SpeakVersion-2", (int [] version) -> {
            telemetry.speak("Version " + version[0] + '-' + version[1] + ".");

            return 0;
        });
        actions.put("STOP-0", (int [] none) -> {
            instructionNum = 1000; // Crude implementation.

            return 0;
        });
    }
}
```
</details>
<details><summary>AutoOctoScriptGamma [TXT]</summary>

```
SpeakVersion 3 3

ClawState 1
WaitSeconds 1000
GetSide
If 1
    // Start Right
    SlidePosition 3
    Drive 1023 1000
    ReadSleeve
    Drive 2647 1800
    Drive 2290 1000

    TurnTo 45 1000

    Drive 2684 1000

    WaitSeconds 500
    ClawState 0

    // Score
    Drive 2317 1000
    SlidePosition 1
    TurnTo -92 1500
    // Old turn to -94.
    Drive 3464 2000

    SlidePositionE -942
    ClawState 1
    WaitSeconds 1000
    SlidePositionE 2694

    Drive 2311 2000
    TurnTo 43 1000
    // Old turn to 38.
    Drive 2646 1000

    WaitSeconds 500
    ClawState 0
    WaitSeconds 500

    Drive 2301 1000
    TurnTo 0 1000

    // Drive 646 1753 2579 3895
    // Drive 87 2238 2083 4403

    // Park
    GetZone
    If 1
        // Blue
        Strafe 1006 1000
    End
    Elseif 2
        // Red
        Drive 2301 1000
    End
    Elseif 3
        // Green
       Strafe 3407 1000
    End
End

Elseif 0
    // Start Left
    SlidePosition 3
    Drive 1023 1000
    ReadSleeve
    Drive 2647 1800
    Drive 2290 1000

    TurnTo -45 1000

    Drive 2684 1000

    WaitSeconds 500
    ClawState 0

    // Score
    Drive 2317 1000
    SlidePosition 1
    TurnTo 92 1500
    // Old turn to -94.
    Drive 3464 2000

    SlidePositionE -942
    ClawState 1
    WaitSeconds 1000
    SlidePositionE 2694

    Drive 2311 2000
    TurnTo -43 1000
    // Old turn to 38.
    Drive 2646 1000

    WaitSeconds 500
    ClawState 0
    WaitSeconds 500

    Drive 2301 1000
    TurnTo 0 1000

    // Drive 646 1753 2579 3895
    // Drive 87 2238 2083 4403

    // Park
    GetZone
    If 1
        // Blue
        Strafe 1006 1000
    End
    Elseif 2
        // Red
        Drive 2301 1000
    End
    Elseif 3
        // Green
       Strafe 3407 1000
    End
End
```
</details>

___

## TeleOp
<details><summary>OctoBeta</summary>

```java
/*
OctoBeta

Current Status: Field-Centric Mecanum Drive with linear slide extender and claw servo.
                The linear slide has a driver enhancement that automatically moves it to levels using encoders.
                This also makes use of a magnetic limit switch beneath the claw to account for error
                    caused by variations in the spooling of the wire.
                Also has a failsafe for when motor strain is detected.

*/

package org.firstinspires.ftc.teamcode.octo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Octo II-VIII", group="Octo")

public class OctoBeta extends OctoWare {

    // Mutable Variables
    double drive, strafe, turn;
    double fl, fr, rl, rr, max;
    double extender;
    double heading, rotX, rotY; // For field-centric drive.
    int level = 0, lastLevel;
    boolean manual = false, rest = true, lastRest;

    // Constants
    final double TURN_SPEED_MAX = 0.9;
    final double CLAW_OPEN = 0.3, CLAW_CLOSE = 0.65;
    final double AUTO_SLIDE_POWER = 1;
    final int [] SLIDE_ENCODER_LEVELS = new int [] {0, 1223, 2120, 3033};

    @Override
    public void runOpMode() {
        setHardware(false);
        waitForStart();

        slideUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            // Controller with Improved Sensitivity
            drive     = -gamepad1.left_stick_y;
            strafe    = gamepad1.left_stick_x;
            turn      = gamepad1.right_stick_x * TURN_SPEED_MAX;
            drive    *= Math.abs(drive);
            strafe   *= Math.abs(strafe);
            turn     *= Math.abs(turn);
            extender  = -gamepad2.left_stick_y;
            lastRest  = rest;
            rest      = clawRest.isPressed();
            heading   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            lastLevel = level;
            if      (gamepad2.a) level = 0;
            else if (gamepad2.b) level = 1;
            else if (gamepad2.y) level = 2;
            else if (gamepad2.x) level = 3;
            if (gamepad2.a || gamepad2.b || gamepad2.y || gamepad2.x) manual = false;

            // Calculations based on GM0.
            rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
            rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
            // Normalize
            max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            fl = (rotY + rotX + turn) / max;
            fr = (rotY - rotX - turn) / max;
            rl = (rotY - rotX + turn) / max;
            rr = (rotY + rotX - turn) / max;

            // Act
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            rearLeft.setPower(rl);
            rearRight.setPower(rr);

            if (manual || extender != 0) {
                slideUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                manual = true;
                if (extender > 0) slideUp.setPower(extender);
                else if (extender < 0 && !rest) slideUp.setPower(extender);
                else slideUp.setPower(0);
            } else {
                if (level != lastLevel) {
                    // Stop if currently running.
                    slideUp.setPower(0);
                    slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    slideUp.setTargetPosition(SLIDE_ENCODER_LEVELS[level]);
                    slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideUp.setPower(AUTO_SLIDE_POWER);
                }
                if (!slideUp.isBusy()) {
                    // Stop if currently running.
                    slideUp.setPower(0);
                    slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            if (rest && !lastRest) slideUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Failsafe
            if (slideUp.isOverCurrent()) manual = true;

            if (gamepad2.left_bumper) claw.setPosition(CLAW_CLOSE);
            else if (gamepad2.right_bumper) claw.setPosition(CLAW_OPEN);

            // Telemetry
            telemetry.addData("Manual", manual);
            telemetry.addData("Current Level", level);
            telemetry.addData("Slide", slideUp.getCurrentPosition());
            telemetry.update();
        }
    }
}
```
</details>
