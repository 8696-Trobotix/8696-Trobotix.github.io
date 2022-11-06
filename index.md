| <image src="Trobotix.png" alt="Trobotix icon." style="max-width: 100%; border: none; box-shadow: none;" /> | <image src="favicon.png" alt="Second icon." style="max-width: 100%; border: none; box-shadow: none;" /> |
| :---: | :---: |
| [**Main Website**](https://sites.google.com/view/trobotix/home) | [**GitHub**](https://github.com/8696-Trobotix) |
## Open Source Projects
- [tralloy](https://8696-trobotix.github.io/tralloy/) - Template for console-based web applications. [Repository](https://github.com/8696-Trobotix/tralloy).
## Seasons
<details><summary><strong>Freight Frenzy [2021 - 2022]</strong></summary>
<image src="images/FreightFrenzyImg.png" alt="Robot from 2022." style="max-width: 50%; border: none; box-shadow: none;" />

### TeleOp
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
### Autonomous
> *On vacation.*
</details>