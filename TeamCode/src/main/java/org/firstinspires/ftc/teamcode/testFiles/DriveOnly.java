/**
 * @author Asher Boymel
 * Drive only Op Mode
 */

package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("unused")
@TeleOp(name="Drive Only", group="Linear OpMode")
public class DriveOnly extends LinearOpMode {

    // Robot hardware variables
    Orientation angles;
    DcMotorEx leftFront, leftBack, rightFront, rightBack; // Drivetrain motors

    // Control Variables
    double percent; // Power percentage for driving

    @Override
    public void runOpMode() {

        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Configure motor directions (ensuring forward movement aligns with joystick input)
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        // Declare control variables
        percent = 65; // Default power percentage


        while (opModeIsActive()) {

            // Retrieve joystick inputs for movement
            double strafe = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            // Emergency stop triggered by back button
            if (gamepad1.back || gamepad2.back) stopAll();

            // Gamepad2 can take over driving if left bumper is pressed
            if (gamepad2.left_bumper) {
                strafe = gamepad2.left_stick_x;
                drive = -gamepad2.left_stick_y;
                rotate = gamepad2.right_stick_x;
            }

            // Adjust driving power percentage with d-pad
            if (percent < 65 && gamepad1.dpadUpWasPressed()) percent += 5;
            if (percent > 0 && gamepad1.dpadDownWasPressed()) percent -= 5;
            if (gamepad1.a) {
                percent = 20; // Low-speed mode
                gamepad1.rumble(0, 1, 2000);
            }
            /*
            if (gamepad1.y) {
                percent = 100; // Full speed
                gamepad1.rumble(1, 0, 2000);
            }
            */
            if (gamepad1.b) percent = 65; // Default speed


            //gamepad1.rumble(gamepad1.left_trigger, gamepad1.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);

            // Execute driving function
            drive(drive, strafe, rotate, percent);


        } // End while loop
    } // End runOpMode

    /**
     * Stops all robot movement.
     * Sets power to zero and stops the arm.
     */
    public void stopAll() {
        percent = 0;
    }

    /**
     * Controls the robot drive system using mecanum wheel calculations.
     *
     * @param drive   Forward/backward movement
     * @param strafe  Left/right movement
     * @param rotate  Rotational movement
     * @param percent Power percentage to scale movement
     */
    public void drive(double drive, double strafe, double rotate, double percent) {
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = -drive + strafe + rotate;
        double backLeftPower = -drive + strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize the values so no value exceeds 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        double scale = percent / 100;

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFront.setPower(scale * frontLeftPower);
        rightFront.setPower(scale * frontRightPower);
        leftBack.setPower(scale * backLeftPower);
        rightBack.setPower(scale * backRightPower);

        updateTelemetry(false, false);
    }

    /**
     * Updates all telemetry data
     *
     * @param fieldCentric boolean for field centric mode
     * @param manual boolean for manual control of the arm
     */
    public void updateTelemetry(boolean fieldCentric, boolean manual) {
        telemetry.addLine("Robot Data");

        telemetry.addLine("\nDrive Data");
        telemetry.addData("Percent Power", percent);
        telemetry.addData("Left Front Power", leftFront.getPower());
        telemetry.addData("Right Front Power", rightFront.getPower());
        telemetry.addData("Left Back Power", leftBack.getPower());
        telemetry.addData("Right Back Power", rightBack.getPower());
        telemetry.update();
    }
}
