package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TesteRUNTOPOSITION extends LinearOpMode {

    private DcMotorEx baseShooter;

    private final int POSIÇÃO = 800;

    @Override
    public void runOpMode(){

        baseShooter = hardwareMap.get(DcMotorEx.class, "baseShooter");

        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a) {
                baseShooter.setTargetPosition(POSIÇÃO);
                baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                baseShooter.setPower(1.0);
            }

            if (gamepad1.b) {
                baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                baseShooter.setTargetPosition(0);
                baseShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                baseShooter.setPower(1.0);
            }
        }
    }
}
