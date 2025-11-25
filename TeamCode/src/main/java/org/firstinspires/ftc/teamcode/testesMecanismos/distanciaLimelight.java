package org.firstinspires.ftc.teamcode.testesMecanismos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Distancia Limelight 1")
public class distanciaLimelight extends OpMode {

    private Limelight3A limelight;

    // Ângulo fixo da câmera
    private static final double CAMERA_ANGLE_DEGREES = 25.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult llResultado = limelight.getLatestResult();

        if (llResultado != null && llResultado.isValid()) {

            double ta = llResultado.getTa();

            // Distância pela área (hipotenusa)
            double distanciaHip = DistanciaPorTA(ta);

            // Conversão para distância horizontal (cateto oposto)
            double distanciaReal = distanciaHip * Math.cos(Math.toRadians(CAMERA_ANGLE_DEGREES));

            telemetry.addData("TA", ta);
            telemetry.addData("Distância Hipotenusa (cm)", distanciaHip);
            telemetry.addData("Distância Real Horizontal (cm)", distanciaReal);
            telemetry.addData("TX", llResultado.getTx());
        }

        telemetry.update();
    }


    // ---------------------------------------------------------
    // DISTÂNCIA COM INTERPOLAÇÃO + EXTRAPOLAÇÃO
    // ---------------------------------------------------------
    public double DistanciaPorTA(double ta) {

        double[] taVals   = {6.6334, 2.6275, 1.3471, 0.8369};
        double[] distVals = {50,     100,    150,    200};

        // EXTRAPOLAÇÃO PERTO DOS 50–100 cm
        if (ta > taVals[0]) {
            double ta1 = taVals[0];
            double ta2 = taVals[1];
            double d1  = distVals[0];
            double d2  = distVals[1];

            return d1 + (d2 - d1) * ((ta - ta1) / (ta2 - ta1));
        }

        // EXTRAPOLAÇÃO ALÉM DOS 200 cm
        if (ta < taVals[3]) {
            double ta1 = taVals[2];
            double ta2 = taVals[3];
            double d1  = distVals[2];
            double d2  = distVals[3];

            return d1 + (d2 - d1) * ((ta - ta1) / (ta2 - ta1));
        }

        // INTERPOLAÇÃO NORMAL ENTRE OS PONTOS
        for (int i = 0; i < taVals.length - 1; i++) {

            double ta1 = taVals[i];
            double ta2 = taVals[i + 1];

            boolean dentroIntervalo =
                    (ta <= ta1 && ta >= ta2) ||
                            (ta >= ta1 && ta <= ta2);

            if (dentroIntervalo) {

                double d1 = distVals[i];
                double d2 = distVals[i + 1];

                return d1 + (d2 - d1) * ((ta - ta1) / (ta2 - ta1));
            }
        }

        return -1;
    }
}