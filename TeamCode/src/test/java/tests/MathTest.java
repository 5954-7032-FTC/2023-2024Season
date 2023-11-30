//package org.firstinspires.ftc.teamcode;

public class MathTest {

    public static void main(String Args[]) {
        System.out.println("Testing to see which is faster \n Math.cos or Math.sqrt");

        double upperLimit = 20000*Math.PI;
        double thetainc = Math.PI/360;

        long current = System.currentTimeMillis();
        for (double theta=0; theta < upperLimit; theta+=thetainc) {
            double sine = Math.sin(theta);
            double cos = Math.cos(theta);
        }
        long elapsed = System.currentTimeMillis() - current;
        System.out.println("Elapsed time(cos):" + elapsed + "ms");

        current = System.currentTimeMillis();
        for (double theta=0; theta < upperLimit; theta+=thetainc) {
            double sine = Math.sin(theta);
            double cos = Math.sqrt(1-sine*sine);
        }
        elapsed = System.currentTimeMillis() - current;
        System.out.println("Elapsed time(Math.sqrt):" + elapsed + "ms");
    }
}
