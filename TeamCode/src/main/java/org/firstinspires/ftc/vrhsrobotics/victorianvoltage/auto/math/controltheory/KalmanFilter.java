package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    //
    private SimpleMatrix x_k_1, P_k_1;
    //system paramters
    private SimpleMatrix F, G, R, Q, H, I, B;

    public KalmanFilter(SimpleMatrix f, SimpleMatrix g, SimpleMatrix r, SimpleMatrix q, SimpleMatrix h) {
        F = f;
        G = g;
        R = r;
        Q = q;
        H = h;
        I = SimpleMatrix.identity(F.numRows());
    }

    public void setInitalPostion(SimpleMatrix x_0, SimpleMatrix P_0, SimpleMatrix B) {
        x_k_1 = x_0;
        P_k_1 = P_0;
        this.B = B;
    }


    /**
     * @param z_k - observation
     * @return
     */
    public SimpleMatrix update(SimpleMatrix z_k,SimpleMatrix U_U) {
        SimpleMatrix x_k_est = F.mult(x_k_1).plus((B.mult(U_U)));
        SimpleMatrix P_k_est = (F.mult(P_k_1).mult((F.transpose()))).plus(Q);

        SimpleMatrix y_k = z_k.minus((H.mult(x_k_est)));
        SimpleMatrix S_k = (H.mult(P_k_est).mult((H.transpose()))).plus(R);
        SimpleMatrix K = P_k_est.mult(((H.transpose()).mult((S_k.invert()))));
        SimpleMatrix x_k = x_k_est.plus((K.mult(y_k)));
        SimpleMatrix P_k = (I.minus((K.mult(H)))).mult(P_k_est);

        this.x_k_1 = x_k;
        this.P_k_1 = P_k;

        return x_k;
    }

    public double getXPosition() {
        return x_k_1.get(0, 0);
    }

    public double getYPosition() {
        return x_k_1.get(1, 0);
    }


}
