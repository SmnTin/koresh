#ifndef PROJECT_KALMAN_H
#define PROJECT_KALMAN_H

class KalmanFilterSimple1D
{
public:
    double X0;// predicted state
    double P0; // predicted covariance

    double F; // factor of real value to previous real value
    double Q; // measurement noise
    double H; // factor of measured value to real value
    double R; // environment noise

    double State;
    double Covariance;

    KalmanFilterSimple1D(double q = 2, double r = 5, double f = 1, double h = 1)
    {
        Q = q;
        R = r;
        F = f;
        H = h;
    }

    void setState(double state, double covariance)
    {
        State = state;
        Covariance = covariance;
    }

    void correct(double data)
    {
        //time update - prediction
        X0 = F*State;
        P0 = F*Covariance*F + Q;

        //measurement update - correction
        double K = H*P0/(H*P0*H + R);
        State = X0 + K*(data - H*X0);
        Covariance = (1 - K*H)*P0;
    }
};

#endif //PROJECT_KALMAN_H
