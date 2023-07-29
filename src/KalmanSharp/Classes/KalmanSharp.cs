using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Distributions;

public class KalmanFilter
{
    private Matrix<float> F; // State transition matrix
    private Matrix<float> B; // Control matrix
    private Matrix<float> H; // Observation matrix
    private Matrix<float> x; // Current state estimate
    private Matrix<float> P; // Current error covariance estimate
    private Matrix<float> Q; // Estimated error in process
    private Matrix<float> R; // Estimated error in measurement

    // Constructor: set initial values and matrices
    public KalmanFilter(Matrix<float> F, Matrix<float> B, Matrix<float> H, Matrix<float> x0, Matrix<float> P0, Matrix<float> Q, Matrix<float> R)
    {
        this.F = F;
        this.B = B; 
        this.H = H;
        this.x = x0;
        this.P = P0;
        this.Q = Q;
        this.R = R;
    }

    // Predict step: update the state and error covariance
    public void Predict(Matrix<float> u)
    {
        x = F * x + B * u;
        P = F * P * F.Transpose() + Q;
    }

    // Update step: update the state and error covariance based on the observation
    public void Update(Matrix<float> z, float outlierThreshold = 0.01f)
    {
        Matrix<float> y = z - H * x;
        Matrix<float> S = H * P * H.Transpose() + R;

        // Calculate the Mahalanobis distance
        float D = (float)System.Math.Sqrt((y.Transpose() * S.Inverse() * y)[0, 0]);

        // Calculate the threshold based on the chi-squared distribution
        float threshold = (float)ChiSquared.InvCDF(y.RowCount, 1 - outlierThreshold);

        // If the Mahalanobis distance is greater than the threshold, consider the observation as an outlier and skip the update step
        if (D > threshold)
        {
            return;
        }

        Matrix<float> K = P * H.Transpose() * S.Inverse();
        x = x + K * y;
        P = (Matrix<float>.Build.DenseIdentity(P.RowCount) - K * H) * P;
    }

    // Get the current state estimate
    public Matrix<float> GetCurrentState()
    {
        return x;
    }
}
