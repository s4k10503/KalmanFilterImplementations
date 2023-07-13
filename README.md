# KalmanSharp

This repository contains a simple implementation of a Kalman filter in C#, using the MathNet.Numerics library.

## Dependencies

This project uses the following libraries:

- [MathNet.Numerics](https://github.com/mathnet/mathnet-numerics) (MIT License)

## Usage

The primary class in this project is KalmanFilter. You can create an instance of the class and then use it as follows:

```csharp
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Distributions;

// Initialize model parameters
Matrix<float> F = ...; // State transition matrix
Matrix<float> H = ...; // Observation matrix
Matrix<float> x = ...; // Initial state estimate
Matrix<float> P = ...; // Initial error covariance estimate
Matrix<float> Q = ...; // Estimated error in process
Matrix<float> R = ...; // Estimated error in measurements

// Create an instance of the KalmanFilter class
var kf = new KalmanFilter(F, H, x, P, Q, R);

// Control vector
Matrix<float> u = ...;

// Perform the prediction step
kf.Predict(u);

// Measurement vector
Matrix<float> z = ...;

// Perform the update step
kf.Update(z);

// Get the current state estimate
var currentState = kf.GetCurrentState();
```

Please replace the ... with appropriate data.
Remember to add using MathNet.Numerics.LinearAlgebra; and using MathNet.Numerics.Distributions; at the beginning of your file.