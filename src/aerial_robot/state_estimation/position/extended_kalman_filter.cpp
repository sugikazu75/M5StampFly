#include <aerial_robot/state_estimation/position/extended_kalman_filter.hpp>

void ExtendedKalmanFilter::initialize()
{
    // Initialize state and covariance matrices
    estimate_state_ = BLA::Zeros<9, 1>();
    predict_state_ = BLA::Zeros<9, 1>();
    control_input_model_ = BLA::Zeros<9, 1>();

    state_transition_model_ = BLA::Zeros<9, 9>();
    state_transition_model_(0, 0) = 1.0;
    state_transition_model_(1, 1) = 1.0;
    state_transition_model_(2, 2) = 1.0;
    state_transition_model_(3, 3) = 1.0;
    state_transition_model_(4, 4) = 1.0;
    state_transition_model_(5, 5) = 1.0;
    state_transition_model_(6, 6) = 1.0 + beta * step;
    state_transition_model_(7, 7) = 1.0 + beta * step;
    state_transition_model_(8, 8) = 1.0 + beta * step;
    state_transition_model_(0, 6) = -step;
    state_transition_model_(1, 7) = -step;
    state_transition_model_(2, 8) = -step;

    for(int i = 0; i < 9; i++)
      for(int j = 0; j < 9; j++)
        state_transition_model_transpose_(i, j) = state_transition_model_(j, i);

    predict_P_ = (float)100.0 * BLA::Eye<9, 9>();
    correction_P_ = predict_P_;

    G_ = BLA::Zeros<9, 6>();
    G_(0, 0) = step;
    G_(1, 1) = step;
    G_(2, 2) = step;
    G_(6, 3) = step;
    G_(7, 4) = step;
    G_(8, 5) = step;

    Q_ = (float)0.01 * BLA::Eye<6, 6>();

    R_ = (float)0.01 * BLA::Eye<3, 3>();

    H_ = BLA::Zeros<3, 9>();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    H_transpose_ = BLA::Zeros<9, 3>();
    H_transpose_(0, 0) = 1.0;
    H_transpose_(1, 1) = 1.0;

    rotation_matrix_ = BLA::Eye<3, 3>();
}

void ExtendedKalmanFilter::update(float x_vel, float y_vel, float altitude, float acc_x, float acc_y, float acc_z, float omega_x, float omega_y, float omega_z)
{
  loop_count_++;
    // get the current state from the attitude estimator
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            state_transition_model_(i + 3, j) = step * rotation_matrix_(i, j);
            state_transition_model_transpose_(j, i + 3) = step * rotation_matrix_(i, j);
            G_(i + 3, j) = step * step * rotation_matrix_(i, j);
            G_transpose_(j, i + 3) = step * step * rotation_matrix_(i, j);
        }
    }

    for(int i = 0; i < 9; i++)
      for(int j = 0; j < 6; j++)
        G_transpose_(j, i) = G_(j, i);

    BLA::Matrix<3, 3> rotation_matrix_transpose;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        rotation_matrix_transpose(i, j) = rotation_matrix_(j, i);

    BLA::Matrix<3, 1> gravity_body = rotation_matrix_transpose * BLA::Matrix<3, 1>(0.0, 0.0, 1.0);

    control_input_model_(0) = gravity_ * (acc_x - gravity_body(0)) * step;
    control_input_model_(1) = gravity_ * (acc_y - gravity_body(1)) * step;
    control_input_model_(2) = gravity_ * (acc_z - gravity_body(2)) * step;

    // observation_(0) = x_vel * altitude / 11.0;
    // observation_(1) = y_vel * altitude / 11.0;
    int n_pix = 30;
    float h = altitude / rotation_matrix_(2, 2);
    if(h < 0.2)
      h = 0.0;

    float tan21 = tan(21.0 * M_PI / 180.0);
    observation_(0) = (h * tan21) / (0.05 * n_pix) * x_vel - h * omega_y;
    observation_(1) = (h * tan21) / (0.05 * n_pix) * y_vel + h * omega_x;
    observation_(2) = altitude;

    if(loop_count_ % 10 == 0)
    {
      // USBSerial.print("control_input_model_: ");
      // USBSerial.print(control_input_model_);
      // USBSerial.print("\n");
      // USBSerial.printf("observation_: %f %f %f\n", observation_(0), observation_(1), observation_(2));
      // USBSerial.print("\n");
    }


    H_(2, 5) = 1.0 / rotation_matrix_(2, 2);
    H_transpose_(5, 2) = 1.0 / rotation_matrix_(2, 2);

    predict();
    correct();

    if(loop_count_ % 10 == 0)
    {
        loop_count_ = 0;
    }
}


void ExtendedKalmanFilter::predict()
{
    // Predict state
    predict_state_ = state_transition_model_ * estimate_state_ + control_input_model_;

    // Predict covariance
    predict_P_ = state_transition_model_ * correction_P_ * state_transition_model_transpose_ + G_ * Q_ * G_transpose_;
}

void ExtendedKalmanFilter::correct()
{
  // Correct step
  BLA::Matrix<3, 1> e = observation_ - H_ * predict_state_;

  // Kalman gain
  BLA::Matrix<3, 3> S = R_ + H_ * predict_P_ * H_transpose_;
  BLA::Matrix<3, 3> S_inverse = Inverse(S);
  BLA::Matrix<9, 3> K = predict_P_ * H_transpose_ * S_inverse;

  // Update state and covariance
  estimate_state_ = predict_state_ + K * e;
  correction_P_ = (BLA::Eye<9, 9>() - K * H_) * predict_P_;

  if(loop_count_ % 10 == 0)
  {
      // USBSerial.print("S: ");
      // USBSerial.print(S);
      // USBSerial.print("\n");
      // USBSerial.print("S_inverse: ");
      // USBSerial.print(S_inverse);
      // USBSerial.print("\n");
      // USBSerial.printf("estimate_state_:\n %f %f %f\n %f %f %f\n %f %f %f\n", estimate_state_(0), estimate_state_(1), estimate_state_(2), estimate_state_(3), estimate_state_(4), estimate_state_(5), estimate_state_(6), estimate_state_(7), estimate_state_(8));
  }
}
