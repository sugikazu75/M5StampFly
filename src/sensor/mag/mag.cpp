#include <sensor/mag/mag.hpp>

void Magnetmeter::initialize()
{
  // TODO: enable to read calib data from flashmemory
  offset_ = {376.80061819, 199.12291455, 483.36028459};

  scaling_matrix_ = {0.01003836, -0.00011478, 0.00073942, 0.0, 0.00943432, -0.00023855, 0.0, 0.0, 0.01126153};
  scaling_matrix_inv_ = Inverse(scaling_matrix_);

  USBSerial.print("mag offset vector: ");
  USBSerial.print(offset_);
  USBSerial.print("\n");
  USBSerial.print("mag scaling matrix: \n");
  USBSerial.print(scaling_matrix_);
  USBSerial.print("\n");
}

void Magnetmeter::update()
{
  readMagData();
  sphereMapping();
}

void Magnetmeter::sphereMapping()
{
  mag_data_ = scaling_matrix_ * (raw_mag_data_ - offset_);
}
