#include <sensor/mag/mag.hpp>

void Magnetmeter::initialize()
{
  // TODO: enable to read calib data from flashmemory
  offset_ = {132, 76, 192};
  scaling_matrix_ = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  scaling_matrix_inv_ = Inverse(scaling_matrix_);

  USBSerial.print("mag offset vector: ");
  USBSerial.print(offset_);
  USBSerial.print("\n");
  USBSerial.print("mag scaling matrix: \n");
  USBSerial.print(scaling_matrix_inv_);
  USBSerial.print("\n");
}

void Magnetmeter::update()
{
  readMagData();
  sphereMapping();
}

void Magnetmeter::sphereMapping()
{
  mag_data_ = scaling_matrix_inv_ * (raw_mag_data_ - offset_);
}
