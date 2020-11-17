/*

bool Adafruit_AS7341::getIsDataReady() {
  Adafruit_BusIO_Register status2_reg    = Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS2);
  Adafruit_BusIO_RegisterBits avalid_bit = Adafruit_BusIO_RegisterBits(&status2_reg, 1, 6);

  return avalid_bit.read();
}

bool Adafruit_AS7341::readAllChannels(uint16_t *readings_buffer) 
{

  setSMUXLowChannels(true);        // Configure SMUX to read low channels
  enableSpectralMeasurement(true); // Start integration
  delayForData(0);                 // I'll wait for you for all time





  Adafruit_BusIO_Register channel_data_reg = Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);
  bool low_success = channel_data_reg.read((uint8_t *)readings_buffer, 12);




  setSMUXLowChannels(false);       // Configure SMUX to read high channels
  enableSpectralMeasurement(true); // Start integration
  delayForData(0);                 // I'll wait for you for all time




  return low_success && channel_data_reg.read((uint8_t *)&readings_buffer[6], 12);
}










void Adafruit_AS7341::setSMUXLowChannels(bool f1_f4) 
{
  enableSpectralMeasurement(false);
  setSMUXCommand(AS7341_SMUX_CMD_WRITE);
  if (f1_f4) {
    setup_F1F4_Clear_NIR();
  } else {
    setup_F5F8_Clear_NIR();
  }
  enableSMUX();
}











bool Adafruit_AS7341::enableSpectralMeasurement(bool enable_measurement) 
{
  Adafruit_BusIO_Register enable_reg          = Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
  Adafruit_BusIO_RegisterBits spec_enable_bit = Adafruit_BusIO_RegisterBits(&enable_reg, 1, 1);
  return spec_enable_bit.write(enable_measurement);
}

bool Adafruit_AS7341::setSMUXCommand(as7341_smux_cmd_t command) 
{
  Adafruit_BusIO_Register cfg6_reg              = Adafruit_BusIO_Register(i2c_dev, AS7341_CFG6);
  Adafruit_BusIO_RegisterBits smux_command_bits = Adafruit_BusIO_RegisterBits(&cfg6_reg, 2, 3);
  return smux_command_bits.write(command);
}   


void Adafruit_AS7341::setup_F1F4_Clear_NIR() {
  // SMUX Config for F1,F2,F3,F4,NIR,Clear
  writeRegister(byte(0x00), byte(0x30)); // F3 left set to ADC2
  writeRegister(byte(0x01), byte(0x01)); // F1 left set to ADC0
  writeRegister(byte(0x02), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x03), byte(0x00)); // F8 left disabled
  writeRegister(byte(0x04), byte(0x00)); // F6 left disabled
  writeRegister(byte(0x05), byte(0x42)); // F4 left connected to ADC3/f2 left connected to ADC1
  writeRegister(byte(0x06), byte(0x00)); // F5 left disbled
  writeRegister(byte(0x07), byte(0x00)); // F7 left disbled
  writeRegister(byte(0x08), byte(0x50)); // CLEAR connected to ADC4
  writeRegister(byte(0x09), byte(0x00)); // F5 right disabled
  writeRegister(byte(0x0A), byte(0x00)); // F7 right disabled
  writeRegister(byte(0x0B), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x0C), byte(0x20)); // F2 right connected to ADC1
  writeRegister(byte(0x0D), byte(0x04)); // F4 right connected to ADC3
  writeRegister(byte(0x0E), byte(0x00)); // F6/F8 right disabled
  writeRegister(byte(0x0F), byte(0x30)); // F3 right connected to AD2
  writeRegister(byte(0x10), byte(0x01)); // F1 right connected to AD0
  writeRegister(byte(0x11), byte(0x50)); // CLEAR right connected to AD4
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x06)); // NIR connected to ADC5
}
void Adafruit_AS7341::setup_F5F8_Clear_NIR() {
  // SMUX Config for F5,F6,F7,F8,NIR,Clear
  writeRegister(byte(0x00), byte(0x00)); // F3 left disable
  writeRegister(byte(0x01), byte(0x00)); // F1 left disable
  writeRegister(byte(0x02), byte(0x00)); // reserved/disable
  writeRegister(byte(0x03), byte(0x40)); // F8 left connected to ADC3
  writeRegister(byte(0x04), byte(0x02)); // F6 left connected to ADC1
  writeRegister(byte(0x05), byte(0x00)); // F4/ F2 disabled
  writeRegister(byte(0x06), byte(0x10)); // F5 left connected to ADC0
  writeRegister(byte(0x07), byte(0x03)); // F7 left connected to ADC2
  writeRegister(byte(0x08), byte(0x50)); // CLEAR Connected to ADC4
  writeRegister(byte(0x09), byte(0x10)); // F5 right connected to ADC0
  writeRegister(byte(0x0A), byte(0x03)); // F7 right connected to ADC2
  writeRegister(byte(0x0B), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x0C), byte(0x00)); // F2 right disabled
  writeRegister(byte(0x0D), byte(0x00)); // F4 right disabled
  writeRegister(byte(0x0E), byte(0x24)); // F8 right connected to ADC2/ F6 right connected to ADC1
  writeRegister(byte(0x0F), byte(0x00)); // F3 right disabled
  writeRegister(byte(0x10), byte(0x00)); // F1 right disabled
  writeRegister(byte(0x11), byte(0x50)); // CLEAR right connected to AD4
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x06)); // NIR connected to ADC5
}

void Adafruit_AS7341::FDConfig() {
  // SMUX Config for Flicker- register (0x13)left set to ADC6 for flicker
  // detection
  writeRegister(byte(0x00), byte(0x00)); // disabled
  writeRegister(byte(0x01), byte(0x00)); // disabled
  writeRegister(byte(0x02), byte(0x00)); // reserved/disabled
  writeRegister(byte(0x03), byte(0x00)); // disabled
  writeRegister(byte(0x04), byte(0x00)); // disabled
  writeRegister(byte(0x05), byte(0x00)); // disabled
  writeRegister(byte(0x06), byte(0x00)); // disabled
  writeRegister(byte(0x07), byte(0x00)); // disabled
  writeRegister(byte(0x08), byte(0x00)); // disabled
  writeRegister(byte(0x09), byte(0x00)); // disabled
  writeRegister(byte(0x0A), byte(0x00)); // disabled
  writeRegister(byte(0x0B), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x0C), byte(0x00)); // disabled
  writeRegister(byte(0x0D), byte(0x00)); // disabled
  writeRegister(byte(0x0E), byte(0x00)); // disabled
  writeRegister(byte(0x0F), byte(0x00)); // disabled
  writeRegister(byte(0x10), byte(0x00)); // disabled
  writeRegister(byte(0x11), byte(0x00)); // disabled
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x60)); // Flicker connected to ADC5 to left of 0x13
}


bool Adafruit_AS7341::enableSMUX(void) {

  Adafruit_BusIO_Register enable_reg          = Adafruit_BusIO_Register(i2c_dev, SM);
  Adafruit_BusIO_RegisterBits smux_enable_bit = Adafruit_BusIO_RegisterBits(&enable_reg, 1, 4);
  bool success = smux_enable_bit.write(true);

  int timeOut = 1000; // Arbitrary value, but if it takes 1000 milliseconds then
                      // something is wrong
  int count = 0;
  while (smux_enable_bit.read() && count < timeOut) {
    delay(1);
    count++;
  }
  if (count >= timeOut)
    return false;
  else
    return success;
}










bool Adafruit_AS7341::getIsDataReady() {
  Adafruit_BusIO_Register status2_reg = Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS2);
  Adafruit_BusIO_RegisterBits avalid_bit = Adafruit_BusIO_RegisterBits(&status2_reg, 1, 6);

  return avalid_bit.read();
}



s
*/

