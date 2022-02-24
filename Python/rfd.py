import serial
from datetime import datetime
import csv

class MotorRx0:
  def __init__(self) -> None:
    self.battVoltage        = 0
    self.battCurrent        = 0
    self.battCurrentDir     = 0
    self.motorCurrentPkAvg  = 0
    self.FETtemp            = 0
    self.motorRPM           = 0
    self.PWMDuty            = 0
    self.LeadAngle          = 0

  def from_bytes(self, buff):
    self.battVoltage        = ((buff[1] & 3) << 8) | (buff[0])
    self.battCurrent        = ((buff[2] & 7) << 6) | (buff[1] >> 2)
    self.battCurrentDir     = ((buff[2] & 8))
    self.motorCurrentPkAvg  = ((buff[3] & 0x3F) << 4) | (buff[2] >> 4)
    self.FETtemp            = ((buff[4] & 7) << 2) | (buff[3] >> 6)
    self.motorRPM           = ((buff[5] & 0x7F) << 5) | (buff[4] >> 3)
    self.PWMDuty            = ((buff[7] & 1) << 9) | (buff[6] << 1) | (buff[5] >> 7)
    self.LeadAngle          = (buff[7] >> 1)

  def __str__(self) -> str:
    return "Battery Voltage: "      + str(self.battVoltage)       + \
        "\nBattery Current: "       + str(self.battCurrent)       + \
        "\nBattery Current Dir: "   + str(self.battCurrentDir)    + \
        "\nMotor Current Pk Avg: "  + str(self.motorCurrentPkAvg) + \
        "\nFET Temperature: "       + str(self.FETtemp)           + \
        "\nMotor RPM: "             + str(self.motorRPM)          + \
        "\nPWM Duty: "              + str(self.PWMDuty)           + \
        "\nLead Angle: "            + str(self.LeadAngle)

class DataIngestor:
  def __init__(self) -> None:
    self.StartChar = 0xFF
    self.EndChar = 0x3F
    self.EscapeChar = 0x2F
    self.data = []
    self.Escaped = False
    self.InProgress = False
  def HandleTransmission(self):
    # get number of messages
    numMessages = self.data[0]
    currentIndex = 1

    for message in range(0, numMessages):
      # get start address
      address = self.data[currentIndex]
      currentIndex += 1
      # get the id
      id = self.data[currentIndex]
      currentIndex += 1
      # get data length
      dataLen = self.data[currentIndex]
      currentIndex += 1

      # fill data in buffer
      dataBuffer = []
      for count in range(0, dataLen):
        dataBuffer.append(self.data[currentIndex])
        currentIndex += 1

      # Check to see if addresses have been found
      rx_time = datetime.utcnow().strftime("%H:%M:%S.%f")[:-3]
      print("Packet received @ ", rx_time)
      if address == 0x04:
        tmp = MotorRx0()
        tmp.from_bytes(dataBuffer)
        print(tmp)

  def DataLink(self, byteIn):
    if not self.Escaped:
      if byteIn == self.EscapeChar:
        self.Escaped = True
        return

      elif (byteIn == self.StartChar):
        if self.InProgress:
          print("Error: There were multiple start conditions")
          return

        self.data.clear()
        self.InProgress = True
        return

      elif byteIn == self.EndChar:
        if not self.InProgress:
          print("Error: There was an end condition without a start")
          return

        # handle transmission
        self.InProgress = False
        self.HandleTransmission()
        return
    else:
      self.Escaped = False

    if not self.InProgress:
      print("Error: There must be a start condition before a transmission can occour")
      return
    self.data.append(byteIn)


def main():
  # Log file setup
  fields = ['Time', 'Battery Voltage', 'Battery Current',
            'Battery Current Direction', 'Motor RPM']
  filename = "TelemLog_" + datetime.utcnow().strftime("%Y%m%d%H%M%S") + '.csv'
  # log = open(filename, 'w', newline='')
  # csvwriter = csv.writer(log)
  # csvwriter.writerow(fields)
  # Serial Setup
  ser = serial.Serial()
  ser.baudrate = 57600
  ser.port = 'COM6'
  ser.open()
  print(ser.name)         # check which port was really used
  ser.write(b'ATO')       # exit AT Commands
  ingestor = DataIngestor()
  try:
    while(1):
      bytes = ser.read(1)
      ingestor.DataLink(ord(bytes))
      
  except KeyboardInterrupt:
    print('Exiting')
  finally:
    ser.close()             # close port
    # log.close()


if __name__ == "__main__":
  main()
