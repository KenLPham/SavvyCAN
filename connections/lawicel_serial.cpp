#include <QCanBusFrame>
#include <QDebug>
#include <QObject>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>
#include <QtNetwork>

#include "lawicel_serial.h"
#include "utility.h"

#define OK '\r'
#define ERROR '\a'

#define TRANSMIT_CMD 't'
#define TRANSMIT_EX_CMD 'T'
#define FD_BRS_CMD 'b'
#define FD_CMD 'd'
#define FD_BRS_EX_CMD 'B'
#define FD_EX_CMD 'D'
#define BUSNUM_CMD 'W'
#define CLOSE_CMD 'C'
#define OPEN_CMD 'O'
#define BITRATE_CMD 'S'

LAWICELSerial::LAWICELSerial(QString portName, int serialSpeed,
                             int lawicelSpeed, bool canFd, int dataRate)
    : CANConnection(portName, "LAWICEL", CANCon::LAWICEL, serialSpeed,
                    lawicelSpeed, canFd, dataRate, 3, 4000, true),
      mTimer(this) /*NB: set this as parent of timer to manage it from working
                      thread */
{
  sendDebug("LAWICELSerial()");

  serial = nullptr;
  isAutoRestart = false;
  this->dataRate = dataRate;
  this->lawicelSpeed = lawicelSpeed;
  this->canFd = canFd;

  readSettings();
}

LAWICELSerial::~LAWICELSerial() {
  stop();
  sendDebug("~LAWICELSerial()");
}

void LAWICELSerial::sendDebug(const QString debugText) {
  qDebug() << debugText;
  debugOutput(debugText);
}

void LAWICELSerial::sendToSerial(const QByteArray &bytes) {
  if (serial == nullptr) {
    sendDebug(
        "Attempt to write to serial port when it has not been initialized!");
    return;
  }

  if (serial && !serial->isOpen()) {
    sendDebug("Attempt to write to serial port when it is not open!");
    return;
  }

  QString buildDebug;
  buildDebug = "Write to serial -> ";
  foreach (int byt, bytes) {
    byt = (unsigned char)byt;
    buildDebug = buildDebug % QString::number(byt, 16) % " ";
  }
  sendDebug(buildDebug);

  if (serial)
    serial->write(bytes);
}

void LAWICELSerial::piStarted() { connectDevice(); }

void LAWICELSerial::piSuspend(bool pSuspend) {
  /* update capSuspended */
  setCapSuspended(pSuspend);

  /* flush queue if we are suspended */
  if (isCapSuspended())
    getQueue().flush();
}

void LAWICELSerial::piStop() {
  mTimer.stop();
  disconnectDevice();
}

bool LAWICELSerial::piGetBusSettings(int pBusIdx, CANBus &pBus) {
  return getBusConfig(pBusIdx, pBus);
}

void LAWICELSerial::piSetBusSettings(int pBusIdx, CANBus bus) {
  /* sanity checks */
  if ((pBusIdx < 0) || pBusIdx >= getNumBuses())
    return;

  /* copy bus config */
  setBusConfig(pBusIdx, bus);
  /*
      qDebug() << "About to update bus " << pBusIdx << " on GVRET";
      if (pBusIdx == 0)
      {
          can0Baud = bus.getSpeed();
          can0Baud |= 0x80000000;
          if (bus.isActive())
          {
              can0Baud |= 0x40000000;
              can0Enabled = true;
          }
          else can0Enabled = false;

          if (bus.isListenOnly())
          {
              can0Baud |= 0x20000000;
              can0ListenOnly = true;
          }
          else can0ListenOnly = false;
      }
  */
  // if (pBusIdx < 2) {
  //   /* update baud rates */
  //   QByteArray buffer;
  //   // sendDebug("Got signal to update bauds. 1: " +
  //   QString::number((can0Baud &
  //   // 0xFFFFFFF)));
  //   buffer[0] = (char)0xF1; // start of a command over serial
  //                           // sendToSerial(buffer);
  // }

  if (!serial)
    return;

  QByteArray buffer;

  // Close bus
  closeBus(pBusIdx, &buffer);
  sendToSerial(buffer);
  waitForReply();
  buffer.clear();

  writeBusSpeed(pBusIdx, &buffer);
  sendToSerial(buffer);
  waitForReply();
  buffer.clear();

  if (bus.isActive()) {
    openBus(pBusIdx, &buffer);
    sendToSerial(buffer);
    waitForReply();
  }
}

bool LAWICELSerial::piSendFrame(const CANFrame &frame) {
  QByteArray buffer;
  int c;
  quint32 ID;

  // qDebug() << "Sending out lawicel frame with id " << frame.ID << " on bus "
  // << frame.bus;

  framesRapid++;

  if (serial == nullptr)
    return false;
  if (serial && !serial->isOpen())
    return false;
  // if (!isConnected) return false;

  // Doesn't make sense to send an error frame
  // to an adapter
  if (frame.frameId() & 0x20000000) {
    return true;
  }

  ID = frame.frameId();
  if (frame.hasExtendedFrameFormat())
    ID |= 1u << 31;

  int idx = 0;
  if (frame.hasFlexibleDataRateFormat()) {
    if (frame.hasExtendedFrameFormat()) {
      if (frame.hasBitrateSwitch())
        buffer[idx] = FD_BRS_EX_CMD;
      else
        buffer[idx] = FD_EX_CMD;
    } else {
      if (frame.hasBitrateSwitch())
        buffer[idx] = FD_BRS_CMD;
      else
        buffer[idx] = FD_CMD;
    }
  } else {
    if (frame.hasExtendedFrameFormat())
      buffer[idx] = TRANSMIT_EX_CMD;
    else
      buffer[idx] = TRANSMIT_CMD;
  }
  idx++;
  buffer[idx] = frame.bus;
  idx++;
  if (frame.hasExtendedFrameFormat()) {
    buffer[idx] = (ID >> 24) & 0xFF;
    idx++;
    buffer[idx] = (ID >> 16) & 0xFF;
    idx++;
  }
  buffer[idx] = (ID >> 8) & 0xFF;
  idx++;
  buffer[idx] = ID & 0xFF;
  idx++;
  buffer[idx] = frame.payload().length();
  idx++;
  for (c = 0; c < frame.payload().length(); c++) {
    buffer[idx] = frame.payload()[c];
    idx++;
  }
  buffer[idx] = OK;

  sendToSerial(buffer);

  return true;
}

/****************************************************************/

void LAWICELSerial::readSettings() { QSettings settings; }

void LAWICELSerial::connectDevice() {
  QSettings settings;

  /* disconnect device */
  if (serial)
    disconnectDevice();

  /* open new device */

  qDebug() << "Serial port: " << getPort();

  serial = new QSerialPort(QSerialPortInfo(getPort()));
  if (!serial) {
    sendDebug("can't open serial port " + getPort());
    return;
  }
  sendDebug("Created Serial Port Object");

  /* connect reading event */
  connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));
  connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
          SLOT(serialError(QSerialPort::SerialPortError)));

  /* configure */
  serial->setBaudRate(mSerialSpeed);
  serial->setDataBits(serial->Data8);

  serial->setFlowControl(serial->HardwareControl);
  // serial->setFlowControl(serial->NoFlowControl);
  if (!serial->open(QIODevice::ReadWrite)) {
    // sendDebug("Error returned during port opening: " +
    // serial->errorString());
  } else {
    // serial->setDataTerminalReady(true); //Seemingly these two lines used to
    // be needed serial->setRequestToSend(true);     //But, really both ends
    // should automatically handle these
    deviceConnected();
  }
}

void LAWICELSerial::deviceConnected() {
  sendDebug("Connecting to LAWICEL Device!");

  QByteArray output;

  // Get number of buses to configure
  output.append(BUSNUM_CMD);
  output.append(OK);
  sendToSerial(output);
  waitForReply();
  output.clear();
  initBuses();

  for (int i = 0; i < mNumBuses; i++) {
    // close the bus in case it was already up
    closeBus(i, &output);
    sendToSerial(output);
    waitForReply();
    output.clear();

    // configure speed for each bus
    writeBusSpeed(i, &output);
    sendToSerial(output);
    waitForReply();
    output.clear();
  }

  // configure CAN FD if enabled
  // TODO: multi bus support
  // WARN: hardware doesnt support this command
  if (this->canFd) {
    switch (this->dataRate) {
    case 1000000:
      output.append("Y1");
      break;
    case 2000000:
      output.append("Y2");
      break;
    case 4000000:
      output.append("Y4");
      break;
    case 5000000:
      output.append("Y5");
      break;
    default:
      output.append("Y2");
      break;
    }
    output.append(OK);
    sendToSerial(output);
    waitForReply();
    output.clear();
  }

  for (int i = 0; i < mNumBuses; i++) {
    // open bus now that we set the speed
    openBus(i, &output);
    sendToSerial(output);
    waitForReply();
    output.clear();
  }

  setStatus(CANCon::CONNECTED);
  CANConStatus stats;
  stats.conStatus = getStatus();
  stats.numHardwareBuses = mNumBuses;
  configureBuses();
  emit status(stats);
}

void LAWICELSerial::initBuses() {
  mBusData.resize(mNumBuses);
  for (int i = 0; i < mNumBuses; i++) {
    mBusData[i].mConfigured = false;
    mBusData[i].mBus.setActive(false);
    mBusData[i].mBus.setSpeed(this->lawicelSpeed);
  }
}

void LAWICELSerial::configureBuses() {
  for (int i = 0; i < mNumBuses; i++) {
    mBusData[i].mConfigured = true;
    mBusData[i].mBus.setActive(true);
  }
}

bool LAWICELSerial::waitForReply() {
  if (!serial->waitForReadyRead(3 * 1000)) {
    sendDebug("Didn't get a reply");
    return false;
  }
  return true;
}

void LAWICELSerial::writeBusSpeed(int pBusIdx, QByteArray *buf) {
  buf->append(BITRATE_CMD);
  buf->append(pBusIdx);
  switch (this->mBusData[pBusIdx].mBus.getSpeed()) {
  case 10000:
    buf->append('0');
    break;
  case 20000:
    buf->append('1');
    break;
  case 50000:
    buf->append('2');
    break;
  case 100000:
    buf->append('3');
    break;
  case 125000:
    buf->append('4');
    break;
  case 250000:
    buf->append('5');
    break;
  case 500000:
    buf->append('6');
    break;
  case 800000:
    buf->append('7');
    break;
  case 1000000:
    buf->append('8');
    break;
  default:
    buf->append('6');
    break;
  }
  buf->append(OK);
}

void LAWICELSerial::openBus(int pBusIdx, QByteArray *buf) {
  buf->append(OPEN_CMD);
  buf->append(pBusIdx);
  buf->append(OK);
}

void LAWICELSerial::closeBus(int pBusIdx, QByteArray *buf) {
  buf->append(CLOSE_CMD);
  buf->append(pBusIdx);
  buf->append(OK);
}

void LAWICELSerial::disconnectDevice() {
  if (serial != nullptr) {
    if (serial->isOpen()) {
      // serial->clear();
      serial->close();
    }
    serial->disconnect(); // disconnect all signals
    delete serial;
    serial = nullptr;
  }

  setStatus(CANCon::NOT_CONNECTED);
  CANConStatus stats;
  stats.conStatus = getStatus();
  stats.numHardwareBuses = mNumBuses;
  emit status(stats);
}

void LAWICELSerial::serialError(QSerialPort::SerialPortError err) {
  QString errMessage;
  bool killConnection = false;
  switch (err) {
  case QSerialPort::NoError:
    return;
  case QSerialPort::DeviceNotFoundError:
    errMessage = "Device not found error on serial";
    killConnection = true;
    piStop();
    break;
  case QSerialPort::PermissionError:
    errMessage = "Permission error on serial port";
    killConnection = true;
    piStop();
    break;
  case QSerialPort::OpenError:
    errMessage = "Open error on serial port";
    killConnection = true;
    piStop();
    break;
  case QSerialPort::ParityError:
    errMessage = "Parity error on serial port";
    break;
  case QSerialPort::FramingError:
    errMessage = "Framing error on serial port";
    break;
  case QSerialPort::BreakConditionError:
    errMessage = "Break error on serial port";
    break;
  case QSerialPort::WriteError:
    errMessage = "Write error on serial port";
    piStop();
    break;
  case QSerialPort::ReadError:
    errMessage = "Read error on serial port";
    piStop();
    break;
  case QSerialPort::ResourceError:
    errMessage = "Serial port seems to have disappeared.";
    killConnection = true;
    piStop();
    break;
  case QSerialPort::UnsupportedOperationError:
    errMessage = "Unsupported operation on serial port";
    killConnection = true;
    break;
  case QSerialPort::UnknownError:
    errMessage = "Beats me what happened to the serial port.";
    killConnection = true;
    piStop();
    break;
  case QSerialPort::TimeoutError:
    errMessage = "Timeout error on serial port";
    killConnection = true;
    break;
  case QSerialPort::NotOpenError:
    errMessage = "The serial port isn't open";
    killConnection = true;
    piStop();
    break;
  }
  /*
  if (serial)
  {
      serial->clearError();
      serial->flush();
      serial->close();
  }*/
  if (errMessage.length() > 1) {
    sendDebug(errMessage);
  }
  if (killConnection) {
    qDebug() << "Shooting the serial object in the head. It deserves it.";
    disconnectDevice();
  }
}

void LAWICELSerial::connectionTimeout() {
  // one second after trying to connect are we actually connected?
  if (CANCon::NOT_CONNECTED == getStatus()) // no?
  {
    // then emit the the failure signal and see if anyone cares
    sendDebug("Failed to connect to LAWICEL at that com port");

    disconnectDevice();
    connectDevice();
  } else {
    /* start timer */
    connect(&mTimer, SIGNAL(timeout()), this, SLOT(handleTick()));
    mTimer.setInterval(250);     // tick four times per second
    mTimer.setSingleShot(false); // keep ticking
    mTimer.start();
  }
}

void LAWICELSerial::readSerialData() {
  QByteArray data;     // buffer
  uint32_t id;         // msg id
  CANFrame buildFrame; // parsed frame
  QString debugBuild;  // debug string

  if (!serial)
    return;

  data = serial->readAll();
  // ACK response
  if (data[0] == OK) {
    sendDebug("OK");
    return;
  } else if (data[0] == ERROR) {
    sendDebug("ERROR");
    return;
  }
  // Bus response
  switch (data[0]) {
  case TRANSMIT_CMD:
    // read bus ID
    buildFrame.bus = (uint8_t)data[1];
    // read message ID
    id = (uint8_t)data[2] << 8 | (uint8_t)data[3];
    buildFrame.setFrameId(id);
    buildFrame.isReceived = true;
    buildFrame.setFrameType(QCanBusFrame::FrameType::DataFrame);
    // read message buffer
    buildFrame.setPayload(data.mid(5, data[4]));
    break;
  case TRANSMIT_EX_CMD:
    buildFrame.setExtendedFrameFormat(true);
    // read bus ID
    buildFrame.bus = (uint8_t)data[1];
    // read message ID
    id = (uint8_t)data[2] << 24 | (uint8_t)data[3] << 16 |
         (uint8_t)data[4] << 8 | (uint8_t)data[5];
    buildFrame.setFrameId(id);
    buildFrame.isReceived = true;
    buildFrame.setFrameType(QCanBusFrame::FrameType::DataFrame);
    // read message buffer
    buildFrame.setPayload(data.mid(7, data[6]));
    break;
  case BUSNUM_CMD:
    mNumBuses = (uint8_t)data[1];
    // print to debug console
    for (int i = 0; i < data.length(); i++) {
      debugBuild =
          debugBuild %
          QString::number((uint8_t)data.at(i), 16).rightJustified(2, '0') % " ";
    }
    sendDebug("Got data from serial. Len = " % QString::number(data.length()));
    debugOutput(debugBuild);
    return;
  case FD_BRS_CMD:
    buildFrame.setBitrateSwitch(true);
    [[fallthrough]];
  case FD_CMD:
    buildFrame.setFlexibleDataRateFormat(true);
    // read bus ID
    buildFrame.bus = (uint8_t)data[1];
    // read message ID
    id = (uint8_t)data[2] << 24 | (uint8_t)data[3] << 16;
    buildFrame.isReceived = true;
    buildFrame.setFrameId(id);
    buildFrame.setFrameType(QCanBusFrame::FrameType::DataFrame);
    // read message buffer
    buildFrame.setPayload(
        data.mid(5, LAWICELSerial::dlc_code_to_bytes((uint8_t)data[4])));
    break;
  case FD_BRS_EX_CMD:
    buildFrame.setBitrateSwitch(true);
    [[fallthrough]];
  case FD_EX_CMD:
    buildFrame.setFlexibleDataRateFormat(true);
    buildFrame.setExtendedFrameFormat(true);
    // read bus ID
    buildFrame.bus = (uint8_t)data[1];
    // read message ID
    id = (uint8_t)data[2] << 24 | (uint8_t)data[3] << 16 |
         (uint8_t)data[4] << 8 | (uint8_t)data[5];
    buildFrame.isReceived = true;
    buildFrame.setFrameId(id);
    buildFrame.setFrameType(QCanBusFrame::FrameType::DataFrame);
    // read message buffer
    buildFrame.setPayload(
        data.mid(7, LAWICELSerial::dlc_code_to_bytes((uint8_t)data[6])));
    break;
  default:
    return;
  }
  if (useSystemTime) {
    buildTimestamp = QDateTime::currentMSecsSinceEpoch() * 1000l;
  } else {
    // TODO: implement timestamp
    // if (data[0] != OK) {
    //   buildTimestamp =
    //       (((uint8_t)data.at(0) << 24) | ((uint8_t)data.at(1) << 16) |
    //        ((uint8_t)data.at(2) << 8) | (uint8_t)data.at(3)) *
    //       1000l;
    // } else {
    //   // Default to system time if timestamps are disabled.
    //   buildTimestamp = QDateTime::currentMSecsSinceEpoch() * 1000l;
    // }
    buildTimestamp = QDateTime::currentMSecsSinceEpoch() * 1000l;
  }
  buildFrame.setTimeStamp(QCanBusFrame::TimeStamp(0, buildTimestamp));
  if (!isCapSuspended()) {
    /* get frame from queue */
    CANFrame *frame_p = getQueue().get();
    if (frame_p) {
      // qDebug() << "Lawicel got frame on bus " << frame_p->bus;
      /* copy frame */
      *frame_p = buildFrame;
      checkTargettedFrame(buildFrame);
      /* enqueue frame */
      getQueue().queue();
    } else
      qDebug() << "can't get a frame, ERROR";
  }

  // print to debug console
  for (int i = 0; i < data.length(); i++) {
    debugBuild =
        debugBuild %
        QString::number((uint8_t)data.at(i), 16).rightJustified(2, '0') % " ";
  }
  sendDebug("Got data from serial. Len = " % QString::number(data.length()));
  debugOutput(debugBuild);
}

// Debugging data sent from connection window. Inject it into Comm traffic.
void LAWICELSerial::debugInput(QByteArray bytes) { sendToSerial(bytes); }

void LAWICELSerial::handleTick() {
  // qDebug() << "Tick!";
}

// Convert a FDCAN_data_length_code to number of bytes in a message
uint8_t LAWICELSerial::dlc_code_to_bytes(int dlc_code) {
  if (dlc_code <= 8)
    return dlc_code;
  else {
    switch (dlc_code) {
    case 9:
      return 12;
    case 10:
      return 16;
    case 11:
      return 20;
    case 12:
      return 24;
    case 13:
      return 32;
    case 14:
      return 48;
    case 15:
      return 64;
    default:
      return 0;
    }
  }
}

uint8_t LAWICELSerial::bytes_to_dlc_code(uint8_t bytes) {
  if (bytes <= 8)
    return bytes;
  else {
    switch (bytes) {
    case 12:
      return 9;
    case 16:
      return 10;
    case 20:
      return 11;
    case 24:
      return 12;
    case 32:
      return 13;
    case 48:
      return 14;
    case 64:
      return 15;
    default:
      return 0;
    }
  }
}
