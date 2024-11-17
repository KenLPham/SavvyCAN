#ifndef LAWICELSERIAL_H
#define LAWICELSERIAL_H

#include <QCanBusDevice>
#include <QSerialPort>
#include <QThread>
#include <QTimer>

/*************/
#include <QDateTime>
/*************/

#include "canconmanager.h"
#include "canconnection.h"
#include "canframemodel.h"

class LAWICELSerial : public CANConnection {
  Q_OBJECT

public:
  LAWICELSerial(QString portName, int serialSpeed, int lawicelSpeed, bool canFd,
                int dataRate);
  virtual ~LAWICELSerial();

protected:
  virtual void piStarted();
  virtual void piStop();
  virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
  virtual bool piGetBusSettings(int pBusIdx, CANBus &pBus);
  virtual void piSuspend(bool pSuspend);
  virtual bool piSendFrame(const CANFrame &);

  void disconnectDevice();

public slots:
  void debugInput(QByteArray bytes);

private slots:
  void connectDevice();
  void connectionTimeout();
  void readSerialData();
  void serialError(QSerialPort::SerialPortError err);
  void deviceConnected();
  void handleTick();

private:
  void openBus(int pBusIdx, QByteArray *buf);
  void closeBus(int pBusIdx, QByteArray *buf);
  void initBuses();
  void configureBuses();
  void readSettings();
  void rebuildLocalTimeBasis();
  void sendToSerial(const QByteArray &bytes);
  void sendDebug(const QString debugText);
  uint8_t dlc_code_to_bytes(int dlc_code);
  uint8_t bytes_to_dlc_code(uint8_t bytes);
  bool waitForReply();
  void writeBusSpeed(int pBusIdx, QByteArray *buf);

protected:
  QTimer mTimer;
  QThread mThread;

  bool isAutoRestart;
  QSerialPort *serial;
  int framesRapid;
  CANFrame buildFrame;
  qint64 buildTimestamp;
  bool can0Enabled;
  bool can0ListenOnly;
  // initial config
  bool canFd;
  int dataRate;
  int lawicelSpeed;
};

#endif // LAWICELSERIAL_H
