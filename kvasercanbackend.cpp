/**
 *  File: kvasercanbackend.cpp
 *
 *
 *  Copyright (c) 2023 Andy Josephson <svcguy@gmail.com>
 *
 *  Based on works by Denis Shienkov <denis.shienkov@gmail.com>
 *  and The Qt Company Ltd.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "kvasercanbackend.h"
#include "kvasercanbackend_p.h"

#include <QtSerialBus/qcanbusdevice.h>

#include <QtCore/QElapsedTimer>
#include <QtCore/QVariant>
#include <QtCore/qcoreevent.h>
#include <QtCore/qloggingcategory.h>
#include <QtCore/qsocketnotifier.h>
#include <QtCore/qtimer.h>

#include "KvaserCAN.h"

QT_BEGIN_NAMESPACE

#define KVASERCAN_READ_TIMEOUT  1

Q_DECLARE_LOGGING_CATEGORY(QT_CANBUS_PLUGINS_KVASERCAN)

class KvaserCanReadNotifier : public QSocketNotifier
{
public:
  explicit KvaserCanReadNotifier(KvaserCanBackendPrivate *d, QObject *parent)
      : QSocketNotifier(d->readHandle, QSocketNotifier::Read, parent)
      , dptr(d)
  {
  }
protected:
  bool event(QEvent *e) override
  {
    if (e->type() == QEvent::SockAct)
    {
      dptr->startRead();
      return true;
    }
    return QSocketNotifier::event(e);
  }
private:
    KvaserCanBackendPrivate *const dptr;
};

class KvaserCanWriteNotifier : public QTimer
{
public:
  KvaserCanWriteNotifier(KvaserCanBackendPrivate *d, QObject *parent)
      : QTimer(parent)
      , dptr(d)
  {
  }
protected:
  void timerEvent(QTimerEvent *e) override
  {
    if (e->timerId() == timerId())
    {
      dptr->startWrite();
      return;
    }
    QTimer::timerEvent(e);
  }
private:
    KvaserCanBackendPrivate *const dptr;
};

struct KvaserCanChannel
{
  char name[21];
  CANAPI_Handle_t index;
};

static const KvaserCanChannel kvasercanChannels[KVASER_BOARDS] =
{
  { "Kvaser CAN Channel 0", KVASER_CAN_CHANNEL0 },
  { "Kvaser CAN Channel 1", KVASER_CAN_CHANNEL1 },
  { "Kvaser CAN Channel 2", KVASER_CAN_CHANNEL2 },
  { "Kvaser CAN Channel 3", KVASER_CAN_CHANNEL3 },
  { "Kvaser CAN Channel 4", KVASER_CAN_CHANNEL4 },
  { "Kvaser CAN Channel 5", KVASER_CAN_CHANNEL5 },
  { "Kvaser CAN Channel 6", KVASER_CAN_CHANNEL6 },
  { "Kvaser CAN Channel 7", KVASER_CAN_CHANNEL7 }
};

static QString
kvaserCanChannelNameFromIndex(quint64 index)
{
  const auto kvasercanChannel = std::find_if(std::begin(kvasercanChannels),
                                             std::end(kvasercanChannels),
                                             [index](KvaserCanChannel channel)
                                             {
                                               return channel.index == index;
                                             });

  if(kvasercanChannel != std::end(kvasercanChannels))
  {
    return kvasercanChannel->name;
  }

  qCWarning(QT_CANBUS_PLUGINS_KVASERCAN,
            "Cannot get channel name for index %llu", index);

  return QStringLiteral("none");
}

QString
configurationKeyCodeToString (QCanBusDevice::ConfigurationKey code)
{
  switch (code)
  {
  case QCanBusDevice::ConfigurationKey::BitRateKey:
    return QString ("BitRateKey");
  case QCanBusDevice::ConfigurationKey::DataBitRateKey:
    return QString ("DataBitRateKey");
  case QCanBusDevice::ConfigurationKey::CanFdKey:
    return QString ("CanFdKey");
  case QCanBusDevice::ConfigurationKey::ErrorFilterKey:
    return QString ("ErrorFilterKey");
  case QCanBusDevice::ConfigurationKey::RawFilterKey:
    return QString ("RawFilterKey");
  case QCanBusDevice::ConfigurationKey::LoopbackKey:
    return QString ("LoopbackKey");
  case QCanBusDevice::ConfigurationKey::ProtocolKey:
    return QString ("ProtocolKey");
  case QCanBusDevice::ConfigurationKey::ReceiveOwnKey:
    return QString ("ReceiveOwnKey");
  case QCanBusDevice::ConfigurationKey::UserKey:
    return QString ("UserKey");
  default:
    return QString ("UnknownKey");
  }
}

QString
channelStateCodeToString (CCanApi::EChannelState code)
{
  switch (code)
  {
  case CCanApi::EChannelState::ChannelAvailable:
    return QString ("Channel Available");
  case CCanApi::EChannelState::ChannelNotAvailable:
    return QString ("Channel Not Available");
  case CCanApi::EChannelState::ChannelNotTestable:
    return QString ("Channel Not Testable");
  case CCanApi::EChannelState::ChannelOccupied:
    return QString ("Channel Occupied");
  default:
    return QString ("Unknown State");
  }
}

QString
errorCodeToString (CCanApi::EErrorCodes code)
{
  switch (code)
  {
  case CCanApi::EErrorCodes::NoError:
    return QString ("CAN - No Error");
  case CCanApi::EErrorCodes::BusOFF:
    return QString ("CAN - busoff status");
  case CCanApi::EErrorCodes::ErrorWarning:
    return QString ("CAN - error warning status");
  case CCanApi::EErrorCodes::BusError:
    return QString ("CAN - bus error");
  case CCanApi::EErrorCodes::ControllerOffline:
    return QString ("CAN - not started");
  case CCanApi::EErrorCodes::ControllerOnline:
    return QString ("CAN - already started");
  case CCanApi::EErrorCodes::MessageLost:
    return QString ("CAN - message lost");
  case CCanApi::EErrorCodes::TransmitterBusy:
    return QString ("USR - transmitter busy");
  case CCanApi::EErrorCodes::ReceiverEmpty:
    return QString ("USR - receiver empty");
  case CCanApi::EErrorCodes::ErrorFrame:
    return QString ("USR - error frame");
  case CCanApi::EErrorCodes::Timeout:
    return QString ("USR - time-out");
  case CCanApi::EErrorCodes::ResourceError:
    return QString ("USR - resource allocation");
  case CCanApi::EErrorCodes::InvalidBaudrate:
    return QString ("USR - illegal baudrate");
  case CCanApi::EErrorCodes::InvalidHandle:
    return QString ("USR - illegal handle");
  case CCanApi::EErrorCodes::IllegalParameter:
    return QString ("USR - illegal parameter");
  case CCanApi::EErrorCodes::NullPointer:
    return QString ("USR - null-pointer assignment");
  case CCanApi::EErrorCodes::NotInitialized:
    return QString ("USR - not initialized");
  case CCanApi::EErrorCodes::AlreadyInitialized:
    return QString ("USR - already initialized");
  case CCanApi::EErrorCodes::InvalidLibrary:
    return QString ("USR - illegal library");
  case CCanApi::EErrorCodes::NotSupported:
    return QString ("USR - not supported");
  case CCanApi::EErrorCodes::FatalError:
    return QString ("USR - other errors");
  case CCanApi::EErrorCodes::VendorSpecific:
    return QString ("USR - vendor specific");
  default:
    return QString ("CAN - unknown error (%1)").arg (code);
  }
}

QString
busStatusCodeToString (QCanBusDevice::CanBusStatus code)
{
  switch (code)
  {
  case QCanBusDevice::CanBusStatus::Good:
    return QString ("CanBusStatus Good");
  case QCanBusDevice::CanBusStatus::Warning:
    return QString ("CanBusStatus Warning");
  case QCanBusDevice::CanBusStatus::Error:
    return QString ("CanBusStatus Error");
  case QCanBusDevice::CanBusStatus::BusOff:
    return QString ("CanBusStatus BusOff");
  case QCanBusDevice::CanBusStatus::Unknown:
    return QString ("CanBusStatus Unknown");
  default:
    return QString ("Unknown can bus status code");
  }
}

QString
frameErrorCodeToString (QCanBusFrame::FrameError code)
{
  switch (code)
  {
  case QCanBusFrame::FrameError::NoError:
    return QString ("NoError ");
  case QCanBusFrame::FrameError::TransmissionTimeoutError:
    return QString ("TxTimeout ");
  case QCanBusFrame::FrameError::LostArbitrationError:
    return QString ("ArbitrationError ");
  case QCanBusFrame::FrameError::ControllerError:
    return QString ("ControllerError ");
  case QCanBusFrame::FrameError::ProtocolViolationError:
    return QString ("ProtocolError ");
  case QCanBusFrame::FrameError::TransceiverError:
    return QString ("TransceiverError ");
  case QCanBusFrame::FrameError::MissingAcknowledgmentError:
    return QString ("AckError ");
  case QCanBusFrame::FrameError::BusOffError:
    return QString ("BusOfflineError ");
  case QCanBusFrame::FrameError::BusError:
    return QString ("BusError ");
  case QCanBusFrame::FrameError::ControllerRestartError:
    return QString ("ControllerRestart ");
  case QCanBusFrame::FrameError::UnknownError:
    return QString ("UnknownError ");
  case QCanBusFrame::FrameError::AnyError:
    return QString ("OtherError ");
  }
}

KvaserCanBackend::KvaserCanBackend(const QString &name, QObject *parent)
    : QCanBusDevice(parent)
    , d_ptr(new KvaserCanBackendPrivate(this))
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D (KvaserCanBackend);

  d->setupChannel (name.toLatin1 ());
  d->setupDefaultConfigurations ();

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

KvaserCanBackend::~KvaserCanBackend()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D (KvaserCanBackend);

  if (d->isOpen)
  {
    setState (QCanBusDevice::UnconnectedState);
    qCInfo (QT_CANBUS_PLUGINS_KVASERCAN, "%s: Closing device", Q_FUNC_INFO);
    d->close ();
  }

  delete d_ptr;
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

bool
KvaserCanBackend::open()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D(KvaserCanBackend);

  if (!d->isOpen)
  {
    if (!d->open())
    {
      qCInfo (QT_CANBUS_PLUGINS_KVASERCAN,
              "%s d->open() returned false", Q_FUNC_INFO);
      return false;
    }

    const auto keys = configurationKeys();

    for (ConfigurationKey key : keys)
    {
      if (key == QCanBusDevice::BitRateKey ||
          key == QCanBusDevice::DataBitRateKey)
      {
        continue;
      }

      const QVariant param = configurationParameter(key);

      qCInfo (QT_CANBUS_PLUGINS_KVASERCAN,
              "Applying configuration key %s with %s",
              qUtf8Printable (configurationKeyCodeToString (key)),
              qUtf8Printable (param.toString ()));

      const bool success = d->setConfigurationParameter (key, param);
      if (!success)
      {
        qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
                   "Cannot apply parameter %s with %s.",
                   qUtf8Printable (configurationKeyCodeToString (key)),
                   qUtf8Printable (param.toString ()));
      }
    }
  }

  setState (QCanBusDevice::ConnectedState);

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN,
           "Exiting: %s, returning [true]", Q_FUNC_INFO);

  return true;
}

void
KvaserCanBackend::close()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D(KvaserCanBackend);

  d->close();

  setState(QCanBusDevice::UnconnectedState);

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

void
KvaserCanBackend::setConfigurationParameter(ConfigurationKey key,
                                             const QVariant &value)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D(KvaserCanBackend);

  qInfo (QT_CANBUS_PLUGINS_KVASERCAN,
           "KvaserCanBackend::setConfigurationParameter(%s, %s)",
           qUtf8Printable (configurationKeyCodeToString (key)),
           qUtf8Printable (value.toString ()));

  if (d->setConfigurationParameter(key, value))
  {
    QCanBusDevice::setConfigurationParameter(key, value);
  }

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

bool
KvaserCanBackend::writeFrame(const QCanBusFrame &newData)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D(KvaserCanBackend);

  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCanBackend::writeFrame(%s)",
          qUtf8Printable(newData.toString()));

  if (state () != QCanBusDevice::ConnectedState)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Attempting to write frame while not connected");

    setError ("Attempting to write frame while not connected",
              QCanBusDevice::WriteError);

    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "%s returning false", Q_FUNC_INFO);
    return false;
  }

  if (!newData.isValid())
  {
    qCWarning(QT_CANBUS_PLUGINS_KVASERCAN, "Attempting to write invalid data");

    setError("Attempting to write invalid data", QCanBusDevice::WriteError);

    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "%s returning false", Q_FUNC_INFO);
    return false;
  }

  if (newData.frameType() != QCanBusFrame::DataFrame
      && newData.frameType() != QCanBusFrame::RemoteRequestFrame)
  {
    qWarning(QT_CANBUS_PLUGINS_KVASERCAN,
            "Attempting to write frame with invalid type");

    setError("Attempting to write frame with invalid type",
              QCanBusDevice::WriteError);

    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "%s returning false", Q_FUNC_INFO);
    return false;
  }

  enqueueOutgoingFrame(newData);

  if (!d->writeNotifier->isActive())
  {
    d->writeNotifier->start();
  }

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "%s return true", Q_FUNC_INFO);
  return true;
}

QString
KvaserCanBackend::interpretErrorFrame(const QCanBusFrame &errorFrame)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  QString compositeError ("FrameError: ");

  if (errorFrame.error ().testFlag (QCanBusFrame::NoError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with NoError flag set received");
    compositeError
        += frameErrorCodeToString (QCanBusFrame::FrameError::NoError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::TransmissionTimeoutError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with TransmissionTimeout flag set received");
    compositeError += frameErrorCodeToString (
        QCanBusFrame::FrameError::TransmissionTimeoutError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::LostArbitrationError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with LostArbitration flag set received");
    compositeError += frameErrorCodeToString (
        QCanBusFrame::FrameError::LostArbitrationError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::ControllerError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with ControllerError flag set received");
    compositeError
        += frameErrorCodeToString (QCanBusFrame::FrameError::ControllerError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::ProtocolViolationError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with ProtocolViolation flag set received");
    compositeError += frameErrorCodeToString (
        QCanBusFrame::FrameError::ProtocolViolationError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::TransceiverError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with TranscieverError flag set received");
    compositeError
        += frameErrorCodeToString (QCanBusFrame::FrameError::TransceiverError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::MissingAcknowledgmentError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with MissingAck flag set received");
    compositeError += frameErrorCodeToString (
        QCanBusFrame::FrameError::MissingAcknowledgmentError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::BusOffError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with BusOff flag set received");
    compositeError
        += frameErrorCodeToString (QCanBusFrame::FrameError::BusOffError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::BusError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with BusError flag set received");
    compositeError
        += frameErrorCodeToString (QCanBusFrame::FrameError::BusError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::ControllerRestartError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with ControllerRestart flag set received");
    compositeError += frameErrorCodeToString (
        QCanBusFrame::FrameError::ControllerRestartError);
  }
  if (errorFrame.error ().testFlag (QCanBusFrame::UnknownError))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Error Frame with UnknownError flag set received");
    compositeError
        += frameErrorCodeToString (QCanBusFrame::FrameError::UnknownError);
  }

  if (compositeError.endsWith (" "))
  {
    compositeError.removeLast ();
  }

  return compositeError;
}

bool
KvaserCanBackend::canCreate(QString *errorReason)
{
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN, "Entering %s", Q_FUNC_INFO);

  char *version = nullptr;

  version = CKvaserCAN::GetVersion ();

  if (version)
  {
    qCInfo (QT_CANBUS_PLUGINS_KVASERCAN, "%s returning true",
             Q_FUNC_INFO);
    return true;
  }
  else
  {
    qCInfo (QT_CANBUS_PLUGINS_KVASERCAN, "%s returning false",
             Q_FUNC_INFO);
    return false;
  }
}

QList<QCanBusDeviceInfo>
KvaserCanBackend::interfaces()
{
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN, "Entering %s", Q_FUNC_INFO);

  QList<QCanBusDeviceInfo> result;

  CANAPI_Return_t ret;
  CKvaserCAN::SChannelInfo info = {};
  CANAPI_OpMode_t op = {};
  CCanApi::EChannelState state;
  bool isFD;
  qint32 ch = 0;

  if (CKvaserCAN::GetFirstChannel(info, nullptr))
  {
    ret = CKvaserCAN::ProbeChannel(ch, op, state);

    if (ret == CCanApi::NoError && state == CCanApi::ChannelAvailable)
    {
      isFD = op.fdoe;

      qCInfo (QT_CANBUS_PLUGINS_KVASERCAN,
              "CH %d, name=%s, isFD=%d",
              ch,
              qUtf8Printable(info.m_szDeviceName),
              isFD);

      result.append (QCanBusDevice::createDeviceInfo(
                                    QStringLiteral("kvasercan"),
                      QLatin1String(info.m_szDeviceName),
                      false,
                      isFD));
      ch++;
    }
    else
    {
      qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
                 "CKvaserCAN::ProbeChannel(ch:%d) failed [error = %s], "
                 "[state = %s]",
                 ch,
                 qUtf8Printable (errorCodeToString (
                     static_cast<CCanApi::EErrorCodes> (ret))),
                 qUtf8Printable (channelStateCodeToString (state)));

      // Return empty list
      return QList<QCanBusDeviceInfo>();
    }
  }
  else
  {
    qCWarning(QT_CANBUS_PLUGINS_KVASERCAN,
               "CKvaserCAN::GetFirstChannel() failed");

    // Return empty list
    return QList<QCanBusDeviceInfo>();
  }

  while (CKvaserCAN::GetNextChannel(info, nullptr))
  {
    ret = CKvaserCAN::ProbeChannel(ch, op, state);

    if (ret == CCanApi::NoError && state == CCanApi::ChannelAvailable)
    {
      isFD = op.fdoe;

      qCInfo(QT_CANBUS_PLUGINS_KVASERCAN,
             "CH %d, name=%s, isFD=%d",
             ch,
             qUtf8Printable(info.m_szDeviceName),
             isFD);

      result.append (QCanBusDevice::createDeviceInfo (
          QStringLiteral ("kvasercan"), QLatin1String (info.m_szDeviceName),
          false, isFD));
      ch++;
    }
    else
    {
      qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
                 "CKvaserCAN::ProbeChannel(ch:%d) failed [error = %s], "
                 "[state = %s]",
                 ch,
                 qUtf8Printable (errorCodeToString (
                     static_cast<CCanApi::EErrorCodes> (ret))),
                 qUtf8Printable (channelStateCodeToString (state)));
      ch++;
    }
  }

  qCInfo(QT_CANBUS_PLUGINS_KVASERCAN,
          "%s found %lld interface(s)", Q_FUNC_INFO, result.length());

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);

  return result;
}

void
KvaserCanBackend::resetController()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  Q_D(KvaserCanBackend);

  close ();
  open ();

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

bool
KvaserCanBackend::hasBusStatus() const
{
  return true;
}

QCanBusDevice::CanBusStatus
KvaserCanBackend::busStatus()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_D (KvaserCanBackend);

  CANAPI_Status_t status;

  d->kc_ptr->GetStatus (status);

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "GetStatus() returned 0x%02X",
           status.byte);

  if (status.byte & CANSTAT_RX_EMPTY)
  {
    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "busStatus = Receiver Empty");
    return QCanBusDevice::CanBusStatus::Good;
  }

  if (status.byte == 0x00)
  {
    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "busStatus = %s",
             qUtf8Printable (busStatusCodeToString (
                 QCanBusDevice::CanBusStatus::Good)));
    return QCanBusDevice::CanBusStatus::Good;
  }

  if (status.byte & CANSTAT_EWRN)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN, "busStatus = %s",
             qUtf8Printable (busStatusCodeToString (
                 QCanBusDevice::CanBusStatus::Warning)));
    return QCanBusDevice::CanBusStatus::Warning;
  }

  if (status.byte & CANSTAT_BOFF)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN, "busStatus = %s",
             qUtf8Printable (busStatusCodeToString (
                 QCanBusDevice::CanBusStatus::BusOff)));
    return QCanBusDevice::CanBusStatus::BusOff;
  }

  if (status.byte
      & (CANSTAT_BERR | CANSTAT_MSG_LST | CANSTAT_QUE_OVR | CANSTAT_RESET))
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN, "busStatus = %s",
             qUtf8Printable (
                 busStatusCodeToString (QCanBusDevice::CanBusStatus::Error)
                 + QString (" status.byte = %1").arg (status.byte)));
    return QCanBusDevice::CanBusStatus::Error;
  }

  qCWarning (QT_CANBUS_PLUGINS_KVASERCAN, "Unknown CAN bus status: 0x%02X",
             status.byte);

  return QCanBusDevice::CanBusStatus::Unknown;
}

QCanBusDeviceInfo
KvaserCanBackend::deviceInfo() const
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  return QCanBusDevice::deviceInfo();
}

QList<QCanBusDeviceInfo>
KvaserCanBackend::interfacesByChannelCondition(Availability available)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  QList<QCanBusDeviceInfo> result;

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
  return result;
}

QList<QCanBusDeviceInfo>
KvaserCanBackend::interfacesByAttachedChannels(Availability available,
                                                bool *ok)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  QList<QCanBusDeviceInfo> result;

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);

  return result;
}

QList<QCanBusDeviceInfo>
KvaserCanBackend::attachedInterfaces(Availability available)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  QList<QCanBusDeviceInfo> result;

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);

  return result;
}

KvaserCanBackendPrivate::KvaserCanBackendPrivate (KvaserCanBackend *q)
    : q_ptr (q), kc_ptr (new CKvaserCAN ()),
      timestampTimer (new QElapsedTimer ())

{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  void *p = new char[256];

  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "===============================================================");

  qCInfo(QT_CANBUS_PLUGINS_KVASERCAN,
         "KvaserCAN-Library Driver Version: %s",
         kc_ptr->GetVersion());

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_CANAPI, p, sizeof(uint16_t));
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library CANAPI Version:\t0x%04X",
          *(static_cast<uint16_t *>(p)));

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_VERSION, p, sizeof(uint16_t));
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library Version:\t\t0x%04X",
          *(static_cast<uint16_t *>(p)));

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_PATCH_NO, p, sizeof(uint8_t));
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library Patch No:\t\t0x%02X",
          *(static_cast<uint8_t *>(p)));

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_BUILD_NO, p, sizeof(uint32_t));
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library Build No:\t\t0x%08X",
          *(static_cast<uint32_t *>(p)));

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_LIBRARY_ID, p, sizeof(uint32_t));
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library Library ID:\t\t0x%08X",
          *(static_cast<uint32_t *>(p)));

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_LIBRARY_NAME, p, sizeof(char) * 256);
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library Library Name:\t%s",
          static_cast<const char *>(p));

  kc_ptr->GetProperty(KVASERCAN_PROPERTY_LIBRARY_VENDOR, p, sizeof(char) * 256);
  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "KvaserCAN-Library Library Vendor:\t%s",
          static_cast<const char *>(p));

  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "===============================================================");

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

bool
KvaserCanBackendPrivate::open()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  Q_Q(KvaserCanBackend);

  CANAPI_Return_t ret;
  CANAPI_Bitrate_t bitrate = {};
  CANAPI_OpMode_t op = {};

  op.byte = CANMODE_DEFAULT;

  const int configuredBitrate =
      q->configurationParameter(QCanBusDevice::BitRateKey).toInt();

  switch (configuredBitrate)
  {
    case 10000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_10K, bitrate);
      break;
    case 20000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_20K, bitrate);
      break;
    case 50000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_50K, bitrate);
      break;
    case 100000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_100K, bitrate);
      break;
    case 125000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_125K, bitrate);
      break;
    case 250000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_250K, bitrate);
      break;
    case 500000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_500K, bitrate);
      break;
    case 800000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_800K, bitrate);
      break;
    case 1000000:
      CKvaserCAN::MapIndex2Bitrate(CANBTR_INDEX_1M, bitrate);
      break;
    default:
      qCWarning(QT_CANBUS_PLUGINS_KVASERCAN,
                 "Invalid bitrate (%d) selected, %s returning false",
                 configuredBitrate,
                 Q_FUNC_INFO);
      return false;
  }

  qCInfo(QT_CANBUS_PLUGINS_KVASERCAN,
          "Initializing Channel %d...", q->d_ptr->channelIndex);

  ret = q->d_ptr->kc_ptr->InitializeChannel(q->d_ptr->channelIndex, op);

  if (ret != CCanApi::NoError)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
                "Could not initialize channel %d [%s], %s returning false",
                q->d_ptr->channelIndex,
                qUtf8Printable (errorCodeToString (
                  static_cast<CCanApi::EErrorCodes> (ret))),
                Q_FUNC_INFO);

    return false;
  }

  qCInfo(QT_CANBUS_PLUGINS_KVASERCAN,
          "Starting controller (bitrate=%d)", configuredBitrate);

  ret = q->d_ptr->kc_ptr->StartController(bitrate);

  if (ret != CCanApi::NoError)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Could not start controller (bitrate=%d) [%s], %s returning false",
               configuredBitrate,
               qUtf8Printable (errorCodeToString (
                   static_cast<CCanApi::EErrorCodes> (ret))),
               Q_FUNC_INFO);

    return false;
  }

  // Setting the read handle to the channel number
  // TODO: Find out if there is a better value here
  quint8 channelToHandle;

  q->d_ptr->kc_ptr->GetProperty(KVASERCAN_PROPERTY_CAN_CHANNEL,
                                static_cast<void *>(&channelToHandle),
                                sizeof(quint8));

  readHandle = channelToHandle;

  writeNotifier = new KvaserCanWriteNotifier(this, q);
  writeNotifier->setInterval (0);

  readNotifier = new KvaserCanReadNotifier(this, q);
  readNotifier->setEnabled (true);

  timestampTimer->restart ();

  isOpen = true;

  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "Started controller successfully");

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "%s returning true", Q_FUNC_INFO);

  return true;
}

void
KvaserCanBackendPrivate::close()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_Q(KvaserCanBackend);

  CANAPI_Return_t ret;

  delete writeNotifier;
  writeNotifier = nullptr;

  delete readNotifier;
  readNotifier = nullptr;

  qCInfo(QT_CANBUS_PLUGINS_KVASERCAN, "Tearing down channel...");

  ret = q->d_ptr->kc_ptr->TeardownChannel();

  if (ret != CCanApi::NoError)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN, "Could not teardown channel [%s}",
               qUtf8Printable (errorCodeToString (
                   static_cast<CCanApi::EErrorCodes> (ret))));

    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);

    return;
  }

  isOpen = false;

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);

  return;
}

bool
KvaserCanBackendPrivate::setConfigurationParameter(QCanBusDevice::ConfigurationKey key,
                                                        const QVariant &value)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_Q (KvaserCanBackend);

  switch (key)
  {
    case QCanBusDevice::BitRateKey:
      return verifyBitRate (value.toInt ());
    case QCanBusDevice::CanFdKey:
      isFlexibleDatarateEnabled = value.toBool ();
      return true;
    case QCanBusDevice::DataBitRateKey:
      return false;
    default:
      qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
                 "Unsupported configuration key: %s",
                 qUtf8Printable (configurationKeyCodeToString (key)));
      q->setError (QString ("Unsupported configuration key %1")
                  .arg (qUtf8Printable (configurationKeyCodeToString (key))),
          QCanBusDevice::ConfigurationError);
      return false;
  }
}

void
KvaserCanBackendPrivate::setupChannel(const QByteArray &interfaceName)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN,
           "Entering: %s", Q_FUNC_INFO);
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN,
           "KvaserCanBackendPrivate::setupChannel(%s)",
           interfaceName.toStdString ().c_str ());

  const KvaserCanChannel *channels = kvasercanChannels;

  while (channels->index < KVASER_BOARDS && channels->name != interfaceName)
  {
    ++channels;
  }

  if(channels->index == KVASER_BOARDS)
  {
    qCWarning(QT_CANBUS_PLUGINS_KVASERCAN,
            "KvaserCanBackendPrivate::setupChannel could not find a channel with specified name");
    channelIndex = 8;
  }

  else
  {
    channelIndex = channels->index;

    qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
        "KvaserCanBackendPrivate::setupChannel(%s) selected channel %d",
        interfaceName.toStdString().c_str(), channelIndex);
  }

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

void
KvaserCanBackendPrivate::setupDefaultConfigurations()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_Q(KvaserCanBackend);

  setConfigurationParameter (QCanBusDevice::BitRateKey, 500000);

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

QString
KvaserCanBackendPrivate::systemErrorString(qint32 errorCode)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  return qUtf8Printable (
      errorCodeToString (static_cast<CCanApi::EErrorCodes> (errorCode)));
}

void
KvaserCanBackendPrivate::startWrite()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);

  Q_Q(KvaserCanBackend);

  CANAPI_Return_t ret;

  if(!q->hasOutgoingFrames())
  {
    writeNotifier->stop();
    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
    return;
  }

  const QCanBusFrame frame = q->dequeueOutgoingFrame();
  const QByteArray payload = frame.payload();
  const qsizetype payloadSize = payload.size();

  CANAPI_Message_t message = {};

  memset (&message, 0x00, sizeof (message));

  message.id = frame.frameId();
  message.dlc = payloadSize;
  if(frame.hasExtendedFrameFormat())
  {
    message.xtd = 1;
  }
  if(frame.frameType() == QCanBusFrame::RemoteRequestFrame)
  {
    message.rtr = 1;
  }
  else
  {
    memcpy(message.data, payload.constData(), payloadSize);
  }

  qCDebug(QT_CANBUS_PLUGINS_KVASERCAN,
          "Writing Frame: %s", qUtf8Printable(frame.toString()));

  ret = q->d_ptr->kc_ptr->WriteMessage(message);

  if(ret != CCanApi::NoError)
  {
    qCWarning (QT_CANBUS_PLUGINS_KVASERCAN,
               "Unable to write frame: %s\n\terror: %s",
               qUtf8Printable (frame.toString ()),
               qUtf8Printable (errorCodeToString (
                   static_cast<CCanApi::EErrorCodes> (ret))));

    q->setError (errorCodeToString (static_cast<CCanApi::EErrorCodes> (ret)),
                 QCanBusDevice::WriteError);
  }

  emit q->framesWritten(quint64(1));

  if(q->hasOutgoingFrames() && !writeNotifier->isActive())
  {
    writeNotifier->start();
  }
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Wrote 1 frame");
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

void
KvaserCanBackendPrivate::startRead()
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_Q(KvaserCanBackend);

  QList<QCanBusFrame> newFrames;
  CANAPI_Message_t message = {};
  CANAPI_Timestamp_t timestamp = {};
  CANAPI_Return_t ret;

  while(true)
  {
    ret = q->d_ptr->kc_ptr->ReadMessage (message, KVASERCAN_READ_TIMEOUT);

    if(ret != CCanApi::NoError)
    {
      if(ret != CCanApi::ReceiverEmpty)
      {
        q->setError (errorCodeToString (
                         static_cast<CCanApi::EErrorCodes> (ret)),
                     QCanBusDevice::ReadError);
      }
      break;
    }

    // Ignore status messages
    if(message.sts == 1)
    {
      qCDebug (QT_CANBUS_PLUGINS_KVASERCAN,
               "%s, Ignorning status frame", Q_FUNC_INFO);
      continue;
    }

    const int size = static_cast<int>(message.dlc);
    QCanBusFrame frame (
        message.id,
        QByteArray (reinterpret_cast<const char *> (message.data), size));

    qint64 now = timestampTimer->nsecsElapsed ();
    timestamp.tv_sec = now / Q_UINT64_C (1000000000);
    timestamp.tv_nsec = now - (timestamp.tv_sec * Q_UINT64_C (1000000000));

    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN,
             "timestamp.tv_sec = %ld, timestamp.tv_nsec = %ld, "
             "timestampTimer.nsecsElapsed() = %lld",
             timestamp.tv_sec, timestamp.tv_nsec, now);

    frame.setTimeStamp (QCanBusFrame::TimeStamp::fromMicroSeconds (
        static_cast<qint64> (now / Q_UINT64_C (1000))));
    if(message.xtd)
    {
      frame.setExtendedFrameFormat(true);
    }
    if(message.rtr)
    {
      frame.setFrameType(QCanBusFrame::RemoteRequestFrame);
    }
    else
    {
      frame.setFrameType(QCanBusFrame::DataFrame);
    }

    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Received new frame: [%.4f] %s",
             static_cast<double> (frame.timeStamp ().seconds ())
                 + static_cast<double> (frame.timeStamp ().microSeconds ()
                                        / 1000000.0F),
             qUtf8Printable (frame.toString ()));

    newFrames.append(std::move(frame));
  }

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Received %lld frame(s)",
           newFrames.length ());
  q->enqueueReceivedFrames(newFrames);

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting: %s", Q_FUNC_INFO);
}

bool
KvaserCanBackendPrivate::verifyBitRate(int bitrate)
{
  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Entering: %s", Q_FUNC_INFO);
  Q_Q(KvaserCanBackend);

  QString errorString;

  if(isOpen)
  {
    errorString = "Cannot change bitrate for already opened device";
    qCWarning(QT_CANBUS_PLUGINS_KVASERCAN, "%s", qUtf8Printable(errorString));
    q->setError(errorString, QCanBusDevice::ConfigurationError);
    qCDebug (QT_CANBUS_PLUGINS_KVASERCAN, "Exiting %s, returning false",
             Q_FUNC_INFO);
    return false;
  }

  bool isValidBitrate = false;

  switch(bitrate)
  {
  case 10000:
    isValidBitrate = true;
    break;
  case 20000:
    isValidBitrate = true;
    break;
  case 50000:
    isValidBitrate = true;
    break;
  case 100000:
    isValidBitrate = true;
    break;
  case 125000:
    isValidBitrate = true;
    break;
  case 250000:
    isValidBitrate = true;
    break;
  case 500000:
    isValidBitrate = true;
    break;
  case 800000:
    isValidBitrate = true;
    break;
  case 1000000:
    isValidBitrate = true;
    break;
  default:
    errorString = QString("Unsupported bitrate selected: %1").arg(bitrate);
    qCWarning(QT_CANBUS_PLUGINS_KVASERCAN, "%s", qUtf8Printable(errorString));
    q->setError(errorString, QCanBusDevice::ConfigurationError);
  }

  qCDebug (QT_CANBUS_PLUGINS_KVASERCAN,
           "Exiting: %s, returning %d", Q_FUNC_INFO, isValidBitrate);
  return isValidBitrate;
}

QT_END_NAMESPACE
