/**
 *  File: kvasercanbackend.h
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

#ifndef KVASERCANBACKEND_H
#define KVASERCANBACKEND_H

#include <QtSerialBus/qcanbusdevice.h>
#include <QtSerialBus/qcanbusdeviceinfo.h>
#include <QtSerialBus/qcanbusframe.h>

QT_BEGIN_NAMESPACE

class KvaserCanBackendPrivate;

class KvaserCanBackend : public QCanBusDevice
{
  Q_OBJECT
  Q_DECLARE_PRIVATE(KvaserCanBackend)
  Q_DISABLE_COPY(KvaserCanBackend)

public:
  explicit KvaserCanBackend(const QString &name, QObject *parent = nullptr);
  ~KvaserCanBackend();

  bool open() override;
  void close() override;

  void setConfigurationParameter(ConfigurationKey key, const QVariant &value) override;

  bool writeFrame(const QCanBusFrame &newData) override;

  QString interpretErrorFrame(const QCanBusFrame &errorFrame) override;

  static bool canCreate(QString *errorReason);
  static QList<QCanBusDeviceInfo> interfaces();

  void resetController() override;
  bool hasBusStatus() const override;
  CanBusStatus busStatus() override;
  QCanBusDeviceInfo deviceInfo() const override;

private:
  enum class Availability
  {
    Available = 1U,
    Occupied  = 2U
  };
  static QList<QCanBusDeviceInfo> interfacesByChannelCondition(Availability available);
  static QList<QCanBusDeviceInfo> interfacesByAttachedChannels(Availability available, bool *ok);
  static QList<QCanBusDeviceInfo> attachedInterfaces(Availability available);

  KvaserCanBackendPrivate * const d_ptr;
};

QT_END_NAMESPACE

#endif // KVASERCANBACKEND_H
