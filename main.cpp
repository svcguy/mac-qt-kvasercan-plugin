/**
 *  File: main.cpp
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

#include <QtSerialBus/qcanbus.h>
#include <QtSerialBus/qcanbusdevice.h>
#include <QtSerialBus/qcanbusfactory.h>

#include <QtCore/qloggingcategory.h>

QT_BEGIN_NAMESPACE

// Warning level messages by default
Q_LOGGING_CATEGORY (QT_CANBUS_PLUGINS_KVASERCAN, "qt.canbus.plugins.kvasercan",
                    QtWarningMsg)

class KvaserCanBusPlugin : public QObject, QCanBusFactory
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QCanBusFactory" FILE "plugin.json")
  Q_INTERFACES(QCanBusFactory)

public:
  QList<QCanBusDeviceInfo>
  availableDevices (QString *errorMessage) const override
  {
    QString errorReason;
    if (Q_UNLIKELY (!KvaserCanBackend::canCreate (&errorReason)))
    {
      return QList<QCanBusDeviceInfo> ();
    }

    if (errorMessage)
    {
      *errorMessage = errorReason;
    }

    return KvaserCanBackend::interfaces();
  }

  QCanBusDevice *
  createDevice (const QString &interfaceName,
                QString *errorMessage) const override
  {
    QString errorReason;
    if (Q_UNLIKELY (!KvaserCanBackend::canCreate (&errorReason)))
    {
      qCWarning (QT_CANBUS_PLUGINS_KVASERCAN, "%s",
                 qUtf8Printable (errorReason));

      if (errorMessage)
      {
        *errorMessage = errorReason;
      }

      return nullptr;
    }

    auto device = new KvaserCanBackend(interfaceName);
    return device;
  }
};

QT_END_NAMESPACE

#include "main.moc"
