/**
 *  File: kvasercanbackend_p.h
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

#ifndef KVASERCANBACKEND_P_H
#define KVASERCANBACKEND_P_H

#include "kvasercanbackend.h"

#include "KvaserCAN.h"

QT_BEGIN_NAMESPACE

class QSocketNotifier;
class QWinEventNotifier;
class QTimer;
class QElapsedTimer;

class KvaserCanBackendPrivate
{
  Q_DECLARE_PUBLIC(KvaserCanBackend)

public:
  KvaserCanBackendPrivate(KvaserCanBackend *q);

  bool open();
  void close();

  bool setConfigurationParameter(QCanBusDevice::ConfigurationKey key, const QVariant &value);

  void setupChannel(const QByteArray &interfaceName);
  void setupDefaultConfigurations();

  QString systemErrorString(qint32 errorCode);

  void startWrite();
  void startRead();

  bool verifyBitRate(int bitrate);

  KvaserCanBackend * const q_ptr;

  bool isFlexibleDatarateEnabled = false;
  bool isOpen = false;
  quint16 channelIndex = 0;
  QTimer *writeNotifier = nullptr;

  QSocketNotifier *readNotifier = nullptr;
  int readHandle = -1;

  QElapsedTimer *timestampTimer;

  CKvaserCAN * const kc_ptr;
};

#endif // KVASERCANBACKEND_P_H
