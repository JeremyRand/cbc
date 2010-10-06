/**************************************************************************
 *  Copyright 2008,2009 KISS Institute for Practical Robotics             *
 *                                                                        *
 *  This file is part of CBC Firmware.                                    *
 *                                                                        *
 *  CBC Firmware is free software: you can redistribute it and/or modify  *
 *  it under the terms of the GNU General Public License as published by  *
 *  the Free Software Foundation, either version 2 of the License, or     *
 *  (at your option) any later version.                                   *
 *                                                                        *
 *  CBC Firmware is distributed in the hope that it will be useful,       *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *  GNU General Public License for more details.                          *
 *                                                                        *
 *  You should have received a copy of the GNU General Public License     *
 *  along with this copy of CBC Firmware.  Check the LICENSE file         *
 *  in the project root.  If not, see <http://www.gnu.org/licenses/>.     *
 **************************************************************************/

// Author: Matthew Thompson (matthewbot@gmail.com)

#ifndef __WIRELESSADAPTER_H__
#define __WIRELESSADAPTER_H__

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QMetaType>
#include <QString>
#include <QStringList>

struct WirelessAdapterStatus {
  WirelessAdapterStatus();
  
  enum AdapterState {
    NOT_DETECTED,
    NOT_UP,
    UP
  };
  AdapterState adapterstate;
  enum ConnectionState {
    NOT_CONNECTED,
    CONNECTING,
    OBTAINING_IP,
    CONNECTED
  };
  ConnectionState connectionstate;
  
  bool scanning;
  
  QString ssid;
  QString ip;
  QString mac;
  
  bool operator==(const WirelessAdapterStatus &other) const;
  inline bool operator!=(const WirelessAdapterStatus &other) const { return !(*this == other); }
};

struct WirelessConnectionSettings {
	enum EncryptionType {
		OPEN,
		WEP,
		WPA,
		WPA2
	};
	
	QString ssid;
	EncryptionType encryption;
	QString key;
};

struct ScanResult {
  QString ssid;
  bool encrypted;
  int quality;
};

class WirelessAdapter : public QThread {
Q_OBJECT

public:
  WirelessAdapter();
  ~WirelessAdapter();
  
  WirelessAdapterStatus getStatus() const;
  QList<ScanResult> getScanResults() const;
  
public slots:
  void startScan();
  void startConnect(WirelessConnectionSettings connsettings);
  
signals:
  void statusChanged();  
  void scanComplete();
  
private:
  virtual void run();
  void updateStatus();
  void up();
  void doScan(QMutexLocker &locker);
  void doConnect(QMutexLocker &locker);
  void doObtainIP(QMutexLocker &locker);

  bool m_startscan;
  bool m_startconnect;
  WirelessConnectionSettings m_connsettings;

  WirelessAdapterStatus m_status;
  QList<ScanResult> m_scanresults;
  mutable QMutex m_mutex;
};

#endif
