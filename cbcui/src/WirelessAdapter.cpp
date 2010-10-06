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

#include "WirelessAdapter.h"
#include <QMutexLocker>
#include <QProcess>
#include <QRegExp>
#include <QDebug>

WirelessAdapter::WirelessAdapter() : m_startscan(false)
{
  start();
}
WirelessAdapter::~WirelessAdapter() { }

WirelessAdapterStatus WirelessAdapter::getStatus() const {
  QMutexLocker locker(&m_mutex);
  WirelessAdapterStatus status = m_status;
  return status;
}

QList<ScanResult> WirelessAdapter::getScanResults() const {
  QMutexLocker locker(&m_mutex);
  QList<ScanResult> scanresults = m_scanresults;
  return scanresults;
}

void WirelessAdapter::startScan() 
{
  m_startscan = true;
}

void WirelessAdapter::startConnect(WirelessConnectionSettings connsettings) 
{
  m_connsettings = connsettings;
  m_startconnect = true;
}

void WirelessAdapter::run() 
{
  QProcess usbnet;
  usbnet.start("insmod /drivers/usbnet.ko");
  usbnet.waitForFinished();

  QProcess asix;
  asix.start("insmod /drivers/asix.ko");
  asix.waitForFinished();
  
  while (true) {    
    QThread::msleep(1000);
    
    QMutexLocker locker(&m_mutex);

    WirelessAdapterStatus oldstatus = m_status;
    updateStatus();
    if (oldstatus != m_status)
      statusChanged();
    
    if (m_status.adapterstate == WirelessAdapterStatus::NOT_UP)
      up();
    
    if (m_status.adapterstate < WirelessAdapterStatus::UP)
      continue;
    
    if (m_startscan || oldstatus.adapterstate != WirelessAdapterStatus::UP) {
      doScan(locker);
      m_startscan = false;
    }
    
    if (m_startconnect) {
      doConnect(locker);
      m_startconnect = false;
    }
  }
}

static const QRegExp essid_regexp("ESSID:\"(.+)\"");

void WirelessAdapter::updateStatus()
{
  m_status = WirelessAdapterStatus();
  static bool wiredconnected;

  QProcess ifconfig;
  QProcess ifconfigwired; // Check wired as well
  ifconfig.start("ifconfig rausb0");
  ifconfig.waitForFinished();
  if (ifconfig.exitCode() != 0) {
    // Check wired as well
    ifconfigwired.start("ifconfig eth0"); // ToDo: will this work if the driver isn't enabled yet?
    ifconfigwired.waitForFinished();
    if (ifconfigwired.exitCode() != 0) {
      m_status.adapterstate = WirelessAdapterStatus::NOT_DETECTED;
      wiredconnected = false;
      return;
    }
  }
  
  if(!wiredconnected && ifconfigwired.exitCode() == 0) {
    QProcess wiredconnect;
    wiredconnect.start("udhcpc -t 5 -n -p /var/run/udhcpc.eth0.pid -i eth0");
    wiredconnect.waitForFinished();
  }

  QString outwired = ifconfigwired.readAllStandardOutput(); // Check wired as well

  QString out = ifconfig.readAllStandardOutput();
  if (!out.contains("UP") && !outwired.contains("UP") ) { // Check wired as well
    m_status.adapterstate = WirelessAdapterStatus::NOT_UP;
    return;
  }
  m_status.adapterstate = WirelessAdapterStatus::UP;
  
  static const QRegExp ip_regexpwired("inet addr:(\\S+)");
  if (ip_regexpwired.indexIn(outwired) != -1) {
    m_status.ip = ip_regexpwired.cap(1);
    wiredconnected = true;
  }
  else {
    wiredconnected = false;
  }

  static const QRegExp mac_regexp("(?:..:){5}..");
  if (wiredconnected && mac_regexp.indexIn(outwired) != -1) // Check wired as well
    m_status.mac = mac_regexp.cap(0);
  else if (mac_regexp.indexIn(out) != -1)
    m_status.mac = mac_regexp.cap(0);
  else
    m_status.mac = "Unknown";

  if(wiredconnected) {
    m_status.ssid = "Wired Ethernet";
    m_status.connectionstate = WirelessAdapterStatus::CONNECTED;
    return;
  }
    
  QProcess iwconfig;
  iwconfig.start("iwconfig rausb0");
  iwconfig.waitForFinished();
  QString iwout = iwconfig.readAllStandardOutput();
  
  if (essid_regexp.indexIn(iwout) != -1)
    m_status.ssid = essid_regexp.cap(1);
  else {
    m_status.connectionstate = WirelessAdapterStatus::NOT_CONNECTED;
    return;
  }
  
  static const QRegExp ip_regexp("inet addr:(\\S+)");
  if (ip_regexp.indexIn(out) != -1)
    m_status.ip = ip_regexp.cap(1);
  else {
    m_status.connectionstate = WirelessAdapterStatus::NOT_CONNECTED;
    return;
  }
  
  m_status.connectionstate = WirelessAdapterStatus::CONNECTED;
  return;
}

void WirelessAdapter::up() 
{
  QProcess::execute("ifconfig rausb0 up");
}

void WirelessAdapter::doScan(QMutexLocker &locker) 
{
  m_status.scanning = true;
  statusChanged();

  QProcess iwlist;
  locker.unlock();
  iwlist.start("iwlist rausb0 scan");
  iwlist.waitForFinished();
  locker.relock();
  QString out = iwlist.readAllStandardOutput();
  
  static const QRegExp scan_regexp("(\\w[\\w\\s]+):\\s?\"?([\\w\\d\\s]*)");
  
  int pos=0;
  m_scanresults.clear();
  
  QStringList lines = out.split("\n");
  for (int i=0; i<lines.size(); i++) {
    if (scan_regexp.indexIn(lines[i]) == -1) { continue; }
    const QString &key = scan_regexp.cap(1);
    const QString &value = scan_regexp.cap(2);
    
    if (key == "ESSID") {
      m_scanresults.append(ScanResult());
      if (value.length() > 0)
        m_scanresults.back().ssid = value;
      else
        m_scanresults.back().ssid = "<Hidden SSID>";
    } else if (key == "Encryption key" && !m_scanresults.isEmpty()) {
      m_scanresults.back().encrypted = (value == "on");
    } else if (key == "Quality" && !m_scanresults.isEmpty()) {
      m_scanresults.back().quality = value.toInt();
    }
    
    pos += scan_regexp.matchedLength();
  }
  
  scanComplete();
  m_status.scanning = false;
  statusChanged();
}

void WirelessAdapter::doConnect(QMutexLocker &locker) {
  m_status.connectionstate = WirelessAdapterStatus::CONNECTING;
  statusChanged();
  
  QProcess::execute("iwpriv rausb0 set TxRate=6");
  QString &ssid = m_connsettings.ssid;
  QProcess::execute("iwpriv rausb0 set \"SSID=" + ssid + "\"");
  
  switch (m_connsettings.encryption) {
    case WirelessConnectionSettings::WEP:
      QProcess::execute("iwpriv rausb0 set AuthMode=WEPAUTO");
      QProcess::execute("iwpriv rausb0 set EncrypType=WEP");
      QProcess::execute("iwpriv rausb0 set Key1=" + m_connsettings.key);
      break;
      
    case WirelessConnectionSettings::WPA:
      QProcess::execute("iwpriv rausb0 set AuthMode=WPAPSK");
      QProcess::execute("iwpriv rausb0 set EncrypType=TKIP");
      QProcess::execute("iwpriv rausb0 set WPAPSK=" + m_connsettings.key);
      break;
      
    case WirelessConnectionSettings::WPA2:
      QProcess::execute("iwpriv rausb0 set AuthMode=WPA2PSK");
      QProcess::execute("iwpriv rausb0 set EncrypType=AES");
      QProcess::execute("iwpriv rausb0 set WPAPSK=" + m_connsettings.key);
      break;
      
    default:
      QProcess::execute("iwpriv rausb0 set AuthMode=OPEN");
      QProcess::execute("iwpriv rausb0 set EncrypType=NONE");
      break;
  }
  
  int i;
  for (i=0;i<50;i++) {
    locker.unlock();
    QThread::msleep(100);
  
    QProcess iwconfig;
    iwconfig.start("iwconfig rausb0");
    iwconfig.waitForFinished();
    locker.relock();
    QString out = iwconfig.readAllStandardOutput();
    
    if (essid_regexp.indexIn(out) != -1 && essid_regexp.cap(1) == ssid) {
      m_status.ssid = ssid;
      doObtainIP(locker);
      return;
    }
  }
  
  m_status.connectionstate = WirelessAdapterStatus::NOT_CONNECTED;
  statusChanged();
}

void WirelessAdapter::doObtainIP(QMutexLocker &locker) {
  m_status.connectionstate = WirelessAdapterStatus::OBTAINING_IP;
  statusChanged();
  
  QProcess udhcpc;
  locker.unlock();
  udhcpc.start("udhcpc -q -f -n -i rausb0");
  udhcpc.waitForFinished();
  locker.relock();
  
  QString out = udhcpc.readAllStandardOutput();
  static const QRegExp lease_regexp("Lease of ((?:\\d{1,3}\\.){3}\\d{1,3}) obtained");
  if (lease_regexp.indexIn(out) == -1) {
    m_status.connectionstate = WirelessAdapterStatus::NOT_CONNECTED;
    statusChanged();
    return;
  }
  
  m_status.connectionstate = WirelessAdapterStatus::CONNECTED;
  m_status.ip = lease_regexp.cap(1);
  statusChanged();
}

WirelessAdapterStatus::WirelessAdapterStatus()
: adapterstate(NOT_DETECTED),
  connectionstate(NOT_CONNECTED),
  scanning(false) { }
  
bool WirelessAdapterStatus::operator==(const WirelessAdapterStatus &other) const {
  if (adapterstate != other.adapterstate || connectionstate != other.connectionstate)
    return false;
    
  if (scanning != other.scanning)
    return false;
    
  if (ssid != other.ssid || ip != other.ip || mac != other.mac)
    return false;
    
  return true;
}

