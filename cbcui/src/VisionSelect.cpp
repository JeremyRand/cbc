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

#include "VisionSelect.h"

#define VISION_ENABLED_KEY   "Vision_Enabled"

VisionSelect::VisionSelect(QWidget *parent) :
        Page(parent),
        m_tracking(parent, &m_vision.m_colorTracker),
        m_setting(parent, m_vision.m_camera,&m_vision.m_rawCameraView),
        m_settings("/mnt/user/cbc_v2.config",QSettings::NativeFormat)
{
    setupUi(this);

    QObject::connect(ui_trackingButton, SIGNAL(clicked()), &m_tracking, SLOT(raisePage()));
    QObject::connect(ui_settingButton, SIGNAL(clicked()), &m_setting, SLOT(raisePage()));
    QObject::connect(ui_enabledCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enabled_changed(int)));
    
    ui_enabledCheckBox->setChecked(m_settings.value(VISION_ENABLED_KEY, true).toBool());
}

VisionSelect::~VisionSelect()
{
}

void VisionSelect::enabled_changed(int newstate)
{
    if (newstate == Qt::Checked) {
        ui_trackingButton->setEnabled(true);
        ui_settingButton->setEnabled(true);
        m_vision.m_camera->requestContinuousFrames();
    } else {
        ui_trackingButton->setEnabled(false);
        ui_settingButton->setEnabled(false);
        m_vision.m_camera->stopFrames();
    }
    
    m_settings.setValue(VISION_ENABLED_KEY, newstate == Qt::Checked);
    m_settings.sync();
}

