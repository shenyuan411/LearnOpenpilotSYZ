/**
 ******************************************************************************
 *
 * @file       outputpage.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @addtogroup
 * @{
 * @addtogroup OutputPage
 * @{
 * @brief
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "outputpagefixedwing.h"
#include "ui_outputpagefixedwing.h"
#include "setupwizard.h"

OutputPageFixedwing::OutputPageFixedwing(SetupWizard *wizard, QWidget *parent) :
    AbstractWizardPage(wizard, parent),

    ui(new Ui::OutputPageFixedwing)
{
    ui->setupUi(this);
}

OutputPageFixedwing::~OutputPageFixedwing()
{
    delete ui;
}

bool OutputPageFixedwing::validatePage()
{
    if (ui->ServoTypeButton->isChecked()) {
        getWizard()->setActuatorType(SetupWizard::SERVO_DIGITAL);
    } else {
        getWizard()->setActuatorType(SetupWizard::SERVO_ANALOG);
    }

    return true;
}
