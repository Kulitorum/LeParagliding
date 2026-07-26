/****************************************************************************

    XFoilAdvancedDlg Class
    Copyright (C) 2009-2016 André Deperrois 

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************/

#include <QLabel>
#include <xdirect/xdirect.h>
#include "xfoiladvanceddlg.h"


XFoilAdvancedDlg::XFoilAdvancedDlg(QWidget *pParent) : QDialog(pParent)
{
    setWindowTitle(tr("XFoil Settings"));
    setupLayout();

    m_IterLimit = 100;
    m_VAccel = 0.001;
    m_bAutoInitBL = true;
    m_bFullReport = false;
}


void XFoilAdvancedDlg::setupLayout()
{
    QVBoxLayout *pMainLayout = new QVBoxLayout;
    {
        QHBoxLayout *pVAccelBoxLayout = new QHBoxLayout;
        {
            QLabel *plab1 = new QLabel(tr("VAccel"));
            plab1->setAlignment(Qt::AlignRight);

            QString tip("<p>Viscous solution acceleration"
                           "The execution of a viscous case requires the solution of a large "
                           "linear system every Newton iteration.  The coefficient matrix of "
                           "this system is 1/3 full, although most of its entries are very small. "
                           "Substantial savings in CPU time (factor of 4 or more) result when "
                           "these small entries are neglected.  SUBROUTINE BLSOLV which solves the "
                           "large Newton system ignores any off-diagonal element whose magnitude "
                           "is smaller than the variable VACCEL.<br>"
                           "A nonzero VACCEL parameter should in principle degrade the convergence rate "
                           "of the viscous solution and thus result in more Newton iterations, although "
                           "the effect is usually too small to notice.  For very low Reynolds number "
                           "cases (less than 100000), it MAY adversely affect the convergence rate "
                           "or stability, and one should try reducing VACCEL or even setting it "
                           "to zero if all other efforts at convergence are unsuccessful.<br>"
                           "The value of VACCEL has absolutely no effect on the final converged "
                           "viscous solution (if attained).)</p>");

            m_pdeVAccel = new DoubleEdit(0.0, 3);     // jx-mod allow 3 decimals fo vaccel according to init value '0.001'
            m_pdeVAccel->setToolTip(tip);
            pVAccelBoxLayout->addStretch(1);
            pVAccelBoxLayout->addWidget(plab1);
            pVAccelBoxLayout->addWidget(m_pdeVAccel);
        }

        QHBoxLayout *pIterBoxLayout = new QHBoxLayout;
        {
            QLabel *plab2 = new QLabel(tr("Iteration Limit"));
            plab2->setAlignment(Qt::AlignRight);
            m_pieIterLimit = new IntEdit;

            pIterBoxLayout->addStretch(1);
            pIterBoxLayout->addWidget(plab2);
            pIterBoxLayout->addWidget(m_pieIterLimit);
        }

        QHBoxLayout *pErrLayout = new QHBoxLayout;
         {
            QLabel *plabCdError = new QLabel("Discard points with Cd<");
            m_pfeCdError = new DoubleEdit(0.001);
            m_pfeCdError->setToolTip("<p>Operating points with drag coefficient less than this value will be considered to be spurious and will be discarded.<br>"
                                     "Recommendation: 0.001</p>");
            pErrLayout->addStretch(1);
            pErrLayout->addWidget(plabCdError);
            pErrLayout->addWidget(m_pfeCdError);
        }


        m_pchInitBL = new QCheckBox(tr("Re-initialize BLs after an unconverged iteration"));
        m_pchFullReport = new QCheckBox(tr("Show full log report for an XFoil analysis"));
        m_pchKeepErrorsOpen = new QCheckBox(tr("Keep Xfoil interface open if analysis errors"));

        QHBoxLayout *pTimerLayout = new QHBoxLayout;
        {
            QLabel *pTimerLabel = new QLabel(tr("Time interval between graph updates"));
            QLabel *pTimerUnitLabel = new QLabel("ms");
            m_pieTimerInterval = new IntEdit(XDirect::timeUpdateInterval(), this);
            m_pieTimerInterval->setMin(0);
            pTimerLayout->addStretch();
            pTimerLayout->addWidget(pTimerLabel);
            pTimerLayout->addWidget(m_pieTimerInterval);
            pTimerLayout->addWidget(pTimerUnitLabel);
        }

        m_pButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::RestoreDefaults, this);
        {
            connect(m_pButtonBox, SIGNAL(clicked(QAbstractButton*)), SLOT(onButton(QAbstractButton*)));
        }


        pMainLayout->addStretch();
        pMainLayout->addLayout(pVAccelBoxLayout);
        pMainLayout->addLayout(pIterBoxLayout);
        pMainLayout->addLayout(pErrLayout);
        pMainLayout->addWidget(m_pchInitBL);
        pMainLayout->addWidget(m_pchFullReport);
        pMainLayout->addWidget(m_pchKeepErrorsOpen);
        pMainLayout->addLayout(pTimerLayout);
        pMainLayout->addStretch();
        pMainLayout->addWidget(m_pButtonBox);
    }

    setLayout(pMainLayout);
}


void XFoilAdvancedDlg::onButton(QAbstractButton *pButton)
{
    if      (m_pButtonBox->button(QDialogButtonBox::Ok)              == pButton)   accept();
    else if (m_pButtonBox->button(QDialogButtonBox::Cancel)          == pButton)   reject();
    else if (m_pButtonBox->button(QDialogButtonBox::RestoreDefaults) == pButton)   resetDefaults();
}


void XFoilAdvancedDlg::resetDefaults()
{
    m_IterLimit = 100;
    m_VAccel = 0.001;
    XFoilTask::setCdError(1.0e-3);
    m_bAutoInitBL = true;
    m_bFullReport = false;
    XDirect::setKeepOpenOnErrors(true);
    XDirect::setTimeUpdateInterval(100);
    initDialog();
}


void XFoilAdvancedDlg::initDialog()
{
    m_pdeVAccel->setValue(m_VAccel);
    m_pchInitBL->setChecked(m_bAutoInitBL);
    m_pieIterLimit->setValue(m_IterLimit);
    m_pfeCdError->setValue(XFoilTask::CdError());
    m_pchFullReport->setChecked(m_bFullReport);
    m_pchKeepErrorsOpen->setChecked(XDirect::bKeepOpenOnErrors());
    m_pieTimerInterval->setValue(XDirect::timeUpdateInterval());
}


void XFoilAdvancedDlg::keyPressEvent(QKeyEvent *pEvent)
{
    switch (pEvent->key())
    {
        case Qt::Key_Return:
        case Qt::Key_Enter:
        {
            if(!m_pButtonBox->hasFocus())
            {
                m_pButtonBox->setFocus();
                return;
            }
            else
            {
                accept();
                return;
            }
            break;
        }
        case Qt::Key_Escape:
        {
            reject();
            return;
        }
        default:
            pEvent->ignore();
    }
}


void XFoilAdvancedDlg::accept()
{
    m_IterLimit = m_pieIterLimit->value();
    m_VAccel = m_pdeVAccel->value();
    XFoilTask::setCdError(m_pfeCdError->value());

    m_bAutoInitBL = m_pchInitBL->isChecked();
    m_bFullReport = m_pchFullReport->isChecked();
    XDirect::setTimeUpdateInterval(m_pieTimerInterval->value());
    XDirect::setKeepOpenOnErrors(m_pchKeepErrorsOpen->isChecked());
    QDialog::accept();
}


