/****************************************************************************

    FoilCoordDlg Class
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


#pragma once

#include <QDialog>
#include <QPushButton>
#include <QStandardItemModel>
#include <QDialogButtonBox>

class CPTableView;
class FloatEditDelegate;


class QSpecEditDlg : public QDialog
{
    friend class XInverse;

    Q_OBJECT
    public:
        QSpecEditDlg();

        void initDialog(const QVector<double> &x, const QVector<double> &y);
        void readData(QVector<double> &X, QVector<double> &Y);

    private:
        void setupLayout();
        void showEvent(QShowEvent *pEvent) override;
        void hideEvent(QHideEvent *pEvent) override;

    private slots:
        void onButton(QAbstractButton *pButton);

    private:
        QDialogButtonBox *m_pButtonBox;

        CPTableView *m_ptvCoordTable;
        QStandardItemModel *m_pCoordModel;
        FloatEditDelegate *m_pFloatDelegate;

        static QByteArray s_Geometry;
};


