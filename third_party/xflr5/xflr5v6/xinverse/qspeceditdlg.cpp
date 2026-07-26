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

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStringList>
#include <QHeaderView>

#include "qspeceditdlg.h"
#include <xflcore/displayoptions.h>
#include <xflwidgets/customwts/cptableview.h>
#include <xflwidgets/customwts/floateditdelegate.h>

QByteArray QSpecEditDlg::s_Geometry;


QSpecEditDlg::QSpecEditDlg()
{
    setWindowTitle("QSpec edition");
    setupLayout();
}


void QSpecEditDlg::setupLayout()
{
    m_ptvCoordTable = new CPTableView(this);
    {
        m_ptvCoordTable->setEditable(true);
        m_ptvCoordTable->setFont(DisplayOptions::tableFont());

        m_pCoordModel = new QStandardItemModel(this);
        m_pCoordModel->setRowCount(10);//temporary
        m_pCoordModel->setColumnCount(2);
        m_pCoordModel->setHeaderData(0, Qt::Horizontal, "X");
        m_pCoordModel->setHeaderData(1, Qt::Horizontal, "Y");

        m_ptvCoordTable->setModel(m_pCoordModel);

        m_pFloatDelegate = new FloatEditDelegate(this);
        m_ptvCoordTable->setItemDelegate(m_pFloatDelegate);

        QVector<int> precision = {5,5};
        m_pFloatDelegate->setPrecision(precision);

        connect(m_pFloatDelegate, SIGNAL(closeEditor(QWidget*)), SLOT(onCellChanged(QWidget*)));

        QItemSelectionModel *pSelectionModel = new QItemSelectionModel(m_pCoordModel);
        m_ptvCoordTable->setSelectionModel(pSelectionModel);
        connect(pSelectionModel, SIGNAL(currentChanged(QModelIndex,QModelIndex)), this, SLOT(onItemClicked(QModelIndex)));
    }

    m_pButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    {
        connect(m_pButtonBox, SIGNAL(clicked(QAbstractButton*)), SLOT(onButton(QAbstractButton*)));
    }

    QVBoxLayout * pMainLayout = new QVBoxLayout(this);
    {
        pMainLayout->addWidget(m_ptvCoordTable);
        pMainLayout->addWidget(m_pButtonBox);
    }
    setLayout(pMainLayout);
}


void QSpecEditDlg::onButton(QAbstractButton *pButton)
{
    if (     m_pButtonBox->button(QDialogButtonBox::Ok)     == pButton)  accept();
    else if (m_pButtonBox->button(QDialogButtonBox::Cancel) == pButton)  reject();
}


void QSpecEditDlg::showEvent(QShowEvent *pEvent)
{
    QDialog::showEvent(pEvent);
    restoreGeometry(s_Geometry);
}


void QSpecEditDlg::hideEvent(QHideEvent *pEvent)
{
    QDialog::hideEvent(pEvent);
    s_Geometry = saveGeometry();
}


void QSpecEditDlg::initDialog(QVector<double> const &x, QVector<double> const &y)
{
    m_pCoordModel->setRowCount(x.size());
    m_pCoordModel->setColumnCount(2);
    for (int i=0; i<x.size(); i++)
    {
        QModelIndex Xindex = m_pCoordModel->index(i, 0, QModelIndex());
        m_pCoordModel->setData(Xindex, x.at(i));

        QModelIndex Yindex =m_pCoordModel->index(i, 1, QModelIndex());
        m_pCoordModel->setData(Yindex, y.at(i));
    }
}


void QSpecEditDlg::readData(QVector<double> &X, QVector<double> &Y)
{
    Q_ASSERT(X.size()==m_pCoordModel->rowCount());
    Q_ASSERT(Y.size()==m_pCoordModel->rowCount());

    for(int row=0; row<m_pCoordModel->rowCount(); row++)
    {
        QModelIndex XIndex =m_pCoordModel->index(row, 0, QModelIndex());
        if(XIndex.isValid())
            X[row] = XIndex.data().toDouble();
        QModelIndex YIndex =m_pCoordModel->index(row, 1, QModelIndex());
        if(YIndex.isValid())
            Y[row] = YIndex.data().toDouble();
    }
}



