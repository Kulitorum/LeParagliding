/****************************************************************************

    FoilSelectionDlg Classe
    Copyright (C) André Deperrois

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

#include <QVBoxLayout>
#include <QHeaderView>
#include <QPushButton>

#include "polarselectiondlg.h"
#include <xflobjects/objects2d/objects2d.h>
#include <xflobjects/objects2d/foil.h>
#include <xflobjects/objects2d/polar.h>
#include <xflwidgets/mvc/expandabletreeview.h>
#include <xflwidgets/mvc/objecttreedelegate.h>
#include <xflwidgets/mvc/objecttreeitem.h>
#include <xflwidgets/mvc/objecttreemodel.h>

PolarSelectionDlg::PolarSelectionDlg(QWidget *pParent) : QDialog(pParent)
{
    m_pFoil = nullptr;
    m_pPolar = nullptr;
    setupLayout();
}


void PolarSelectionDlg::setupLayout()
{
    QVBoxLayout *pMainLayout = new QVBoxLayout;
    {
        m_pStruct = new ExpandableTreeView;

        QStringList labels;
        labels << tr("Object")  << "1234567"<< "";


        m_pModel = new ObjectTreeModel(this);
        m_pModel->setHeaderData(0, Qt::Horizontal, "Objects", Qt::DisplayRole);
        m_pModel->setHeaderData(1, Qt::Horizontal, "1234567890123", Qt::EditRole);
        m_pModel->setHeaderData(1, Qt::Horizontal, "1234567890123", Qt::DisplayRole);
        m_pModel->setHeaderData(2, Qt::Horizontal, "123", Qt::DisplayRole);
        m_pModel->setHeaderData(2, Qt::Horizontal, Qt::AlignRight, Qt::TextAlignmentRole);

        m_pStruct->setModel(m_pModel);

        m_pStruct->header()->hide();
        m_pStruct->header()->setStretchLastSection(false);
        m_pStruct->header()->setSectionResizeMode(0, QHeaderView::Stretch);
        m_pStruct->header()->setSectionResizeMode(1, QHeaderView::Fixed);
        m_pStruct->header()->setSectionResizeMode(2, QHeaderView::Fixed);
        int av = m_pStruct->treeFontStruct().averageCharWidth();
    #ifdef Q_OS_WIN
        m_pStruct->header()->resizeSection(1, 11*av);
        m_pStruct->header()->resizeSection(2, 5*av);
    #else
        m_pStruct->header()->resizeSection(1, 7*av);
        m_pStruct->header()->resizeSection(2, 3*av);
    #endif

        m_pDelegate = new ObjectTreeDelegate(this);
        m_pStruct->setItemDelegate(m_pDelegate);

        connect(m_pStruct->selectionModel(), SIGNAL(currentRowChanged(QModelIndex,QModelIndex)), this, SLOT(onCurrentRowChanged(QModelIndex,QModelIndex)));

        m_pButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Discard);
        {
            connect(m_pButtonBox, SIGNAL(clicked(QAbstractButton*)), SLOT(onButton(QAbstractButton*)));
        }
        pMainLayout->addWidget(m_pStruct);
        pMainLayout->addWidget(m_pButtonBox);
    }
    setLayout(pMainLayout);
}


void PolarSelectionDlg::initDialog()
{
    m_pModel->removeRows(0, m_pModel->rowCount());

    ObjectTreeItem *pRootItem = m_pModel->rootItem();

    m_pStruct->selectionModel()->blockSignals(true);

    for(int i=0; i<Objects2d::foilCount(); i++)
    {
        Foil const *pFoil = Objects2d::foilAt(i);
        if(!pFoil) continue;

        LineStyle ls(pFoil->theStyle());
        ObjectTreeItem *pFoilItem = m_pModel->appendRow(pRootItem, pFoil->name(), pFoil->theStyle(), Qt::Unchecked);
        fillPolars(pFoilItem, pFoil);
    }

    m_pStruct->selectionModel()->blockSignals(false);
}


void PolarSelectionDlg::fillPolars(ObjectTreeItem *pFoilItem, Foil const *pFoil)
{
    if(!pFoil || !pFoilItem) return;

    for(int iPolar=0; iPolar<Objects2d::polarCount(); iPolar++)
    {
        Polar *pPolar = Objects2d::polarAt(iPolar);
        if(pPolar && pPolar->foilName().compare(pFoil->name())==0)
        {
            Polar *pPolar = Objects2d::polarAt(iPolar);
            if(!pPolar) continue;
            if(pPolar && pPolar->foilName().compare(pFoil->name())==0)
            {
                LineStyle ls(pPolar->theStyle());
                ls.m_bIsEnabled = true;
                m_pModel->appendRow(pFoilItem, pPolar->name(), ls, Qt::Unchecked);
            }
        }
    }
}


void PolarSelectionDlg::onCurrentRowChanged(QModelIndex index, QModelIndex)
{

    ObjectTreeItem *pSelectedItem = nullptr;

    if(index.column()==0)
    {
        pSelectedItem = m_pModel->itemFromIndex(index);
    }
    else if(index.column()>=1)
    {
        QModelIndex ind = index.sibling(index.row(), 0);
        pSelectedItem = m_pModel->itemFromIndex(ind);
    }

    if(!pSelectedItem) return;

    if(pSelectedItem->level()==1)
    {
        m_pFoil = Objects2d::foil(pSelectedItem->name());
        m_pPolar = nullptr;
    }
    else if(pSelectedItem->level()==2)
    {
        ObjectTreeItem const*pFoilItem = pSelectedItem->parentItem();
        m_pFoil = Objects2d::foil(pFoilItem->name());
        m_pPolar = Objects2d::getPolar(m_pFoil, pSelectedItem->name());
    }
    else
    {
        m_pFoil = nullptr;
        m_pPolar = nullptr;
    }
}


void PolarSelectionDlg::onButton(QAbstractButton *pButton)
{
    if (     m_pButtonBox->button(QDialogButtonBox::Ok)      == pButton)  accept();
    else if (m_pButtonBox->button(QDialogButtonBox::Discard) == pButton)  reject();
}


void PolarSelectionDlg::accept()
{

    QDialog::accept();
}

