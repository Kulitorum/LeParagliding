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


#pragma once

#include <QDialog>
#include <QDialogButtonBox>
#include <QModelIndex>

class Foil;
class Polar;
class ExpandableTreeView;
class ObjectTreeDelegate;
class ObjectTreeModel;
class ObjectTreeItem;

class PolarSelectionDlg : public  QDialog
{
    Q_OBJECT
    public:
        PolarSelectionDlg(QWidget *pParent);

        void initDialog();

        Foil const *selectedFoil() const {return m_pFoil;}
        Polar const *selectedPolar() const {return m_pPolar;}

    private:
        void setupLayout();
        void filltreeView();
        void fillPolars(ObjectTreeItem *pFoilItem, Foil const *pFoil);

        void accept() override;
        QSize sizeHint() const override {return QSize(550,900);}

    private slots:
        void onCurrentRowChanged(QModelIndex currentIndex, QModelIndex previousIndex);
        void onButton(QAbstractButton *pButton);

    private:
        ExpandableTreeView *m_pStruct;
        ObjectTreeModel *m_pModel;
        ObjectTreeDelegate *m_pDelegate;

        QDialogButtonBox *m_pButtonBox;

        Foil *m_pFoil;
        Polar *m_pPolar;
};


