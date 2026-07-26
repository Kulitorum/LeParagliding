/****************************************************************************

    xflr5 v6
    Copyright (C) André Deperrois 
    GNU General Public License v3

*****************************************************************************/
#pragma once

#include <QCheckBox>

class CrossCheckBox : public QWidget
{
    Q_OBJECT

    public:
        CrossCheckBox(QWidget *pParent = nullptr);

    signals:
        void clicked(bool);

    public:
        QSize sizeHint() const override;
        QSize minimumSizeHint() const override;
        void paintEvent(QPaintEvent *) override;
        void mouseReleaseEvent(QMouseEvent *pEvent) override;

        void setCheckState(Qt::CheckState state) {m_State=state;}
        Qt::CheckState checkState() const {return m_State;}

    private:
        Qt::CheckState m_State;
};

