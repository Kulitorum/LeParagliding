/****************************************************************************

    xflr5 v6
    Copyright (C) André Deperrois 
    GNU General Public License v3

*****************************************************************************/

#include "crosscheckbox.h"
#include <xflcore/displayoptions.h>
#include <xflwidgets/wt_globals.h>

CrossCheckBox::CrossCheckBox(QWidget *pParent) : QWidget(pParent)
{
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    m_State = Qt::Unchecked;
}


void CrossCheckBox::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    QColor backcolor    = palette().window().color();
    QColor crosscolor   = palette().windowText().color();
    QColor contourcolor = palette().mid().color();


    drawCheckBox(&painter, m_State, rect(), DisplayOptions::treeFontStruct().height(), false, true, crosscolor, backcolor, contourcolor);

}


QSize CrossCheckBox::minimumSizeHint() const
{
    return sizeHint();
}


QSize CrossCheckBox::sizeHint() const
{
    return QSize(DisplayOptions::treeFontStruct().height()*1.5, DisplayOptions::treeFontStruct().height()*1.5);
}


void CrossCheckBox::mouseReleaseEvent(QMouseEvent *)
{
    if(m_State==Qt::Checked)
    {
        m_State = Qt::Unchecked;
        emit clicked(false);
    }
    else
    {
        m_State = Qt::Checked;
        emit clicked(true);
    }
    update();
}

