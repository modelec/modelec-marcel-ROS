#pragma once

#include <QHBoxLayout>
#include <QPushButton>
#include <qspinbox.h>
#include <QWidget>

namespace ModelecGUI
{
    class ActionWidget : public QWidget
    {
        Q_OBJECT
    public:
        ActionWidget(QWidget *parent = 0);

        ~ActionWidget();

        void SetButtonText(const QString& text);
        void SetSpinBoxValue(double value);
        void SetSpinBoxRange(double min, double max);
        void SetSpinBoxStep(double step);

        double GetSpinBoxValue();

    protected slots:
        void OnButtonClicked();

    signals:
        void ButtonClicked(double value);

    private:

        QHBoxLayout* layout_;
        QDoubleSpinBox* spinBox_;
        QPushButton* pushButton_;
    };
}