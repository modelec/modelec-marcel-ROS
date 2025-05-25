#include <modelec_gui/widget/action_widget.hpp>

namespace ModelecGUI
{
    ActionWidget::ActionWidget(QWidget* parent) : QWidget(parent)
    {
        layout_ = new QHBoxLayout(this);
        setLayout(layout_);

        spinBox_ = new QDoubleSpinBox(this);
        pushButton_ = new QPushButton(this);
        connect(pushButton_, &QPushButton::clicked, this, &ActionWidget::OnButtonClicked);

        layout_->addWidget(spinBox_);
        layout_->addWidget(pushButton_);

        layout_->setStretch(0, 4);
        layout_->setStretch(1, 1);
    }

    ActionWidget::~ActionWidget()
    {
        delete layout_;
        delete spinBox_;
        delete pushButton_;
    }

    void ActionWidget::SetButtonText(const QString& text)
    {
        if (pushButton_)
        {
            pushButton_->setText(text);
        }
    }

    void ActionWidget::SetSpinBoxValue(double value)
    {
        if (spinBox_)
        {
            spinBox_->setValue(value);
        }
    }

    void ActionWidget::SetSpinBoxRange(double min, double max)
    {
        if (spinBox_)
        {
            spinBox_->setRange(min, max);
        }
    }

    void ActionWidget::SetSpinBoxStep(double step)
    {
        if (spinBox_)
        {
            spinBox_->setSingleStep(step);
        }
    }

    double ActionWidget::GetSpinBoxValue()
    {
        return spinBox_ ? spinBox_->value() : 0.0;
    }

    void ActionWidget::OnButtonClicked()
    {
        emit ButtonClicked(GetSpinBoxValue());
    }
}
