/********************************************************************************
** Form generated from reading UI file 'formzUFtmF.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef FORMZUFTMF_H
#define FORMZUFTMF_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGridLayout *gridLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QComboBox *predefinedPoseComboBox;
    QPushButton *predefinedPosePushButton;
    QPushButton *goToPlayPosePushButton;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QPushButton *closeGripperPushButton;
    QPushButton *openGripperPushButton;
    QSlider *gripperSlider;
    QPushButton *goToTargetPushButton;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(487, 471);
        gridLayout = new QGridLayout(Form);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox = new QGroupBox(Form);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        predefinedPoseComboBox = new QComboBox(groupBox);
        predefinedPoseComboBox->setObjectName(QString::fromUtf8("predefinedPoseComboBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(predefinedPoseComboBox->sizePolicy().hasHeightForWidth());
        predefinedPoseComboBox->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(predefinedPoseComboBox, 0, 0, 1, 1);

        predefinedPosePushButton = new QPushButton(groupBox);
        predefinedPosePushButton->setObjectName(QString::fromUtf8("predefinedPosePushButton"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(predefinedPosePushButton->sizePolicy().hasHeightForWidth());
        predefinedPosePushButton->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(predefinedPosePushButton, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox, 4, 0, 1, 2);

        goToPlayPosePushButton = new QPushButton(Form);
        goToPlayPosePushButton->setObjectName(QString::fromUtf8("goToPlayPosePushButton"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(1);
        sizePolicy3.setHeightForWidth(goToPlayPosePushButton->sizePolicy().hasHeightForWidth());
        goToPlayPosePushButton->setSizePolicy(sizePolicy3);

        gridLayout->addWidget(goToPlayPosePushButton, 3, 1, 1, 1);

        groupBox_2 = new QGroupBox(Form);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        sizePolicy.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        closeGripperPushButton = new QPushButton(groupBox_2);
        closeGripperPushButton->setObjectName(QString::fromUtf8("closeGripperPushButton"));
        sizePolicy3.setHeightForWidth(closeGripperPushButton->sizePolicy().hasHeightForWidth());
        closeGripperPushButton->setSizePolicy(sizePolicy3);

        gridLayout_3->addWidget(closeGripperPushButton, 0, 0, 1, 1);

        openGripperPushButton = new QPushButton(groupBox_2);
        openGripperPushButton->setObjectName(QString::fromUtf8("openGripperPushButton"));
        sizePolicy3.setHeightForWidth(openGripperPushButton->sizePolicy().hasHeightForWidth());
        openGripperPushButton->setSizePolicy(sizePolicy3);

        gridLayout_3->addWidget(openGripperPushButton, 0, 1, 1, 1);

        gripperSlider = new QSlider(groupBox_2);
        gripperSlider->setObjectName(QString::fromUtf8("gripperSlider"));
        gripperSlider->setMaximum(100);
        gripperSlider->setSingleStep(1);
        gripperSlider->setValue(100);
        gripperSlider->setSliderPosition(100);
        gripperSlider->setTracking(false);
        gripperSlider->setOrientation(Qt::Horizontal);
        gripperSlider->setTickPosition(QSlider::TicksBelow);
        gripperSlider->setTickInterval(5);

        gridLayout_3->addWidget(gripperSlider, 1, 0, 1, 2);


        gridLayout->addWidget(groupBox_2, 1, 0, 1, 2);

        goToTargetPushButton = new QPushButton(Form);
        goToTargetPushButton->setObjectName(QString::fromUtf8("goToTargetPushButton"));
        sizePolicy3.setHeightForWidth(goToTargetPushButton->sizePolicy().hasHeightForWidth());
        goToTargetPushButton->setSizePolicy(sizePolicy3);

        gridLayout->addWidget(goToTargetPushButton, 3, 0, 1, 1);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QCoreApplication::translate("Form", "Form", nullptr));
        groupBox->setTitle(QCoreApplication::translate("Form", "Predefined Pose", nullptr));
        predefinedPosePushButton->setText(QCoreApplication::translate("Form", "Go", nullptr));
        goToPlayPosePushButton->setText(QCoreApplication::translate("Form", "Go to Play Pose", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("Form", "GroupBox", nullptr));
        closeGripperPushButton->setText(QCoreApplication::translate("Form", "Close Gripper", nullptr));
        openGripperPushButton->setText(QCoreApplication::translate("Form", "Open Gripper", nullptr));
        goToTargetPushButton->setText(QCoreApplication::translate("Form", "Go to Target", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // FORMZUFTMF_H
