/********************************************************************************
** Form generated from reading UI file 'formponphQ.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef FORMPONPHQ_H
#define FORMPONPHQ_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGridLayout *gridLayout;
    QGroupBox *groupBox_5;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QPushButton *closeGripperPushButton;
    QPushButton *openGripperPushButton;
    QSlider *gripperSlider;
    QPushButton *goToPlayPosePushButton;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *raiseHeadPushButton;
    QPushButton *nodHeadPushButton;
    QPushButton *lowerHeadPushButton;
    QSlider *headSlider;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QComboBox *predefinedPoseComboBox;
    QPushButton *predefinedPosePushButton;
    QPushButton *goToTargetPushButton;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(487, 471);
        gridLayout = new QGridLayout(Form);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox_5 = new QGroupBox(Form);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_5->sizePolicy().hasHeightForWidth());
        groupBox_5->setSizePolicy(sizePolicy);

        gridLayout->addWidget(groupBox_5, 5, 1, 1, 1);

        groupBox_2 = new QGroupBox(Form);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(1);
        sizePolicy1.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy1);
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        closeGripperPushButton = new QPushButton(groupBox_2);
        closeGripperPushButton->setObjectName(QString::fromUtf8("closeGripperPushButton"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(1);
        sizePolicy2.setHeightForWidth(closeGripperPushButton->sizePolicy().hasHeightForWidth());
        closeGripperPushButton->setSizePolicy(sizePolicy2);

        gridLayout_3->addWidget(closeGripperPushButton, 0, 0, 1, 1);

        openGripperPushButton = new QPushButton(groupBox_2);
        openGripperPushButton->setObjectName(QString::fromUtf8("openGripperPushButton"));
        sizePolicy2.setHeightForWidth(openGripperPushButton->sizePolicy().hasHeightForWidth());
        openGripperPushButton->setSizePolicy(sizePolicy2);

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

        goToPlayPosePushButton = new QPushButton(Form);
        goToPlayPosePushButton->setObjectName(QString::fromUtf8("goToPlayPosePushButton"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(1);
        sizePolicy3.setVerticalStretch(1);
        sizePolicy3.setHeightForWidth(goToPlayPosePushButton->sizePolicy().hasHeightForWidth());
        goToPlayPosePushButton->setSizePolicy(sizePolicy3);

        gridLayout->addWidget(goToPlayPosePushButton, 3, 1, 1, 1);

        groupBox_3 = new QGroupBox(Form);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        sizePolicy3.setHeightForWidth(groupBox_3->sizePolicy().hasHeightForWidth());
        groupBox_3->setSizePolicy(sizePolicy3);
        horizontalLayout = new QHBoxLayout(groupBox_3);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        raiseHeadPushButton = new QPushButton(groupBox_3);
        raiseHeadPushButton->setObjectName(QString::fromUtf8("raiseHeadPushButton"));

        verticalLayout->addWidget(raiseHeadPushButton);

        nodHeadPushButton = new QPushButton(groupBox_3);
        nodHeadPushButton->setObjectName(QString::fromUtf8("nodHeadPushButton"));

        verticalLayout->addWidget(nodHeadPushButton);

        lowerHeadPushButton = new QPushButton(groupBox_3);
        lowerHeadPushButton->setObjectName(QString::fromUtf8("lowerHeadPushButton"));

        verticalLayout->addWidget(lowerHeadPushButton);


        horizontalLayout->addLayout(verticalLayout);

        headSlider = new QSlider(groupBox_3);
        headSlider->setObjectName(QString::fromUtf8("headSlider"));
        headSlider->setMinimum(-100);
        headSlider->setMaximum(100);
        headSlider->setTracking(false);
        headSlider->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(headSlider);


        gridLayout->addWidget(groupBox_3, 5, 0, 1, 1);

        groupBox = new QGroupBox(Form);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy1);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        predefinedPoseComboBox = new QComboBox(groupBox);
        predefinedPoseComboBox->setObjectName(QString::fromUtf8("predefinedPoseComboBox"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(predefinedPoseComboBox->sizePolicy().hasHeightForWidth());
        predefinedPoseComboBox->setSizePolicy(sizePolicy4);

        gridLayout_2->addWidget(predefinedPoseComboBox, 0, 0, 1, 1);

        predefinedPosePushButton = new QPushButton(groupBox);
        predefinedPosePushButton->setObjectName(QString::fromUtf8("predefinedPosePushButton"));
        QSizePolicy sizePolicy5(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(predefinedPosePushButton->sizePolicy().hasHeightForWidth());
        predefinedPosePushButton->setSizePolicy(sizePolicy5);

        gridLayout_2->addWidget(predefinedPosePushButton, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox, 4, 0, 1, 2);

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
        groupBox_5->setTitle(QCoreApplication::translate("Form", "Neck", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("Form", "Gripper", nullptr));
        closeGripperPushButton->setText(QCoreApplication::translate("Form", "Close Gripper", nullptr));
        openGripperPushButton->setText(QCoreApplication::translate("Form", "Open Gripper", nullptr));
        goToPlayPosePushButton->setText(QCoreApplication::translate("Form", "Go to Play Pose", nullptr));
        groupBox_3->setTitle(QCoreApplication::translate("Form", "Head", nullptr));
        raiseHeadPushButton->setText(QCoreApplication::translate("Form", "Raise", nullptr));
        nodHeadPushButton->setText(QCoreApplication::translate("Form", "Nod", nullptr));
        lowerHeadPushButton->setText(QCoreApplication::translate("Form", "Lower", nullptr));
        groupBox->setTitle(QCoreApplication::translate("Form", "Predefined Pose", nullptr));
        predefinedPosePushButton->setText(QCoreApplication::translate("Form", "Go", nullptr));
        goToTargetPushButton->setText(QCoreApplication::translate("Form", "Go to Target", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // FORMPONPHQ_H
