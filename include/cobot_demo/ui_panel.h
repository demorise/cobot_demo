/********************************************************************************
** Form generated from reading UI file 'formVjXzza.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef FORMVJXZZA_H
#define FORMVJXZZA_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGridLayout *gridLayout;
    QPushButton *goToPlayPosePushButton;
    QPushButton *goToTargetPushButton;
    QPushButton *closeGripperPushButton;
    QPushButton *openGripperPushButton;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QComboBox *predefinedPoseComboBox;
    QPushButton *predefinedPosePushButton;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(487, 471);
        gridLayout = new QGridLayout(Form);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        goToPlayPosePushButton = new QPushButton(Form);
        goToPlayPosePushButton->setObjectName(QString::fromUtf8("goToPlayPosePushButton"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(goToPlayPosePushButton->sizePolicy().hasHeightForWidth());
        goToPlayPosePushButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(goToPlayPosePushButton, 1, 1, 1, 1);

        goToTargetPushButton = new QPushButton(Form);
        goToTargetPushButton->setObjectName(QString::fromUtf8("goToTargetPushButton"));
        sizePolicy.setHeightForWidth(goToTargetPushButton->sizePolicy().hasHeightForWidth());
        goToTargetPushButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(goToTargetPushButton, 1, 0, 1, 1);

        closeGripperPushButton = new QPushButton(Form);
        closeGripperPushButton->setObjectName(QString::fromUtf8("closeGripperPushButton"));
        sizePolicy.setHeightForWidth(closeGripperPushButton->sizePolicy().hasHeightForWidth());
        closeGripperPushButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(closeGripperPushButton, 0, 1, 1, 1);

        openGripperPushButton = new QPushButton(Form);
        openGripperPushButton->setObjectName(QString::fromUtf8("openGripperPushButton"));
        sizePolicy.setHeightForWidth(openGripperPushButton->sizePolicy().hasHeightForWidth());
        openGripperPushButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(openGripperPushButton, 0, 0, 1, 1);

        groupBox = new QGroupBox(Form);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(1);
        sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy1);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        predefinedPoseComboBox = new QComboBox(groupBox);
        predefinedPoseComboBox->setObjectName(QString::fromUtf8("predefinedPoseComboBox"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(predefinedPoseComboBox->sizePolicy().hasHeightForWidth());
        predefinedPoseComboBox->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(predefinedPoseComboBox, 0, 0, 1, 1);

        predefinedPosePushButton = new QPushButton(groupBox);
        predefinedPosePushButton->setObjectName(QString::fromUtf8("predefinedPosePushButton"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(predefinedPosePushButton->sizePolicy().hasHeightForWidth());
        predefinedPosePushButton->setSizePolicy(sizePolicy3);

        gridLayout_2->addWidget(predefinedPosePushButton, 0, 1, 1, 1);


        gridLayout->addWidget(groupBox, 2, 0, 1, 2);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QCoreApplication::translate("Form", "Form", nullptr));
        goToPlayPosePushButton->setText(QCoreApplication::translate("Form", "Go to Play Pose", nullptr));
        goToTargetPushButton->setText(QCoreApplication::translate("Form", "Go to Target", nullptr));
        closeGripperPushButton->setText(QCoreApplication::translate("Form", "Close Gripper", nullptr));
        openGripperPushButton->setText(QCoreApplication::translate("Form", "Open Gripper", nullptr));
        groupBox->setTitle(QCoreApplication::translate("Form", "Predefined Pose", nullptr));
        predefinedPosePushButton->setText(QCoreApplication::translate("Form", "Go", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // FORMVJXZZA_H
