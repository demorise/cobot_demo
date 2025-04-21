/********************************************************************************
** Form generated from reading UI file 'formpWaMlF.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef FORMPWAMLF_H
#define FORMPWAMLF_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGridLayout *gridLayout;
    QPushButton *goToTargetPushButton;
    QPushButton *goToPlayPosePushButton;
    QPushButton *closeGripperPushButton;
    QPushButton *openGripperPushButton;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(487, 471);
        gridLayout = new QGridLayout(Form);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        goToTargetPushButton = new QPushButton(Form);
        goToTargetPushButton->setObjectName(QString::fromUtf8("goToTargetPushButton"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(goToTargetPushButton->sizePolicy().hasHeightForWidth());
        goToTargetPushButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(goToTargetPushButton, 1, 0, 1, 1);

        goToPlayPosePushButton = new QPushButton(Form);
        goToPlayPosePushButton->setObjectName(QString::fromUtf8("goToPlayPosePushButton"));
        sizePolicy.setHeightForWidth(goToPlayPosePushButton->sizePolicy().hasHeightForWidth());
        goToPlayPosePushButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(goToPlayPosePushButton, 1, 1, 1, 1);

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


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QCoreApplication::translate("Form", "Form", nullptr));
        goToTargetPushButton->setText(QCoreApplication::translate("Form", "Go to Target", nullptr));
        goToPlayPosePushButton->setText(QCoreApplication::translate("Form", "Go to Play Pose", nullptr));
        closeGripperPushButton->setText(QCoreApplication::translate("Form", "Close Gripper", nullptr));
        openGripperPushButton->setText(QCoreApplication::translate("Form", "Open Gripper", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // FORMPWAMLF_H
