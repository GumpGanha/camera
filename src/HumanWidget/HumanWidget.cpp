#include "HumanWidget.h"

#include "ui_HumanWidget.h"
#include<QString>
#include <QMessageBox>
#include <QFileDialog>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <QMatrix>
#include <QTextStream>
#include <QDebug>

//#pragma execution_character_set("utf-8")
using std::ifstream;

HumanWidget::HumanWidget(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::HumanWidget)
{

    ui->setupUi(this);
    m_StereoVisionSystem = boost::make_shared<StereoVisionSystem>();


    connect(ui->startButton, &QPushButton::clicked, this, &HumanWidget::on_startButton_clicked);
    connect(ui->stopButton, &QPushButton::clicked, this, &HumanWidget::on_stopButton_clicked);
    connect(ui->closeButton, &QPushButton::clicked, this, &HumanWidget::on_closeButton_clicked);


    connect(&timer, &QTimer::timeout, this, &HumanWidget::updateDisplay);


}

HumanWidget::~HumanWidget()
{
    delete ui;
}
void HumanWidget::on_startButton_clicked() {
    if (!isRunning) {
       m_StereoVisionSystem->initializeCamera();
        timer.start();
        isRunning = true;
        ui->state->setText("The state of camera:open");
     
    }
    timer.setInterval(100); // 100 ms
}


void HumanWidget::on_stopButton_clicked() {
    if (isRunning) {
        timer.stop();
        isRunning = false;
    
    }
    ui->state->setText("The state of camera:stop");
}

void HumanWidget::on_closeButton_clicked() {
    if (isRunning) {
        timer.stop();
        isRunning = false;
    }
    m_StereoVisionSystem->close();
    this->close();
}

void HumanWidget::updateDisplay() {
    SendMat result = m_StereoVisionSystem->pointcapture();
    QString objectText = matToString(result.object2camera);
    QString glassesText = matToString(result.glasses2camera);

    qDebug()<< "objectText" << endl << objectText << endl;

    ui->object_2->setText(objectText);
    ui->glasses->setText(glassesText);
}



QString HumanWidget::matToString(const cv::Mat& mat) {
    QString result;
    QTextStream stream(&result);
    stream.setFieldAlignment(QTextStream::AlignLeft);
    stream.setPadChar(' ');

    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            uchar value = mat.at<uchar>(i, j);
            stream << value<< ' ';  
        }
        stream << '\n';
    }
    return result;
}