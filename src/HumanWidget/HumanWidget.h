#ifndef HUMANWIDGET_H
#define HUMANWIDGET_H


#include  "ImageCapture.h"
#include    "SendMat.h"

#include    "ui_HumanWidget.h"
#include <QTimer>
#include <QWidget>
#include <iostream>
#include <QLabel>

using std::cout;
using std::endl;


using std::vector;



QT_BEGIN_NAMESPACE
namespace Ui {
    class HumanWidgetUI;
}
QT_END_NAMESPACE

class HumanWidget : public QWidget
{
    Q_OBJECT

public:
   explicit HumanWidget(QWidget* parent = nullptr);
    ~HumanWidget();

private slots:
    void on_startButton_clicked();
    void on_stopButton_clicked();
    void on_closeButton_clicked();
    void updateDisplay();


private:

    Ui::HumanWidget*  ui;

    QTimer timer;
    bool isRunning = false;


    SendMat  m_SendMat;
    StereoVisionSystem_spointer  m_StereoVisionSystem;
   
    QString matToString(const cv::Mat& mat);



};
#endif // WIDGET_H