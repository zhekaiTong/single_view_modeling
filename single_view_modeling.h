#ifndef SINGLE_VIEW_MODELING_H
#define SINGLE_VIEW_MODELING_H

#include <stdio.h>
#include <QMainWindow>
#include <QFileDialog>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include <vector>
#include <QPainter>
#include <QMouseEvent>
#include <QInputDialog>
#include <eigen3/Eigen/Dense>

#include <QMainWindow>

using namespace cv;
using namespace std;

namespace Ui {
class single_view_modeling;
}

class single_view_modeling : public QMainWindow
{
    Q_OBJECT

public:
    explicit single_view_modeling(QWidget *parent = 0);
    ~single_view_modeling();

private slots:
    void on_actionOpen_triggered();

    void show_image(Mat const& src, int n);

    bool eventFilter(QObject *obj, QEvent *event);

    void compute_vanish(vector<cv::Point2d> point);

    //void input2vec(string str);

    void computeRefX(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> r);

    void computeRefY(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> r);

    void computeRefZ(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> r);

    double compute3DX(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> tb);

    double compute3DY(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> tb);

    double compute3DZ(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> tb);

    Eigen::Matrix3d computeHmat(vector<cv::Point2d> img_uv, vector<cv::Point2d> real_xy, int plane);

    void textureMap(vector<cv::Point2d> real_xy, Eigen::Matrix3d H, int pl);

    void on_XButton_clicked();

    void on_YButton_clicked();

    void on_ZButton_clicked();

    void on_XYZButton_clicked();

    void on_refXButton_clicked();

    void on_refYButton_clicked();

    void on_refZButton_clicked();

private:
    Ui::single_view_modeling *ui;

    Mat src, img;
    vector<cv::Point2d> para_points, origin, img_uv, real_xy;
    vector<cv::Point3d> van_p, recon3D;
    Eigen::Matrix3d H1, H2, H3;
    double sx, sy, cx, cy, cz;
    int plane;
    double ax, ay, az;
};

#endif // SINGLE_VIEW_MODELING_H
