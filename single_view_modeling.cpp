#include "single_view_modeling.h"
#include "ui_single_view_modeling.h"

using namespace cv;
using namespace std;
using namespace Eigen;

single_view_modeling::single_view_modeling(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::single_view_modeling)
{
    ui->setupUi(this);
    qApp->installEventFilter(this);
    this->setMouseTracking(true);
}

single_view_modeling::~single_view_modeling()
{
    delete ui;
}

QImage Mat2QImage(cv::Mat const& src)
{
    cv::Mat temp;
    cvtColor(src, temp,CV_BGR2RGB);
    QImage q_image((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    q_image.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return q_image;
}

void single_view_modeling::show_image(Mat const& src, int n)
{
    //imshow( "image", src );

    QPixmap contour_pixmap = QPixmap::fromImage(Mat2QImage( src ));

    int w = contour_pixmap.width();
    int h = contour_pixmap.height();
    ui->imgLabel->resize(w,h);
    ui->imgLabel->setPixmap(contour_pixmap);

    //ui->scrollArea->show();
}

/*void single_view_modeling::input2vec(string str)
{
    char *s_input = (char *)str.c_str();
    const char * split = ",";
    char *p = strtok(s_input, split);
    int a;
    vector<int> nums;
    while(p != NULL)
    {
        // char * -> int
        sscanf(p, "%d", &a);
        nums.push_back(a);
        p=strtok(NULL, split);
    }
    Point3d pt(nums[0], nums[1], nums[2]);
    real_xy.push_back(pt);

   cout << "Your select points are: " << real_xy << endl;
}*/

void single_view_modeling::compute_vanish(vector<cv::Point2d> point)
{
    Point3d e1(point[point.size()-6].x, point[point.size()-6].y, 1);
    Point3d e2(point[point.size()-5].x, point[point.size()-5].y, 1);
    Point3d e3(point[point.size()-4].x, point[point.size()-4].y, 1);
    Point3d e4(point[point.size()-3].x, point[point.size()-3].y, 1);
    Point3d e5(point[point.size()-2].x, point[point.size()-2].y, 1);
    Point3d e6(point[point.size()-1].x, point[point.size()-1].y, 1);
    Point3d line1(0, 0, 0);
    Point3d line2(0, 0, 0);
    Point3d line3(0, 0, 0);

    line1 = e1.cross(e2);
    line2 = e3.cross(e4);
    line3 = e5.cross(e6);
    cout << "line1: " << line1 << endl;
    cout << "line2: " << line2 << endl;
    cout << "line3: " << line3 << endl;
//     Mat M1 = (Mat_<double>(3,3) << line1.x*line1.x, line1.x*line1.y, line1.x*line1.z,\
                                   line1.x*line1.y, line1.y*line1.y, line1.y*line1.z,\
                                   line1.x*line1.z, line1.y*line1.z, line1.z*line1.z);
    Matrix3d M1;
    M1 << line1.x*line1.x, line1.x*line1.y, line1.x*line1.z,
          line1.x*line1.y, line1.y*line1.y, line1.y*line1.z,
          line1.x*line1.z, line1.y*line1.z, line1.z*line1.z;
    cout << "M1 = " << endl << " " << M1 << endl << endl;
//     Mat M2 = (Mat_<double>(3,3) << line2.x*line2.x, line2.x*line2.y, line2.x*line2.z,\
                                   line2.x*line2.y, line2.y*line2.y, line2.y*line2.z,\
                                   line2.x*line2.z, line2.y*line2.z, line2.z*line2.z);
//     Mat M3 = (Mat_<double>(3,3) << line3.x*line3.x, line3.x*line3.y, line3.x*line3.z,\
                                   line3.x*line3.y, line3.y*line3.y, line3.y*line3.z,\
                                   line3.x*line3.z, line3.y*line3.z, line3.z*line3.z);
    Matrix3d M2;
    M2 << line2.x*line2.x, line2.x*line2.y, line2.x*line2.z,
          line2.x*line2.y, line2.y*line2.y, line2.y*line2.z,
          line2.x*line2.z, line2.y*line2.z, line2.z*line2.z;
    cout << "M2 = " << endl << " " << M2 << endl << endl;
    Matrix3d M3;
    M3 << line3.x*line3.x, line3.x*line3.y, line3.x*line3.z,
          line3.x*line3.y, line3.y*line3.y, line3.y*line3.z,
          line3.x*line3.z, line3.y*line3.z, line3.z*line3.z;
    cout << "M3 = " << endl << " " << M3 << endl << endl;
    Matrix3d M = M1 + M2 + M3;

    SelfAdjointEigenSolver<Matrix3d> eigensolver(M);
    if (eigensolver.info() != Success) abort();
    cout << "The eigenvalues of M are:\n" << eigensolver.eigenvalues() << endl;
    cout << "Here's a matrix whose columns are eigenvectors of M \n"
         << "corresponding to these eigenvalues:\n"
         << eigensolver.eigenvectors() << endl;
    Matrix3d Vec_M = eigensolver.eigenvectors();
    Point3d minEigVec(Vec_M.col(0)[0], Vec_M.col(0)[1], Vec_M.col(0)[2]);
    minEigVec = minEigVec / minEigVec.z;
    cout << "output minimum eig vector:" << minEigVec << endl;
    van_p.push_back(minEigVec);
    cout << "vanishing point: " << van_p << endl;
}

void single_view_modeling::computeRefX(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> r)
{
    cout << "r0.x: " << r[0].x << endl;
    cout << "obeginx: " << o.begin()->x << endl;
    ax = 0.00016;//((r[0].x - o.begin()->x) / (v[0].x - r[0].x)) / 300;
    cout << "try ax: " << ax << endl;
}

void single_view_modeling::computeRefY(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> r)
{
    cout << "r0.x: " << r[0].x << endl;
    cout << "obeginx: " << o.begin()->x << endl;
    ay = 0.000178;//((r[0].x - o.begin()->x) / (v[1].x - r[0].x)) / 400;
    cout << "try ay: " << ay << endl;
}

void single_view_modeling::computeRefZ(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> r)
{
    az = 0.000235;//abs((r[0].x - o.begin()->x) / (v[2].x - r[0].x)) / 183;
    cout << "try az: " << az << endl;
}

double single_view_modeling::compute3DX(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> tb)
{
    Point3d ori(o.begin()->x, o.begin()->y, 1);
    Point3d lyz(0, 0, 0);
    //lyz.x = (v[1].y * v[2].z) - (v[1].z * v[2].y);
    //lyz.y = (v[1].z * v[2].x) - (v[1].x * v[2].z);
    //lyz.z = (v[1].x * v[2].y) - (v[1].y * v[2].x);
    //vector<cv::Point3d> lyzVec;
    lyz = v[1].cross(v[2]);
    //lyzVec.push_back(lyz);

    Point3d bCrst(0, 0, 0);
    //bCrst.x = (tb[0].y * tb[1].z) - (tb[0].z * tb[1].y);
    //bCrst.y = (tb[0].z * tb[1].x) - (tb[0].x * tb[1].z);
    //bCrst.z = (tb[0].x * tb[1].y) - (tb[0].y * tb[1].x);
    //vector<cv::Point3d> bCrstVec;
    bCrst = tb[1].cross(tb[0]);
    //bCrstVec.push_back(bCrst);

    Point3d vCrst(0, 0, 0);
    vCrst = v[0].cross(tb[0]);

    double num = ori.dot(lyz) * sqrt(pow(bCrst.x,2) + pow(bCrst.y,2) + pow(bCrst.z,2));
    double den = tb[1].dot(lyz) * sqrt(pow(vCrst.x,2) + pow(vCrst.y,2) + pow(vCrst.z,2));
    double X = num / (den * ax);
    return X;
}

double single_view_modeling::compute3DY(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> tb)
{
    Point3d ori(o.begin()->x, o.begin()->y, 1);
    Point3d lxz(0, 0, 0);
    lxz = v[0].cross(v[2]);

    Point3d bCrst(0, 0, 0);
    bCrst = tb[1].cross(tb[0]);

    Point3d vCrst(0, 0, 0);
    vCrst = v[1].cross(tb[0]);

    double num = ori.dot(lxz) * sqrt(pow(bCrst.x,2) + pow(bCrst.y,2) + pow(bCrst.z,2));
    double den = tb[1].dot(lxz) * sqrt(pow(vCrst.x,2) + pow(vCrst.y,2) + pow(vCrst.z,2));
    double Y = num / (den * ay);
    return Y;
}

double single_view_modeling::compute3DZ(vector<cv::Point2d> o, vector<cv::Point3d> v, vector<cv::Point3d> tb)
{
    Point3d ori(o.begin()->x, o.begin()->y, 1);
    Point3d lxy(0, 0, 0);
    lxy = v[0].cross(v[1]);

    Point3d bCrst(0, 0, 0);
    bCrst = tb[1].cross(tb[0]);

    Point3d vCrst(0, 0, 0);
    vCrst = v[2].cross(tb[0]);

    double num = ori.dot(lxy) * sqrt(pow(bCrst.x,2) + pow(bCrst.y,2) + pow(bCrst.z,2));
    double den = tb[1].dot(lxy) * sqrt(pow(vCrst.x,2) + pow(vCrst.y,2) + pow(vCrst.z,2));
    double Z = num / (den * az);
    return Z;
}


Matrix3d single_view_modeling::computeHmat(vector<cv::Point2d> img_uv, vector<cv::Point2d> real_xy, int pl)
{
    cout << "in computeHmat function, plane is:" << pl << endl;
    int uv_start;
    if(pl == 1)
        uv_start = 0;
    if(pl == 2)
        uv_start = 4;
    if(pl == 3)
        uv_start = 8;
    MatrixXd A(8, 8);
    VectorXd xx(8);
    VectorXd AA(8);
    for(int i=uv_start; i<uv_start+4; i++)
    {
        VectorXd v1(8);
        v1(0) = img_uv[i].x;
        v1(1) = img_uv[i].y;
        v1(2) = 1;
        v1(3) = 0;
        v1(4) = 0;
        v1(5) = 0;
        v1(6) = -img_uv[i].x * real_xy[i].x;
        v1(7) = -img_uv[i].y * real_xy[i].x;

        VectorXd v2(8);
        v2(0) = 0;
        v2(1) = 0;
        v2(2) = 0;
        v2(3) = img_uv[i].x;
        v2(4) = img_uv[i].y;
        v2(5) = 1;
        v2(6) = -img_uv[i].x * real_xy[i].y;
        v2(7) = -img_uv[i].y * real_xy[i].y;
        cout << "test v1:" << v1 << endl;
        cout << "test v2: " << v2 << endl;
        //v(img_uv[0].x, img_uv[0].y, 1.0, 0.0, 0.0, 0.0, -img_uv[0].x*real_xy[0].x, -img_uv[0].y*real_xy[0].x, -real_xy[0].x );
        //A.row(i) = VectorXd(9);
        for(int j=0; j<8; j++)
        {
            A.row(i-uv_start)[j] = v1(j);
            A.row(i-uv_start+4)[j] = v2(j);
        }
        xx(i-uv_start) = real_xy[i].x;
        xx(i-uv_start+4) = real_xy[i].y;
    }
    cout << "check matrix A: " << A << endl;
    cout << "check matrix xx:" << xx << endl;
    AA = A.inverse() * xx;
    cout << "check projective mapping matrix in vector AA: " << AA << endl;
    VectorXd HVec(9);
    for(int k=0; k<8; k++)
        HVec(k) = AA(k);
    HVec(8) = 1;
    Map<Matrix3d> H(HVec.data(), 3,3);
    //cout << "the minimum eigen vector of AA is: " << minVec << endl;
    cout << "H matrix is: " << H << endl;
    return H;
}

void single_view_modeling::textureMap(vector<cv::Point2d> real_xy, Eigen::Matrix3d H, int pl)
{
    vector<cv::Point2d> xy;
    vector<int> xy_x;
    vector<int> xy_y;
    for(int i=0; i<4; i++)
    {
//        cout << "enter textureMap xy.x: " << round(real_xy[(pl-1)*4 + i].x) << endl;
//        cout << "enter textureMap xy.y: " << round(real_xy[(pl-1)*4 + i].y) << endl;
        xy.push_back(Point2d(round(real_xy[(pl-1)*4 + i].x), round(real_xy[(pl-1)*4 + i].y)));
        xy_x.push_back(xy[i].x);
        xy_y.push_back(xy[i].y);
        //xy[i].x = round(real_xy[(pl-1)*4 + i].x);
        //xy[i].y = round(real_xy[(pl-1)*4 + i].y);
//        cout << "export xy[0]: " << xy[i] << endl;
    }
    cout << "check xy: " << xy << endl;
    for(int i=0; i<4; i++)
    {
        cout << "check xy_x: " << xy_x[i] << endl;
        cout << "check xy_y: " << xy_y[i] << endl;
    }
//    cout << "break1" << endl;
    auto result_x = minmax_element (xy_x.begin(),xy_x.end());
    auto result_y = minmax_element (xy_y.begin(),xy_y.end());
    int minLen = *result_x.first;
    int maxLen = *result_x.second;
    int minWid = *result_y.first;
    int maxWid = *result_y.second;
    cout << minLen << maxLen << minWid << maxWid << endl;
//    cout << "break2 " << im_len << "  " << im_wid << endl;
    Mat texture(maxLen-minLen+1, maxWid-minWid+1, CV_8UC3, Scalar(255,255,255));
//    cout << "break3" << endl;
    Vector3d uv;
    Vector2d uv_floor;
    double a, b;
//    cout << "break4" << endl;
    for(int i=minLen; i<maxLen+1; i++)
        for(int j=minWid; j<maxWid+1; j++)
        {
            uv = H.transpose() * Vector3d(i, j, 1);
            uv = uv / uv(2);
//            cout << "break5" << endl;
            uv_floor(0) = floor(uv(0));
            uv_floor(1) = floor(uv(1));
            a = uv(0) - uv_floor(0);
            b = uv(1) - uv_floor(1);
//            cout << "break6" << endl;
//            texture.at<Vec3b>(i,j) = img.at<Vec3b>(uv_floor(1), uv_floor(0));
//            cout << i-minLen+1 << endl;
            texture.at<Vec3b>(i-minLen,j) = (1-a) * (1-b) * img.at<Vec3b>(uv_floor(1), uv_floor(0))
                                     + a * (1-b) * img.at<Vec3b>(uv_floor(1), uv_floor(0)+1)
                                     + a * b * img.at<Vec3b>(uv_floor(1)+1, uv_floor(0)+1)
                                     + b * (1-a) * img.at<Vec3b>(uv_floor(1)+1, uv_floor(0));
//            cout << "break7" << endl;
        }
    imshow("reconstruct image"+to_string(pl), texture);
    string file = "box" + to_string(pl) + ".jpeg";
    imwrite( file, texture );
//    cout << "break8" << endl;
}


bool single_view_modeling::eventFilter(QObject *obj, QEvent *event)
{

    if (qobject_cast<QLabel*>(obj)==ui->imgLabel && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        sx = mouseEvent->pos().x();
        sy = mouseEvent->pos().y();
        Point2d point(sx, sy);
        if(para_points.size() < 18)
        {
            para_points.push_back(point);
            circle(img, para_points.back(), 1, Scalar(255,0,0), 2, 8, 0);
            cout << "point list" << para_points << endl;

            if(para_points.size() % 2 == 0)
            {
                line(img, para_points[para_points.size()-2], para_points[para_points.size()-1], Scalar(255,0,0),2, 8, 0);
                if(para_points.size() % 6 == 0)
                    compute_vanish(para_points);
            }
            show_image(img, 0);
            if(para_points.size() == 18)
                cout << "Please choose the origion:" << endl;
        }

        else
        {
            if(origin.empty())
            {
                origin.push_back(point);
                circle(img, origin.back(), 1, Scalar(0,255,0), 2, 8, 0);
                show_image(img, 0);
                cout << "Your origin is: " << origin << endl;
                cout << "Please select 4 points for xy plane" << endl;
                //cout << "Select your first point:" << endl;
                plane = 1;
            }
            else
            {
                if(plane == 1)
                {
                    if(img_uv.size() < 4)
                    {
                        circle(img, point, 1, Scalar(0,0,255), 2, 8, 0);
                        show_image(img, 0);
                        img_uv.push_back(point);
                        if(img_uv.size() == 4)
                        {
                            cout << "Your four point for xy plane is: " << img_uv << endl;
                            //cout << "3D reconstruct for selected points (select b and t): " << endl;
                            cout << "set reference points to compute ax, ay, az: " << endl;
                        }
                    }
                    // calculate 3d real world coordinates
                    else
                    {
                        circle(img, point, 1, Scalar(230,230,250), 2, 8, 0);
                        show_image(img, 0);
                        Point3d p3d(sx, sy, 1);
                        recon3D.push_back(p3d);
                    }
                }
                else if(plane == 2)
                {
                    if(img_uv.size() >= 4 && img_uv.size() < 8)
                    {
                        circle(img, point, 1, Scalar(0,0,255), 2, 8, 0);
                        show_image(img, 0);
                        img_uv.push_back(point);
                        if(img_uv.size() == 8)
                        {
                            cout << "Your four point for xz plane is: " << img_uv << endl;
                            cout << "3D reconstruct for selected points (select b and t): " << endl;
                        }
                    }
                    // calculate 3d real world coordinates
                    else
                    {
                        circle(img, point, 1, Scalar(176,224,230), 2, 8, 0);
                        show_image(img, 0);
                        Point3d p3d(sx, sy, 1);
                        recon3D.push_back(p3d);
                    }
                }
                else if(plane == 3)
                {
                    if(img_uv.size() >= 8 && img_uv.size() < 12)
                    {
                        circle(img, point, 1, Scalar(0,0,255), 2, 8, 0);
                        show_image(img, 0);
                        img_uv.push_back(point);
                        if(img_uv.size() == 12)
                        {
                            cout << "Your four point for yz plane is: " << img_uv << endl;
                            cout << "3D reconstruct for selected points (select t and b): " << endl;
                        }
                    }
                    // calculate 3d real world coordinates
                    else
                    {
                        circle(img, point, 1, Scalar(135,206,235), 3, 8, 0);
                        show_image(img, 0);
                        Point3d p3d(sx, sy, 1);
                        recon3D.push_back(p3d);
                    }
                }


//                double dist = sqrt(pow(sx-origin[0].x, 2) + pow(sy-origin[0].y, 2));
//                QString qstr = QString::fromStdString(to_string(dist));
//                bool ok;
//                QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
//                                                     tr("Input real world coordinate(x, y): "), QLineEdit::Normal,
//                                                     qstr, &ok);
//                if (ok && !text.isEmpty())
//                    ui->dlglabel->setText(text);
//                string sstr = text.toUtf8().constData();

            }

        }
    }


    if (qobject_cast<QLabel*>(obj)==ui->imgLabel && event->type() == QEvent::MouseMove)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        statusBar()->showMessage(QString("Mouse move (%1,%2)").arg(mouseEvent->pos().x()).arg(mouseEvent->pos().y()));

    }
      //update();
      return false;
}

void single_view_modeling::on_actionOpen_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose"), "", tr("Images(*.png *.jpg *.jpeg *.bmp *.gif)"));

    if (QString::compare(filename, QString()) != 0)
    {
        //QImage image;
        string file = filename.toUtf8().constData();
        src = imread(file,1);
        img = src.clone();
        show_image(src,0);
        cout << "please select lines parallel to X Y Z:" << endl;
    }
}

void single_view_modeling::on_XButton_clicked()
{
    cx = compute3DX(origin, van_p, recon3D);
    cout << "X coordinate is: " << cx << endl;
    recon3D.clear();
}

void single_view_modeling::on_YButton_clicked()
{
    cy = compute3DY(origin, van_p, recon3D);
    cout << "Y coordinate is: " << cy << endl;
    recon3D.clear();
}

void single_view_modeling::on_ZButton_clicked()
{
    cz = compute3DZ(origin, van_p, recon3D);
    cout << "Z coordinate is: " << cz << endl;
    recon3D.clear();

}

void single_view_modeling::on_XYZButton_clicked()
{
    Point2d r_xy(0, 0);
    if(plane == 1)
        r_xy = Point2d(cx, cy);
    if(plane == 2)
        r_xy = Point2d(cx, cz);
    if(plane == 3)
        r_xy = Point2d(cy, cz);
    real_xy.push_back(r_xy);
    cout << "XYZ coordinates of the point is: " << "x:" << cx << "y:" << cy << "z:" << cz<< endl;
    cout << "coordinates on projection plane is: " << real_xy<< endl;
    if(!real_xy.empty() && real_xy.size() % 4 == 0)
    {
        //computeHmat(img_uv, real_xy); // for H1
        //real_xy.clear();
        //img_uv.clear();
        if(plane == 1)
        {
            cout << "now compute H1" << endl;
            //H1 = computeHmat(img_uv, real_xy, 1); // for H1
            H1 = computeHmat(real_xy, img_uv, 1); // for H1
            //real_xy.clear();
            //img_uv.clear();
            plane = 2;

        }
        else if(plane == 2)
        {
            cout << "now compute H2" << endl;
            //H2 = computeHmat(img_uv, real_xy, 2); // for H2
            H2 = computeHmat(real_xy, img_uv, 2); // for H2
            //real_xy.clear();
            //img_uv.clear();
            plane = 3;
        }
        else if(plane == 3)
        {
            cout << "now compute H3" << endl;
            //H3 = computeHmat(img_uv, real_xy, 3); // for H3
            H3 = computeHmat(real_xy, img_uv, 3); // for H3
            plane = 0;
            // do texture mapping
            textureMap(real_xy, H1, 1);
            textureMap(real_xy, H2, 2);
            textureMap(real_xy, H3, 3);
        }
    }
}


void single_view_modeling::on_refXButton_clicked()
{
    computeRefX(origin, van_p, recon3D);
    recon3D.clear();
}

void single_view_modeling::on_refYButton_clicked()
{
    computeRefY(origin, van_p, recon3D);
    recon3D.clear();
}

void single_view_modeling::on_refZButton_clicked()
{
    computeRefZ(origin, van_p, recon3D);
    recon3D.clear();
    cout << "3D reconstruct for selected points on xy plane (select b and t): " << endl;
}
