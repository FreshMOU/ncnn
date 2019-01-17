#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>

#include "net.h"

struct bbox{
    int x1,y1,x2,y2,x3,y3,x4,y4;
    float score;
};

typedef struct point_sh{
    int x,y;
    float sina;
    int q;
}Point;

//排斥实验
bool IsRectCross(const Point &p1,const Point &p2,const Point &q1,const Point &q2)
{
    bool ret = std::min(p1.x,p2.x) <= std::max(q1.x,q2.x)    &&
                std::min(q1.x,q2.x) <= std::max(p1.x,p2.x) &&
                std::min(p1.y,p2.y) <= std::max(q1.y,q2.y) &&
                std::min(q1.y,q2.y) <= std::max(p1.y,p2.y);
    return ret;
}
//跨立判断
bool IsLineSegmentCross(const Point &pFirst1,const Point &pFirst2,const Point &pSecond1,const Point &pSecond2)
{
    long line1,line2;
    line1 = pFirst1.x * (pSecond1.y - pFirst2.y) +
        pFirst2.x * (pFirst1.y - pSecond1.y) +
        pSecond1.x * (pFirst2.y - pFirst1.y);
    line2 = pFirst1.x * (pSecond2.y - pFirst2.y) +
        pFirst2.x * (pFirst1.y - pSecond2.y) + 
        pSecond2.x * (pFirst2.y - pFirst1.y);
    if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
        return false;

    line1 = pSecond1.x * (pFirst1.y - pSecond2.y) +
        pSecond2.x * (pSecond1.y - pFirst1.y) +
        pFirst1.x * (pSecond2.y - pSecond1.y);
    line2 = pSecond1.x * (pFirst2.y - pSecond2.y) + 
        pSecond2.x * (pSecond1.y - pFirst2.y) +
        pFirst2.x * (pSecond2.y - pSecond1.y);
    if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
        return false;
    return true;
}

bool GetCrossPoint(const Point &p1,const Point &p2,const Point &q1,const Point &q2,long &x,long &y)
{
    if(IsRectCross(p1,p2,q1,q2))
    {
        if (IsLineSegmentCross(p1,p2,q1,q2))
        {
            //求交点
            long tmpLeft,tmpRight;
            tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
            tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

            x = (int)((double)tmpRight/(double)tmpLeft);

            tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
            tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x- p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y); 
            y = (int)((double)tmpRight/(double)tmpLeft);
            return true;
        }
    }
    return false;
}

//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
bool IsPointInPolygon(std::vector<Point> poly, Point pt)
{
    int i,j;
    bool c = false;
    for (i = 0,j = poly.size() - 1;i < poly.size();j = i++)
    {
        if ((((poly[i].y <= pt.y) && (pt.y < poly[j].y)) ||
            ((poly[j].y <= pt.y) && (pt.y < poly[i].y)))
            && (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y)/(poly[j].y - poly[i].y) + poly[i].x))
        {
            c = !c;
        }
    }
    return c;
}

//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
bool PointCmp(const Point &a,const Point &b,const Point &center)
{
    if (a.x >= 0 && b.x < 0)
        return true;
    if (a.x == 0 && b.x == 0)
        return a.y > b.y;
    //向量OA和向量OB的叉积
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    //向量OA和向量OB共线，以距离判断大小
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}
void ClockwiseSortPoints(std::vector<Point> &vPoints)
{
    //计算重心
    Point center;
    double x = 0,y = 0;
    for (int i = 0;i < vPoints.size();i++)
    {
        x += vPoints[i].x;
        y += vPoints[i].y;
    }
    center.x = (int)x/vPoints.size();
    center.y = (int)y/vPoints.size();

    //冒泡排序
    for(int i = 0;i < vPoints.size() - 1;i++)
    {
        for (int j = 0;j < vPoints.size() - i - 1;j++)
        {
            if (PointCmp(vPoints[j],vPoints[j+1],center))
            {
                Point tmp = vPoints[j];
                vPoints[j] = vPoints[j + 1];
                vPoints[j + 1] = tmp;
            }
        }
    }
}

//多边形交点
bool PolygonClip(const std::vector<Point> &poly1, const std::vector<Point> &poly2, std::vector<Point> &interPoly)
{
    if (poly1.size() < 3 || poly2.size() < 3)
    {
        return false;
    }

    long x,y;
    //计算多边形交点
    for (int i = 0;i < poly1.size();i++)
    {
        int poly1_next_idx = (i + 1) % poly1.size();
        for (int j = 0;j < poly2.size();j++)
        {
            int poly2_next_idx = (j + 1) % poly2.size();
            if (GetCrossPoint(poly1[i],poly1[poly1_next_idx],
                poly2[j],poly2[poly2_next_idx],
                x,y))
            {
                Point temp1;
                temp1.x = x;
                temp1.y = y;
                interPoly.push_back(temp1);
            }
        }
    }

    //计算多边形内部点
    for(int i = 0;i < poly1.size();i++)
    {
        if (IsPointInPolygon(poly2,poly1[i]))
        {
            interPoly.push_back(poly1[i]);
        }
    }
    for (int i = 0;i < poly2.size();i++)
    {
        if (IsPointInPolygon(poly1,poly2[i]))
        {
            interPoly.push_back(poly2[i]);
        }
    }
    
    if(interPoly.size() <= 0)
        return false;
        
    //点集排序 
    ClockwiseSortPoints(interPoly);
    return true;
}

inline int compute_q(std::vector<point_sh>& pt, float x0, float y0)
{
    for (int i=0; i<pt.size(); i++)
    {
        point_sh*p = &pt[i];
        if (p->x - x0 > 0)
        {
            if (p->y - y0 > 0)
                p->q = 1;
            else
                p->q = 4;
        }
        else
        {
            if (p->y - y0 > 0)
                p->q = 2;
            else
                p->q = 3;        
        }
    }

    return 0;
}

float polygon_area(std::vector<point_sh> pt)
{
    if (pt.size() < 3)
        return -100;

    float x0,y0;
    x0 = (pt[0].x + pt[1].x + pt[2].x) / 3.0;
    y0 = (pt[0].y + pt[1].y + pt[2].y) / 3.0;
    //std::vector<float> sins;
    float dx, dy, ds, sina;
    for (int i=0; i<pt.size(); i++)
    {
        dx = pt[i].x - x0;
        dy = pt[i].y - y0;
        ds = std::sqrt(dx * dx + dy * dy);
        pt[i].sina = dy / ds;
    }

    compute_q(pt, x0, y0);
    //简单排序，如有需要可以优化成堆排序
    int j=0;
    while(1)
    {
        if (j > pt.size())
            break;

        for (int i=1; i<pt.size(); i++)
        {
            if (pt[j].q < pt[i].q)
                continue;
            
            if (pt[j].q > pt[i].q)
                std::swap(pt[j], pt[i]);
            else if ((pt[j].q == 1 || pt[j].q == 4) && pt[j].sina > pt[i].sina ||
                     (pt[j].q == 1 || pt[j].q == 4) && pt[j].sina < pt[i].sina)
                        std::swap(pt[j], pt[i]);
        }
        j++;
    }

    point_sh Fpt = pt[0];
    float area = 0;
    for (int i=1; i<pt.size()-1; i++)
    {
        area += std::fabs(0.5f * (Fpt.x * (pt[i].y - pt[i+1].y) + pt[i].x * (Fpt.y - pt[i].y) + pt[i+1].x * (pt[i+1].y - Fpt.y)));
    }

    return area;
}

inline std::vector<point_sh> poly2point(bbox polygon)
{
    std::vector<point_sh> ps;
    point_sh p1,p2,p3,p4;
    p1.x = polygon.x1;
    p1.y = polygon.y1;
    p2.x = polygon.x2;
    p2.y = polygon.y2;
    p3.x = polygon.x3;
    p3.y = polygon.y3;
    p4.x = polygon.x4;
    p4.y = polygon.y4;
    ps.push_back(p1);
    ps.push_back(p2);
    ps.push_back(p3);
    ps.push_back(p4);
    return ps; 
}


float polygon_iou(bbox polygon1, bbox polygon2)
{
    float iou, inter_area, area1, area2;
    std::vector<point_sh> ps1, ps2;
    ps1 = poly2point(polygon1);
    ps2 = poly2point(polygon2);
    std::vector<Point> inter_poly;
    if (PolygonClip(ps1, ps2, inter_poly))
    {
        area1 = polygon_area(ps1);
        area2 = polygon_area(ps2);
        inter_area = polygon_area(inter_poly);
        iou = inter_area / (area1 + area2);
    }
    else
        iou = 0;
    return iou;
}

bool compare(bbox a, bbox b)
{
    return a.score < b.score;
}

static int nms(std::vector<bbox>& polygons, float overlap)
{
    std::sort(polygons.begin(), polygons.end(), compare);
    int j=0;
    while(1)
    {
        bbox Fpolygon = polygons[j];//try..catch
        j++;
        if (j > polygons.size())
            break;
        
        for (int i=j; i<polygons.size(); i++)
        {
            float iou = polygon_iou(Fpolygon, polygons[i]);
            if (iou > overlap)
                polygons.erase(polygons.begin() + i);
        }
    }
    return 0;
}

static int detect_textboxes(cv::Mat& raw_img, float show_threshold)
{
    ncnn::Net vgg_net;

    int img_h = raw_img.size().height;
    int img_w = raw_img.size().width;
    vgg_net.load_param("cdar.param");
    vgg_net.load_model("cdar.bin");
    int input_size = 768;
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(raw_img.data, ncnn::Mat::PIXEL_BGR, raw_img.cols, raw_img.rows, input_size, input_size);

    const float mean_vals[3] = {104.f, 117.f, 123.f};
    const float norm_vals[3] = {1.0/104.f, 1.0/117.f, 1.0/123.f};
    in.substract_mean_normalize(mean_vals, NULL);

    ncnn::Mat out;
    
    ncnn::Extractor ex = vgg_net.create_extractor();
    ex.set_light_mode(true);

    ex.input("data", in);
    ex.extract("detection_out",out);

    std::vector<bbox> polygons;
    cv::Point recs[200][1][4]; 
    int j=0;
    fprintf(stderr, "%d %d %d\n", out.w, out.h, out.c);
    for (int i=0; i<out.h; i++)
    {
        const float *value = out.row(i);
        float score = value[1];
        float x1,y1,x2,y2,x3,y3,x4,y4;
        x1 = std::max(1, std::min(int(value[6] * img_w), img_w-1));
        y1 = std::max(1, std::min(int(value[7] * img_h), img_h-1));
        x2 = std::max(1, std::min(int(value[8] * img_w), img_w-1));
        y2 = std::max(1, std::min(int(value[9] * img_h), img_h-1));
        x3 = std::max(1, std::min(int(value[10]* img_w), img_w-1));
        y3 = std::max(1, std::min(int(value[11]* img_h), img_h-1));
        x4 = std::max(1, std::min(int(value[12]* img_w), img_w-1));
        y4 = std::max(1, std::min(int(value[13]* img_h), img_h-1));
        if (score>show_threshold)
        {
            bbox polygon;
            polygon.x1 = x1;
            polygon.y1 = y1;
            polygon.x2 = x2;
            polygon.y2 = y2;
            polygon.x3 = x3;
            polygon.y3 = y3;
            polygon.x4 = x4;
            polygon.y4 = y4;
            polygon.score = score;
            polygons.push_back(polygon);

            j++;
        }
    }

    nms(polygons, 0.1);
    if (polygons.empty())
        return -100;
    for (int i=0; i<polygons.size(); i++)
    {
        recs[i][0][0] = cv::Point(polygons[i].x1, polygons[i].y1);
        recs[i][0][1] = cv::Point(polygons[i].x2, polygons[i].y2);
        recs[i][0][2] = cv::Point(polygons[i].x3, polygons[i].y3);
        recs[i][0][3] = cv::Point(polygons[i].x4, polygons[i].y4);
    }
    std::cout << "before:" << j << "final:" << polygons.size() << std::endl;
    
    int npt[] = { 4 };
    for (int i=0; i<polygons.size(); i++)
    {
        const cv::Point *ppt[1] = {recs[i][0]};
        cv::polylines(raw_img, ppt, npt, 1, 1, cv::Scalar(255, 255, 0));
    }

    cv::imwrite("test.jpg", raw_img);
    return 0;
}

int main(int argc, char** argv)
{
    const char* imagepath = argv[1];

    cv::Mat m = cv::imread(imagepath, CV_LOAD_IMAGE_COLOR);
    if (m.empty())
    {
        fprintf(stderr, "cv::imread %s failed\n", imagepath);
        return -1;
    }

    time_t start, end;
    start = time(NULL);
    detect_textboxes(m,0.1);
    end = time(NULL);                  
    printf( "%ld seconds\n", end - start );
    return 0;
}