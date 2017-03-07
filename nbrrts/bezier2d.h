#ifndef BEZIER3TEST
#define BEZIER3TEST

#include <qdebug.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
float bezier3funcX(float uu,Vector2f *controlP);
float bezier3funcY(float uu,Vector2f *controlP);

/*
 *
 *
 *
 */
void createCurve(vector<Vector2f> originPoint,int originCount,vector<Vector2f> &curvePoint){
    //控制点收缩系数 ，经调试0.6较好
    float scale = 0.5;
    Vector2f midpoints[originCount];
    //生成中点
    for(int i = 0 ;i < originCount ; i++){
        int nexti = (i + 1) % originCount;
        midpoints[i].x() = (originPoint[i].x() + originPoint[nexti].x())/2.0;
        midpoints[i].y() = (originPoint[i].y() + originPoint[nexti].y())/2.0;
    }

    //平移中点
    Vector2f extrapoints[2 * originCount];
    for(int i = 0 ;i < originCount ; i++){
         int nexti = (i + 1) % originCount;
         int backi = (i + originCount - 1) % originCount;
         Vector2f midinmid;
         midinmid.x() = (midpoints[i].x() + midpoints[backi].x())/2.0;
         midinmid.y() = (midpoints[i].y() + midpoints[backi].y())/2.0;
         int offsetx = originPoint[i].x() - midinmid.x();
         int offsety = originPoint[i].y() - midinmid.y();
         int extraindex = 2 * i;
         extrapoints[extraindex].x() = midpoints[backi].x() + offsetx;
         extrapoints[extraindex].y() = midpoints[backi].y() + offsety;
         //朝 originPoint[i]方向收缩
         int addx = (extrapoints[extraindex].x() - originPoint[i].x()) * scale;
         int addy = (extrapoints[extraindex].y() - originPoint[i].y()) * scale;
         extrapoints[extraindex].x() = originPoint[i].x() + addx;
         extrapoints[extraindex].y() = originPoint[i].y() + addy;

         int extranexti = (extraindex + 1)%(2 * originCount);
         extrapoints[extranexti].x() = midpoints[i].x() + offsetx;
         extrapoints[extranexti].y() = midpoints[i].y() + offsety;
         //朝 originPoint[i]方向收缩
         addx = (extrapoints[extranexti].x() - originPoint[i].x()) * scale;
         addy = (extrapoints[extranexti].y() - originPoint[i].y()) * scale;
         extrapoints[extranexti].x() = originPoint[i].x() + addx;
         extrapoints[extranexti].y() = originPoint[i].y() + addy;

    }

    Vector2f controlPoint[4];
    //生成4控制点，产生贝塞尔曲线
    for(int i = 0 ;i < originCount-1 ; i++){
           controlPoint[0] = originPoint[i];
           int extraindex = 2 * i;
           controlPoint[1] = extrapoints[extraindex + 1];
           int extranexti = (extraindex + 2) % (2 * originCount);
           controlPoint[2] = extrapoints[extranexti];
           int nexti = (i + 1) % originCount;
           controlPoint[3] = originPoint[nexti];
           float u = 1;
           while(u >= 0){
               int px = bezier3funcX(u,controlPoint);
               int py = bezier3funcY(u,controlPoint);
               //u的步长决定曲线的疏密
               u -= 0.05;
               Vector2f tempP = Vector2f(px,py);
               //存入曲线点
               curvePoint.push_back(tempP);
           }
    }
}

//三次贝塞尔曲线
float bezier3funcX(float uu,Vector2f *controlP){
   float part0 = controlP[0].x() * uu * uu * uu;
   float part1 = 3 * controlP[1].x() * uu * uu * (1 - uu);
   float part2 = 3 * controlP[2].x() * uu * (1 - uu) * (1 - uu);
   float part3 = controlP[3].x() * (1 - uu) * (1 - uu) * (1 - uu);
   return part0 + part1 + part2 + part3;
}
float bezier3funcY(float uu,Vector2f *controlP){
   float part0 = controlP[0].y() * uu * uu * uu;
   float part1 = 3 * controlP[1].y() * uu * uu * (1 - uu);
   float part2 = 3 * controlP[2].y() * uu * (1 - uu) * (1 - uu);
   float part3 = controlP[3].y() * (1 - uu) * (1 - uu) * (1 - uu);
   return part0 + part1 + part2 + part3;
}



Vector2f BezierPoint(vector<Vector2f> cp, float t)
{
    float xa, xb;
    float ya, yb;
    float tsquare;

    xa = cp[2].x() + cp[0].x() - 2 * cp[1].x();
    xb = 2 * cp[1].x() - 2 * cp[0].x();

    ya = cp[2].y() + cp[0].y() - 2 * cp[1].y();
    yb = 2 * cp[1].y() - 2*cp[0].y();

    tsquare = t * t;


    double x = xa * tsquare + t * xb +cp[0].x();
    double y = ya * tsquare + t * yb +cp[0].y();

    Vector2f result(x,y);
    qDebug()<<"result"<<result.x()<<result.y();
    return result;
}

void ComputeBezier2( vector<Vector2f> cp, int numberOfPoints, vector<Vector2f> &curve )
{
    float   dt;
    int    i;

    dt = 1.0 / ( numberOfPoints - 1 );

    for( i = 0; i < numberOfPoints; i++)
        curve.push_back( BezierPoint( cp, i*dt ) );
}



/*
 *
 *
 *
 */
Vector2f PointOnCubicBezier3( vector<Vector2f> cp, float t )
{
    float   ax, bx, cx;
    float   ay, by, cy;
    float   tSquared, tCubed;
    Vector2f result;

    /*計算多項式係數*/

    cx = 3.0 * (cp[1].x() - cp[0].x());
    bx = 3.0 * (cp[2].x() - cp[1].x()) - cx;
    ax = cp[3].x() - cp[0].x() - cx - bx;

    cy = 3.0 * (cp[1].y() - cp[0].y());
    by = 3.0 * (cp[2].y() - cp[1].y()) - cy;
    ay = cp[3].y() - cp[0].y() - cy - by;

    /*計算位於參數值t的曲線點*/

    tSquared = t * t;
    tCubed = tSquared * t;

    result.x() = (ax * tCubed) + (bx * tSquared) + (cx * t) + cp[0].x();
    result.y() = (ay * tCubed) + (by * tSquared) + (cy * t) + cp[0].y();

    return result;
}

/*
 ComputeBezier以控制點cp所產生的曲線點，填入Vector2f結構的陣列。
 呼叫者必須分配足夠的記憶體以供輸出結果，其為<sizeof(Vector2f) numberOfPoints>
*/
void ComputeBezier3( vector<Vector2f> cp, int numberOfPoints, vector<Vector2f> &curve)
{
    float   dt;
    int    i;

    dt = 1.0 / ( numberOfPoints - 1 );

    for( i = 0; i < numberOfPoints; i++)
        curve.push_back(PointOnCubicBezier3( cp, i*dt ));
}
#endif // BEZIER3TEST

