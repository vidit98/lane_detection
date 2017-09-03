#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stack>
#include <iostream>
#include <math.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

int minl=30,maxg=3,thr=40;

struct slope
{
	float m;
	float x1;
	float y1;
	float x2;
	float y2;

};
 
vector <slope> fl;

void threshhold(Mat image)
{	
	
	vector<Vec4i> lines;
	namedWindow( "a", WINDOW_AUTOSIZE);
	createTrackbar("thr","a",&thr,180);
    createTrackbar("minl","a",&minl,700);
    createTrackbar("maxg","a",&maxg,500);
    imshow("b",image);
	while(1)
	{
		Mat dst1(image.rows,image.cols,CV_8UC3,Scalar(0,0,0));
		HoughLinesP(image,lines,1,CV_PI/180,thr+1,minl,maxg);

		for( size_t i = 0; i < lines.size(); i++ )
	    {
	        line( dst1, Point(lines[i][0], lines[i][1]),
	            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
	    }

	    imshow("a",dst1);

	    waitKey(1);

	}

}

float cal_slp(int x1,int y1,int x2,int y2)
{

	float m;
	if (x1==x2) return CV_PI/2;
	m=atan((float)(y1-y2)/(float)(x1-x2));
	return m;

}


void sol_le(float (*b)[3],float ans_arr[])
{
	float q = b[0][0]*b[1][1] - b[0][1]*b[1][0];
	ans_arr[0] = ((b[1][1]*b[0][2]) - (b[0][1]*b[1][2]))/q;
	ans_arr[1] = ((-b[1][0]*b[0][2]) + (b[0][0]*b[1][2]))/q;

}

int find_median(int ar[],int size)
{
	int m;
	sort(ar , ar + size);
	if(size % 2 == 0)
	{
		m= (ar[size/2] + ar[(size/2 -1)])/2 ;
	}
	else
	{
		m= ar[size/2];
	}
	return m;


}

int get_lanes(vector<slope> sl , Mat img)
{
	int col=img.cols , row = img.rows;
	Mat dst10(row,col,CV_8UC3,Scalar(0,0,0));
	float xyarr[2];
    float eqarr[2][3],y1;
    int c_flag=0,i,k,count=0,avg[100] ,med;
    
   	for(  i = 0; i < sl.size() - 1; i++ )
   	{
		eqarr[0][1]=1;
		eqarr[0][0]=((sl[i].y1-sl[i].y2)/(sl[i].x1-sl[i].x2));
		eqarr[0][2]=-((sl[i].y2*sl[i].x1 - sl[i].y1*sl[i].x2)/(sl[i].x1-sl[i].x2));
		
		
		
		for(k=i+1; k < sl.size();k++)
		{
			eqarr[1][1]=1;
			eqarr[1][0]=((sl[k].y1-sl[k].y2)/(sl[k].x1-sl[k].x2));
			eqarr[1][2]=-((sl[k].y2*sl[k].x1 - sl[k].y1*sl[k].x2)/(sl[k].x1-sl[k].x2));
			
			sol_le(eqarr,xyarr);
			
		
			if((xyarr[0] > col * 0.3) && (xyarr[0] < col * 0.64) && (-xyarr[1] < row * 0.7) && (-xyarr[1] > 0) && (sl[k].y1 >= -xyarr[1]) && (sl[k].y2 >= -xyarr[1]))
			{
				c_flag=1;
				avg[count] = -xyarr[1];
				y1 = xyarr[1];
				count++;
				fl.push_back(sl[k]);
				line(img,Point(sl[k].x1,sl[k].y1),Point(sl[k].x2,sl[k].y2),Scalar(0,0,255),2,8);
				line(dst10,Point(sl[k].x1,sl[k].y1),Point(sl[k].x2,sl[k].y2),Scalar(0,0,255),2,8);


			}

		}

		if(c_flag==1) 
		{
			if ((sl[i].y1 >= -y1 ) && (sl[i].y2 >= -y1))
			{
				avg[count] = -xyarr[1];

				count++;
				c_flag=0;
				fl.push_back(sl[i]);
				
			}
		}
	}

	med = find_median(avg , count) ;

	
	return med;
	

}


int main(int argc , char **argv)
{
	Mat img=imread(argv[1],1);
	Mat img0 = imread(argv[1],1);
	Mat img1,img2,dst,dst0;
	Mat dst1(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	Mat dst2(img.rows,img.cols,CV_8UC1,Scalar(0));
	Mat dst3(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	Mat road(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
	vector<Vec4i> hierarchy1;
    vector<vector<Point> > contours1;
	int i,k;
	

	vector<Vec4i> lines;
	slope a;
	vector<slope> sl; //storing slope and 2 pts of line


	cvtColor(img,img,CV_RGB2GRAY);
	
	medianBlur(img,img1,5);
	
	for(i=0;i < img.rows;i++)
	{
		for(k=0;k < img.cols;k++)
		{
			if (img1.at<uchar>(i,k) < 80)
			{
				img1.at<uchar>(i,k)=100;
			}
		}
	}
	
	GaussianBlur(img1,img2,Size(3,3),0,0);
	
	Canny(img2,dst,50,100);
	
	
	HoughLinesP(dst,lines,1,CV_PI/180,thr+1,minl,maxg);
	cout << lines.size();
	for(  i = 0; i < lines.size(); i++ )
    {	
        line( dst1, Point(lines[i][0], lines[i][1]),Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
	}

   
    a.m=cal_slp(lines[0][0],-lines[0][1],lines[0][2],-lines[0][3]);
    a.x1=lines[0][0];
    a.y1=lines[0][1];
    a.x2=lines[0][2];
    a.y2=lines[0][3];

    sl.push_back(a);
    int flag=0;
    unsigned int j=0;
	for( i = 0; i < lines.size(); i++)
    {

    	flag=0;
    	

    	a.m=cal_slp(lines[i][0],-lines[i][1],lines[i][2],-lines[i][3]);
	    a.x1=lines[i][0];
	    a.y1=lines[i][1];
	    a.x2=lines[i][2];
	    a.y2=lines[i][3];

	    
	    if((abs(a.m) > .1744) && (a.x1!=a.x2))
	    {
	    	for( j=0; j< sl.size();j++)
	    	{
	    		
	    		if( abs(a.m - sl[j].m) <= 0.04)
	    		{	
	    			flag=1;
	    			break;
				}
			}

	    	if(flag == 0)
	    	{
	    		sl.push_back(a);
	    	}
	    }
    }

    if(abs(sl[0].m) <= .1744)
    {
    	sl.erase(sl.begin());
    }

    for(  i = 0; i < sl.size(); i++ )
    {
        line( dst1, Point(sl[i].x1, sl[i].y1),Point(sl[i].x2, sl[i].y2), Scalar(0,255,0), 1, 8 );
    }
  
   int horizon = get_lanes(sl,img);
   int x1,x2;
   for(  i = 0; i < fl.size(); i++ )
   {	
   		x1 = (-(horizon+20) + ((fl[i].y2*fl[i].x1 - fl[i].y1*fl[i].x2)/(fl[i].x1-fl[i].x2)) ) / tan(fl[i].m);
   		x2 = (-img.rows + ((fl[i].y2*fl[i].x1 - fl[i].y1*fl[i].x2)/(fl[i].x1-fl[i].x2)) ) / tan(fl[i].m);
   		line( img0, Point(x1,horizon + 20),Point(x2,img.rows), Scalar(0,255,0), 2, 8 );
   }
   
 
   imshow("roads" , img0);
   waitKey(0);
	return 0;
}