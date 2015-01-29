#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class histo
{
public:
	histo();
	histo(int n);
	~histo();
	histo operator+(const histo& a);

private:
	int size;
	float* data;
	int MIN_cmp(int a, int b);
	int max(int a, int b, int c);
	int min(int a, int b, int c);
	void rgb2hsv(int r, int g, int b, float& h, float& s, float& v);
};

histo::histo(int n)
{
	size = n;
	data = new float(size);
}

histo::~histo()
{
	delete data;
}

histo histo::operator+(const histo& a)
{
	int size = a.size <= this->size? a.size: this->size;
	histo re(size);
	for (int i = 0; i < re.size; i++)
	{
		re.data[i] = a.data[i] + this->data[i];
	}
	return re;
}

void histo::rgb2hsv(int r, int g, int b, float& h, float& s, float& v)
{
  int max_ = max( r, g, b );
  int min_ = min( r, g, b );
  v = max_;
  if ( max_ == 0 )
    s = 0;
  else
  s = ( max_ - min_ ) / (float)max_;
  if ( max_ == min_ )
    h = 0;
  if ( r == max_ )
    h = ( g - b ) / (float)( max_ - min_ ) * 60;
  if ( g == max_ )
    h = 120 + ( b - r ) / (float)( max_ - min_ ) * 60;
  if ( b == max_ )
    h = 240 + ( r - g ) / (float)( max_ - min_ ) * 60;
  if ( h < 0 )
    h += 360;
}

int histo::MIN_cmp(int a, int b)
{
	return a>b?a:b;
}

int histo::max(int a, int b, int c)
{
  if ( a > b )
    return a>c?a:c;
  else
    return b>c?b:c;
}

int histo::min( int a, int b, int c )
{
  if ( a < b )
    return a<c?a:c;
  else
    return b<c?b:c;
}
