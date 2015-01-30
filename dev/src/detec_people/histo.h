#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class histo
{
public:
	histo();
	histo(int n);
  histo(histo&);
	~histo();
	histo operator+(const histo& a);
  void set_size(int n);
  histo* calc_histogram(PointCloudT::Ptr& cloud, float max_y, float min_y);
  float histo_dist_sq(histo* a);
  void normalize();

private:
	int size;
	float* data;
  int sum;
	int MIN_cmp(int a, int b);
  int histo_bin(float h, float s, float v);
	int max(int a, int b, int c);
	int min(int a, int b, int c);
	void rgb2hsv(int r, int g, int b, float& h, float& s, float& v);

  const static int N = 110; //NH = NS = NV = 10; NH*NS + NV
  const static float S_THRESH = 0.1;
  const static float V_THRESH = 0.2;
  const static float H_MAX = 360.0;
  const static float S_MAX = 1.0;
  const static float V_MAX = 1.0;
  const static int NH = 10;
  const static int NS = 10;
  const static int NV = 10;
};

histo::histo()
{
  size = 0;
  data = NULL;
  sum = 0;
}

histo::histo(histo& a)
{
  size = a.size;
  data = new float[size];
  for (int i = 0; i < size; i++)
    data[i] = a.data[i];
  sum = 0;
}

histo::histo(int n)
{
	size = n;
	data = new float[size];
  for (int i = 0; i < size; i++)
    data[i] = 0;
  sum = 0;
}

histo::~histo()
{
	delete data;
}

void histo::set_size(int n)
{
  size = n;
  if (data != NULL)
    delete data;
  data = new float[n];
  for (int i = 0; i < size; i++)
    data[i] = 0;
}

histo histo::operator+(const histo& a)
{
	int size = a.size <= this->size? a.size: this->size;
	histo re(size);
	for (int i = 0; i < re.size; i++)
	{
		re.data[i] = a.data[i] + this->data[i];
	}
  re.sum = this->sum + a.sum;
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

int histo::histo_bin(float h, float s, float v)
{
  int hd, sd, vd;

  /* if S or V is less than its threshold, return a "colorless" bin */
  vd = MIN_cmp( (int)(v * NV / V_MAX), NV-1 );
  if( s < S_THRESH  ||  v < V_THRESH )
    return NH * NS + vd;

  /* otherwise determine "colorful" bin */
  hd = MIN_cmp( (int)(h * NH / H_MAX), NH-1 );
  sd = MIN_cmp( (int)(s * NS / S_MAX), NS-1 );
  return sd * NH + hd;
}

void histo::normalize()
{
  for (int i = 0; i < this->size; i++)
    this->data[i] = this->data[i] / (float)this->sum;
}

float histo::histo_dist_sq(histo* a)
{
  float sum_a = 0;
  int n = this->size < a->size ? this->size : a->size;
  for (int i = 0; i < n; i++)
  {
    sum_a += sqrt(this->data[i] * a->data[i]);
  }
  return 1.0 - sum_a / (float)n;
}

histo* histo::calc_histogram(PointCloudT::Ptr& cloud, float max_y, float min_y)
{
  histo* re = new histo(3 * N);
  float h, s, v;

  float head_y = max_y - (max_y - min_y) / 8.0;
  float body_y = max_y - (max_y - min_y) / 2.0;

  for (int i = 0; i < cloud->points.size(); i++)
  {
    rgb2hsv((int)cloud->points[i].r, (int)cloud->points[i].g,
     (int)cloud->points[i].b, h, s, v);
    int bin = histo_bin(h, s, v);
    if (cloud->points[i].y > head_y)
      re->data[bin]++;
    else if (cloud->points[i].y > body_y)
      re->data[bin + N]++;
    else
      re->data[bin + 2 * N]++;
  }
  re->sum = cloud->points.size();
  this->normalize();
  return re;
}
