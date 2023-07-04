#include "corner_event_detector/evHarris_ANMS_detector.h"

namespace corner_event_detector
{

evHarrisANMSDetector::evHarrisANMSDetector(bool connect)
: Detector(connect)
{
  detector_name_ = "evHarris_ANMS";

  // parameters
  queue_size_ = 25;
  window_size_ = 4;
  kernel_size_ = 5;
  harris_threshold_ = 8.0;

  queues_ = new DistinctQueue(window_size_, queue_size_, true);

  Eigen::VectorXd Dx = Eigen::VectorXd(kernel_size_);
  Eigen::VectorXd Sx = Eigen::VectorXd(kernel_size_);
  for (int i=0; i<kernel_size_; i++)
  {
    Sx[i] = factorial(kernel_size_ - 1)/
            (factorial(kernel_size_ - 1 - i) * factorial(i));
    Dx[i] = pasc(i, kernel_size_-2) - pasc(i-1, kernel_size_-2);
  }
  Gx_ = Sx * Dx.transpose();
  Gx_ = Gx_ / Gx_.maxCoeff();

  const double sigma = 1.;
  const double A = 1./(2.*M_PI*sigma*sigma);
  const int l2 = (2*window_size_+2-kernel_size_)/2;
  h_ = Eigen::MatrixXd(2*l2+1, 2*l2+1);
  for (int x=-l2; x<=l2; x++)
  {
    for (int y=-l2; y<=l2; y++)
    {
      const double h_xy = A * exp(-(x*x+y*y)/(2*sigma*sigma));
      h_(l2+x, l2+y) = h_xy;
    }
  }
  h_ /= h_.sum();
  
  // Start : Code 1 for ANMS
  sae_[0] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
  sae_[1] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
  sae_latest_[0] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
  sae_latest_[1] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
  ssae_[0] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
  ssae_[1] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
  // End : Code 1 for ANMS
}

evHarrisANMSDetector::~evHarrisANMSDetector()
{
}

struct IndexedValue{
    double value;
    int x;
    int y;
};

bool IVCompare(const IndexedValue& Ihs, const IndexedValue& Rhs){
        return Ihs.value < Rhs.value;
}

double lambda_sae(double tival,double tau_smooth)
{
    if (tival < 4*tau_smooth)
    {
        return exp(-tival/(20*tau_smooth));
    }
    else
    {
        return 0.0;
    }
}

bool evHarrisANMSDetector::isFeature(const dvs_msgs::Event &e, int & F1, int & F2)
{
  // update queues
  queues_->newEvent(e.x, e.y, e.polarity);

  // check if queue is full
  double score = harris_threshold_ - 10.;
  if (queues_->isFull(e.x, e.y, e.polarity))
  {
    // check if current event is a feature
    score = getHarrisScore(e.x, e.y, e.polarity);

    last_score_ = score;
  }

  F1 = score>= harris_threshold_ ? 1 : 0;
  
  
  // Start : Code 2 for ANMS
  const int pol = e.polarity ? 1 : 0;
  const int pol_inv = (!e.polarity) ? 1 : 0;
  double & t_last = sae_latest_[pol](e.x,e.y);
  double & t_last_inv = sae_latest_[pol_inv](e.x, e.y);
  double et=e.ts.toSec();
  
  // SSAE updated with SAE*
  score = score > 16.0 ? 16.0 : score; // Top limit
  score = score < harris_threshold_ ? 0.0 : score; // Bottom limit
  
  
  F2=1;
  if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) 
  {
    t_last = et;
    sae_[pol](e.x, e.y) = et;
    //ssae_[pol](e.x,e.y) = score;
  } 
  else 
  {
    t_last = et;
    sae_[pol](e.x, e.y) = et;
    //ssae_[pol](e.x,e.y) = score;
    //return false;
  }
  
  if (score < harris_threshold_)
  {
    F2 = 0;
    return false;
  }
  
// start Tau

  std::vector<IndexedValue> local_sae;
  
  for (int i=-tau_size_;i<=tau_size_;++i)
  {
      for (int j=-tau_size_;j<tau_size_;++j)
      {
          if (i==0 && j==0)
            continue;
          local_sae.push_back({et-sae_[pol](e.x+i,e.y+j),e.x+i,e.y+j});
      }
  }
  
  std::partial_sort(local_sae.begin(),local_sae.begin()+5,local_sae.end(),IVCompare);
  double tau_aver = (local_sae[0].value + local_sae[1].value + local_sae[2].value + local_sae[3].value + local_sae[4].value)/5.0;
  tau_aver = std::min(std::max(tau_aver,5e-5),0.05);
  
//end tau 


  
// start ANMS
  ssae_[pol](e.x,e.y) = score;
  int nms_dssae =0;
  for (int i=-nms_size_;i<=nms_size_;++i)
  {
    for (int j=-nms_size_;j<nms_size_;++j)
    {
        if (i==0 && j==0)
            continue;
        //if (exp(-(et-sae_[pol](e.x+i,e.y+j))/(20*tau_smooth))*ssae_[pol](e.x+i,e.y+j) > score )
        if (lambda_sae(et-sae_[pol](e.x+i,e.y+j),tau_aver)*ssae_[pol](e.x+i,e.y+j) > score)
        {
            
            nms_dssae ++;
        }
    }
  }
  
  
  if (nms_dssae > 1 || F1==0)
  {
      F2=0;
      return false;
  }
//end ANMS
  
  return true;
  // End : Code 3 for ANMS
}

double evHarrisANMSDetector::getHarrisScore(int img_x, int img_y, bool polarity)
{
  // do not consider border
  if (img_x<window_size_ or img_x>sensor_width_-window_size_ or
      img_y<window_size_ or img_y>sensor_height_-window_size_)
  {
    // something below the threshold
    return harris_threshold_ - 10.;
  }

  const Eigen::MatrixXi local_frame = queues_->getPatch(img_x, img_y, polarity);

  const int l = 2*window_size_+2-kernel_size_;
  Eigen::MatrixXd dx = Eigen::MatrixXd::Zero(l, l);
  Eigen::MatrixXd dy = Eigen::MatrixXd::Zero(l, l);
//  Eigen::MatrixXd dxy = Eigen::MatrixXd::Zero(l, l);
  for (int x=0; x<l; x++)
  {
    for (int y=0; y<l; y++)
    {
      for (int kx=0; kx<kernel_size_; kx++)
      {
        for (int ky=0; ky<kernel_size_; ky++)
        {
          dx(x, y) += local_frame(x+kx, y+ky)*Gx_(kx, ky);
          dy(x, y) += local_frame(x+kx, y+ky)*Gx_(ky, kx);
        }
      }
    }
  }

  double a=0., b=0., d=0.;
  for (int x=0; x<l; x++)
  {
    for (int y=0; y<l; y++)
    {
      a += h_(x, y) * dx(x, y) * dx(x, y);
      b += h_(x, y) * dx(x, y) * dy(x, y);
      d += h_(x, y) * dy(x, y) * dy(x, y);
    }
  }

  const double score = a*d-b*b - 0.04*(a+d)*(a+d);

  return score;
}


int evHarrisANMSDetector::factorial(int n) const
{
  if (n > 1)
  {
    return n * factorial(n - 1);
  }
  else
  {
    return 1;
  }
}

int evHarrisANMSDetector::pasc(int k, int n) const
{
  if (k>=0 && k<=n)
  {
    return factorial(n)/(factorial(n-k)*factorial(k));
  }
  else
  {
    return 0;
  }
}

} // namespace
