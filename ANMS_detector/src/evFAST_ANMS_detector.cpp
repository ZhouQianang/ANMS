#include "corner_event_detector/evFAST_ANMS_detector.h"

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

namespace corner_event_detector
{

evFASTANMSDetector::evFASTANMSDetector(bool connect)
: Detector(connect),
  circle3_ {{0, 3}, {1, 3}, {2, 2}, {3, 1},
            {3, 0}, {3, -1}, {2, -2}, {1, -3},
            {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
            {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
  circle4_ {{0, 4}, {1, 4}, {2, 3}, {3, 2},
            {4, 1}, {4, 0}, {4, -1}, {3, -2},
            {2, -3}, {1, -4}, {0, -4}, {-1, -4},
            {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
            {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}}
{
  detector_name_ = "evFAST_ANMS";

  // allocate SAE matrices
  sae_[0] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  sae_[1] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  sae_latest_[0] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  sae_latest_[1] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  ssae_[0] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  ssae_[1] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
}

evFASTANMSDetector::~evFASTANMSDetector()
{
}



bool evFASTANMSDetector::isFeature(const dvs_msgs::Event &e, int & F1, int & F2)
{
  // update SAE and SAE*
  const int pol = e.polarity ? 1 : 0;
  const int pol_inv = (!e.polarity) ? 1 : 0;
  double & t_last = sae_latest_[pol](e.x,e.y);
  double & t_last_inv = sae_latest_[pol_inv](e.x, e.y);
  double et=e.ts.toSec();
    
  if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) 
  {
    t_last = et;
    sae_[pol](e.x, e.y) = et;
  } 
  else 
  {
    t_last = et;
    sae_[pol](e.x, e.y) = et;
    //return false;
  }

  const int max_scale = 1;

  // only check if not too close to border
  const int cs = max_scale*4;
  if (e.x < cs || e.x >= sensor_width_-cs ||
      e.y < cs || e.y >= sensor_height_-cs)
  {
    return false;
  }
  
  // FAST score
  double score1=0.0,score2=0.0,score=0.0;

  bool found_streak = false;

  for (int i=0; i<16; i++)
  {
    for (int streak_size = 3; streak_size<=6; streak_size++)
    {
      // check that streak event is larger than neighbor
      if (sae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]) <  sae_[pol](e.x+circle3_[(i-1+16)%16][0], e.y+circle3_[(i-1+16)%16][1]))
        continue;

      // check that streak event is larger than neighbor
      if (sae_[pol](e.x+circle3_[(i+streak_size-1)%16][0], e.y+circle3_[(i+streak_size-1)%16][1]) <          sae_[pol](e.x+circle3_[(i+streak_size)%16][0], e.y+circle3_[(i+streak_size)%16][1]))
        continue;

      double min_t = sae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]);
      for (int j=1; j<streak_size; j++)
      {
        const double tj = sae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);
        if (tj < min_t)
          min_t = tj;
      }

      bool did_break = false;
      for (int j=streak_size; j<16; j++)
      {
        const double tj = sae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);

        if (tj >= min_t)
        {
          did_break = true;
          break;
        }
      }

      if (!did_break)
      {
        found_streak = true;
        // FAST score on circle 3
        score1 = double(16 - streak_size);
        break;
      }

    }
    if (found_streak)
    {
      break;
    }
  }

  if (found_streak)
  {
    found_streak = false;
    for (int i=0; i<20; i++)
    {
      for (int streak_size = 4; streak_size<=8; streak_size++)
      {
        // check that first event is larger than neighbor
        if (sae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]) <  sae_[pol](e.x+circle4_[(i-1+20)%20][0], e.y+circle4_[(i-1+20)%20][1]))
          continue;

        // check that streak event is larger than neighbor
        if (sae_[pol](e.x+circle4_[(i+streak_size-1)%20][0], e.y+circle4_[(i+streak_size-1)%20][1]) <          sae_[pol](e.x+circle4_[(i+streak_size)%20][0], e.y+circle4_[(i+streak_size)%20][1]))
          continue;

        double min_t = sae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]);
        for (int j=1; j<streak_size; j++)
        {
          const double tj = sae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
          if (tj < min_t)
            min_t = tj;
        }

        bool did_break = false;
        for (int j=streak_size; j<20; j++)
        {
          const double tj = sae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
          if (tj >= min_t)
          {
            did_break = true;
            break;
          }
        }

        if (!did_break)
        {
          found_streak = true;
          // FAST score on circle 4
          score2 = double(20-streak_size);
          break;
        }
      }
      if (found_streak)
      {
        break;
      }
    }
  }

  F1 = found_streak ? 1 : 0;
  
  // ANMS
  if(F1)
  {
      F2=1;
    score = (score1 + score2 - 15.0);
    
    
    // Tau same as evHarris_ANMS
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
    // end Tau
    
    ssae_[pol](e.x,e.y) = score;
    int nnms_dssae = 0;
    for (int i=-nms_size_;i<=nms_size_;++i)
    {
        for (int j=-nms_size_;j<nms_size_;++j)
        {
            if (i==0 && j==0)
                continue;
          
            if (lambda_sae(et-sae_[pol](e.x+i,e.y+j),tau_aver)*ssae_[pol](e.x+i,e.y+j) > score)
            {
                nnms_dssae ++;
            }
        }
    }
    if (nnms_dssae > 1)
    {
        F2 = 0;
    }
  } 
  else
  {
     ssae_[pol](e.x,e.y) = 0.0;
  }
  
  return true;
}

} // namespace
