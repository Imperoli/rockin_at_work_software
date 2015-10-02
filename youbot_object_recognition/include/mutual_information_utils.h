#ifndef MUTUAL_INFORMATION_UTILS_
#define MUTUAL_INFORMATION_UTILS_

#include "utils.h"
#include <iostream>
#include <random>

void projOnImage(const std::vector<cv::Point2f>& proj_pts, float scale, cv::Mat& im)
{
  for(int i=0; i<proj_pts.size(); i++){
    cv::Point2f p=proj_pts[i];
    p.x/=scale; p.y/=scale;
    if(p.x>0&&p.x<im.cols&&p.y>0&&p.y<im.rows)
    {
      im.at<float>(p)=0;
    }
  }
}

void extractParticles(const std::vector<ObjectCandidate>& candidates, const std::vector<double>& priors, std::vector<ObjectCandidate>& particles)
{
  particles.clear();
  do{
    for(int i=0; i<candidates.size(); i++){
      double r = (double)(rand() % 1000000)/1000000.0f;
      if(r>priors[i]) continue;
      
      std::default_random_engine generator;
      
      std::normal_distribution<double> distribution_tx(candidates[i].pose.translation()(0),1.0);
      std::normal_distribution<double> distribution_ty(candidates[i].pose.translation()(1),1.0);
      std::normal_distribution<double> distribution_tz(candidates[i].pose.translation()(2),1.0);
      Eigen::Vector3d t(distribution_tx(generator), distribution_ty(generator), distribution_tz(generator));
      
      Eigen::Quaterniond q(candidates[i].pose.linear());
      double qw=q.w(); double qx=q.x(); double qy=q.y(); double qz=q.z();
      if((qw>=0&&qw<.001)||(qw<0&&qw>-.001)) continue;
      qx/=qw; qy/=qw; qz/=qw; qw=1;
      
      std::normal_distribution<double> distribution_qx(qx,.01);
      std::normal_distribution<double> distribution_qy(qy,.01);
      std::normal_distribution<double> distribution_qz(qz,.01);
      qx=distribution_qx(generator); qy=distribution_qy(generator); qz=distribution_qz(generator);
      
      double normalization=sqrt((qw*qw) + (qx*qx) + (qy*qy) + (qz*qz));
      qw/=normalization; qx/=normalization; qy/=normalization; qz/=normalization;
      
      Eigen::Quaterniond q_new(qw,qx,qy,qz);
      
      Eigen::Affine3d T; T.linear()=Eigen::Matrix3d(q_new); T.translation()=t;
      ObjectCandidate particle; particle.pose=T;
      particles.push_back(particle);
    }
  }while(particles.size()<70);
}

void computeObservations(RasterObjectModel3DPtr& obj_model_ptr, const cv::Size& im_size, const std::vector<ObjectCandidate>& candidates,  const std::vector<Eigen::Affine3d>& views, cv::Mat& indices, std::vector<cv::Mat>& observations)
{
  observations.clear();
  indices=cv::Mat(candidates.size(),views.size(),CV_32SC1,cv::Scalar(-1));
  int idx=0;
  for(int c=0; c<candidates.size(); c++){
    for(int v=0; v<views.size(); v++){
      cv::Mat im=cv::Mat::ones(im_size, CV_32FC1);
      Eigen::Affine3d T=views[v].inverse()*candidates[c].pose;
      Eigen::Quaterniond q(T.linear());
      obj_model_ptr->setModelView(q,T.translation());
      obj_model_ptr->storeModelView();
      indices.at<int>(c,v)=idx;
      std::vector<cv::Point2f> proj_pts;
      obj_model_ptr->projectRasterPoints (q, T.translation(), proj_pts);
      projOnImage(proj_pts, 2, im);
      cv::distanceTransform(im,im, CV_DIST_L1, 3);
      observations.push_back(im);
      idx++;
    }
  }
}

/*void computePriorProb(const std::vector<ObjectCandidate>& candidates, std::vector<double>& priors)
{
  priors.clear();
  double tot=0;
  for(int i=0; i<candidates.size(); i++){
    double prob=1/(1+candidates[i].avgDist);
    tot+=prob;
    priors.push_back(prob);
  }
  for(int i=0;i<priors.size();i++){
    priors[i]=priors[i]/tot;
  }
}*/

void computePriorProb(const std::vector<ObjectCandidate>& candidates, std::vector<double>& priors)
{
  priors.clear();
  double max=0;
  for(int i=0; i<candidates.size(); i++){
    double prob=1/(1+candidates[i].avgDist);
    if(max<prob) max=prob;
    priors.push_back(prob);
  }
  for(int i=0;i<priors.size();i++){
    priors[i]=priors[i]/max;
  }
}

float computeDistance(const cv::Mat& dist_map, const std::vector<cv::Point2f>& proj_pts)
{
  float dist=0;
  int count=0;
  for(int i=0; i<proj_pts.size(); i++){
    if(proj_pts[i].x==-1||proj_pts[i].y==-1) continue;
    dist+=dist_map.at<float>(proj_pts[i]);
    count++;
  }
  dist/=count;
  return dist;
}

float computeTotalDistance(const cv::Mat& dist_map, const std::vector<cv::Point2f>& proj_pts)
{
  float dist=0;
  for(int i=0; i<proj_pts.size(); i++){
    if(proj_pts[i].x==-1||proj_pts[i].y==-1) continue;
    dist+=dist_map.at<float>(proj_pts[i]);
  }
  return dist;
}

void computeLikelihoodj(RasterObjectModel3DPtr& obj_model_ptr, const int i, const int k, const std::vector<cv::Mat>& observations, const cv::Mat& indices, std::vector<double>& likelihoodj)
{
  likelihoodj.clear();
  double tot=0;
  std::vector<cv::Point2f> proj_pts;
  obj_model_ptr->projectRasterPoints (indices.at<int>(i,k), proj_pts);
  for(int j=0; j<observations.size(); j++){
    double lik=1/(1+(double)computeDistance(observations[j], proj_pts));
    likelihoodj.push_back(lik);
    tot+=lik;
  }
  for(int j=0; j<likelihoodj.size(); j++){
    likelihoodj[j]/=tot;
  }
}

/*double computeOjDatoSk(const int n_particles, const std::vector< std::vector<double> >&likelihood, const int j)
{
  double ojsk=0;
  for(int i=0; i<n_particles; i++){
    //ojsk+=likelihood[i][j]*priors[i];
    ojsk+=likelihood[i][j];
  }
  ojsk/=(double)n_particles;
  return ojsk;
}*/

void computeOjDatoSk(const int n_particles, const int n_observations, const std::vector< std::vector<double> >&likelihood, std::vector<double>& ojsk)
{
  ojsk.clear();
  for(int j=0; j<n_observations; j++){
    double ojsk_j=0;
    for(int i=0; i<n_particles; i++){
      ojsk_j+=likelihood[i][j];
    }
    ojsk_j/=(double)n_particles;
    ojsk.push_back(ojsk_j);
  }
}

double computeLikelihood(RasterObjectModel3DPtr& obj_model_ptr, const int k, const int n_particles, const std::vector<cv::Mat>& observations, const cv::Mat& indices, std::vector< std::vector<double> >& likelihood)
{
  likelihood.clear();
  for(int i=0; i<n_particles; i++){
    std::vector<double> likelihoodj;
    computeLikelihoodj(obj_model_ptr, i, k, observations, indices, likelihoodj);
    likelihood.push_back(likelihoodj);
  }
}

double computeIk( const int n_particles, const int n_observation, const std::vector< std::vector<double> >& likelihood, const std::vector<double>& ojsk)
{
  double I=0;
  for(int i=0; i<n_particles; i++){
    for(int j=0; j<n_observation; j++){
      I+=likelihood[i][j]*log(likelihood[i][j]/ojsk[j]);
    }
  }
  double n_inv=1/(double)n_particles;
  I*=n_inv;
  return I;
}

void bayesian_update(RasterObjectModel3DPtr& obj_model_ptr, const cv_ext::PinholeCameraModel& cam_model, const cv::Mat& image,
                      const std::vector<ObjectCandidate>& candidates, const Eigen::Affine3d& current_view, std::vector<double>& belief)
{
  
  ImageTensorPtr dist_map_tensor_ptr;
  computeDistanceMapTensor ( image, dist_map_tensor_ptr );
  
  DirectionalChamferMatching dc_matching(cam_model, dist_map_tensor_ptr);
  dc_matching.setTemplateModel( obj_model_ptr );
  dc_matching.enableVerbouseMode ( false);
  
  std::vector<double> priors;
  double max=0;
  Eigen::Affine3d current_view_inv=current_view.inverse();
  for (int i=0; i<candidates.size(); i++){
    Eigen::Affine3d T=current_view_inv*candidates[i].pose;
    Eigen::Quaterniond q(T.linear());

    double avg_d=dc_matching.getAvgDistance( q, T.translation() );
    double prob=1/(1+avg_d);
    if(max<prob) max=prob;
    priors.push_back(prob);
  }
  for(int i=0;i<priors.size();i++){
    priors[i]=priors[i]/max;
  }
  
  double tot=0;
  for (int i=0; i<belief.size(); i++){
    belief[i]*=priors[i];
    tot+=belief[i];
  }
  for (int i=0; i<belief.size(); i++){
    belief[i]/=tot;
  }
  
}

#endif


