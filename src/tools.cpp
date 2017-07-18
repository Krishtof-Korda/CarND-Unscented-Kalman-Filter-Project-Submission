#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  int n = estimations.size();
  int g = ground_truth.size();
  //cout << "number of estimations = " << n << endl;
  //cout << "number of ground truths = " << g << endl;
  
  if(n==0 || n!=g) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for(int i=0; i < n; ++i) {
    
    //cout << "i = " << i << endl;
    VectorXd residuals = estimations[i]-ground_truth[i];
    residuals = residuals.cwiseProduct(residuals);
    rmse += residuals;
    //cout << "residuals = " << residuals << endl << "sum = " << rmse << endl;
  }
  
  //calculate the mean
  rmse = rmse/n;
  //cout << "mean = " << rmse << endl;
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  //cout << "RMSE = " << rmse << endl;
  
  //return the result
  return rmse;
}
