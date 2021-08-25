/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  num_particles = 50;  // TODO: Set the number of particles
 
  
  std::default_random_engine gen;
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  Particle part_temp;	
  for (int i = 0; i < num_particles; i++) {
    
    
    part_temp.x = dist_x(gen);
    part_temp.y = dist_y(gen);
    part_temp.theta = dist_theta(gen);
    
    part_temp.weight=1;
    particles.push_back(part_temp);
    
  }
  
  is_initialized=true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  double meas_x,meas_y,meas_theta;
  
  for (int i = 0; i < num_particles; i++) {
      
      if (yaw_rate > .01){
      meas_x =  particles[i].x+velocity/yaw_rate*(sin(particles[i].theta+delta_t*yaw_rate)-sin(particles[i].theta));
      meas_y =  particles[i].y+velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+delta_t*yaw_rate));
      meas_theta= particles[i].theta+delta_t*yaw_rate;}
    else {
       meas_x =  particles[i].x+velocity*delta_t*cos(particles[i].theta);
      meas_y =  particles[i].y+velocity*delta_t*sin(particles[i].theta);
      meas_theta= particles[i].theta+delta_t*yaw_rate;
    }
      //add noise
      normal_distribution<double> dist_x(meas_x, std_pos[0]);
 	  normal_distribution<double> dist_y(meas_y, std_pos[1]);
 	  normal_distribution<double> dist_theta(meas_theta, std_pos[2]);
    
      particles[i].x = dist_x(gen);
      particles[i].y = dist_y(gen);
      particles[i].theta = dist_theta(gen);  
      //std::cout << particles[i].x <<" "<< particles[i].y <<  std::endl;  
     
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  //should assign each sensor observation, the map landmark associated with it. from lesson 5, Explanation of Project Code
  

  
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  //predict measurements to all the map landmarks with the sensor range for each particle, 
  //use above measurements to associate sensor measurements to map landmarks
/* code in pyhton
prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob*/
  
  double dist_tonearest_lm,prob;
  vector<LandmarkObs> transformed_obs;
  std::vector<int> landmarks_inrange;
  double weights_sum=0;
  for (int p = 0; p < num_particles; p++) {
  particles[p].associations.clear();
    particles[p].sense_x.clear();
    particles[p].sense_y.clear();
    
  prob=1;
   
  landmarks_inrange.clear();  
  transformed_obs.clear();  
  //perform transformations of observation  
  for (int i = 0; i < observations.size(); i++) { 
    //std::cout <<particles[p].x<<"  x<--particle-->y  "<<particles[p].y<< std::endl;
    //std::cout <<observations[i].y<<"  observations[i]  "<<observations[i].y<< std::endl;
    transformed_obs.push_back(  TransformObs(particles[p], observations[i])   );}  
    //calculate landmarks in sensor range only
  
  for (int i = 0; i < map_landmarks.landmark_list.size(); i++) {
     
    
    if (   (   fabs(particles[p].x - map_landmarks.landmark_list[i].x_f)  < sensor_range) &&
      	   (   fabs(particles[p].y - map_landmarks.landmark_list[i].y_f)  < sensor_range)
       ){
      landmarks_inrange.push_back(i);
         
     
      }
     
    
  //end of   
  }
  
  
  
  //associate each transformed obser with the nearest landmark  
  // loop through each observation and find nearest neighbor landmark in range  
  for (int i = 0; i < transformed_obs.size(); i++) {
    //loop through landmarks_inrange to associate each trans obs with the nearest neighbor
    dist_tonearest_lm=sensor_range*100; 
    double dist_to_lm;
    for (int j = 0; j < landmarks_inrange.size(); j++){
    	int landmark_index=landmarks_inrange[j];
    	
    	dist_to_lm=dist(transformed_obs[i].x,transformed_obs[i].y,
                               map_landmarks.landmark_list[landmark_index].x_f,
                               map_landmarks.landmark_list[landmark_index].y_f);
      
       if (dist_to_lm <dist_tonearest_lm){ 
        transformed_obs[i].id=landmark_index+1; //landmark id start from 1 not 0
        dist_tonearest_lm = dist_to_lm;
    }
      
  }
    //MultivProb(double sig_x, double sig_y, double x_obs, double y_obs,
                   //double mu_x, double mu_y)
  
    
    prob*=MultivProb(std_landmark[0],std_landmark[1],
                     transformed_obs[i].x,transformed_obs[i].y,
                     map_landmarks.landmark_list[transformed_obs[i].id-1].x_f,//index=id-1
                     map_landmarks.landmark_list[transformed_obs[i].id-1].y_f);//index=id-1
    particles[p].associations.push_back(transformed_obs[i].id);
    particles[p].sense_x.push_back(transformed_obs[i].x);
    particles[p].sense_y.push_back(transformed_obs[i].y);
    
      
   
  }
	particles[p].weight=prob;
    weights_sum+=particles[p].weight;
    //std::cout <<"particle "<<p<<" weight "<<prob*1e15;
  }// end of particles loop
  //normalize weights
  for (int p = 0; p < num_particles; p++) {  
    particles[p].weight=particles[p].weight/weights_sum;     }
  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	//inspired by https://stackoverflow.com/questions/65377120/using-c-discrete-distribution-with-weights-taken-from-a-data-structure
  std::default_random_engine gen;
  std::vector<double> weights;
  int resampled_id; 
  std::vector<Particle> resampled_particles;
 
  //create a weight vector for resampling
    for (int p = 0; p < num_particles; p++) {  
    weights.push_back(particles[p].weight);}
    
    std::discrete_distribution<> d(weights.begin(),weights.end());
    
    for (int i = 0; i < num_particles; i++){
        resampled_id=d(gen);  
        resampled_particles.push_back(particles[resampled_id]);}
        
    particles= resampled_particles; 
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}