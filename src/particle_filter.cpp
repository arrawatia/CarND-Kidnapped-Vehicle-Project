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

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;

  // Don't run it twice.
  if (is_initialized)
  {
    return;
  }

  // Noise
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Generate particles with initial value around the GPS measurement.
  // The values are sampled from a normal distributions with mean around the initial GPS measurement
  // and given standard dev.
  for (int i = 0; i < num_particles; i++)
  {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  double epsilon = 0.001;
  default_random_engine gen;

  for (int i = 0; i < num_particles; i++)
  {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    double pred_x;
    double pred_y;
    double pred_theta;

    // Use the correct set of equations for the motion model.
    if (fabs(yaw_rate) < epsilon)
    {
      pred_x = x + velocity * cos(theta) * delta_t;
      pred_y = y + velocity * sin(theta) * delta_t;
      pred_theta = theta;
    }
    else
    {
      pred_x = x + (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
      pred_y = y + (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
      pred_theta = theta + (yaw_rate * delta_t);
    }

    // Noise
    normal_distribution<double> dist_x(pred_x, std_pos[0]);
    normal_distribution<double> dist_y(pred_y, std_pos[1]);
    normal_distribution<double> dist_theta(pred_theta, std_pos[2]);

    // Sample the particle from the predicted position and heading to simulate sensor noise.
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  int n_obs = observations.size();
  int n_pred = predicted.size();

  for (int i = 0; i < n_obs; i++)
  {

    // Initialize min distance. Choose a big number because we want to find the lowest value.
    double min_distance = numeric_limits<double>::max();

    int map_id = -1;

    // Find the closest landmark
    for (int j = 0; j < n_pred; j++)
    {

      double x_distance = observations[i].x - predicted[j].x;
      double y_distance = observations[i].y - predicted[j].y;

      double distance = x_distance * x_distance + y_distance * y_distance;

      if (distance < min_distance)
      {
        min_distance = distance;
        map_id = predicted[j].id;
      }
    }

    // Update the observation.
    observations[i].id = map_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
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

  for (int i = 0; i < num_particles; i++)
  {

    double px = particles[i].x;
    double py = particles[i].y;
    double ptheta = particles[i].theta;

    // Step 1: Transform observations from vehicle coordinates to map coordinates.
    vector<LandmarkObs> xformed_obs;
    for (int j = 0; j < observations.size(); j++)
    {
      double t_x = cos(ptheta) * observations[j].x - sin(ptheta) * observations[j].y + px;
      double t_y = sin(ptheta) * observations[j].x + cos(ptheta) * observations[j].y + py;
      xformed_obs.push_back(LandmarkObs{observations[j].id, t_x, t_y});
    }

    //Step 2: Filter map landmarks to keep locations predicted to be within sensor range of the particle
    vector<LandmarkObs> predictions;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      float lx = map_landmarks.landmark_list[j].x_f;
      float ly = map_landmarks.landmark_list[j].y_f;
      int lid = map_landmarks.landmark_list[j].id_i;

      if (fabs(lx - px) <= sensor_range && fabs(ly - py) <= sensor_range)
      {
        predictions.push_back(LandmarkObs{lid, lx, ly});
      }
    }

    // Step 3: Associate transformed observations to predicted landmarks
    dataAssociation(predictions, xformed_obs);

    // Step 4: Calculate the importance weight of each particle
    particles[i].weight = 1.0;

    for (int j = 0; j < xformed_obs.size(); j++)
    {

      double ox = xformed_obs[j].x;
      double oy = xformed_obs[j].y;

      double px = 0;
      double py = 0;

      // Get coordinates of the prediction associated with the current observation
      int current_obs = xformed_obs[j].id;
      for (int k = 0; k < predictions.size(); k++)
      {
        if (predictions[k].id == current_obs)
        {
          px = predictions[k].x;
          py = predictions[k].y;
        }
      }

      // Calculate weight for this observation with multivariate Gaussian
      double sx = std_landmark[0];
      double sy = std_landmark[1];
      double obs_w = (1 / (2 * M_PI * sx * sy)) * exp(-(pow(px - ox, 2) / (2 * pow(sx, 2)) + (pow(py - oy, 2) / (2 * pow(sy, 2)))));

      // Multiply the weight with the total
      particles[i].weight *= obs_w;
    }
  }
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<Particle> particle_sample;

  vector<double> weights;
  for (int i = 0; i < num_particles; i++)
  {
    weights.push_back(particles[i].weight);
  }

  // Generate random starting index for resampling wheel
  default_random_engine gen;
  uniform_int_distribution<int> particle_index(0, num_particles - 1);
  int index = particle_index(gen);

  double max_weight = *max_element(weights.begin(), weights.end());

  // Uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> random_weight(0.0, max_weight);

  double beta = 0.0;

  // Spin the resampling wheel
  for (int i = 0; i < num_particles; i++)
  {
    beta += random_weight(gen) * 2.0;

    while (beta > weights[index])
    {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    particle_sample.push_back(particles[index]);
  }
  particles = particle_sample;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}