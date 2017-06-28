/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 50;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {

		Particle particle;

		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;

		particles.push_back(particle);
		weights.push_back(1);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double new_x;
	double new_y;
	double new_theta;

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	// Calculate new state
	for (int i = 0; i < num_particles; ++i) {
		if (fabs(yaw_rate) < 0.00001) {
		new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
		new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
		new_theta = particles[i].theta;
		}
		else
		{
		new_x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
		new_y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
		new_theta = particles[i].theta + yaw_rate * delta_t;
		}

		particles[i].x = new_x + dist_x(gen);
		particles[i].y = new_y + dist_y(gen);
		particles[i].theta = new_theta + dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); i++) {

		// Current observation
		LandmarkObs cur_obs = observations[i];

		double min_dist = 1000000.0;

		int map_id = -1;

		for (int j = 0; j < predicted.size(); j++) {
			// Current prediction
			LandmarkObs cur_pred = predicted[j];

			// Distance between observed and predicted landmarks
			double cur_dist = dist(cur_obs.x, cur_obs.y, cur_pred.x, cur_pred.y);

			// Associate predicted landmark nearest the current observed landmark
			if (cur_dist < min_dist) {
				min_dist = cur_dist;
				map_id = cur_pred.id;
			}
		}

		// set the observation's id to the nearest predicted landmark's id
		observations[i].id = map_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// For every particle associate a landmark and calculate a weight
	for (int i = 0; i < num_particles; i++) {
		// Particle x, y coordinates
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		// Hold the map landmark locations predicted to be within sensor range of the particle
		vector<LandmarkObs> predictions;

		// For each map landmark find particles inside the sensor range
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			float lm_x = map_landmarks.landmark_list[j].x_f;
			float lm_y = map_landmarks.landmark_list[j].y_f;
			int lm_id = map_landmarks.landmark_list[j].id_i;

			// Find the landmarks inside the sensor range
			if (dist(lm_x, lm_y, p_x, p_y) <= sensor_range) {
				predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
			}
		}

		// Translate and rotate observations in vehicle coordinates to map coordinates
		vector<LandmarkObs> transformed_obs;
		for (int j = 0; j < observations.size(); j++) {
			double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
			double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
			transformed_obs.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
		}

		// Perform dataAssociation to find observations closest to Landmarks
		dataAssociation(predictions, transformed_obs);

		particles[i].weight = 1.0;

		for (int j = 0; j < transformed_obs.size(); j++) {
			double obs_x, obs_y, pred_x, pred_y;
			obs_x = transformed_obs[j].x;
			obs_y = transformed_obs[j].y;

			int associated_prediction = transformed_obs[j].id;

			for (int k = 0; k < predictions.size(); k++) {
				if (predictions[k].id == associated_prediction) {
				  pred_x = predictions[k].x;
				  pred_y = predictions[k].y;
				}
			}

			// Calculate weight for this observation with multivariate Gaussian
			double std_x = std_landmark[0];
			double std_y = std_landmark[1];
			double obs_w = ( 1/(2*M_PI*std_x*std_y)) * exp( -( pow(pred_x-obs_x,2)/(2*pow(std_x, 2)) + (pow(pred_y-obs_y,2)/(2*pow(std_y, 2))) ) );

			particles[i].weight *= obs_w;
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<double> weights;
	for (int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
	}

	std::discrete_distribution<> distribution(weights.begin(), weights.end());

	std::vector<Particle> new_particles;
	new_particles.clear();

	for (int i = 0; i < num_particles; i++) {
		int sampled_index = distribution(gen);
		new_particles.push_back(particles[sampled_index]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
// associations: The landmark id that goes along with each listed association
// sense_x: the associations x mapping already converted to world coordinates
// sense_y: the associations y mapping already converted to world coordinates

//Clear the previous associations
particle.associations.clear();
particle.sense_x.clear();
particle.sense_y.clear();

particle.associations= associations;
 particle.sense_x = sense_x;
 particle.sense_y = sense_y;

 return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
vector<int> v = best.associations;
stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
vector<double> v = best.sense_x;
stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
vector<double> v = best.sense_y;
stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

