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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 100;
	default_random_engine gen;
	
	// This line creates normal (Gaussian) distributions for x, y and theta.
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	

	for (int i = 0; i < num_particles; i++) {
		Particle part;
		part.id = i;
		part.x = dist_x(gen);
		part.y = dist_y(gen);
		part.theta = dist_theta(gen);
		part.weight = 1.0;
		weights.push_back(1.0);
		particles.push_back(part);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	
	// Gaussian noise for x, y and theta.
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {
			double x_p, y_p, yaw;
			x_p = particles[i].x;
			y_p = particles[i].y;
			yaw = particles[i].theta;

			if (fabs(yaw_rate) > 0.001) {
        		x_p += (velocity/yaw_rate) * ( sin (yaw + yaw_rate*delta_t) - sin(yaw)) + dist_x(gen);
        		y_p += (velocity/yaw_rate) * ( cos(yaw) - cos(yaw+yaw_rate*delta_t) ) + dist_y(gen);
    		}
    		else {
        		x_p += velocity*delta_t*cos(yaw) + dist_x(gen);
        		y_p += velocity*delta_t*sin(yaw) + dist_y(gen);
    		}
    		yaw += yaw_rate*delta_t + dist_theta(gen);
    		particles[i].x = x_p;
    		particles[i].y = y_p;
    		particles[i].theta = yaw;

	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

// this function is not well-suited since "observations" are passed in the car coordinates

/*
	for (int o = 0; o < observations.size(); o++) {
		for (int p = 0; p < predicted.size(); p++) {
			double dist = (observations[o].x - predicted[p].x)*(observations[o].x - predicted[p].x) + (observations[o].y - predicted[p].y)*(observations[o].y - predicted[p].y);
			if (m == 0) {
				double dist_min = dist;
				observations[o].id = predicted[p].id;
			}
			else if (dist < dist_min) {
				dist_min = dist;
				observations[o].id = predicted[p].id;
			}
		}
	}
	*/
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];
	double gauss_norm = 1/(2*M_PI*sig_x*sig_y);
	double exponent;

	for (int i = 0; i < num_particles; i++) {

		// particle coordinates in map frame
		double x_p_m, y_p_m, yaw_m;
		x_p_m = particles[i].x;
		y_p_m = particles[i].y;
		yaw_m = particles[i].theta;

		double new_weight = 1.0;

		for (int o = 0; o < observations.size(); o++) {


			// transformation of observation from car to map coordinates
			double x_obs_m, y_obs_m;
			x_obs_m = x_p_m + cos(yaw_m)*observations[o].x - sin(yaw_m)*observations[o].y;
			y_obs_m = y_p_m + sin(yaw_m)*observations[o].x + cos(yaw_m)*observations[o].y;

			// using squared euclidean distance for nearest neighbor match
			double distsq_min, x_difsq_min, y_difsq_min;
			distsq_min = sensor_range*sensor_range;
			x_difsq_min = y_difsq_min = 0.5*distsq_min;

			for (int m = 0; m < map_landmarks.landmark_list.size(); m++) {

				double x_l_m = map_landmarks.landmark_list[m].x_f;
				double y_l_m = map_landmarks.landmark_list[m].y_f;

				double x_difsq = (x_obs_m-x_l_m)*(x_obs_m-x_l_m);
				double y_difsq = (y_obs_m-y_l_m)*(y_obs_m-y_l_m);
				double distsq =  x_difsq + y_difsq;

				if (distsq < distsq_min) {
					distsq_min = distsq;
					x_difsq_min = x_difsq;
					y_difsq_min = y_difsq;
				}
			}

			exponent = x_difsq_min/(2*sig_x*sig_x) + y_difsq_min/(2*sig_y*sig_y);
			new_weight *= gauss_norm*exp(-exponent);
		}
		// update particle weight
	    particles[i].weight = weights[i] = new_weight;	
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
    discrete_distribution<> weights_dist(weights.begin(), weights.end());
   
    vector<Particle> new_particles;
    
    for (int i = 0; i < num_particles; i++) {
        new_particles.push_back(particles[weights_dist(gen)]);
    }

    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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
