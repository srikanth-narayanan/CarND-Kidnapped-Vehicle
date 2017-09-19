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

// Random Engine global value
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
    
    num_particles = 10;
    
    // resize particle and weights
    particles.resize(num_particles);
    weights.resize(num_particles);
    
    // Standard deviations for x, y, and yaw
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];
    
	// Ditribution to add random Gaussian noise to each particle.
    normal_distribution<double> dist_x(x,std_x);
    normal_distribution<double> dist_y(y,std_y);
    normal_distribution<double> ang_theta(theta,std_theta);
    
    
    //Initialise all Particles
    for (int i = 0; i < num_particles; i++){
        
        Particle ptl;
        
        ptl.id = i;
        ptl.x = dist_x(gen);
        ptl.y = dist_y(gen);
        ptl.theta = ang_theta(gen);
        ptl.weight = 1.0;
        
        particles.push_back(ptl);
    }
    
    is_initialized = true;
    
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // Standard deviations for x, y, and yaw
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];
    
    // Ditribution to add random Gaussian noise to each particle.
    normal_distribution<double> dist_x(0,std_x);
    normal_distribution<double> dist_y(0,std_y);
    normal_distribution<double> ang_theta(0,std_theta);
    
    // Predict all particle x, y and theta based on velocity and Yawrate
    // A simple bicycle model is used ( Equations change depends on Yawrate is zero or greater
    for(int i = 0; i < num_particles; i++){
        // if yaw rate is equal to zero
        if (fabs(yaw_rate < 0.00001)){
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        }
        else{
            double up_theta = particles[i].theta + yaw_rate * delta_t;
            particles[i].x += (velocity / yaw_rate) * (sin(up_theta) - sin(particles[i].theta));
            particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(up_theta));
            particles[i].theta = up_theta;
        }
        
        // Include Gaussian noise in predicted value using sensor uncertainity
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += ang_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    for (int i = 0; i < observations.size(); i++){
        // Current observation
        LandmarkObs curr_obs = observations[i];
        
        // temp variable minimum distance. Initialise a very high value to start
        double mindist = numeric_limits<double>::max();
        
        // temp variable to hold map id
        int mid = -1;
        
        for (int j = 0; j < predicted.size(); j++){
            
            //current predicted
            LandmarkObs curr_pred = predicted[j];
            
            //calculate distance
            double curr_dist = dist(curr_obs.x, curr_obs.y, curr_pred.x, curr_pred.y);
            
            //check if lesser than min distance
            if(curr_dist < mindist){
                mindist = curr_dist;
                mid = curr_pred.id;
            }
        }
        observations[i].id = mid;
    }
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
    
    // Update weight of every particle
    for (int i = 0; i < num_particles; i++){
        
        // current particle values
        double px = particles[i].x;
        double py = particles[i].y;
        double ptheta = particles[i].theta;
        
        
        // Landmarks within the sensor range to hold predicted value
        std::vector<LandmarkObs> predicted;
        
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++){
            
            // get current data
            float lnd_x = map_landmarks.landmark_list[j].x_f;
            float lnd_y = map_landmarks.landmark_list[j].y_f;
            int lnd_id = map_landmarks.landmark_list[j].id_i;
            
            // consider landmark only within sensor range
            double lnd_dist = dist(px, py, lnd_x, lnd_y);
            if(lnd_dist <= sensor_range){
                predicted.push_back(LandmarkObs{lnd_id, lnd_x, lnd_y});
            }
        }
        
        // convert list of observation from vehicle co-ordinate to map co-ordinate
        std::vector<LandmarkObs> trans_observed;
        
        for (int k = 0; k < observations.size(); k++){
            double trs_x = px + (cos(ptheta) * observations[k].x - sin(ptheta) * observations[k].y);
            double trs_y = py + (sin(ptheta) * observations[k].x + cos(ptheta) * observations[k].y);
            
            trans_observed.push_back(LandmarkObs{observations[k].id, trs_x, trs_y});
        }
        
        // data association of the observed data to the predicted data with min distance
        dataAssociation(predicted, trans_observed);
        
        // Reinit weight
        particles[i].weight = 1.0;
        
        // update weight for each particle
        for (int m = 0; m < trans_observed.size(); m++){
            
            // current observed landmark value
            double obs_x = trans_observed[m].x;
            double obs_y = trans_observed[m].y;
            int obs_id = trans_observed[m].id;
            
            double prd_x, prd_y;
            
            for(int l = 0; l < predicted.size(); l++){
                if(predicted[l].id == obs_id){
                    prd_x = predicted[l].x;
                    prd_y = predicted[l].y;
                }
            }
            
            // inputs for multivariate gaussian distribution
            double sig_x = std_landmark[0];
            double sig_y = std_landmark[1];
            
            // calculate normal term
            double gauss_norm = (1 / (2 * M_PI * sig_x * sig_y));
            
            // calculate exponent
            double exponent = (pow(obs_x-prd_x,2) / (2 * pow(sig_x, 2))) + (pow(obs_y - prd_y,2) / (2 * pow(sig_y, 2)));
            
            // calculate weight of a particle
            double p_weight = gauss_norm * exp(-exponent);
            
            // update weight
            particles[i].weight += p_weight;
        }
        weights[i] = particles[i].weight;
    }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    // new resample particles
    std::vector<Particle> resampled_particle;
    resampled_particle.resize(num_particles);
    
    //Random particle distribution with probability based on weight
    std::discrete_distribution<> discdist(weights.begin(), weights.end());
    
    // draw random particles based on the importance weight probability distribution
    for (int i = 0; i < num_particles; ++i){
        resampled_particle[i] = particles[discdist(gen)];
    }
    
    particles = resampled_particle;
    
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
