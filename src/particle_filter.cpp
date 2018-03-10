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
#define _USE_MATH_DEFINES
#include <iomanip>

#include "particle_filter.h"

using namespace std;
int haha;
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	int n_particles = 40;
	float std_x = std[0];
	float std_y = std[1];
	float std_theta = std[2];
	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);
	for (int i = 0; i< n_particles;i++){
		 Particle part;
		 part.id = i;
		 part.x = x + dist_x(gen);
		 part.y= y + dist_y(gen);
		 part.theta = theta + dist_theta(gen);
		 part.weight = 1.0;
		 particles.push_back(part);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine
	float std_x = std_pos[0];
	float std_y = std_pos[1];
	float std_theta = std_pos[2];
	vector<Particle> particles_new;
	
	for (int i = 0;i<particles.size();i++){
		float x_position = particles[i].x;
		float y_position = particles[i].y;
		float theta = particles[i].theta;
		float new_x_position;
		float new_y_position;
		float new_theta;
		if (fabs(yaw_rate)< 0.00001){
			new_x_position = velocity*delta_t*cos(particles[i].theta)+x_position;
			new_y_position = velocity*delta_t*sin(particles[i].theta)+y_position;
			new_theta = theta;
		} 
		else
		{
			new_x_position = x_position + (velocity/yaw_rate)*(sin(theta+yaw_rate*delta_t)-sin(theta));
			new_y_position = y_position + (velocity/yaw_rate)*(cos(theta)-cos(theta+yaw_rate*delta_t));
			new_theta = theta + yaw_rate*delta_t;
		}
		normal_distribution<double> dist_x(0, std_x);
		normal_distribution<double> dist_y(0, std_y);
		normal_distribution<double> dist_theta(0, std_theta);
		particles[i].x = new_x_position +dist_x(gen);
		particles[i].y = new_y_position +dist_y(gen);
		particles[i].theta = new_theta + dist_theta(gen);
		if(i==2){
		//cout<<"XXX "<< particles[i].x<<endl;
		//cout<<"YYYY "<< particles[i].y<<endl;
		//cout<<"new x "<< new_x_position<<endl;
		//cout<<"new y "<< new_y_position<<endl;
		//cout<<"old x "<< x_position<<endl;
		//cout<<"old y "<< y_position<<endl;
		//cout<<"velo "<< velocity<<endl;
		//cout<<"theta "<< theta<<endl;
		//cout<<"yaw rate "<< yaw_rate<<endl;
		//cout<<"delta"<< delta_t<<endl;
		//cout<<"dist x "<< particles[i].y<<endl;
		//cout<<"dist y "<< particles[i].y<<endl;
}
		particles_new.push_back(particles[i]);

	}
	particles = particles_new;
	
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i =0;i< observations.size();i++){
		LandmarkObs pred_min_dist;
		float min_dist=numeric_limits<double>::max();;
		for(int j =0;j< predicted.size();j++){
			//from helper file
			float distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
			if (distance < min_dist){
				min_dist = distance;
				pred_min_dist = predicted[j];
			}
		}
	observations[i].id = pred_min_dist.id;
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
	for (int i = 0; i< particles.size();i++){
		float x_position = particles[i].x;
		float y_position = particles[i].y;
		float theta = particles[i].theta;
		float obs_map_part_x;
		float obs_map_part_y;
		vector<LandmarkObs> observations_particle;
		vector<LandmarkObs> landmarks_particle_in_range;
		landmarks_particle_in_range.clear();
		observations_particle.clear();
		for (int j = 0; j< observations.size();j++){
			float obs_x = observations[j].x;
			float obs_y = observations[j].y;
			obs_map_part_x = x_position + cos(theta)*obs_x-sin(theta)*obs_y;
			obs_map_part_y = y_position + sin(theta)*obs_x+cos(theta)*obs_y;
			observations_particle.push_back(LandmarkObs{ j, obs_map_part_x, obs_map_part_y});
		}
		for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
			float x_map = map_landmarks.landmark_list[k].x_f;
			float y_map = map_landmarks.landmark_list[k].y_f;
			int id_map = map_landmarks.landmark_list[k].id_i;
			float dist_part_map = dist(x_position,y_position,x_map,y_map);
			if (dist_part_map <= sensor_range){
				landmarks_particle_in_range.push_back(LandmarkObs{ id_map, x_map,y_map});
			}
		}
		//Function assigns id of closest landmark to id of list landmarks_particle_in_range
		dataAssociation(landmarks_particle_in_range,observations_particle);
		particles[i].weight = 1.0;
		for (int j = 0; j< observations_particle.size();j++){
			float mu_x = map_landmarks.landmark_list[observations_particle[j].id-1].x_f;
			float mu_y = map_landmarks.landmark_list[observations_particle[j].id-1].y_f;
			//double gauss_norm = (1/(2*M_PI*std_landmark[0]*std_landmark[1]));
			//cout<< gauss_norm<<endl;
			//double exponent = (pow((observations_particle[j].x-mu_x),2))/(2*pow(std_landmark[0],2))+(pow((observations_particle[j].y-mu_y),2))/(2*pow(std_landmark[1],2));
			//double weight = gauss_norm *exp(-exponent);
			double weight = ( 1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp( -( pow(observations_particle[j].x-mu_x,2)/(2*pow(std_landmark[0], 2)) + (pow(observations_particle[j].y-mu_y,2)/(2*pow(std_landmark[1], 2))) ) );
			particles[i].weight *= weight;
			//cout<<particles[i].weight<<endl;
		}
	}
}

void ParticleFilter::resample(){ 
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<Particle> particles_resampled;
	vector<double> weights_particles;
	//weights.clear();
	//particles_resampled.clear();
	double max_weight = 0;
	for(int j = 0;j<particles.size();j++){
		double temp_weight = particles[j].weight;
		weights_particles.push_back(particles[j].weight);
		if (temp_weight> max_weight){
			max_weight = temp_weight;
		}
	}
	uniform_real_distribution<double> distribution(0,2*max_weight);
	int index = int(rand() % particles.size());
	double beta = 0.0;
	for(int j = 0;j<particles.size();j++){
		beta += distribution(gen);
		while(beta >weights_particles[index]){
			beta -=weights_particles[index];
			index = (index+1)%particles.size();
		}
		particles_resampled.push_back(particles[index]);
	}
	cout<<particles.size()<<endl;
	particles = particles_resampled;
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

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y){
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;}
