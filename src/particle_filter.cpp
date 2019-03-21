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
#include <float.h>

#include "helper_functions.h"

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    num_particles = 100;

    std::default_random_engine gen;
    std::normal_distribution<double> noise_x(x, std[0]);
    std::normal_distribution<double> noise_y(y, std[1]);
    std::normal_distribution<double> noise_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++)
    {
        Particle p;
        p.id = i;
        p.x = noise_x(gen);
        p.y = noise_y(gen);
        p.theta = noise_theta(gen);
        p.weight = 1.0;

        particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
    double velocity, double yaw_rate)
{
    std::default_random_engine gen;
    std::normal_distribution<double> noise_x(0, std_pos[0]);
    std::normal_distribution<double> noise_y(0, std_pos[1]);
    std::normal_distribution<double> noise_theta(0, std_pos[2]);

    double vel_yawrate = (yaw_rate != 0) ? velocity / yaw_rate : 0;
    double yawrate_deltat = yaw_rate * delta_t;

    std::vector<Particle>::iterator iter = particles.begin();
    while (iter != particles.end())
    {
        if (yaw_rate != 0)
        {
            iter->x = iter->x + vel_yawrate * (sin(iter->theta + yawrate_deltat) - sin(iter->theta)) + noise_x(gen);
            iter->y = iter->y + vel_yawrate * (cos(iter->theta) - cos(iter->theta + yawrate_deltat)) + noise_y(gen);
            iter->theta = iter->theta + yawrate_deltat + noise_theta(gen);
        }
        else
        {
            iter->x = iter->x + velocity * delta_t * cos(iter->theta) + noise_x(gen);
            iter->y = iter->y + velocity * delta_t * sin(iter->theta) + noise_y(gen);
            iter->theta = iter->theta + noise_theta(gen);
        }

        iter++;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    const std::vector<LandmarkObs> &observations,
    const Map &map_landmarks)
{
    std::vector<Particle>::iterator part = particles.begin();
    while (part != particles.end())
    {
        std::vector<int> associations;
        std::vector<double> sense_x;
        std::vector<double> sense_y;

        double weight = 1;

        //transform observed landmarks to global CS
        std::vector<LandmarkObs>::const_iterator obs = observations.begin();
        while (obs != observations.end())
        {
            //transform observed landmarks to global CS
            LandmarkObs obs_map;
            obs_map.x = part->x + cos(part->theta) * obs->x - sin(part->theta) * obs->y;
            obs_map.y = part->y + sin(part->theta) * obs->x + cos(part->theta) * obs->y;

            //associate each observation with a landmark using nearest neighbour
            double min_dist = DBL_MAX;
            Map::single_landmark_s bastLandmark;

            std::vector<Map::single_landmark_s>::const_iterator mark = map_landmarks.landmark_list.begin();
            while (mark != map_landmarks.landmark_list.end())
            {
                double distance = dist(mark->x_f, mark->y_f, obs_map.x, obs_map.y);
                if (distance < min_dist)
                {
                    min_dist = distance;
                    bastLandmark = *mark;
                }
                mark++;
            }
            obs_map.id = bastLandmark.id_i;

            //compute weight
            weight *= multivar_gauss(obs_map.x, obs_map.y, bastLandmark.x_f, bastLandmark.y_f, std_landmark[0], std_landmark[1]);
            
            //associate particle with observations for visualisation purposes
            associations.push_back(obs_map.id);
            sense_x.push_back(obs_map.x);
            sense_y.push_back(obs_map.y);

            obs++;
        }

        SetAssociations(*part, associations, sense_x, sense_y);

        // update weight
        part->weight = weight;
        part++;
    }
}

void ParticleFilter::resample()
{
    //normalize weights and save max
    double w_sum = 0;
    std::vector<Particle>::iterator part = particles.begin();
    while (part != particles.end())
    {
        w_sum += part->weight;
        part++;
    }
    if (w_sum == 0)
    {
        printf("\nERROR: invalid weights\n\n");
        return;
    }

    double w_max = -DBL_MAX;
    part = particles.begin();
    while (part != particles.end())
    {
        part->weight /= w_sum;
        if (w_max < part->weight)
        {
            w_max = part->weight;
        }
        part++;
    }

    std::default_random_engine gen;
    std::uniform_int_distribution<int> index_distr(0, num_particles-1);
    std::uniform_real_distribution<double> weight_distr(0, 2 * w_max);

    int index = index_distr(gen);
    double beta = 0;

    std::vector<Particle> particles_new;

    for (unsigned int i = 0; i < num_particles; i++)
    {
        beta += weight_distr(gen);
        while (particles[index].weight < beta)
        {
            beta -= particles[index].weight;

            index++;
            if (index >= num_particles)
            {
                index %= num_particles;
            }
        }
        particles_new.push_back(particles[index]);        
    }

    particles = particles_new;
}

void ParticleFilter::SetAssociations(Particle& particle,
    const std::vector<int>& associations,
    const std::vector<double>& sense_x,
    const std::vector<double>& sense_y)
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

std::string ParticleFilter::getAssociations(Particle best)
{
    std::vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

std::string ParticleFilter::getSenseCoord(Particle best, std::string coord)
{
    std::vector<double> v;

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
    std::string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}