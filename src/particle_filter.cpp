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

    double vel_yawrate = velocity / yaw_rate;
    double yawrate_deltat = yaw_rate * delta_t;

    std::vector<Particle>::iterator iter = particles.begin();
    while (iter != particles.end())
    {
        iter->x = iter->x + vel_yawrate * (sin(iter->theta + yawrate_deltat) - sin(iter->theta)) + noise_x(gen);
        iter->y = iter->y + vel_yawrate * (cos(iter->theta) - cos(iter->theta + yawrate_deltat)) + noise_y(gen);
        iter->theta = iter->theta + yawrate_deltat + noise_theta(gen);

        iter++;
    }
}

//void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)
void ParticleFilter::dataAssociation(const Map &map_landmarks, std::vector<LandmarkObs>& observations)
{
    /**
     * TODO: Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */
    

    std::vector<Map::single_landmark_s>::const_iterator mark;
    std::vector<LandmarkObs>::iterator obs = observations.begin();
    while (obs != observations.end())
    {
        double min_dist = DBL_MAX;
        Map::single_landmark_s* min_dist_lm = NULL;

        mark = map_landmarks.landmark_list.begin();
        while (mark != map_landmarks.landmark_list.end())
        {
            double distance = dist(mark->x_f, mark->y_f, obs->x, obs->y);
            if (distance < min_dist)
            {
                min_dist = distance;
                min_dist_lm = mark._Ptr;
            }
            mark++;
        }

        obs->association = min_dist_lm;
        obs->id = min_dist_lm->id_i;
        obs++;
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

            //test eigen
            //Eigen::Vector3d obsv(obs->x, obs->y, 1);
            //Eigen::Vector3d obs_mapv = transform(obsv, part->x, part->y, part->theta, false);

            //associate each observation with a landmark
            double min_dist = DBL_MAX;
            Map::single_landmark_s* min_dist_lm = NULL;

            std::vector<Map::single_landmark_s>::const_iterator mark = map_landmarks.landmark_list.begin();
            while (mark != map_landmarks.landmark_list.end())
            {
                double distance = dist(mark->x_f, mark->y_f, obs_map.x, obs_map.y);
                if (distance < min_dist)
                {
                    min_dist = distance;
                    min_dist_lm = mark._Ptr;
                }
                mark++;
            }

            obs_map.association = min_dist_lm;
            obs_map.id = min_dist_lm->id_i;

            //compute weight
            weight *= multivar_gauss(obs_map.x, obs_map.y, obs_map.association->x_f, obs_map.association->y_f, std_landmark[0], std_landmark[1]);
            
            //associate particle with observations
            associations.push_back(obs_map.id);
            sense_x.push_back(obs_map.x);
            sense_y.push_back(obs_map.y);

            obs++;
        }

        SetAssociations(*part, associations, sense_x, sense_y);

        //TODO: use eigen to tranform map to particle CS and associate "precited" map with observations

        //transform from particle's CS to map CS
        //std::vector<Map::single_landmark_s>::const_iterator mark = map_landmarks.landmark_list.begin();
        //while (mark != map_landmarks.landmark_list.end())
        //{
        //    if (dist(mark->x_f, mark->y_f, part->x, part->y) > sensor_range)
        //        continue;

        //    LandmarkObs obs; //predicted observation of map landmark in map CS
        //    obs.x = part->x + cos(part->theta * obs->x) - sin(part->theta * obs->y);
        //    obs.y = part->y + cos(part->theta * obs->x) + sin(part->theta * obs->y);
        //    
        //    map++;
        //    map_local++;
        //}

        //associate observations with map
        //dataAssociation(map_landmarks, observations_map);

        //double weight = 1;
        ////compute probability density of each observation
        //obs_map = observations_map.begin();
        //while (obs_map != observations_map.end())
        //{
        //    double w = multivar_gauss(obs_map.x, obs_map.y, obs_map.association->x_f, obs_map.association->y_f, std_landmark[0], std_landmark[1]);
        //    if (w == 0)
        //        w = DBL_MIN;
        //    weight *= w;
        //    obs_map++;
        //}

        part->weight = weight;
        part++;
    }
}

void ParticleFilter::resample()
{
    /**
     * TODO: Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     *
     * //Resample Wheel from Lesson 4
     * index = int(random.uniform(0,N-1))
     * print(index)
     * print(len(w))
     * beta = 0
     * for i in range(N):
     *     beta += random.uniform(0, 2* w_max)
     *     while w[index] < beta:
     *         beta -= w[index]
     *         index += 1
     *         index %= N
     *     p3.append(p[index])
     */

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
    printf("index= %d", index);
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