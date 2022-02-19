#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Sampling startegies:
         * Uniform
         * Bridge
         * Gaussian
         * Magic
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler();

            ~YourSampler() override = default;

            ::rl::math::Vector generate() override;

            ::rl::math::Vector generateCollisionFree() override;

            ::rl::math::Vector bridgeSampler();

            ::rl::math::Vector gaussSampler();

            virtual void seed(const ::std::mt19937::result_type& value);

            ::rl::math::Vector sigma;

            /** Probability of choosing bridge sample. */
            ::rl::math::Real ratio;

        protected:
            ::std::uniform_real_distribution<::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution<::rl::math::Real> randDistribution;

            ::std::mt19937 randEngine;

            ::std::normal_distribution< ::rl::math::Real>::result_type gauss();

            ::std::normal_distribution< ::rl::math::Real> gaussDistribution;

            ::std::mt19937 gaussEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
