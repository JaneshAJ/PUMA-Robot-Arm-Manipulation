#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
                Sampler(),
                randDistribution(0, 1),
                randEngine(::std::random_device()()),
                gaussDistribution(0, 1),
                gaussEngine(::std::random_device()()),
                ratio(0.99f)
        {
        }

        ::std::normal_distribution<::rl::math::Real>::result_type
        YourSampler::gauss()
        {
            return this->gaussDistribution(this->gaussEngine);
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            ::rl::math::Vector rand(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
                rand(i) = this->rand();
            }

            return this->model->generatePositionUniform(rand);
        }

        ::rl::math::Vector
        YourSampler::generateCollisionFree()
        {
            return this->gaussSampler();
        }

        ::rl::math::Vector
        YourSampler::bridgeSampler() {
            // Based on bridge sampler from the Robotics Library

            this->sigma.resize(this->model->getDof());
            // setting the variance
            this->sigma << 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f;

            if (this->rand() > this->ratio) {
                return this->generate();
            } else {
                ::rl::math::Vector q(this->model->getDof());
                ::rl::math::Vector q3(this->model->getDof());

                while (true) {
                    ::rl::math::Vector q2 = this->generate();

                    this->model->setPosition(q2);
                    this->model->updateFrames();

                    //check q2 for collision
                    if (this->model->isColliding()) {
                        // generate smaple from gaussian distribution with mean q2
                        for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
                            q3(i) = this->gauss() * this->sigma(i) + q2(i);
                        }

                        this->model->clip(q3);

                        this->model->setPosition(q3);
                        this->model->updateFrames();

                        // if q3 is colliding, and the middle-point between q2 and q3 is not, return the middle-point
                        if (this->model->isColliding()) {
                            this->model->interpolate(q2, q3, 0.5f, q);

                            this->model->setPosition(q);
                            this->model->updateFrames();

                            if (!this->model->isColliding()) {
                                return q;
                            }
                        }
                    }
                }
            }
        }

        ::rl::math::Vector
        YourSampler::gaussSampler() {
            // Based on gaussaian sampler from the Robotics Library

            this->sigma.resize(this->model->getDof());
            this->sigma << 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f;

            ::rl::math::Vector q2(this->model->getDof());

            while (true) {
                // generating an uniform sample
                ::rl::math::Vector q = this->generate();

                // generating other sample with mean "q"
                for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
                    q2(i) = this->gauss() * this->sigma(i) + q(i);
                }

                this->model->setPosition(q);
                this->model->updateFrames();

                // checking whether q is colliding
                if(this->model->isColliding()) {

                    this->model->setPosition(q2);
                    this->model->updateFrames();

                    // returning q2 if it is not colliding
                    if(!this->model->isColliding()){
                        return q2;
                    }
                } else {
                    // return q if q2 is colliding
                    this->model->setPosition(q2);
                    this->model->updateFrames();

                    if(this->model->isColliding()){
                        return q;
                    }
                }
            }
        }

        ::std::uniform_real_distribution<::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
            this->gaussEngine.seed(value);
        }
    }
}

