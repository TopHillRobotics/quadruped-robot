// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_GEOMETRY_H
#define QR_GEOMETRY_H

#include <cmath>
#include <iostream>

#include <type_traits>

#include "qr_algebra.h"
#include "qr_c_types.h"
#include "qr_se3.h"

    namespace math {

        /** 
         * @brief Linear interpolate between two point in state space.
         * @example 
         *  robotics::math::qrSegment<float, Vec6<float>> qrSegment;
         *  Vec6<float> s = Vec6<float>::Zero();
         *  Vec6<float> t;
         *  t<< 1, 1, 1, 1, 0, 0;
         *  qrSegment.Reset(s,t);
         *  Vec6<float> midpoint = qrSegment.GetPoint(0.9);
         *  cout << midpoint << endl;
         * 
         */
        template <class t=float, class T=Vec6<t>>
        class qrSegment {
            public:
                qrSegment() {}
                qrSegment(T source_, T dest_) {
                    source = source_;
                    dest = dest_;
                }
                ~qrSegment() = default;
                T GetPoint(float phase) {
                    if (phase > 1.0) {
                        phase = 1.0;
                    } else if (phase < 0.0) {
                        phase =0.0;
                    }
                    // std::cout << "phase " << phase << " source " << source.transpose() << " dest " << dest.transpose() <<std::endl;
                    T point = phase*dest + (1.0-phase)*source;
                    return point;
                }
                void Reset(T source_, T dest_) {
                    source = source_;
                    dest = dest_;
                }
            private:
                T source;
                T dest;
        };


        /** @brief 
         * @ref https://math.stackexchange.com/questions/2217654/interpolation-in-so3-different-approaches
         * @ref https://math.stackexchange.com/questions/1832019/what-is-the-angular-velocity-in-an-inertial-frame-given-the-angular-velocity-in
         * 
         */
        class qrCubicSplineInSO3 {
        public:
            qrCubicSplineInSO3() {}

            qrCubicSplineInSO3(Mat3<float> R1, Mat3<float>R2, float t0, float duration, Mat3<float> inertial);

            ~qrCubicSplineInSO3() = default;

            void GetPoint(float currentTime, Mat3<float>& outR, Vec3<float>& outwb);

        private:
            float t0;
            float dt;
            float lastTime;
            Mat3<float> R1;
            Mat3<float> lastR;
            Mat3<float> R2;
            Mat3<float> dR1;
            Mat3<float> dR2;

            Mat3<float> M0;
            Mat3<float> M1;
            Mat3<float> M2;
            Mat3<float> M3;
            Mat3<float> W;
        };

        /**
         * @brief Spline class defines an abstract class for different spline interpolations
         */
        class qrSpline {
        public:
            /**
             * @brief record the spatial information, including the time dervative.
             */
            struct qrPoint {
                qrPoint() : x(0.0), xd(0.0), xdd(0.0)
                {}

                qrPoint(float p, float v = 0.0, float a = 0.0) : x(p), xd(v), xdd(a)
                {}

                void setZero()
                {
                    x = 0.0;
                    xd = 0.0;
                    xdd = 0.0;
                }
                float x;
                float xd;
                float xdd;
            };

            qrSpline::qrPoint operator=(const qrSpline::qrPoint &rhs)
            {
                qrSpline::qrPoint out;
                out.x = rhs.x;
                out.xd = rhs.xd;
                out.xdd = rhs.xdd;
                return out;
            }

            /** 
             * @brief Constructor function 
             */
            qrSpline();

            /**
             * @brief Constructor function
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const qrPoint& Start point
             * @param const qrPoint& End point
             */
            qrSpline(const float &initial_time,
                    const float &duration,
                    const qrPoint &start,
                    const qrPoint &end);
            
            qrSpline(const float &initial_time,
                    const float &duration,
                    float start_p,
                    float end_p);

            /** @ Destructor function */
            virtual ~qrSpline() = 0;

            /**
             * @brief Sets the boundary of the spline
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const qrPoint& Start point
             * @param const qrPoint& End point
             */
            void setBoundary(const float &initial_time,
                                const float &duration,
                                const qrPoint &start_p,
                                const qrPoint &end_p);

            /**
             * @brief Sets the boundary of the spline
             * @param const float& Initial time
             * @param const float& Duration of the spline
             * @param const float& Start point
             * @param const float& End point
             */
            void setBoundary(const float &initial_time,
                                const float &duration,
                                const float &start_p,
                                const float &end_p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param Point& Point value
             */
            virtual bool getPoint(const float &current_time,
                                    qrPoint &p) = 0;

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param const float& Current time
             * @param float& Point value
             */
            virtual bool getPoint(const float &current_time,
                                    float &p) = 0;

            bool isTimeElapsed(float &time);

        protected:
            /** 
             * @brief Initial time of the spline 
             */
            float initial_time_;

            /** 
             * @brief Duration of the spline 
             */
            float duration_;

            /** 
             * @brief Start point of the spline 
             */
            qrPoint start_;

            /** 
             * @brief End point of the spline 
             */
            qrPoint end_;
        };

        /** 
         * @brief if timeout return true else return false. 
         */
        inline bool qrSpline::isTimeElapsed(float &t)
        {
            //this makes sense only without the time interval
            if ((t - initial_time_) > duration_)
                return true;
            else
                return false;
        }

        /**
         * @brief CubicSpline class defines a cubic spline interpolation
         */
        class qrCubicSpline : public qrSpline {
        public:
            /** 
             * @brief Constructor function 
             */
            qrCubicSpline();

            /**
             * @brief Constructor function
             * @param initial_time Initial time
             * @param duration Duration of the spline
             * @param start Start point
             * @param end End point
             */
            qrCubicSpline(const float &initial_time,
                        const float &duration,
                        const qrPoint &start,
                        const qrPoint &end);

            /**
             * @brief Constructor function
             * @param initial_time Initial time
             * @param duration Duration of the spline
             * @param start Start point
             * @param end End point
             */
            qrCubicSpline(const float &initial_time,
                        const float &duration,
                        float start,
                        float end);

            /** 
             * @brief Destructor function 
             */
            ~qrCubicSpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param current_time Current time
             * @param p Point value
             */
            bool getPoint(const float &current_time,
                            qrPoint &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param current_time Current time
             * @param p Point value
             */
            bool getPoint(const float &current_time,
                            float &p);
        };

        /**
         * @brief FifthOrderPolySpline class defines a 5-order spline interpolation
         */
        class qrFifthOrderPolySpline : public qrSpline {
        public:
            /** 
             * @brief Constructor function 
             */
            qrFifthOrderPolySpline();

            /**
             * @brief Constructor function
             * @param initial_time Initial time
             * @param duration Duration of the spline
             * @param start Start point
             * @param end End point
             */
            qrFifthOrderPolySpline(const float &initial_time,
                                    const float &duration,
                                    const qrPoint &start,
                                    const qrPoint &end);

            /** 
             * @brief Destructor function 
             */
            ~qrFifthOrderPolySpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param current_time Current time
             * @param p Point value
             */
            bool getPoint(const float &current_time,
                            qrPoint &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param current_time Current time
             * @param p Point value
             */
            bool getPoint(const float &current_time,
                            float &p);
        };

        /**
         * @brief LinearSpline class defines a linear spline interpolation
         */
        class qrLinearSpline : public qrSpline {
        public:
            /** 
             * @brief Constructor function 
             */
            qrLinearSpline();

            /**
             * @brief Constructor function
             * @param initial_time Initial time
             * @param duration Duration of the spline
             * @param start Start point
             * @param end End point
             */
            qrLinearSpline(const float &initial_time,
                            const float &duration,
                            const qrPoint &start,
                            const qrPoint &end);

            /** 
             * @brief Destructor function 
             */
            ~qrLinearSpline();

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param current_time Current time
             * @param p Point value
             */
            bool getPoint(const float &current_time, qrPoint &p);

            /**
             * @brief Gets the value of the point according to the spline interpolation
             * @param current_time Current time
             * @param p value
             */
            bool getPoint(const float &current_time, float &p);
        };
    } //  namespace math
#endif // QR_GEOMETRY_H
