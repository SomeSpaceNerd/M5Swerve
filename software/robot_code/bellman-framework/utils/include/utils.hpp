#pragma once

#include <algorithm>
#include <vector>

namespace bellman
{
    namespace utils 
    {
        namespace controllers // Controllers based on FRC230's award-winning control systems
        {
            class VelocityController
            {
                public:
                    /**
                     * @brief Constructor for a velocity controller
                     *
                     * @param kv Velocity Constant, how fast the motor can get to at full speed
                     * @param kr Robot Constant, how fast the motor can go
                     * @param tau Time Constant, how long it takes the motor to get to full speed
                     *
                     * @return VelocityController
                     */
                    VelocityController(double kv, double kr, double tau) : m_kv(kv), m_kr(kr), m_tau(tau), m_setpoint(0.0), m_integral(0.0) {}

                    /**
                     * @brief Set the controller's setpoint
                     *
                     * @param sp setpoint
                     */
                    void SetSetpoint(double sp)
                    {
                        m_setpoint = sp;
                    }

                    /**
                     * @brief Reset the controller's integral to 0
                     */
                    void Reset()
                    {
                        m_integral = 0.0;
                    }

                    /**
                     * @brief Get the output of the controller based on a measurement
                     *
                     * @param measurement The current velocity
                     * @param dt Time delta between previous and current measurement
                     *
                     * @return The output of the controller
                     */
                    double Update(double measurement, double dt)
                    {
                        if (dt <= 0.0 || m_kv == 0.0 || m_kr == 0.0) { return 0.0; }

                        const double error = m_setpoint - measurement;
                        m_integral += error * dt;
                        const double integralLimit = m_kr / m_kv;
                        m_integral = std::clamp(m_integral, -integralLimit, integralLimit);
                        return (m_kv / m_kr) * (m_tau * error + m_integral);
                    }

                private:
                    double m_kv;
                    double m_kr;
                    double m_tau;
                    double m_setpoint;
                    double m_integral;
            };

            class PositionController
            {
                public:
                    /**
                     * @brief Constructor for a position controller
                     *
                     * @param kp Positional Constant, how much we want it to respond to error
                     * @param kv Velocity Constant, how fast we want it to get to full speed
                     * @param kr Robot Constant, how fast the thing can go
                     * @param tau Time Constant, how long it takes to get there
                     *
                     * @return VelocityController
                     */
                    PositionController(double kp, double kv, double kr, double tau) : m_kp(kp), m_kv(kv), m_kr(kr), m_tau(tau), m_setpoint(0.0), m_integral(0.0) {}

                    /**
                     * @brief Set the controller's setpoint
                     *
                     * @param sp setpoint
                     */
                    void SetSetpoint(double sp)
                    {
                        m_setpoint = sp;
                    }

                    /**
                     * @brief Reset the controller's integral to 0
                     */
                    void Reset()
                    {
                        m_integral = 0.0;
                    }

                    /**
                     * @brief Get the output of the controller based on a measurement
                     *
                     * @param position The current position of the motor
                     * @param velocity The current velocity of the motor
                     * @param dt Time delta between previous and current measurement
                     *
                     * @return The output of the controller
                     */
                    double Update(double position, double velocity, double dt)
                    {
                        if (dt <= 0.0 || m_kv == 0.0 || m_kr == 0.0) { return 0.0; }

                        const double perror = m_setpoint - position;
                        const double vcmd = m_kp * perror;
                        const double verror = vcmd - velocity;
                        m_integral += verror * dt;
                        const double integralLimit = m_kr / m_kv;
                        m_integral = std::clamp(m_integral, -integralLimit, integralLimit);
                        return (m_kv / m_kr) * (m_tau * verror + m_integral);
                    }

                private:
                    double m_kp;
                    double m_kv;
                    double m_kr;
                    double m_tau;
                    double m_setpoint;
                    double m_integral;
            };
        }

        class interpolationTable 
        {
            public:
                /**
                 * @brief Add a single point to the interpolation table
                 *
                 * @param x The input value for the interpolation table
                 * @param y The output value for the interpolation table
                 */
                void AddPoint(double x, double y) 
                {
                    m_points.emplace_back(x, y);
                    std::sort(m_points.begin(), m_points.end(), [](const auto& a, const auto& b) {return a.first < b.first; }); // Keep vector sorted by x
                }

                /**
                 * @brief Add multiple points to the interpolation table
                 *
                 * @param pts A vector of pairs of 2 doubles representing a list of 2D points
                 */
                void AddPoints(const std::vector<std::pair<double, double>>& pts)
                {
                    m_points.insert(m_points.end(), pts.begin(), pts.end());
                    std::sort(m_points.begin(), m_points.end(), [](const auto& a, const auto& b) {return a.first < b.first; }); // Keep vector sorted by x
                }

                /**
                 * @brief Gets the interpolated value from the table with a given input
                 *
                 * @param x Input value for the table
                 *
                 * @return The output of the interpolation table for the given x
                 */
                double GetValue(double x) const 
                {
                    if (m_points.empty()) { return 0.0; }

                    if (x <= m_points.front().first) { return m_points.front().second; }
                    if (x >= m_points.back().first) { return m_points.back().second; }

                    for (size_t i = 0; i < m_points.size() - 1; ++i) 
                    {
                        const double x0 = m_points[i].first;
                        const double y0 = m_points[i].second;
                        const double x1 = m_points[i + 1].first;
                        const double y1 = m_points[i + 1].second;

                        if (x >= x0 && x <= x1) 
                        {
                            const double t = (x - x0) / (x1 - x0);
                            return y0 + t * (y1 - y0);
                        }
                    }
                    return 0.0; // Fallthrough
                }

            private:
                std::vector<std::pair<double, double>> m_points;
        };
    }
}