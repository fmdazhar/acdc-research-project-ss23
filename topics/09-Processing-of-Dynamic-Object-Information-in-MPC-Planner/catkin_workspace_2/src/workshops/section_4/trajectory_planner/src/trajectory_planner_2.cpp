/*
 * trajectory_planner.cpp
 *
 * ACDC (2021)
 *
 * Author:
 * Guido Küppers, guido.kueppers@ika.rwth-aachen.de
 *
 * Description:
 * This file contains the code to be viewed and modified by the students.
 *
 */

#include "trajectory_planner_2.hpp"
#include <Eigen/Geometry>

CppAD::AD<CppAD::cg::CG<double>> ref_val;
CppAD::AD<CppAD::cg::CG<double>> previous_val;
/*
 *
 * Set control constraints
 *
 */
void PLANNER::getControlConstraints(Eigen::VectorXd &u_lb, Eigen::VectorXd &u_ub, Eigen::VectorXi &sparsity)
{
    sparsity << 1, 1;
    u_lb.resize(2);
    u_ub.resize(2);
    u_lb(0) = -5.0;
    u_ub(0) = 5.0;
    u_lb(1) = -0.4;
    u_ub(1) = 0.4;
}

/*
 *
 * Set state constraints
 *
 */
void PLANNER::getStateConstraints(Eigen::VectorXd &x_lb, Eigen::VectorXd &x_ub, Eigen::VectorXi &sparsity)
{
    sparsity << 0, 0, 1, 1, 1, 0, 1;
    x_lb.resize(4);
    x_ub.resize(4);
    x_lb(0) = 0.0;
    x_ub(0) = 999999999;
    x_lb(1) = 0.0;
    x_ub(1) = 999999999;
    x_lb(2) = -3.5;
    x_ub(2) = 3.5;
    x_lb(3) = -0.4956735;
    x_ub(3) = 0.4956735;
}

/*
 *
 * Implements the intermediate cost functions.
 * Derivatives are automatically computed using Algorithmic Differentiation.
 *
 * x[0]-x[6] is the state vector (active variables), the others are passive parameters
 * (weights, reference values, dyn obj coordinates and reference path values [x-y-v-x-y-v-...])
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class MPC_NODE, typename SCALAR_EVAL, typename SCALAR>
template <typename SC>
SC CostTermIntermediate<STATE_DIM, CONTROL_DIM, MPC_NODE, SCALAR_EVAL, SCALAR>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC &t)
{
    // Find nearest sample in reference path
    const size_t index = MPC_NODE::state_dim + MPC_NODE::params_dim_cfg;
    SC distance = CppAD::sqrt(CppAD::pow(x[index] - x[0], 2) + CppAD::pow(x[index + 1] - x[1], 2));
    SC velocity = x[index + 2];
    for (size_t i = index + 3; i < STATE_DIM - 2; i += 3)
    {
        SC currDist = CppAD::sqrt(CppAD::pow(x[i] - x[0], 2) + CppAD::pow(x[i + 1] - x[1], 2));
        velocity = CppAD::CondExpLt(currDist, distance, x[i + 2], velocity);
        distance = CppAD::CondExpLt(currDist, distance, currDist, distance);
    }

    // Ref path term
    SC pathRef = x[MPC_NODE::WEIGHTS::PATH_REF];
    SC pathCost = distance / pathRef;
    SC pathWeight = x[MPC_NODE::WEIGHTS::PATH];
    SC pathTerm = CppAD::pow(pathCost * pathWeight, 2);

    // TODO: INSERT CODE HERE
    // use helping comments from Wiki and README.md

    //System State Vector:
    // x[0]: x -> Position X
    // x[1]: y -> Position Y
    // x[2]: s -> Distance
    // x[3]: v -> Vehicle Velocity
    // x[4]: a -> Vehicle Acceleration
    // x[5]: psi -> Vehicle Heading
    // x[6]: delta -> Steering Angle

    //Control Vector:
    // u[0]: j_lon -> longitudinal jerk
    // u[1]: alpha -> Steering Rate

    // TASK 5.2.2
    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    // Longitudinal jerk term
    SC jerkRef = x[MPC_NODE::WEIGHTS::JERK_REF];
    SC jerkLonCost = u[0] / jerkRef;
    SC jerkLonWeight = x[MPC_NODE::WEIGHTS::JERK];
    SC jerkLonTerm = CppAD::pow(jerkLonCost * jerkLonWeight, 2);

    // Alpha term
    SC alphaRef = x[MPC_NODE::WEIGHTS::ALPHA_REF];
    SC alphaCost = u[1] / alphaRef;
    SC alphaWeight = x[MPC_NODE::WEIGHTS::ALPHA];
    SC alphaTerm = CppAD::pow(alphaCost * alphaWeight, 2);

    // Lateral jerk term
    //The vehicles wheel-base is defined by the variable wheelBase
    double wheelBase = MPC_NODE::systemDynamics::wheelBase;
    SC jLat = (1 / wheelBase) * (2 * x[3] * CppAD::tan(x[6]) * x[4] + CppAD::pow(x[3], 2) * (CppAD::pow(CppAD::tan(x[6]), 2) + 1) * u[1]);
    SC jerkLatCost = jLat / jerkRef;
    SC jerkLatWeight = x[MPC_NODE::WEIGHTS::JERK];
    SC jerkLatTerm = CppAD::pow(jerkLatCost * jerkLatWeight, 2);
    //____________________________________________________

    // TASK 5.2.3
    // Velocity Term
    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    SC vScale = CppAD::CondExpGt(velocity, SC(10.0 / 3.6), velocity, SC(10.0 / 3.6));
    SC vCost = (velocity - x[3]) / vScale;
    SC vWeight = x[MPC_NODE::WEIGHTS::VEL];
    SC velTerm = CppAD::pow(vCost * vWeight, 2);
    //____________________________________________________

    // TASK 5.2.4
    // Dyn obj
    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    SC dynObjX = x[MPC_NODE::DYNOBJCOORDS::X];
    SC dynObjY = x[MPC_NODE::DYNOBJCOORDS::Y];
    SC dynObjVx = x[MPC_NODE::DYNOBJCOORDS::Vx];
    SC dynObjVy = x[MPC_NODE::DYNOBJCOORDS::Vy];
    SC dynObjAx = x[MPC_NODE::DYNOBJCOORDS::Ax];
    SC dynObjAy = x[MPC_NODE::DYNOBJCOORDS::Ay];
    SC ttc_val = x[MPC_NODE::DYNOBJCOORDS::ttc];
    SC dynObjRef = x[MPC_NODE::WEIGHTS::DYNOBJ_REF];

    // SECTION 1:- Old distance using coordinates only calculation
    SC dynObjDist = CppAD::sqrt(CppAD::pow(dynObjX - x[0], 2) + CppAD::pow(dynObjY - x[1], 2));
    SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2)) + 1, SC(0.0));
    SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2);

    // SECTION 2:- Distance including dynamic object velocity calculation 
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t)) - x[1], 2));
    // SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2)) + 1, SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2);

    // SECTION 3:- Distance including velocity and acceleration calculation 
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t) + (0.5 * dynObjAx * t * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t) + (0.5 * dynObjAy * t * t)) - x[1], 2));
    // SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2)) + 1, SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2);

    // SECTION 4:- Addition of speed dependent term and subsequent cost function
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t) + (0.5 * dynObjAx * t * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t) + (0.5 * dynObjAy * t * t)) - x[1], 2));
    // SC speedTerm  = CppAD::CondExpGt(x[3], SC(4.0), CppAD::pow(x[3], 2)/CppAD::pow(4.0, 2), SC(1.0));
    // SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, (speedTerm * CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2))) + speedTerm, SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2);

    // SECTION 5:- Addition of relative speed dependent term and subsequent cost function
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t) + (0.5 * dynObjAx * t * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t) + (0.5 * dynObjAy * t * t)) - x[1], 2));
    // SC dynObjResultantVel = CppAD::sqrt(CppAD::pow(dynObjVx, 2) + CppAD::pow(dynObjVy, 2));
    // SC relativeSpeed = CppAD::CondExpGt((x[3] - dynObjResultantVel), SC(1.0), (x[3] - dynObjResultantVel), x[3]);
    // SC speedTerm  = CppAD::CondExpGt(relativeSpeed, SC(4.0), CppAD::pow(x[3], 2)/CppAD::pow(4.0, 2), SC(1.0));
    // SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, (speedTerm * CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2))) + speedTerm, SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2);

    // SECTION 6:- Addition of relative speed dependent term and capped (threshold) cost function
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t) + (0.5 * dynObjAx * t * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t) + (0.5 * dynObjAy * t * t)) - x[1], 2));
    // SC dynObjResultantVel = CppAD::sqrt(CppAD::pow(dynObjVx, 2) + CppAD::pow(dynObjVy, 2));
    // SC relativeSpeed = CppAD::CondExpGt((x[3] - dynObjResultantVel), SC(1.0), (x[3] - dynObjResultantVel), x[3]);
    // SC speedTerm = CppAD::tanh((CppAD::pow(relativeSpeed, 2)/CppAD::pow(4.0, 2)) - SC(2.0)) + SC(2.0);
    // SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, (speedTerm * CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2))) + speedTerm, SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2);

    // SECTION 7:- Addition of parallax angle based error metric
    // SC L_r = wheelBase / SC(2.0);                                                // Distance from CG to rear wheel axis
    // SC car_half_width = SC(0.5);                                                  // Car half width assumed to be 0.5 m
    // SC theta_rl = CppAD::atan((dynObjX + L_r) / (car_half_width - dynObjY));      // Theta angle to dynobj from rear left point
    // SC theta_rr = CppAD::atan((dynObjX + L_r) / (car_half_width + dynObjY));      // Theta angle to dynobj from rear right point
    // SC theta = SC(M_PI) - (theta_rl + theta_rr);
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(theta * dynObjWeight, 2);

    // SECTION 8:- Inverse Time to collision based cost function (Distance based conditional activation)
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t) + (0.5 * dynObjAx * t * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t) + (0.5 * dynObjAy * t * t)) - x[1], 2));
    // SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, (SC(1.0)/(SC(1.0) + (CppAD::pow(SC(2.71828),SC(30.0) * ((SC(-1.0) * (SC(1.0)/ttc_val)) + SC(0.3)))))),SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2); 

    // SECTION 9:- Inverse Time to collision based cost function (TTC based conditional activation)
    // SC dynObjDist = CppAD::sqrt(CppAD::pow((dynObjX + (dynObjVx * t) + (0.5 * dynObjAx * t * t)) - x[0], 2) + CppAD::pow((dynObjY + (dynObjVy * t) + (0.5 * dynObjAy * t * t)) - x[1], 2));
    // SC dynObjCost = CppAD::CondExpLt(ttc_val, SC(100.0),(SC(1.0)/(SC(1.0) + (CppAD::pow(SC(2.71828),SC(30.0) * ((SC(-1.0) * (SC(1.0)/ttc_val)) + SC(0.3)))))),SC(0.0));
    // SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    // SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2); 

    //____________________________________________________

    // TASK 6.2.3.2
    // Traffic Light
    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    SC TrafficLightX = x[MPC_NODE::TRAFFICLIGHT::X_TL];
    SC TrafficLightY = x[MPC_NODE::TRAFFICLIGHT::Y_TL];
    SC TrafficLightState = x[MPC_NODE::TRAFFICLIGHT::STATE];
    SC TrafficLightRef = x[MPC_NODE::WEIGHTS::TRAFFICLIGHT_REF];
    SC TrafficLightDist = CppAD::sqrt(CppAD::pow(TrafficLightX - x[0], 2) + CppAD::pow(TrafficLightY - x[1], 2));
    SC TrafficLightCost = CppAD::CondExpEq(TrafficLightState,SC(1.0),CppAD::CondExpLt(TrafficLightDist, TrafficLightRef, CppAD::cos(SC(M_PI) * CppAD::pow(TrafficLightDist, 2) / CppAD::pow(TrafficLightRef, 2)) + 1, SC(0.0)),SC(0.0));
    SC TrafficLightWeight = x[MPC_NODE::WEIGHTS::TRAFFICLIGHT];
    SC TrafficLightTerm = CppAD::pow(TrafficLightCost * TrafficLightWeight, 2);
    //____________________________________________________

    // Return sum
    // int pos_vec = test_function();
    // std::cout<<"Calculated the values"<< dynObjDist <<std::endl;
    // std::cout<<"Calculated the values"<< ref_val <<std::endl;
    return pathTerm + jerkLonTerm + jerkLatTerm + alphaTerm + velTerm + dynObjTerm + TrafficLightTerm;
}

/*
*Calculate velocity
*/

/*
 *
 * Implements the final cost functions.
 * Derivatives are automatically computed using Algorithmic Differentiation.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class MPC_NODE, typename SCALAR_EVAL, typename SCALAR>
template <typename SC>
SC CostTermFinal<STATE_DIM, CONTROL_DIM, MPC_NODE, SCALAR_EVAL, SCALAR>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC &t)
{
    //
    // Yaw deviation
    //

    // Find nearest sample in reference path and approx its orientation
    const size_t index = MPC_NODE::state_dim + MPC_NODE::params_dim_cfg + 3;
    SC distance = CppAD::sqrt(CppAD::pow(x[index] - x[0], 2) + CppAD::pow(x[index + 1] - x[1], 2));
    SC dx = x[index] - x[index - 3];
    SC dy = x[index + 1] - x[index - 2];
    for (size_t i = index + 3; i < STATE_DIM - 2; i += 3)
    {
        SC currDist = CppAD::sqrt(CppAD::pow(x[i] - x[0], 2) + CppAD::pow(x[i + 1] - x[1], 2));
        dx = CppAD::CondExpLt(currDist, distance, x[i] - x[i - 3], dx);
        dy = CppAD::CondExpLt(currDist, distance, x[i + 1] - x[i - 2], dy);
        distance = CppAD::CondExpLt(currDist, distance, currDist, distance);
    }

    return CppAD::pow((x[5] - CppAD::atan2(dy, dx)) * x[MPC_NODE::WEIGHTS::YAW], 2);
}

/*
 *
 * Implements the system dynamics.
 *
 */
template <typename SCALAR>
void ACDC_VehcicleSystem<SCALAR>::computeControlledDynamics(const StateVector<STATE_DIM, SCALAR> &state, const time_t &t, const ControlVector<CONTROL_DIM, SCALAR> &control, StateVector<STATE_DIM, SCALAR> &derivative)
{
    // TASK 5.2.1
    // System Dynamics
    // TODO: INSERT CODE HERE
    // use helping comments from Wiki
    //System State Vector:
    // state(0): x -> Position X
    // state(1): y -> Position Y
    // state(2): s -> Distance
    // state(3): v -> Vehicle Velocity
    // state(4): a -> Vehicle Acceleration
    // state(5): psi -> Vehicle Heading
    // state(6): delta -> Steering Angle

    //Control Vector:
    // control(0): j_lon -> longitudinal jerk
    // control(1): alpha -> Steering Rate

    //The vehicles wheel-base is defined by the class variable wheelBase

    derivative(0) = state(3) * cos(state(5)); //derivative of x
    derivative(1) = state(3) * sin(state(5)); //derivative of y
    derivative(2) = state(3); //derivative of s
    derivative(3) = state(4); //derivative of v
    derivative(4) = control(0); //derivative of a
    derivative(5) = (state(3) / wheelBase) * tan(state(6)); //derivative of psi
    derivative(6) = control(1); //derivative of delta

    //____________________________________________________
}

/*
 *
 * Callback function for received SPATs.
 *
 */
void PLANNER::callbackSPAT(const definitions::v2x_SPAT& msg)
{
    // TASK 6.2.3.1
    // Process SPATEM

    // Loop all intersections in message
    for(int i = 0; i < msg.spatData_intersections.size(); i++) {
        definitions::v2x_SPAT_IntersectionState spat_intsctn = msg.spatData_intersections[i];
        // Loop all movement states to get signal groups
        for(int m = 0; m < spat_intsctn.states.size(); m++) {
            //Loop all traffic lights stored in the map data
            for(int k = 0; k<trafficlights.size(); k++)
            {
                if(trafficlights[k].sig_id == spat_intsctn.states[m].signalGroup)
                {
                    trafficlights[k].last_spat = ros::Time::now();
                    if(spat_intsctn.states[m].state_time_speed[0].eventState == 5 || spat_intsctn.states[m].state_time_speed[0].eventState == 6)
                    {
                        trafficlights[k].red = false;
                    }
                    else
                    {
                        trafficlights[k].red = true;
                    }
                }
            }
        }
    }

    //____________________________________________________
}

/*
 *
 * Called with our main frequency. Evaluates the MPC and sends new trajectory to controller.
 *
 */
void PLANNER::runMPC()
{
    // Checking rosbag record duration and stopping
    // record_end = std::chrono::steady_clock::now();
    // if(((record_end - record_start)/std::chrono::seconds(120)) > 10 && recording){
    //     system("rosnode kill performanceAnalysisBag");
    //     recording = false;
    // }

    // Temporary data for initialization
    StateVectorArray<state_dim> x_ref_init(K + 1);
    FeedbackArray<state_dim, control_dim> u0_fb(K);
    ControlVectorArray<control_dim> u0_ff(K);

    // Check for vehicleData2 time
    ros::Duration vehcileDataAge = ros::Time::now() - vehicleData2.header.stamp;
    if (vehcileDataAge > ros::Duration(1.0 / cfg.mainFrequency))
    {
        ROS_WARN("Outdated vehicle data!");
        bReset = true;
    }

    // We have a valid MPC solution from the last iteration
    if (!bReset)
    {
        // Adjust last trajectories according to odometry
        for (size_t i = 0; i < x_ref_init.size(); i++)
        {
            // Gather interpolation parameters
            size_t index;
            double factor;
            getInterpolationParameters(lastValidTrajectory.getReferenceStateTrajectory().getDataArray(), lastValidTrajectory.getReferenceStateTrajectory().getDataArray()[i][2] + ds, index, factor);

            // Interpolate state
            interpolateValues(lastValidTrajectory.getReferenceStateTrajectory().getDataArray(), factor, index, x_ref_init[i]);

            // Transform state into new frame
            x_ref_init[i][0] -= dx;
            x_ref_init[i][1] -= dy;
            x_ref_init[i][5] -= dYaw;
            const double ds_tmp = std::sqrt(x_ref_init[i][0] * x_ref_init[i][0] + x_ref_init[i][1] * x_ref_init[i][1]);
            const double theta = std::atan2(x_ref_init[i][1], x_ref_init[i][0]);
            x_ref_init[i][0] = ds_tmp * std::cos(theta - dYaw);
            x_ref_init[i][1] = ds_tmp * std::sin(theta - dYaw);

            // Interpolate feedback and feedforward controls
            if (i < u0_fb.size())
            {
                index = std::min(index, u0_fb.size() - 2);
                interpolateValues(lastValidTrajectory.getFeedbackTrajectory().getDataArray(), factor, index, u0_fb[i]);
                interpolateValues(lastValidTrajectory.getFeedforwardTrajectory().getDataArray(), factor, index, u0_ff[i]);
            }
        }

        // Recompute s
        x_ref_init.front()[2] = 0.0;
        for (size_t i = 1; i < x_ref_init.size(); i++)
        {
            x_ref_init[i][2] = x_ref_init[i - 1][2] + std::sqrt(std::pow(x_ref_init[i - 1][0] - x_ref_init[i][0], 2) + std::pow(x_ref_init[i - 1][1] - x_ref_init[i][1], 2));
        }

        // Check for too high deviations to odometry
        bool bLatReinit = false;
        bool bLonReinit = false;
        if (std::abs(x_ref_init.front()[3] - vehicleData2.velocity) > cfg.deviationMaxV / 3.6 || std::abs(x_ref_init.front()[4] - vehicleData2.acceleration) > cfg.deviationMaxA)
        {
            x_ref_init.front()[3] = vehicleData2.velocity;
            x_ref_init.front()[4] = vehicleData2.acceleration;
            bLonReinit = true;
        }
        if (std::abs(x_ref_init.front()[1]) > std::abs(cfg.deviationMaxY) || std::abs(x_ref_init.front()[5]) > std::abs(cfg.deviationMaxYaw * M_PI / 180.0))
        {
            x_ref_init.front()[1] = 0.0;
            x_ref_init.front()[5] = 0.0;
            x_ref_init.front()[6] = vehicleData2.steering_angle;
            bLatReinit = true;
        }
        else if (std::abs(x_ref_init.front()[6] - vehicleData2.steering_angle) > cfg.deviationMaxDelta * M_PI / 180.0)
        {
            x_ref_init.front()[6] = vehicleData2.steering_angle;
        }
        x_ref_init.front()[0] = 0.0;
        x_ref_init.front()[2] = 0.0;

        bReset = bLonReinit && bLatReinit;
    }

    // In case of reset, we initialize everything with zeros
    if (bReset)
    {
        x_ref_init.setConstant(StateVector<state_dim>::Zero());
        u0_fb.setConstant(FeedbackMatrix<state_dim, control_dim>::Zero());
        u0_ff.setConstant(ControlVector<control_dim>::Zero());

        // Set vehicle data
        x_ref_init.front()[3] = vehicleData2.velocity;
        x_ref_init.front()[4] = vehicleData2.acceleration;
        x_ref_init.front()[6] = vehicleData2.steering_angle;
    }

    // Time measurement start
    auto start_time = std::chrono::high_resolution_clock::now();

    // Update the environment data
    updateEnvironmentData();

    std::cout<<"Recurring loop"<<std::endl;
    

    // Run real-time iteration loop for this timestep
    NLOptConSolver<state_dim, control_dim>::Policy_t initialGuess(x_ref_init, u0_ff, u0_fb, ilqr_settings_mpc.dt);
    ilqr_mpc->setInitialGuess(initialGuess);
    ilqr_mpc->prepareMPCIteration();
    ilqr_mpc->changeInitialState(x_ref_init.front());
    bool success;
    for (size_t i = 0; i < cfg.mpcIterations; i++)
    {
        success = ilqr_mpc->finishMPCIteration();
        if (success && i < cfg.mpcIterations - 1)
            ilqr_mpc->prepareMPCIteration();
        else
            break;
    }

    // Time measurement stop
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (!success || ilqr_mpc->getBackend()->getSummary().totalCosts.back() > cfg.objectiveTolerance || std::isnan(ilqr_mpc->getSolution().getReferenceStateTrajectory().getDataArray()[1][0]))
    {
        std::cout << "Trajectory optimization FAILED after " << duration << "s !!!" << std::endl;
        if (success)
            std::cout << "Final cost value is " << ilqr_mpc->getBackend()->getSummary().totalCosts.back() << std::endl;
        bReset = true;
    }
    else
    {
        std::cout << "Trajectory optimization SUCCESSFUL after " << duration << "s." << std::endl;
        lastValidTrajectory = ilqr_mpc->getSolution();
        bReset = false;
    }


    /*template <size_t STATE_DIM, class MPC_NODE>
    template <typename V>
    V CalculateObjVel<STATE_DIM,MPC_NODE>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const V &t){
        V veldynObjX = x[MPC_NODE::DYNOBJCOORDS::X];
        V veldynObjY = x[MPC_NODE::DYNOBJCOORDS::Y];
        std::cout<<"The x distance of object is" << veldynObjX <<endl;
        return 0;
    }*/
    Eigen::RowVectorXd vec1(3);
    vec1 << 1, 2, 3;
    //extern CppAD::cg::CG<double> ref_val;
    //std::cout << "object x distance = " << ref_val << std::endl;

    /*calculateObjVel();*/

    publishTrajectory2();
}

/*
 *
 * Entry point of the ROS node.
 *
 */
int main(int argc, char **argv)
{
    PLANNER mpc(argc, argv);
    mpc.init();

    return 0;
}
