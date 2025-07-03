/*
 * trajectory_planner.h
 *
 * ACDC (2021)
 *
 * Author:
 * Guido Küppers, guido.kueppers@ika.rwth-aachen.de
 *
 * Description:
 * Class declarations and definitions as the backbone of the ROS Node.
 * Should not be viewed or modified by students.
 *
 */
#pragma once

#define EIGEN_STACK_ALLOCATION_LIMIT 131072 * 2 // 256KB

#include "ros/ros.h"

// Parameters
#include <dynamic_reconfigure/server.h>
#include <trajectory_planner/trajectory_plannerConfig.h>


//Added for ACDC RP
#include <nav_msgs/Odometry.h>

// I/O
#include <visualization_msgs/MarkerArray.h>
#include <definitions/FlatlandVehicleState.h>
#include <definitions/IkaObjectList.h>
#include <definitions/IkaTpTrajectoryInterface.h>
#include <definitions/v2x_SPAT.h>

#include <definitions/utility/ika_utilities.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>

// Control Toolbox
#include <ct/optcon/optcon.h>
#include <ct/core/core.h>

#include "CostFunctionADParams.hpp"
#include "CostFunctionADParams-impl.hpp"

#include <vector>
#include <std_msgs/Float64.h>

// Cost function analysis 
#include <definitions/CostFuncAnalysisMsg.h>
#include <cmath>
#include <chrono>

using namespace ct::core;
using namespace ct::optcon;

// Forward declare
class PLANNER;

int test_function();
/*
 *
 *  Constraint class
 *
 */
template <class VEHICLESYSTEM>
class ConstraintTerm : public ct::optcon::ConstraintBase<VEHICLESYSTEM::STATE_DIM, VEHICLESYSTEM::CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ct::optcon::ConstraintBase<VEHICLESYSTEM::STATE_DIM, VEHICLESYSTEM::CONTROL_DIM> Base;
    typedef ct::core::StateVector<VEHICLESYSTEM::STATE_DIM> state_vector_t;
    typedef ct::core::ControlVector<VEHICLESYSTEM::CONTROL_DIM> control_vector_t;

    //! constructor with hard-coded constraint boundaries.
    ConstraintTerm()
    {
        Base::lb_.resize(1);
        Base::ub_.resize(1);
        Base::lb_.setConstant(0.0);
        Base::ub_.setConstant(81.0); // Squared
    }

    //! deep cloning
    virtual ConstraintTerm *clone() const override { return new ConstraintTerm(); }
    virtual size_t getConstraintSize() const override { return 1; }

    virtual Eigen::VectorXd evaluate(const state_vector_t &x, const control_vector_t &u, const double t) override
    {
        Eigen::Matrix<double, 1, 1> val;
        auto wheelBaseRez = 1.0 / VEHICLESYSTEM::wheelBase;
        auto tanDelta = tan(x(6));
        val.template segment<1>(0) << x(4) * x(4) + x(3) * x(3) * x(3) * x(3) * wheelBaseRez * wheelBaseRez * tanDelta * tanDelta;
        return val;
    }

    virtual Eigen::Matrix<ct::core::ADCGScalar, Eigen::Dynamic, 1> evaluateCppadCg(
        const ct::core::StateVector<VEHICLESYSTEM::STATE_DIM, ct::core::ADCGScalar> &x,
        const ct::core::ControlVector<VEHICLESYSTEM::CONTROL_DIM, ct::core::ADCGScalar> &u,
        ct::core::ADCGScalar t) override
    {
        Eigen::Matrix<ct::core::ADCGScalar, 1, 1> val;
        auto wheelBaseRez = 1.0 / VEHICLESYSTEM::wheelBase;
        auto tanDelta = CppAD::tan(x(6));
        val.template segment<1>(0) << x(4) * x(4) + x(3) * x(3) * x(3) * x(3) * wheelBaseRez * wheelBaseRez * tanDelta * tanDelta;
        return val;
    }
};

/*
 *
 * Cost function class (Intermediate)
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class MPC_NODE, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class CostTermIntermediate : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    //! constructor
    CostTermIntermediate(MPC_NODE *mpc) : mpcNode(mpc) {}

    //! copy constructor
    CostTermIntermediate(const CostTermIntermediate &arg) : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg), mpcNode(arg.mpcNode) {}

    //! deep cloning
    CostTermIntermediate *clone() const override
    {
        return new CostTermIntermediate(*this);
    }

    /**
     * @brief      Evaluates the term at x, u, t
     *
     * @param[in]  x     The current state
     * @param[in]  u     The current control
     * @param[in]  t     The current time
     *
     * @return     The evaluatated cost term
     */
    SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
                    const SCALAR &t) override
    {
        return evalLocal<SCALAR>(x, u, t);
    }

    /**
     * @brief      The evaluate method used for jit compilation in CostfunctionAD
     *
     * @param[in]  x     The state vector
     * @param[in]  u     The control vector
     * @param[in]  t     The time
     *
     * @return     The evaluated cost
     */
    virtual ct::core::ADCGScalar evaluateCppadCg(const ct::core::StateVector<STATE_DIM, ct::core::ADCGScalar> &x,
                                                 const ct::core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar> &u,
                                                 ct::core::ADCGScalar t) override
    {
        return evalLocal<ct::core::ADCGScalar>(x, u, t);
    }

    
    int test_function();

protected:
    // Defined in .cpp file.


    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC &t);



    MPC_NODE *mpcNode;
};



/*
 *
 * Cost function class (Final)
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class MPC_NODE, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class CostTermFinal : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    //! constructor
    CostTermFinal(MPC_NODE *mpc) : mpcNode(mpc) {}

    //! copy constructor
    CostTermFinal(const CostTermFinal &arg) : TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>(arg), mpcNode(arg.mpcNode) {}

    //! deep cloning
    CostTermFinal *clone() const override
    {
        return new CostTermFinal(*this);
    }

    /**
     * @brief      Evaluates the term at x, u, t
     *
     * @param[in]  x     The current state
     * @param[in]  u     The current control
     * @param[in]  t     The current time
     *
     * @return     The evaluatated cost term
     */
    SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1> &x,
                    const Eigen::Matrix<SCALAR, CONTROL_DIM, 1> &u,
                    const SCALAR &t) override
    {
        return evalLocal<SCALAR>(x, u, t);
    }

    /**
     * @brief      The evaluate method used for jit compilation in CostfunctionAD
     *
     * @param[in]  x     The state vector
     * @param[in]  u     The control vector
     * @param[in]  t     The time
     *
     * @return     The evaluated cost
     */
    virtual ct::core::ADCGScalar evaluateCppadCg(const ct::core::StateVector<STATE_DIM, ct::core::ADCGScalar> &x,
                                                 const ct::core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar> &u,
                                                 ct::core::ADCGScalar t) override
    {
        return evalLocal<ct::core::ADCGScalar>(x, u, t);
    }

protected:
    // Defined in .cpp file.
    template <typename SC>
    SC evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC &t);

    MPC_NODE *mpcNode;
};

/*
 *
 * Vehicle model class
 *
 */
template <typename SCALAR>
class ACDC_VehcicleSystem : public ControlledSystem<7, 2, SCALAR>
{
public:
    static const size_t STATE_DIM = 7;   //!< state dimension (x-pos, y-pos, distance, velocity, longitudinal acceleration, orientation, steering angle)
    static const size_t CONTROL_DIM = 2; //!< control dimension (longitudinal jerk, steering rate)
    typedef ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;
    typedef typename Base::time_t time_t;

    static constexpr SCALAR wheelBase = 2.711;

    //! default constructor
    ACDC_VehcicleSystem()
        : ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(SYSTEM_TYPE::SECOND_ORDER) {}

    //! destructor
    virtual ~ACDC_VehcicleSystem(){};

    //! deep cloning
    ACDC_VehcicleSystem *clone() const override
    {
        return new ACDC_VehcicleSystem<SCALAR>(*this);
    }

    //! evaluate the system dynamics
    /*!
     * @param state current state (position, velocity)
     * @param t current time (gets ignored)
     * @param control control action
     * @param derivative derivative (velocity, acceleration)
     */
    // Defined in .cpp file.
    virtual void computeControlledDynamics(const StateVector<STATE_DIM, SCALAR> &state,
                                           const time_t &t,
                                           const ControlVector<CONTROL_DIM, SCALAR> &control,
                                           StateVector<STATE_DIM, SCALAR> &derivative) override;
};

/*
 *
 * ROS Node class
 *
 */



class PLANNER
{
public:
    // Constants for tweaking
    static const size_t params_dim = 77; // (Passive) parameter amount used in the AD cost functions. They are used for configurable settings (e.g. weights, see amount below) and the rest is filled with reference path samples.
                                         // Increasing this will allow more reference path samples to be used, but increases the JIT compilation time when initially starting the MPC.
    double interpolation_s;
    static constexpr ct::core::Time timeHorizon = 4.0; // Time horizon of the MPC.
    static constexpr double mpc_dt = 0.2;              // Time discretization of the MPC.

    static const size_t params_dim_cfg = 17; // Total amount of configurable passive parameters in the cost functions (besides ref path samples, but including dynamic object).
    enum WEIGHTS
    { // Indices of the weights in the cost function state vector. 0-6 is used by the system state.
        VEL = 7,
        PATH = 8,
        YAW = 9,
        JERK = 10,
        ALPHA = 11,
        DYNOBJ = 12,
        PATH_REF = 13,
        JERK_REF = 14,
        ALPHA_REF = 15,
        DYNOBJ_REF = 16,
        TRAFFICLIGHT_REF = 17,
        TRAFFICLIGHT = 18
    };

    enum DYNOBJCOORDS
    {
        X = 19,
        Y = 20,
        Vx = 21,
        Vy = 22,
        Ax = 23,
        Ay = 24,
        len = 25,
        
    };

    enum TRAFFICLIGHT
    {
        X_TL = 26,
        Y_TL = 27,
        STATE = 28
    };


    //TTC variable initialization
    std::vector<float> target_pos_vec_;
    std::vector<float> ego_pos_vec_;
    std::vector<float> target_vel_vec_;
    std::vector<float> ego_vel_vec_;
    float target_heading_;
    float ego_heading_;
    bool no_target;

    struct TrafficLight
    {
        std::vector<geometry_msgs::Point> ingress_lane;
        int sig_id;
        int tl_id;
        bool red;
        ros::Time last_spat;
    };

    struct ObjectWithPriority {
    definitions::IkaObject object;
    float priority;
    };

    std::vector<TrafficLight> trafficlights;
    bool use_spats = false;

    tf2_ros::Buffer *transform_buffer_ = nullptr;

    // Other typedefs / constants
    typedef ACDC_VehcicleSystem<double> systemDynamics;
    static const size_t state_dim = systemDynamics::STATE_DIM;
    static const size_t control_dim = systemDynamics::CONTROL_DIM;
    static const size_t params_dim_refPath = params_dim - params_dim_cfg;

    PLANNER(int argc, char **argv) : _argc(argc), _argv(argv), ilqr_mpc(nullptr) {}
    ~PLANNER() { delete ilqr_mpc; }

    // Init the ROS node.
    void init()
    {
        ros::init(_argc, _argv, "acdc_mpc");
        ros::NodeHandle n;

        // Subscribers
        std::string default_subscribe_topic_lanes = "/vehicle/lane_markings";
        std::string subscribe_topic_lanes;
        n.param<std::string>("controller/lanes_topic", subscribe_topic_lanes, default_subscribe_topic_lanes);
        subscriber_lanes = n.subscribe(subscribe_topic_lanes, 1, &PLANNER::callbackLaneMarkings, this);

        std::string default_subscribe_topic_vehicle = "/vehicle/wheel_sensor";
        std::string subscribe_topic_vehicle;
        n.param<std::string>("controller/wheel_topic", subscribe_topic_vehicle, default_subscribe_topic_vehicle);
        subscriber_vehicle_data = n.subscribe(subscribe_topic_vehicle, 1, &PLANNER::callbackVehicleData, this);

        std::string default_subscribe_topic_dyn = "/vehicle/ikaObjectList";
        std::string subscribe_topic_dyn;
        n.param<std::string>("/vehicle/ikaObjectList", subscribe_topic_dyn, default_subscribe_topic_dyn);
        subscriber_dyn = n.subscribe(subscribe_topic_dyn, 1, &PLANNER::callbackDynObjects, this);

        std::string default_subscribe_topic_spat = "/TopicSPATs"; // fill in your SPaT topic
        std::string subscribe_topic_spat;
        n.param<std::string>("/TopicSPATs", subscribe_topic_spat, default_subscribe_topic_spat);
        subscriber_spat = n.subscribe(subscribe_topic_spat, 1, &PLANNER::callbackSPAT, this);

        n.param<bool>("/trajectory_planner/use_spats", use_spats, true);

        std::string subscribe_topic_ego_pose = "/odometry/ground_truth";
        subscriber_ego_pose = n.subscribe(subscribe_topic_ego_pose, 1, &PLANNER::callbackEgoPose, this);


        // Publishers
        publisher_trj = n.advertise<definitions::IkaTpTrajectoryInterface>("/mpc/trajectory_interface", 1000);
        publisher_rviz = n.advertise<visualization_msgs::Marker>("/mpc/visualization_marker_array", 0);
        analysis_publisher = n.advertise<std_msgs::Float64>("dist", 10);
        cost_pub_ = n.advertise<std_msgs::Float64>("cost", 10);
        ttc_publisher = n.advertise<std_msgs::Float64>("ttc", 10);

        // Dynamic reconfigure
        dynamic_reconfigure::Server<trajectory_planner::trajectory_plannerConfig> server;
        dynamic_reconfigure::Server<trajectory_planner::trajectory_plannerConfig>::CallbackType f;
        f = boost::bind(&PLANNER::callbackDynReconf, this, _1, _2);
        server.setCallback(f);

        // Init vars
        dYaw = 0.0;
        dx = 0.0;
        dy = 0.0;
        ds = 0.0;

        //TF
        transform_buffer_ = new tf2_ros::Buffer;
        tf2_ros::TransformListener tfListener(*transform_buffer_);

        TrafficLight tl;
        //Define Traffic Light Positions (e.g. Map Data)
        geometry_msgs::Point p;
        p.z = 0.0;

        tl.red = true;
        p.x = 405.5;
        p.y = 156.0;
        tl.ingress_lane.push_back(p);
        p.x = 410.5;
        p.y = 160.0;
        tl.ingress_lane.push_back(p);
        tl.sig_id = 1;
        tl.tl_id = 1;
        trafficlights.push_back(tl);
        tl.ingress_lane.clear();
        p.x = 417.5;
        p.y = 171.6;
        tl.ingress_lane.push_back(p);
        p.x = 411.41;
        p.y = 166.36;
        tl.ingress_lane.push_back(p);
        tl.sig_id = 1;
        tl.tl_id = 2;
        trafficlights.push_back(tl);
        tl.ingress_lane.clear();
        p.x = 416.8;
        p.y = 178.8;
        tl.ingress_lane.push_back(p);
        p.x = 408.68;
        p.y = 169.09;
        tl.ingress_lane.push_back(p);
        tl.sig_id = 1;
        tl.tl_id = 3;
        trafficlights.push_back(tl);
        tl.ingress_lane.clear();
        p.x = 419.0;
        p.y = 153.0;
        tl.ingress_lane.push_back(p);
        p.x = 415.45;
        p.y = 157.27;
        tl.ingress_lane.push_back(p);
        tl.sig_id = 2;
        tl.tl_id = 4;
        trafficlights.push_back(tl);
        tl.ingress_lane.clear();
        p.x = 401.3;
        p.y = 173.7;
        tl.ingress_lane.push_back(p);
        p.x = 405.05;
        p.y = 169.09;
        tl.ingress_lane.push_back(p);
        tl.sig_id = 2;
        tl.tl_id = 5;
        trafficlights.push_back(tl);
        tl.ingress_lane.clear();

        // Proceed
        initMPC();
        run();

        delete transform_buffer_;
    }

    /*
     *
     * Called when parameters are updated.
     *
     */
    void callbackDynReconf(trajectory_planner::trajectory_plannerConfig &config, uint32_t level)
    {
        cfg = config;

        if (level)
            ROS_INFO("Reconfigured parameters received.");
        else
            ROS_INFO("Parameters received.");

        if (ilqr_mpc && ilqr_settings_mpc.printSummary != cfg.printSummary)
        {
            ilqr_settings_mpc.printSummary = cfg.printSummary;
            ilqr_mpc->configure(ilqr_settings_mpc);
        }

        bReset = true;
    }

    /*
     *
     * Vehicle data callback.
     *
     */
    void callbackVehicleData(const definitions::FlatlandVehicleState &msg)
    {
        // Update odometry
        const double dT = (msg.header.stamp - vehicleData.header.stamp).toSec();
        const double s = msg.velocity * dT;
        const double yaw = -msg.yaw_rate * dT;
        dx += std::cos(dYaw + yaw / 2.0) * s;
        dy += std::sin(dYaw + yaw / 2.0) * s;
        ds += s;
        dYaw += yaw;

        // Save
        vehicleData = msg;
        vehicleData.steering_angle *= -1;
        vehicleData.yaw_rate *= -1;

        // Cost function analysis
        egoVel = vehicleData.velocity;
        egoAcc = vehicleData.acceleration;
    }
    
    void callbackEgoPose(const nav_msgs::Odometry msg)
    {
        geometry_msgs::Quaternion orient_;
        geometry_msgs::Point pos__;
        geometry_msgs::Vector3 velocity_vec;
        orient_ = msg.pose.pose.orientation;
        pos__ = msg.pose.pose.position;
        velocity_vec = msg.twist.twist.linear;
   //the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(orient_, quat);

   //the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

   //the found angles are written in a geometry_msgs::Vector3
        geometry_msgs::Vector3 rpy;
        rpy.x = roll;
        rpy.y = pitch;
        rpy.z = yaw;
    //For ttc calculation
        ego_heading_ = yaw;
        ego_pos_vec_ = {pos__.x, pos__.y};
        ego_vel_vec_ = {velocity_vec.x, velocity_vec.y};
    }
    /*
     *
     * Lane Markings callback. Triggers the MPC.
     *
     */
    void callbackLaneMarkings(const visualization_msgs::Marker &msg)
    {
        if (msg.points.empty())
            return;

        if (msg.id)
            boundaryRight = msg.points;
        else
            boundaryLeft = msg.points;

        if (msg.id && boundaryLeft.size() && ros::Time::now() - lastStartTime >= ros::Duration(1.0 / cfg.mainFrequency))
        {
            computeReferencePath();
            computeVelocitySamples();
            lastStartTime = ros::Time::now();
            runMPC();
            visualizeReferencePath();

            // Reset odometry
            dYaw = 0.0;
            dx = 0.0;
            dy = 0.0;
            ds = 0.0;
        }
    }

    /*
     *
     * Dynamic Object callback.
     *
     */
    void callbackDynObjects(const definitions::IkaObjectList &msg)
    {
        dynObjectList = msg;
        dynObjectList.header.stamp = ros::Time::now(); // No timestamp set by default?!
    }

    /*
     *
     * Set up the MPC.
     * Specify settings, vehicle model, constraints and cost terms.
     *
     */
    void initMPC()
    {
        //
        // Settings
        //

        // settings for the iLQR instance used in MPC.
        ilqr_settings_mpc.dt = mpc_dt; // the control discretization in [sec]
        ilqr_settings_mpc.integrator = ct::core::IntegrationType::RK4;
        ilqr_settings_mpc.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
        ilqr_settings_mpc.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::GNMS; // Gauss-Newton-Multiple-Shooting
        ilqr_settings_mpc.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;
        ilqr_settings_mpc.max_iterations = 1; // In MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
        ilqr_settings_mpc.printSummary = bReset && cfg.printSummary;

        //
        // Setup
        //

        // System
        vehicleDynamics = std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>>(new systemDynamics());
        adLinearizer = std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>>(new ct::core::SystemLinearizer<state_dim, control_dim>(vehicleDynamics));

        // Constraints
        std::shared_ptr<ConstraintContainerAD<state_dim, control_dim>> constraints(new ct::optcon::ConstraintContainerAD<state_dim, control_dim>());
        Eigen::VectorXd u_lb, u_ub, x_lb, x_ub;
        Eigen::VectorXi sparsity_u(control_dim), sparsity_x(state_dim);
        getControlConstraints(u_lb, u_ub, sparsity_u);
        getStateConstraints(x_lb, x_ub, sparsity_x);
        std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlConstraint(new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub, sparsity_u));
        std::shared_ptr<StateConstraint<state_dim, control_dim>> stateConstraint(new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sparsity_x));
        std::shared_ptr<ConstraintTerm<systemDynamics>> accConstraint(new ConstraintTerm<systemDynamics>());
        constraints->addIntermediateConstraint(controlConstraint, false);
        constraints->addIntermediateConstraint(stateConstraint, false);
        constraints->addIntermediateConstraint(accConstraint, false);
        constraints->initialize();

        std::cout << "Creating cost functions!" << std::endl;

        // Cost functions
        intermediateTerm = std::shared_ptr<CostTermIntermediate<state_dim + params_dim, control_dim, PLANNER, double, ct::core::ADCGScalar>>(new CostTermIntermediate<state_dim + params_dim, control_dim, PLANNER, double, ct::core::ADCGScalar>(this));
        finalTerm = std::shared_ptr<CostTermFinal<state_dim + params_dim, control_dim, PLANNER, double, ct::core::ADCGScalar>>(new CostTermFinal<state_dim + params_dim, control_dim, PLANNER, double, ct::core::ADCGScalar>(this));
        costFunction = std::shared_ptr<CostFunctionADParams<state_dim, control_dim, params_dim>>(new CostFunctionADParams<state_dim, control_dim, params_dim>());
        costFunction->addIntermediateADTermParam(intermediateTerm);
        costFunction->addFinalADTermParam(finalTerm);
        costFunction->initialize();

        std::cout << "Creating cost functions done!" << std::endl;

        // Initial state
        StateVector<state_dim> x0;
        x0.setZero();

        // Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
        ContinuousOptConProblem<state_dim, control_dim> optConProblem(timeHorizon, x0, vehicleDynamics, costFunction, adLinearizer);
        optConProblem.setGeneralConstraints(constraints);
        ilqr_mpc = new NLOptConSolver<state_dim, control_dim>(optConProblem, ilqr_settings_mpc);
        K = ilqr_settings_mpc.computeK(timeHorizon);
        bReset = true;
    }

    /*
     *
     * Main loop, wait for input.
     *
     */
    void run()
    {
        ros::spin();
    }

    /*
     *
     * Compute the discrete reference path from the boundaries
     *
     * It is assumed that the boundaries are mostly parallel.
     * Interpolation since the laser scanner provides arbitrary sampling.
     *
     */
    void computeReferencePath()
    {
        referencePath.clear();

        if (boundaryLeft.size() <= 1 || boundaryRight.size() <= 1)
        {
            std::cout << "Cannot extract reference path!" << std::endl;
            return;
        }

        // Compute s of boundaries
        std::vector<double> s_left(boundaryLeft.size()), s_right(boundaryRight.size());
        s_left[0] = s_right[0] = 0.0;
        for (size_t i = 1; i < boundaryLeft.size(); i++)
            s_left[i] = s_left[i - 1] + std::sqrt(std::pow(boundaryLeft[i].x - boundaryLeft[i - 1].x, 2) + std::pow(boundaryLeft[i].y - boundaryLeft[i - 1].y, 2));
        for (size_t i = 1; i < boundaryRight.size(); i++)
            s_right[i] = s_right[i - 1] + std::sqrt(std::pow(boundaryRight[i].x - boundaryRight[i - 1].x, 2) + std::pow(boundaryRight[i].y - boundaryRight[i - 1].y, 2));

        // Determine size
        size_t size = params_dim_refPath / 3;
        if (s_left.back() <= s_right.back())
            interpolation_s = s_left.back() / size;
        else
            interpolation_s = s_right.back() / size;
        if (size <= 1)
        {
            std::cout << "Cannot extract reference path!" << std::endl;
            return;
        }
        referencePath.resize(size);

        // Interpolate
        int lastSampleLeft = 0;
        int lastSampleRight = 0;
        for (size_t i = 0; i < referencePath.size(); i++)
        {
            geometry_msgs::Point interpolatedPointLeft = interpolateBoundary(boundaryLeft, s_left, lastSampleLeft, interpolation_s * i);
            geometry_msgs::Point interpolatedPointRight = interpolateBoundary(boundaryRight, s_right, lastSampleRight, interpolation_s * i);
            referencePath[i].x = (interpolatedPointLeft.x + interpolatedPointRight.x) / 2.0;
            referencePath[i].y = (interpolatedPointLeft.y + interpolatedPointRight.y) / 2.0;
            referencePath[i].z = 0.0;
        }
    }

    /*
     *
     * Computes the Menger curvature (https://en.wikipedia.org/wiki/Menger_curvature)
     * Also see https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
     *
     */
    double approxCurvature(const geometry_msgs::Point &A, const geometry_msgs::Point &B, const geometry_msgs::Point &C)
    {
        // Use Heron's formular to compute triangle area
        // Compute side lengths of triangle
        const double a = std::sqrt(std::pow(A.x - B.x, 2) + std::pow(A.y - B.y, 2));
        const double b = std::sqrt(std::pow(A.x - C.x, 2) + std::pow(A.y - C.y, 2));
        const double c = std::sqrt(std::pow(B.x - C.x, 2) + std::pow(B.y - C.y, 2));
        const double s = (a + b + c) / 2.0;
        const double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

        return 4 * area / (a * b * c);
    }

    /*
     *
     * Compute velocity for each reference path sample.
     *
     */
    void computeVelocitySamples()
    {
        referenceVelocity.clear();
        referenceVelocity.resize(referencePath.size(), cfg.targetVelocity / 3.6);

        // Ego velocity
        referenceVelocity[0] = std::min(vehicleData.velocity + 1.0, referenceVelocity[0]);

        // Curvature dependent reference velocity
        for (size_t i = 1; i < referencePath.size() - 1; i++)
        {
            double curvature = approxCurvature(referencePath[i - 1], referencePath[i], referencePath[i + 1]);
            curvature = std::max(curvature, 1e-6); // safeguard to avoid division by zero
            const double curvVMax = std::sqrt(cfg.aCurvRefVel / curvature);
            referenceVelocity[i] = std::min(referenceVelocity[i], curvVMax);
        }

        // Apply smoothing step according to aRef in longitudinal direction (backward sweep)
        for (int i = referenceVelocity.size() - 2; i >= 0; i--)
        {
            const double vT = referenceVelocity[i + 1];
            const double v = referenceVelocity[i];
            if (vT < v)
            {
                const double vGradDependend = std::sqrt(vT * vT + 2 * cfg.aGradRefVel * interpolation_s);
                referenceVelocity[i] = std::min(v, vGradDependend);
            }
        }

        // Apply smoothing step according to aRef in longitudinal direction (forward sweep)
        for (size_t i = 0; i < referenceVelocity.size() - 1; i++)
        {
            const double vT = referenceVelocity[i + 1];
            const double v = referenceVelocity[i];
            if (vT > v)
            {
                const double vGradDependend = std::sqrt(v * v + 2 * cfg.aGradRefVel * interpolation_s);
                referenceVelocity[i + 1] = std::min(vT, vGradDependend);
            }
        }
    }

    /*
     *
     * Linearly interpolates a point at distance desired_s on the discrete boundary.
     *
     */
    static geometry_msgs::Point interpolateBoundary(const std::vector<geometry_msgs::Point> &points, const std::vector<double> &s_vec, int &start, const double desired_s)
    {
        geometry_msgs::Point interp;

        // Locate
        while (start < points.size() && s_vec[start] <= desired_s)
            start++;
        start--;

        if (start == points.size() - 1 || start < 0)
        {
            std::cout << "Boundary interpolation error!" << std::endl;
            interp.x = interp.y = interp.z = 0.0;
            return interp;
        }

        // Interpolate
        double factor;
        if (s_vec[start + 1] - s_vec[start] == 0.0)
            factor = 0;
        else
            factor = (desired_s - s_vec[start]) / (s_vec[start + 1] - s_vec[start]);

        interp.x = points[start].x + (points[start + 1].x - points[start].x) * factor;
        interp.y = points[start].y + (points[start + 1].y - points[start].y) * factor;
        interp.z = points[start].z + (points[start + 1].z - points[start].z) * factor;

        return interp;
    }

    /*
     *
     * Visualize reference path in rviz.
     *
     */
    void visualizeReferencePath()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser_front_ref";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.ns = "mpc_referencePath";
        marker.lifetime = ros::Duration(0.2);
        marker.id = 0;

        for (size_t i = 0; i < params_dim_refPath / 3; i++)
        {
            geometry_msgs::Point p = referencePath[i];
            p.z = referenceVelocity[i];
            marker.points.push_back(p);
        }
        publisher_rviz.publish(marker);
    }

    /*
     *
     * Publish resulting trajectory to both, controller and rviz.
     *
     */
    void publishTrajectory()
    {
        // To controller
        definitions::IkaTpTrajectoryInterface trjForCtrl;
        trjForCtrl.TIME.resize(100);
        trjForCtrl.X.resize(100);
        trjForCtrl.Y.resize(100);
        trjForCtrl.THETA.resize(100);
        trjForCtrl.V.resize(100);
        trjForCtrl.KAPPA.resize(100);
        trjForCtrl.DKAPPA.resize(100);
        trjForCtrl.S.resize(100);
        trjForCtrl.A.resize(100);

        trjForCtrl.valid = !bReset;
        if (!bReset)
        {
            StateTrajectory<state_dim> trajectory = lastValidTrajectory.getReferenceStateTrajectory();
            for (size_t i = 0; i < K + 1; i++)
            {
                trjForCtrl.TIME[i] = ilqr_settings_mpc.dt * i;
                trjForCtrl.X[i] = trajectory[i][0];
                trjForCtrl.Y[i] = trajectory[i][1];
                trjForCtrl.THETA[i] = trajectory[i][5];
                trjForCtrl.V[i] = trajectory[i][3];
                trjForCtrl.KAPPA[i] = std::tan(trajectory[i][6]) / systemDynamics::wheelBase;
                const double cosDelta = std::cos(trajectory[i][6]);
                trjForCtrl.DKAPPA[i] = trajectory[i][6] / (cosDelta * cosDelta * systemDynamics::wheelBase);
                trjForCtrl.S[i] = trajectory[i][2];
                trjForCtrl.A[i] = trajectory[i][4];
            }
        }

        trjForCtrl.num_Elements = static_cast<uint16_t>(K);
        trjForCtrl.sampling_Mode = static_cast<uint8_t>(0);
        trjForCtrl.ActivationModeLongitudinal = 0;
        trjForCtrl.ActivationModeLateral = 0;
        trjForCtrl.timestamp = ros::Time::now().toSec();

        publisher_trj.publish(trjForCtrl);

        // To rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser_front_ref";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.ns = "mpc_trajectory";
        marker.lifetime = ros::Duration(0.2);
        marker.id = 0;

        if (!bReset)
        {
            StateTrajectory<state_dim> trajectory = lastValidTrajectory.getReferenceStateTrajectory();

            for (size_t i = 0; i < K + 1; i++)
            {
                geometry_msgs::Point p;
                p.x = trajectory[i][0];
                p.y = trajectory[i][1];
                p.z = trajectory[i][3];
                marker.points.push_back(p);
            }
        }
        publisher_rviz.publish(marker);


        auto cost = ilqr_mpc->getBackend()->getSummary().totalCosts.back();
        std_msgs::Float64 cost_msg;
        cost_msg.data = cost;  // Assuming 'cost' is the variable holding the computed cost
        cost_pub_.publish(cost_msg);

    }

    void getInterpolationParameters(const StateVectorArray<state_dim> &traj, const double &s, size_t &index, double &factor)
    {
        for (index = 0; index < traj.size() - 1; index++)
        {
            if (std::abs(traj[index + 1][2] > std::abs(s)))
                break;
        }
        if (index == traj.size() - 1)
        {
            factor = 1;
            index--;
        }
        else if (traj[index + 1][2] - traj[index][2] == 0)
        {
            factor = 0;
        }
        else
        {
            factor = (s - traj[index][2]) / (traj[index + 1][2] - traj[index][2]);
        }
    }

    // intpTraj can be a state/control vector or a feedback matrix
    template <typename TrajType>
    void interpolateValues(const TrajType &traj, const double &factor, const size_t &index, typename TrajType::value_type &intpTraj)
    {
        intpTraj = traj[index] + (traj[index + 1] - traj[index]) * factor;
    }

    float previous_value = 0.0;
    int curr_target = 0;
    

    void updateEnvironmentData()
    {
        Eigen::Matrix<double, params_dim, 1> paramVector;

        // Weights
        paramVector(WEIGHTS::VEL - state_dim) = cfg.velocityWeight;
        paramVector(WEIGHTS::PATH - state_dim) = cfg.pathWeight;
        paramVector(WEIGHTS::YAW - state_dim) = cfg.yawWeight;
        paramVector(WEIGHTS::JERK - state_dim) = cfg.jerkWeight;
        paramVector(WEIGHTS::ALPHA - state_dim) = cfg.alphaWeight;
        paramVector(WEIGHTS::DYNOBJ - state_dim) = cfg.dynObjWeight;
        paramVector(WEIGHTS::PATH_REF - state_dim) = cfg.pathRef;
        paramVector(WEIGHTS::JERK_REF - state_dim) = cfg.jerkRef;
        paramVector(WEIGHTS::ALPHA_REF - state_dim) = cfg.alphaRef * M_PI / 180.0;
        paramVector(WEIGHTS::DYNOBJ_REF - state_dim) = cfg.dynObjRef;
        paramVector(WEIGHTS::TRAFFICLIGHT_REF - state_dim) = cfg.trafficLightRef;
        paramVector(WEIGHTS::TRAFFICLIGHT - state_dim) = cfg.trafficLightWeight;

        // Dyn object
        if (ros::Time::now() - dynObjectList.header.stamp > ros::Duration(1.0) || dynObjectList.objects.empty())
        {
            paramVector(DYNOBJCOORDS::X - state_dim) = 999999;
            paramVector(DYNOBJCOORDS::Y - state_dim) = 999999;
            paramVector(DYNOBJCOORDS::Vx - state_dim) = 999999;
            paramVector(DYNOBJCOORDS::Vy - state_dim) = 999999;
            paramVector(DYNOBJCOORDS::Ax - state_dim) = 999999;
            paramVector(DYNOBJCOORDS::Ay - state_dim) = 999999;
            paramVector(DYNOBJCOORDS::len - state_dim) = 20.0;
            no_target = true;
        }
        else
        {
            // definitions::IkaObject relTarget = getRelevantTarget(dynObjectList.objects);
            definitions::IkaObject relTarget = getHighestPriorityObject(dynObjectList.objects, ego_pos_vec_, ego_vel_vec_, ego_heading_);
            std::vector<float> pos_vec = IkaUtilities::getObjectPosition(relTarget);
            std::vector<float> vel_vec = IkaUtilities::getObjectVelocity(relTarget);
            std::vector<float> acc_vec = IkaUtilities::getObjectAcceleration(relTarget);
            std::vector<float> size = IkaUtilities::getObjectSize(relTarget);
            std::vector<float> obj_size_vec = {size[0], size[1]};
            no_target = false; //setting flag to target detected
            target_pos_vec_ = pos_vec; //setting postion vector of dynamic object
            target_vel_vec_ = vel_vec; //setting velocity vector of dynamic object
            double val_d = IkaUtilities::getObjectHeading(relTarget);


            target_heading_ = val_d;
            float ttc_val = getTimeToCollision(target_pos_vec_, ego_pos_vec_, target_vel_vec_, ego_vel_vec_, target_heading_, ego_heading_, no_target);
            std_msgs::Float64 ttc_msg;
            ttc_msg.data = ttc_val;  // Assuming 'cost' is the variable holding the computed cost
            ttc_publisher.publish(ttc_msg);

            ROS_INFO_STREAM("Rel Target ID: " << relTarget.IdInternal);
            paramVector(DYNOBJCOORDS::X - state_dim) = pos_vec[0];
            paramVector(DYNOBJCOORDS::Y - state_dim) = pos_vec[1];
            paramVector(DYNOBJCOORDS::Vx - state_dim) = vel_vec[0];
            paramVector(DYNOBJCOORDS::Vy - state_dim) = vel_vec[1];
            paramVector(DYNOBJCOORDS::Ax - state_dim) = acc_vec[0];
            paramVector(DYNOBJCOORDS::Ay - state_dim) = acc_vec[1];
            paramVector(DYNOBJCOORDS::len - state_dim) = obj_size_vec[0];

            // Cost function analysis
            dist2DynObj = sqrt(pow((pos_vec[0] - dx), 2) + pow((pos_vec[1] - dy), 2));
            dynObjVel = sqrt(pow(vel_vec[0], 2) + pow(vel_vec[1], 2));
            dynObjAcc = sqrt(pow(acc_vec[0], 2) + pow(acc_vec[1], 2));

            std_msgs::Float64 dist_msg;
            dist_msg.data = dist2DynObj;  // Assuming 'cost' is the variable holding the computed cost
            analysis_publisher.publish(dist_msg);
            
        }
        
        std::vector<TrafficLight> loc_tl;
        loc_tl = trafficlights;
        geometry_msgs::PoseStamped pose_in, pose_out;
        geometry_msgs::Point point_out;
        pose_in.header.frame_id = "map";
        pose_in.pose.orientation.w = 1.0;
        pose_out.pose.orientation.w = 1.0;
        //Traffic Light
        for(int i = 0; i<trafficlights.size(); i++)
        {

            //Check for SPAT Timeout
            if((ros::Time::now()-trafficlights[i].last_spat).toSec()>=10.0 && use_spats)
            {
                ROS_WARN_STREAM("SPAT Timeout for Traffic Light ID: " << trafficlights[i].tl_id);
                trafficlights[i].red = true;
                loc_tl[i].red = true;
            }

            if(!use_spats)
            {
                trafficlights[i].red = false;
                loc_tl[i].red = false;
            }
            //Transform TL Positions into vehicle coordinates
            loc_tl[i].ingress_lane.clear();
            for(int j = 0; j<trafficlights[i].ingress_lane.size(); j++)
            {
                pose_in.pose.position = trafficlights[i].ingress_lane[j];
                transform_buffer_->transform(pose_in, pose_out, "vehicle_body");
                point_out = pose_out.pose.position;
                loc_tl[i].ingress_lane.push_back(point_out);
            }
        }
        //Select relevant traffic light
        TrafficLight rel_tl = getRelevantTrafficLight(loc_tl);
        double dist = std::sqrt(std::pow(rel_tl.ingress_lane.back().x,2.0)+std::pow(rel_tl.ingress_lane.back().y,2.0));
        if(dist <= paramVector(WEIGHTS::TRAFFICLIGHT_REF - state_dim) && use_spats)
        {
            ROS_INFO_STREAM("Relevant Traffic Light ID: " << rel_tl.tl_id << " Signal Group ID: " << rel_tl.sig_id << " State Red: " << rel_tl.red);
        }
        paramVector(TRAFFICLIGHT::X_TL - state_dim) = rel_tl.ingress_lane.back().x;
        paramVector(TRAFFICLIGHT::Y_TL - state_dim) = rel_tl.ingress_lane.back().y;
        paramVector(TRAFFICLIGHT::STATE - state_dim) = (double)rel_tl.red;

        // Ref path + velocity
        paramVector.tail(params_dim - params_dim_cfg).setConstant(999999);
        auto refPathSample = referencePath.begin();
        auto refVelSample = referenceVelocity.begin();
        for (size_t i = params_dim_cfg; i < params_dim - 2; i += 3)
        {
            paramVector(i) = refPathSample->x;
            paramVector(i + 1) = refPathSample->y;
            paramVector(i + 2) = *refVelSample;
            refPathSample++;
            refVelSample++;
        }
        costFunction->setCurrentParameters(paramVector);
        ilqr_mpc->changeCostFunction(costFunction);
    }

    definitions::IkaObject getHighestPriorityObject(std::vector<definitions::IkaObject> objects, std::vector<float> ego_pos_vec, std::vector<float> ego_vel_vec, float ego_heading) {
    if(objects.size()<1) {
        return objects[0];
    }
    
    std::vector<float> priorities(objects.size(), 0);
    float totalInverseTTC = 0.0;
    
    for(int i = 0; i < objects.size(); i++) {
        definitions::IkaObject obj = objects[i];
        
        std::vector<float> obj_pos_vec = IkaUtilities::getObjectPosition(obj);
        std::vector<float> obj_vel_vec = IkaUtilities::getObjectVelocity(obj);
       
        float obj_heading = IkaUtilities::getObjectHeading(obj);
        
        float ttc = getTimeToCollision(obj_pos_vec, ego_pos_vec, obj_vel_vec, ego_vel_vec, obj_heading, ego_heading, false);
        
        priorities[i] = 1.0 / ttc;
        
        totalInverseTTC += priorities[i];
    }

        if(totalInverseTTC == 0) {
        ROS_INFO_STREAM("No high priority objects found; all objects have zero priority.");
        float min_distance = std::numeric_limits<float>::max();
        int min_distance_index = 0;
        
        for(int i = 0; i < objects.size(); i++) {
            std::vector<float> obj_pos_vec = IkaUtilities::getObjectPosition(objects[i]);
            float distance = std::sqrt(pow(obj_pos_vec[0] - ego_pos_vec[0], 2.0) + pow(obj_pos_vec[1] - ego_pos_vec[1], 2.0));
            
            if(distance < min_distance) {
                min_distance = distance;
                min_distance_index = i;
            }
        }
        ROS_INFO_STREAM("Distance based Object Index: " << min_distance_index);
        return objects[min_distance_index];
        }

        for(size_t i = 0; i < priorities.size(); i++) {
        priorities[i] /= totalInverseTTC;
        ROS_INFO_STREAM("Object Index: " << i << ", Priority: " << priorities[i]);
        }
        
    int maxIndex = std::max_element(priorities.begin(), priorities.end()) - priorities.begin();
    return objects[maxIndex];
    }


    float getTimeToCollision(std::vector<float> target_pos_vec, std::vector<float> ego_pos_vec, std::vector<float> target_vel_vec, std::vector<float> ego_vel_vec, float target_heading, float ego_heading, bool tgt_flag){
        float ttc= 100;
        if (!tgt_flag) {
        double vel_ego = std::sqrt(pow((ego_vel_vec[0]),2.0)+pow((ego_vel_vec[1]),2.0));
        double vel_target = std::sqrt(pow((target_vel_vec[0]),2.0)+pow((target_vel_vec[1]),2.0));
        double d = std::sqrt(pow((target_pos_vec[0]),2.0) + pow((target_pos_vec[1] ),2.0));
        ttc = d/vel_ego;
        }
        return ttc;
    }

    definitions::IkaObject getRelevantTarget(std::vector<definitions::IkaObject> objects)
    {
        if(objects.size()<1) //Only one Object in List
        {
            return objects[0];
        }
        else //Search for relevant Target
        {
            double wDist = 1.0;
            double wHead = 5.0;
            double wBehind = 5.0;
            std::vector<double> costs;
            costs.resize(objects.size());
            double lowest_cost = INFINITY;
            int id_lowest_cost = 0;
            //Calculate costs for each object which indicates the "probability" of being the relevant target
            for(int i = 0; i<objects.size(); i++)
            {
                std::vector<float> obj_pos_vec = IkaUtilities::getObjectPosition(objects[i]);
                float obj_heading = IkaUtilities::getObjectHeading(objects[i]);
                costs[i] = 0;
                //1. Euclidean Distance
                double dist = std::sqrt(pow(obj_pos_vec[0],2.0)+pow(obj_pos_vec[1],2.0));
                costs[i] += wDist * dist;

                //2. Heading Deviation
                costs[i] += wHead * (std::exp(std::fabs(obj_heading) * 2.0)-1);

                //3. X-Dist (is object behind us?)
                if(obj_pos_vec[0]<0.0)
                {
                    costs[i] += wBehind * std::fabs(obj_pos_vec[0]);
                }

                if(costs[i]<lowest_cost)
                {
                    lowest_cost = costs[i];
                    id_lowest_cost = i;
                }
            }
            return objects[id_lowest_cost];
        }
    }




    TrafficLight getRelevantTrafficLight(std::vector<TrafficLight> tls)
    {
        if(tls.size()<1) //Only one TrafficLight in List
        {
            return tls[0];
        }
        else //Search for relevant Target
        {
            double wDist = 1.0;
            double wY = 5.0;
            double wBehind = 5.0;
            std::vector<double> costs;
            costs.resize(tls.size());
            double lowest_cost = INFINITY;
            int id_lowest_cost = 0;
            //Calculate costs for each traffic light which indicates the "probability" of being the relevant target
            for(int i = 0; i<tls.size(); i++)
            {
                costs[i] = 0;
                //1. Euclidean Distance
                double dist = std::sqrt(pow(tls[i].ingress_lane.back().x,2.0)+pow(tls[i].ingress_lane.back().y,2.0));
                costs[i] += wDist * dist;

                //2. y-Deviation
                costs[i] += wY * (std::exp(std::fabs(tls[i].ingress_lane.back().y) * 2.0)-1);

                //3. X-Dist (is object behind us?)
                if(tls[i].ingress_lane.back().x<0.0)
                {
                    costs[i] += wBehind * std::fabs(tls[i].ingress_lane.back().x);
                }

                if(costs[i]<lowest_cost)
                {
                    lowest_cost = costs[i];
                    id_lowest_cost = i;
                }
            }
            return tls[id_lowest_cost];
        }
    }

    


    /*
     *
     * Defined in .cpp file
     *
     */
    void callbackSPAT(const definitions::v2x_SPAT& msg);
    void runMPC();
    void getControlConstraints(Eigen::VectorXd &u_lb, Eigen::VectorXd &u_ub, Eigen::VectorXi &sparsity);
    void getStateConstraints(Eigen::VectorXd &x_lb, Eigen::VectorXd &x_ub, Eigen::VectorXi &sparsity);



protected:
    int _argc;
    char **_argv;

    ros::Subscriber subscriber_lanes;
    ros::Subscriber subscriber_vehicle_data;
    ros::Subscriber subscriber_dyn;
    ros::Subscriber subscriber_spat;
    ros::Subscriber subscriber_ego_pose;
    ros::Publisher publisher_trj;
    ros::Publisher publisher_rviz;

    // Cost function analysis parameters
    ros::Publisher analysis_publisher;
    ros::Publisher cost_pub_;
    ros::Publisher ttc_publisher;


    // Odometry
    definitions::FlatlandVehicleState vehicleData;
    double dYaw;
    double dx;
    double dy;
    double ds;

    // Cost function analysis parameters and variables
    double egoVel;
    double egoAcc;
    double dist2DynObj;
    double dynObjVel;
    double dynObjAcc;

    // Time stamp related
    std::chrono::time_point<std::chrono::steady_clock> record_start;
    std::chrono::time_point<std::chrono::steady_clock> record_end;
    bool recording;

    NLOptConSettings ilqr_settings_mpc;
    NLOptConSolver<state_dim, control_dim> *ilqr_mpc;
    size_t K;
    ros::Time lastStartTime;
    bool bReset;

    StateFeedbackController<state_dim, control_dim> lastValidTrajectory;

    std::vector<geometry_msgs::Point> boundaryLeft;
    std::vector<geometry_msgs::Point> boundaryRight;

    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> vehicleDynamics;
    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer;
    std::shared_ptr<CostTermIntermediate<state_dim + params_dim, control_dim, PLANNER, double, ct::core::ADCGScalar>> intermediateTerm;
    std::shared_ptr<CostTermFinal<state_dim + params_dim, control_dim, PLANNER, double, ct::core::ADCGScalar>> finalTerm;
    std::shared_ptr<CostFunctionADParams<state_dim, control_dim, params_dim>> costFunction;

public:
    std::vector<geometry_msgs::Point> referencePath;
    std::vector<double> referenceVelocity;
    definitions::IkaObjectList dynObjectList;
    trajectory_planner::trajectory_plannerConfig cfg;

};

