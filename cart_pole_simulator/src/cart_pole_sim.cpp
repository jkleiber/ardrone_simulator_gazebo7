
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

#include <std_msgs/Empty.h>
#include <gtddp_drone_msgs/ctrl_data.h>
#include <gtddp_drone_msgs/state_data.h>

// Constants
const double gravity = 9.81;

// Initialization flags
bool sim_init = false;
bool sim_started = false;

// System parameters
double bob_mass;
double cart_mass;
double max_force;
double pole_length;

// Track state and control
Eigen::Vector4d state;
Eigen::VectorXd _u = Eigen::VectorXd::Zero(1);

// ODE solver
boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd, double, Eigen::VectorXd, double, boost::numeric::odeint::vector_space_algebra> stepper;

// System state Publisher
ros::Publisher state_pub;

// Time tracking
ros::Time prev_time;



void dynamics(const Eigen::Vector4d& x, Eigen::VectorXd& dxdt, double t)
{
    // Name states to make it easier
    double x_dot     = x(1);
    double theta     = x(2);
    double theta_dot = x(3);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double ctrl = _u(0);

    // Set up dx
    Eigen::VectorXd dx(4);
    dx.setZero();

    // Apply dynamics to calculate dx
    dx(0) =  x_dot;
    dx(1) =  (-bob_mass*pole_length*pow(theta_dot,2)*sin_theta + bob_mass*gravity*sin_theta*cos_theta + ctrl)/(cart_mass+bob_mass*pow(sin_theta,2));
    dx(2) =  theta_dot;
    dx(3) =  (-bob_mass*pole_length*pow(theta_dot,2)*sin_theta*cos_theta + (cart_mass+bob_mass)*gravity*sin_theta + ctrl*cos_theta) / (cart_mass*pole_length + bob_mass*pole_length*pow(sin_theta,2));
    dxdt=dx;
}


void updateState()
{
    // Find timestep
    double dt = (ros::Time::now() - prev_time).toSec();

    // Set the placeholder state
    Eigen::Vector4d x = state;

    // Calculate the current state of the system using an ODE solver
    // Integrate from time 0 every time because we only want to forward propagate one step
    stepper.do_step(dynamics, x, 0, dt);

    // Update state
    state = x;

    // Update the time tracking
    prev_time = ros::Time::now();
}


void startCallback(const std_msgs::Empty::ConstPtr& empty)
{
    sim_started = true;
    std::cout << "Cart-Pole Simulation started!\n";
}

void stopCallback(const std_msgs::Empty::ConstPtr& empty)
{
    sim_started = false;
    std::cout << "Cart-Pole Simulation paused!\n";
}


void controlCallback(const gtddp_drone_msgs::ctrl_data::ConstPtr& ctrl)
{
    // Update the current force
    double force = ctrl->ctrl[0];

    // Make sure the force stays within the boundaries
    if(force < -max_force)
    {
        force = -max_force;
    }
    else if(force > max_force)
    {
        force = max_force;
    }

    // Update the dynamics given the control
    _u(0) = force;
    updateState();
}


void statePubCallback(const ros::TimerEvent event)
{
    // Make the state data message
    gtddp_drone_msgs::state_data state_msg;

    // Only publish while the simulation is active
    if(sim_started)
    {
        // Set the state data message to the right size
        state_msg.state.resize(4);

        // Populate the state
        for(int i = 0; i < 4; ++i)
        {
            state_msg.state[i] = state(i);
        }

        // Publish
        state_pub.publish(state_msg);
    }
}


void simLoop(const ros::TimerEvent& event)
{
    // Make the timer run once before calculating dt
    // Also don't run the simulator unless the start command has been sent
    if(!sim_init || !sim_started)
    {
        sim_init = true;
        prev_time = ros::Time::now();
        return;
    }

    // Update the simulation state consistently
    // updateState();
}


int main(int argc, char **argv)
{
    // Initialize
    ros::init(argc, argv, "cart_pole_sim_node");

    // Define the node
    ros::NodeHandle cpsim;

    // Initialize the state
    std::vector<double> init_state;

    if(!cpsim.getParam("/init_state", init_state))
    {
        ROS_WARN("Warning: no initial state provided");

        // Start with the default start state (pendulum up, cart sitting still)
        state << 0, 0, 0, 0;
    }
    else
    {
        // Copy initial conditions from the temporary vector
        for(int i=0; i < 4; ++i)
        {
            state(i) = init_state[i];
        }
    }

    // Initialize the system paramters
    bob_mass = cpsim.param<double>("/mass", 0.50);
    cart_mass = cpsim.param<double>("/cart_mass", 10.0);
    max_force = cpsim.param<double>("/max_force", 100.0);
    pole_length = cpsim.param<double>("/length", 1.0);

    std::cout << bob_mass << " " << cart_mass << " " << max_force << " " << pole_length << std::endl;

    // Subscribe to the control signal
    ros::Subscriber ctrl_sub = cpsim.subscribe(cpsim.resolveName("/cart/control"), 1, &controlCallback);
    ros::Subscriber start_sub = cpsim.subscribe(cpsim.resolveName("/cart/start"), 1, &startCallback);
    ros::Subscriber stop_sub = cpsim.subscribe(cpsim.resolveName("/cart/stop"), 1, &stopCallback);

    // Publisher for the system state
    state_pub = cpsim.advertise<gtddp_drone_msgs::state_data>(cpsim.resolveName("/cart/state"), 1);

    // Initialize the time
    prev_time = ros::Time::now();

    // Simulator main thread and state estimator are run on timers
    ros::Timer sim_timer = cpsim.createTimer(ros::Duration(0.001), &simLoop);   // 1000 Hz
    ros::Timer state_pub_timer = cpsim.createTimer(ros::Duration(0.01), &statePubCallback); // 100 Hz

    // Keep thread alive / pump callbacks
    ros::spin();

    return 0;
}
