#ifndef msr_AirLibUnitTests_RovHydrodynamicsTest_hpp
#define msr_AirLibUnitTests_RovHydrodynamicsTest_hpp

#include "TestBase.hpp"
#include "vehicles/rov/RovPhysicsBody.hpp"
#include "vehicles/rov/RovParamsFactory.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "common/SteppableClock.hpp"
#include <iostream>
#include <cmath>

namespace msr
{
namespace airlib
{

    class RovHydrodynamicsTest : public TestBase
    {
    public:
        virtual void run() override
        {
            std::cout << "Starting ROV Fossen Hydrodynamics & Depth Safety Tests..." << std::endl;

            auto clock = std::make_shared<SteppableClock>(3E-3f);
            ClockFactory::get(clock);

            AirSimSettings::VehicleSetting vehicle_setting("RovSimple", AirSimSettings::kVehicleTypeRovSimple);
            std::unique_ptr<RovBodyParams> params = RovBodyParamsFactory::createConfig(
                &vehicle_setting,
                std::make_shared<SensorFactory>());
            auto api = params->createRovApi();

            std::unique_ptr<Kinematics> kinematics;
            std::unique_ptr<Environment> environment;
            Kinematics::State initial_kinematic_state = Kinematics::State::zero();
            initial_kinematic_state.pose = Pose();
            kinematics.reset(new Kinematics(initial_kinematic_state));

            Environment::State initial_environment;
            initial_environment.position = initial_kinematic_state.pose.position;
            initial_environment.geo_point = GeoPoint();
            environment.reset(new Environment(initial_environment));

            RovPhysicsBody rov(params.get(), api.get(), kinematics.get(), environment.get());
            rov.reset();

            testWrenchVertexCount(rov, *params);
            testNeutralBuoyancy(rov, *kinematics, *environment, *params);
            testRestoringMoments(rov, *kinematics, *environment, *params);
            testHydrodynamicDamping(rov, *kinematics, *params);
            testDepthSafetyBoundary(rov, *kinematics, *params);

            std::cout << "All ROV Hydrodynamics & Depth Safety Tests PASSED successfully!" << std::endl;
        }

    private:
        void testWrenchVertexCount(RovPhysicsBody& rov, const RovBodyParams& params)
        {
            uint expected_vertices = params.getParams().rotor_count + 3;
            testAssert(rov.wrenchVertexCount() == expected_vertices,
                       "wrenchVertexCount must include rotors + buoyancy + damping + bed_contact vertices");
            std::cout << "  [PASS] Wrench vertex count: " << rov.wrenchVertexCount()
                      << " (8 thrusters + 3 hydro vertices)" << std::endl;
        }

        void testNeutralBuoyancy(RovPhysicsBody& rov, Kinematics& kinematics, const Environment& environment, const RovBodyParams& params)
        {
            // Reset to level orientation
            Kinematics::State state = Kinematics::State::zero();
            state.pose = Pose(Vector3r::Zero(), Quaternionr::Identity());
            kinematics.setState(state);
            rov.update();

            const real_T mass = params.getParams().mass;
            const real_T g = environment.getState().gravity.z();
            const real_T expected_buoyancy = mass * g;

            const Wrench& b_wrench = rov.getBuoyancyVertex().getWrench();
            // In NED body frame with level attitude, buoyancy force is straight up along -Z:
            testAssert(std::abs(b_wrench.force.z() - (-expected_buoyancy)) < 1e-2f,
                       "Buoyancy force in body frame must equal -m*g along Z");
            testAssert(std::abs(b_wrench.force.x()) < 1e-4f && std::abs(b_wrench.force.y()) < 1e-4f,
                       "Buoyancy force XY components must be zero when level");

            // Evaluate net body wrench through FastPhysicsEngine
            Wrench body_wrench = rov.getBodyWrench();
            Vector3r net_accel = (body_wrench.force / mass) + environment.getState().gravity;
            testAssert(net_accel.norm() < 1e-2f,
                       "Net linear acceleration under neutral buoyancy and gravity must be zero (canceling gravity)");

            std::cout << "  [PASS] Neutral buoyancy: Mass=" << mass << "kg, Gravity=" << g
                      << "m/s^2, BuoyancyForce=" << expected_buoyancy
                      << "N, NetAccel=" << net_accel.norm() << "m/s^2 (Zero)" << std::endl;
        }

        void testRestoringMoments(RovPhysicsBody& rov, Kinematics& kinematics, const Environment& environment, const RovBodyParams& params)
        {
            const real_T off_z = params.getParams().off_z;
            const real_T mass = params.getParams().mass;
            const real_T g = environment.getState().gravity.z();

            // 1. Roll Test (+15 deg tilt, starboard down)
            real_T roll_rad = Utils::degreesToRadians(15.0f);
            Quaternionr q_roll = VectorMath::toQuaternion(0.0f, roll_rad, 0.0f);
            Kinematics::State state = Kinematics::State::zero();
            state.pose.orientation = q_roll;
            kinematics.setState(state);
            rov.update();

            Wrench wrench_roll = rov.getBodyWrench();
            // Restoring torque about body X must be strictly negative to restore port-side up
            testAssert(wrench_roll.torque.x() < -0.01f,
                       "Roll restoring torque tau_x must be strictly negative when vehicle rolls to starboard");
            real_T expected_roll_tau = -std::abs(off_z) * mass * g * std::sin(roll_rad);
            testAssert(std::abs(wrench_roll.torque.x() - expected_roll_tau) < 0.1f,
                       "tau_x must match Fossen restoring moment formula: -off_z * m * g * sin(phi)");

            // 2. Pitch Test (+15 deg tilt, nose up)
            real_T pitch_rad = Utils::degreesToRadians(15.0f);
            Quaternionr q_pitch = VectorMath::toQuaternion(pitch_rad, 0.0f, 0.0f);
            state.pose.orientation = q_pitch;
            kinematics.setState(state);
            rov.update();

            Wrench wrench_pitch = rov.getBodyWrench();
            // Restoring torque about body Y must be strictly negative to restore nose-down
            testAssert(wrench_pitch.torque.y() < -0.01f,
                       "Pitch restoring torque tau_y must be strictly negative when vehicle pitches up");
            real_T expected_pitch_tau = -std::abs(off_z) * mass * g * std::sin(pitch_rad);
            testAssert(std::abs(wrench_pitch.torque.y() - expected_pitch_tau) < 0.1f,
                       "tau_y must match Fossen restoring moment formula: -off_z * m * g * sin(theta)");

            std::cout << "  [PASS] Fossen metacentric self-righting restoring moments: "
                      << "Roll tau_x=" << wrench_roll.torque.x() << " Nm (restoring), "
                      << "Pitch tau_y=" << wrench_pitch.torque.y() << " Nm (restoring)" << std::endl;
        }

        void testHydrodynamicDamping(RovPhysicsBody& rov, Kinematics& kinematics, const RovBodyParams& params)
        {
            Kinematics::State state = Kinematics::State::zero();
            state.pose.orientation = Quaternionr::Identity();
            state.twist.linear = Vector3r(1.0f, 1.5f, 0.8f); // Surge, sway, heave
            state.twist.angular = Vector3r(0.6f, -0.5f, 0.4f); // Roll rate, pitch rate, yaw rate
            kinematics.setState(state);
            rov.update();

            const Wrench& d_wrench = rov.getDampingVertex().getWrench();
            const auto& p = params.getParams();

            // Check linear damping forces oppose velocity
            testAssert(d_wrench.force.x() < 0.0f, "Surge damping force must oppose positive forward velocity");
            testAssert(d_wrench.force.y() < 0.0f, "Sway damping force must oppose positive starboard velocity");
            testAssert(d_wrench.force.z() < 0.0f, "Heave damping force must oppose positive downward velocity");

            // Verify damping equation: -|D_L|*v - |D_Q|*|v|*v
            real_T expected_fx = -std::abs(p.damping_linear.x()) * 1.0f - std::abs(p.damping_linear_q.x()) * 1.0f * 1.0f;
            testAssert(std::abs(d_wrench.force.x() - expected_fx) < 1e-3f, "Surge drag matches Fossen formula");

            // Check angular damping torques oppose angular rates
            testAssert(d_wrench.torque.x() < 0.0f, "Roll damping torque must oppose positive roll rate");
            testAssert(d_wrench.torque.y() > 0.0f, "Pitch damping torque must oppose negative pitch rate");
            testAssert(d_wrench.torque.z() < 0.0f, "Yaw damping torque must oppose positive yaw rate");

            std::cout << "  [PASS] Hydrodynamic damping: Force=[" << d_wrench.force.x() << ", "
                      << d_wrench.force.y() << ", " << d_wrench.force.z() << "] N, Torque=["
                      << d_wrench.torque.x() << ", " << d_wrench.torque.y() << ", " << d_wrench.torque.z()
                      << "] Nm (All strictly dissipative)" << std::endl;
        }

        void testDepthSafetyBoundary(RovPhysicsBody& rov, Kinematics& kinematics, const RovBodyParams& params)
        {
            const real_T bed_depth = std::abs(params.getParams().z_bed); // 30.0m

            // 1. Normal depth (z = 10m): No contact reaction
            Kinematics::State state = Kinematics::State::zero();
            state.pose.position = Vector3r(0.0f, 0.0f, 10.0f);
            state.twist.linear = Vector3r(0.0f, 0.0f, 1.0f);
            kinematics.setState(state);
            rov.update();
            testAssert(rov.getBedContactVertex().getWrench().force.norm() < 1e-4f,
                       "No bed contact force when vehicle is operating safely above bed");

            // 2. Approaching cushion (z = 29.8m, vz = 1.0 m/s): Smooth cushion damping active
            state.pose.position = Vector3r(0.0f, 0.0f, 29.8f);
            state.twist.linear = Vector3r(0.0f, 0.0f, 1.0f);
            kinematics.setState(state);
            rov.update();
            const Wrench& cushion_wrench = rov.getBedContactVertex().getWrench();
            testAssert(cushion_wrench.force.z() < 0.0f,
                       "Cushion damping force must be upward (-Z) to smoothly decelerate descent");

            // 3. Bed penetration (z = 30.1m, delta_z = 0.1m, vz = 0.5 m/s): Penalty spring-damper reaction
            state.pose.position = Vector3r(0.0f, 0.0f, 30.1f);
            state.twist.linear = Vector3r(0.0f, 0.0f, 0.5f);
            kinematics.setState(state);
            rov.update();
            const Wrench& reaction_wrench = rov.getBedContactVertex().getWrench();
            real_T expected_rebound = params.getParams().bed_stiffness * 0.1f + params.getParams().bed_damping * 0.5f;
            testAssert(std::abs(reaction_wrench.force.z() - (-expected_rebound)) < 1.0f,
                       "Bed contact force must match penalty spring-damper model (-kp*delta_z - kd*vz)");

            // 4. Hard safety clamp (z = 31.0m, beyond hard threshold 30.3m): Hard clamp activates
            state.pose.position = Vector3r(0.0f, 0.0f, 31.0f);
            state.twist.linear = Vector3r(0.0f, 0.0f, 2.5f);
            kinematics.setState(state);
            rov.update(); // triggers enforceDepthClamp()
            testAssert(rov.getPose().position.z() <= bed_depth + 1e-3f,
                       "Hard depth clamp must clamp vehicle position to bed depth");
            testAssert(rov.getTwist().linear.z() <= 0.0f,
                       "Hard depth clamp must eliminate downward penetration velocity");

            std::cout << "  [PASS] Depth safety boundary: Soft cushion + Spring-Damper reaction ("
                      << reaction_wrench.force.z() << " N) + Hard kinematic clamp to "
                      << bed_depth << "m successfully verified!" << std::endl;
        }
    };

}
} // namespace

#endif
