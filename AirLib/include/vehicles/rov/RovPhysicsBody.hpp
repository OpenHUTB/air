// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_rovbody_hpp
#define msr_airlib_rovbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "RotorTiltable.hpp"
#include "AeroVertex.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "RovBodyParams.hpp"
#include "physics/PhysicsBody.hpp"
#include <cmath>

namespace msr
{
namespace airlib
{

    class RovPhysicsBody;

    // Fossen 6-DOF Hydrodynamic Buoyancy Vertex
    // Generates upward neutral buoyancy in world frame balancing gravity: F_b = -m * g
    // Applied at Center of Buoyancy (COB), which is offset from Center of Mass (COM) along Z: r_COB = [0, 0, -|off_z|]
    // The offset naturally creates passive pitch and roll righting moments: tau_restoring = r_COB x F_b_body
    class RovBuoyancyVertex : public PhysicsBodyVertex
    {
    public:
        RovBuoyancyVertex() = default;

        void initialize(const RovPhysicsBody* body);

    protected:
        virtual void setWrench(Wrench& wrench) override;

    private:
        const RovPhysicsBody* body_ = nullptr;
    };

    // Fossen 6-DOF Hydrodynamic Damping Vertex
    // Computes linear and quadratic fluid resistance forces and damping moments:
    //   D(nu)*nu = D_linear * nu + D_quadratic * |nu| * nu
    // Resists surge, sway, heave (linear) and roll, pitch, yaw (angular) in water
    class RovDampingVertex : public PhysicsBodyVertex
    {
    public:
        RovDampingVertex() = default;

        void initialize(const RovPhysicsBody* body);

    protected:
        virtual void setWrench(Wrench& wrench) override;

    private:
        const RovPhysicsBody* body_ = nullptr;
    };

    // Seabed Depth Safety Boundary Vertex (Soft-Hard Contact Model)
    // Applies smooth spring-damper contact reaction when approaching/penetrating the configured seabed depth limit Z_bed
    // Guarantees ROV never falls into the void or penetrates through boundaries
    class RovBedContactVertex : public PhysicsBodyVertex
    {
    public:
        RovBedContactVertex() = default;

        void initialize(const RovPhysicsBody* body);

    protected:
        virtual void setWrench(Wrench& wrench) override;

    private:
        const RovPhysicsBody* body_ = nullptr;
    };

    class RovPhysicsBody : public PhysicsBody
    {
    public:
        RovPhysicsBody(RovBodyParams* params, VehicleApiBase* vehicle_api,
                       Kinematics* kinematics, Environment* environment)
            : params_(params), vehicle_api_(vehicle_api)
        {
            setName("RovPhysicsBody");
            vehicle_api_->setParent(this);
            initialize(kinematics, environment);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            //reset rotors, kinematics and environment
            PhysicsBody::resetImplementation();

            //reset sensors last after their ground truth has been reset
            resetSensors();
        }

        virtual void update() override
        {
            //enforce depth clamp hard limit check
            enforceDepthClamp();

            //update forces on vertices that we will use next
            PhysicsBody::update();

            //Note that controller gets updated after kinematics gets updated in updateKinematics
            //otherwise sensors will have values from previous cycle causing lags which will appear
            //as crazy jerks whenever commands like velocity is issued
        }

        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            PhysicsBody::reportState(reporter);

            reportSensors(*params_, reporter);

            //report rotors
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                reporter.startHeading("", 1);
                reporter.writeValue("RotorTiltable", rotor_index);
                reporter.endHeading(false, 1);
                rotors_.at(rotor_index).reportState(reporter);
            }

            //report AeroVertex
            reporter.startHeading("", 1);
            reporter.writeValue("AeroVertex", 0);
            reporter.endHeading(false, 1);
            aero_vertex_.reportState(reporter);

            //report Hydrodynamics
            reporter.startHeading("Hydrodynamics", 1);
            reporter.writeValue("BuoyancyForce_Body", buoyancy_vertex_.getWrench().force);
            reporter.writeValue("DampingForce_Body", damping_vertex_.getWrench().force);
            reporter.writeValue("DampingTorque_Body", damping_vertex_.getWrench().torque);
            reporter.writeValue("BedContactForce_Body", bed_contact_vertex_.getWrench().force);
            reporter.writeValue("BedDepthLimit", std::abs(params_->getParams().z_bed));
            reporter.endHeading(false, 1);
        }
        //*** End: UpdatableState implementation ***//

        //Fast Physics engine calls this method to set next kinematics
        virtual void updateKinematics(const Kinematics::State& kinematics) override
        {
            PhysicsBody::updateKinematics(kinematics);

            enforceDepthClamp();

            updateSensorsAndController();
        }

        //External Physics engine calls this method to keep physics bodies updated and move rotors
        virtual void updateKinematics() override
        {
            PhysicsBody::updateKinematics();

            enforceDepthClamp();

            updateSensorsAndController();
        }

        void updateSensorsAndController()
        {
            updateSensors(*params_, getKinematics(), getEnvironment());

            //update controller which will update actuator control signal
            vehicle_api_->update();

            //update all control inputs
            //for now assuming actuation order is {flap1, flap2, flap3, rotor1thr, rotor1ang, rotor2thr, rotor2ang, ...}
            vector<real_T> rov_inputs = { 0.0, 0.0, 0.0 };
            //vector<real_T> rov_inputs = { vehicle_api_->getActuation(0), vehicle_api_->getActuation(1), vehicle_api_->getActuation(2) };
            aero_vertex_.setFlapInputs(rov_inputs);

            //transfer new input values from controller to rotors
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(rotor_index));
                //rotors_.at(rotor_index).setAngleSignal(vehicle_api_->getActuation(rotor_index));

                //rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(3 + 2 * rotor_index));
                //rotors_.at(rotor_index).setAngleSignal(vehicle_api_->getActuation(3 + 2 * rotor_index + 1));
            }
        }

        //this gets called in getDragWrench function in physics engine
        virtual void setAirspeedBody(const Vector3r airspeed_body_vector)
        {
            aero_vertex_.setAirspeedVertex(airspeed_body_vector);

            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                rotors_[rotor_index].setAirspeedRotor(airspeed_body_vector);
            }
        }

        void overwriteRotorTilts(const vector<float>& angles, bool spin_props)
        {
            for (uint i = 0; i < rotors_.size(); ++i) {
                auto& rotor = rotors_.at(i);
                rotor.overwriteTilt(angles[i], spin_props);
            }
        }

        //sensor getter
        const SensorCollection& getSensors() const
        {
            return params_->getSensors();
        }

        const RovBodyParams& getParams() const
        {
            return *params_;
        }

        RovBodyParams& getParams()
        {
            return *params_;
        }

        //physics body interface
        virtual uint wrenchVertexCount() const override
        {
            // Thrusters + Buoyancy Vertex + Damping Vertex + Bed Contact Safety Vertex
            return params_->getParams().rotor_count + 3;
        }

        virtual PhysicsBodyVertex& getWrenchVertex(uint index) override
        {
            const uint rotor_count = params_->getParams().rotor_count;
            if (index < rotor_count) {
                return rotors_.at(index);
            }
            else if (index == rotor_count) {
                return buoyancy_vertex_;
            }
            else if (index == rotor_count + 1) {
                return damping_vertex_;
            }
            else if (index == rotor_count + 2) {
                return bed_contact_vertex_;
            }
            throw std::out_of_range("wrench vertex index out of range");
        }

        virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
        {
            const uint rotor_count = params_->getParams().rotor_count;
            if (index < rotor_count) {
                return rotors_.at(index);
            }
            else if (index == rotor_count) {
                return buoyancy_vertex_;
            }
            else if (index == rotor_count + 1) {
                return damping_vertex_;
            }
            else if (index == rotor_count + 2) {
                return bed_contact_vertex_;
            }
            throw std::out_of_range("wrench vertex index out of range");
        }

        virtual real_T getRestitution() const override
        {
            return params_->getParams().restitution;
        }
        virtual real_T getFriction() const override
        {
            return params_->getParams().friction;
        }

        RotorTiltable::TiltOutput getRotorOutput(uint index) const
        {
            return rotors_.at(index).getOutput();
        }

        AeroVertex::Output getAeroOutput() const
        {
            return aero_vertex_.getOutput();
        }

        uint rotorCount() const
        {
            return params_->getParams().rotor_count;
        }

        const RovBuoyancyVertex& getBuoyancyVertex() const
        {
            return buoyancy_vertex_;
        }

        const RovDampingVertex& getDampingVertex() const
        {
            return damping_vertex_;
        }

        const RovBedContactVertex& getBedContactVertex() const
        {
            return bed_contact_vertex_;
        }

        // Aggregate total wrench (forces transformed to world frame, torques in body frame)
        Wrench getBodyWrench() const
        {
            Wrench wrench = Wrench::zero();
            const Quaternionr& orientation = getKinematics().pose.orientation;

            for (uint i = 0; i < wrenchVertexCount(); ++i) {
                const PhysicsBodyVertex& vertex = getWrenchVertex(i);
                const auto& vertex_wrench = vertex.getWrench();
                wrench += vertex_wrench;
                wrench.torque += vertex.getPosition().cross(vertex_wrench.force);
            }

            wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);
            return wrench;
        }

        virtual ~RovPhysicsBody() = default;

    private: //methods
        void initialize(Kinematics* kinematics, Environment* environment)
        {
            PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

            createRotors(*params_, rotors_, environment);
            createAeroVertex(*params_, aero_vertex_, environment, kinematics);

            // Initialize Fossen hydrodynamics and depth safety vertices
            buoyancy_vertex_.initialize(this);
            damping_vertex_.initialize(this);
            bed_contact_vertex_.initialize(this);

            initSensors(*params_, getKinematics(), getEnvironment());
        }

        static void createRotors(const RovBodyParams& params, vector<RotorTiltable>& rotors, const Environment* environment)
        {
            rotors.clear();
            //for each rotor pose
            for (uint rotor_index = 0; rotor_index < params.getParams().rotor_configs.size(); ++rotor_index) {
                const RovBodyParams::RotorTiltableConfiguration& rotor_config = params.getParams().rotor_configs.at(rotor_index);
                rotors.emplace_back(rotor_config.position, rotor_config.normal_nominal, rotor_config.direction, rotor_config.is_fixed, rotor_config.rotation_axis, rotor_config.max_angle, rotor_config.params, environment, rotor_index);
            }
        }

        static void createAeroVertex(const RovBodyParams& params, AeroVertex& aero_vertex, const Environment* environment, const Kinematics* kinematics)
        {
            AirState air_state;
            air_state.zero();
            aero_vertex.initialize(params.getParams().aero_params, environment, kinematics, air_state);
        }

        void reportSensors(RovBodyParams& params, StateReporter& reporter)
        {
            params.getSensors().reportState(reporter);
        }

        void updateSensors(RovBodyParams& params, const Kinematics::State& state, const Environment& environment)
        {
            unused(state);
            unused(environment);
            params.getSensors().update();
        }

        void initSensors(RovBodyParams& params, const Kinematics::State& state, const Environment& environment)
        {
            params.getSensors().initialize(&state, &environment);
        }

        void resetSensors()
        {
            params_->getSensors().reset();
        }

        // Hard depth clamp fail-safe to absolutely prevent falling through world or entering the void
        void enforceDepthClamp()
        {
            if (!params_->getParams().enable_depth_clamp) return;

            const real_T bed_depth = std::abs(params_->getParams().z_bed);
            const real_T hard_limit = bed_depth + params_->getParams().bed_clamp_margin;

            if (getKinematics().pose.position.z() > hard_limit) {
                Pose pose = getPose();
                pose.position.z() = bed_depth;
                setPose(pose);

                Twist twist = getTwist();
                if (twist.linear.z() > 0.0f) {
                    twist.linear.z() = 0.0f; // eliminate downward velocity
                }
                setTwist(twist);
            }
        }

    private: //fields
        RovBodyParams* params_;
        AeroVertex aero_vertex_; //one aerovertex at the center of mass for calculating aerodynamic forces and moments
        vector<RotorTiltable> rotors_;

        // Hydrodynamic vertices
        RovBuoyancyVertex buoyancy_vertex_;
        RovDampingVertex damping_vertex_;
        RovBedContactVertex bed_contact_vertex_;

        VehicleApiBase* vehicle_api_;
    };

    // Implementation of RovBuoyancyVertex methods
    inline void RovBuoyancyVertex::initialize(const RovPhysicsBody* body)
    {
        body_ = body;
        if (!body_) return;
        const real_T off_z = body_->getParams().getParams().off_z;
        // Center of Buoyancy (COB) is located above Center of Mass (COM).
        // In NED coordinate frame (+Z is down), being above COM means negative Z:
        // r_COB = [0, 0, -|off_z|]
        const Vector3r cob_pos(0.0f, 0.0f, -std::abs(off_z));
        PhysicsBodyVertex::initialize(cob_pos, Vector3r(0.0f, 0.0f, -1.0f));
    }

    inline void RovBuoyancyVertex::setWrench(Wrench& wrench)
    {
        wrench = Wrench::zero();
        if (!body_) return;

        const auto& p = body_->getParams().getParams();
        if (!p.enable_neutral_buoyancy) return;

        // Ensure COB position is synchronized with current params
        const real_T off_z = p.off_z;
        setPosition(Vector3r(0.0f, 0.0f, -std::abs(off_z)));

        // Gravitational acceleration in NED world frame (typically [0, 0, +9.80665])
        const Vector3r& gravity = body_->getEnvironment().getState().gravity;
        const real_T mass = body_->getMass();

        // In neutral buoyancy model, buoyancy force magnitude equals vehicle weight:
        // F_buoyancy_world = - mass * gravity = [0, 0, -mass * g]
        const Vector3r f_buoyancy_world = -mass * gravity;

        // Transform world buoyancy force into body-fixed frame
        const Quaternionr& orientation = body_->getKinematics().pose.orientation;
        const Vector3r f_buoyancy_body = VectorMath::transformToBodyFrame(f_buoyancy_world, orientation);

        // Set body-frame force.
        // In FastPhysicsEngine::getBodyWrench:
        //   1) wrench.force += f_buoyancy_body (which transforms back to f_buoyancy_world, perfectly balancing gravity)
        //   2) wrench.torque += r_COB x f_buoyancy_body (generating exact restoring torque for pitch and roll)
        wrench.force = f_buoyancy_body;
        wrench.torque = Vector3r::Zero();
    }

    // Implementation of RovDampingVertex methods
    inline void RovDampingVertex::initialize(const RovPhysicsBody* body)
    {
        body_ = body;
        // Damping forces act at Center of Mass (0, 0, 0)
        PhysicsBodyVertex::initialize(Vector3r::Zero(), Vector3r::Zero());
    }

    inline void RovDampingVertex::setWrench(Wrench& wrench)
    {
        wrench = Wrench::zero();
        if (!body_) return;

        const auto& p = body_->getParams().getParams();
        if (!p.enable_hydrodynamic_damping) return;

        const Quaternionr& orientation = body_->getKinematics().pose.orientation;

        // Linear velocity in world frame transformed into body frame: nu1 = [u, v, w]^T
        const Vector3r vel_world = body_->getKinematics().twist.linear;
        const Vector3r vel_body = VectorMath::transformToBodyFrame(vel_world, orientation);

        // Angular velocity in body frame: nu2 = [p, q, r]^T
        const Vector3r omega_body = body_->getKinematics().twist.angular;

        // Fossen hydrodynamic damping:
        //   D(nu)*nu = D_linear * nu + D_quadratic * |nu| * nu
        // Coefficients strictly oppose motion using -|coeff| * vel
        Vector3r force_damping(
            -std::abs(p.damping_linear.x()) * vel_body.x() - std::abs(p.damping_linear_q.x()) * std::abs(vel_body.x()) * vel_body.x(),
            -std::abs(p.damping_linear.y()) * vel_body.y() - std::abs(p.damping_linear_q.y()) * std::abs(vel_body.y()) * vel_body.y(),
            -std::abs(p.damping_linear.z()) * vel_body.z() - std::abs(p.damping_linear_q.z()) * std::abs(vel_body.z()) * vel_body.z()
        );

        Vector3r torque_damping(
            -std::abs(p.damping_angular.x()) * omega_body.x() - std::abs(p.damping_angular_q.x()) * std::abs(omega_body.x()) * omega_body.x(),
            -std::abs(p.damping_angular.y()) * omega_body.y() - std::abs(p.damping_angular_q.y()) * std::abs(omega_body.y()) * omega_body.y(),
            -std::abs(p.damping_angular.z()) * omega_body.z() - std::abs(p.damping_angular_q.z()) * std::abs(omega_body.z()) * omega_body.z()
        );

        wrench.force = force_damping;
        wrench.torque = torque_damping;
    }

    // Implementation of RovBedContactVertex methods
    inline void RovBedContactVertex::initialize(const RovPhysicsBody* body)
    {
        body_ = body;
        PhysicsBodyVertex::initialize(Vector3r::Zero(), Vector3r(0.0f, 0.0f, -1.0f));
    }

    inline void RovBedContactVertex::setWrench(Wrench& wrench)
    {
        wrench = Wrench::zero();
        if (!body_) return;

        const auto& p = body_->getParams().getParams();
        if (!p.enable_depth_clamp) return;

        const real_T bed_depth = std::abs(p.z_bed);
        const real_T current_z = body_->getKinematics().pose.position.z();
        const real_T vz_world = body_->getKinematics().twist.linear.z();

        const real_T penetration = current_z - bed_depth;
        const real_T margin = p.bed_contact_margin;

        real_T f_bed_z_world = 0.0f;

        if (penetration > 0.0f) {
            // Penetration below seabed: penalty spring-damper contact reaction
            real_T f_spring = p.bed_stiffness * penetration;
            real_T f_damper = (vz_world > 0.0f) ? (p.bed_damping * vz_world) : 0.0f;
            // Upward repulsive contact force (-Z in NED world frame)
            f_bed_z_world = -(f_spring + f_damper);
        }
        else if (penetration > -margin && vz_world > 0.0f && margin > 1e-4f) {
            // Smooth approaching cushion buffer: cubic Hermite smoothstep transition
            real_T s = (penetration + margin) / margin; // 0 to 1
            real_T smooth_factor = s * s * (3.0f - 2.0f * s);
            f_bed_z_world = -smooth_factor * (p.bed_damping * 0.5f * vz_world);
        }

        if (std::abs(f_bed_z_world) > 1e-4f) {
            Vector3r f_contact_world(0.0f, 0.0f, f_bed_z_world);
            const Quaternionr& orientation = body_->getKinematics().pose.orientation;
            wrench.force = VectorMath::transformToBodyFrame(f_contact_world, orientation);
            wrench.torque = Vector3r::Zero();
        }
    }

}
} //namespace

#endif
