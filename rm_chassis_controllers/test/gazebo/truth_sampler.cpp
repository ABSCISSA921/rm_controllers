// Test-only Gazebo system plugin. Reads physics state at WorldUpdateEnd; it never
// writes poses, forces, controller state, contacts, or physics parameters.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>

namespace gazebo
{
class BaseLinkObserver : public SystemPlugin
{
public:
  void Load(int, char**) override
  {
    const char* directory = std::getenv("BASE_LINK_GAZEBO_TRUTH_DIR");
    if (!directory)
      return;
    directory_ = directory;
    const char* model = std::getenv("BASE_LINK_GAZEBO_MODEL");
    model_name_ = model ? model : "series_legged2";
    state_.open(directory_ + "/physics.csv");
    contacts_.open(directory_ + "/contacts.csv");
    state_ << std::setprecision(17);
    contacts_ << std::setprecision(17);
    state_ << "iteration,sim_sec,sim_nsec";
    for (const auto& name : { "base", "left_wheel", "right_wheel" })
      for (const auto& field : { "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "wx", "wy", "wz" })
        state_ << ',' << name << '_' << field;
    for (const auto& name : joint_names_)
      state_ << ',' << name << "_q," << name << "_dq," << name << "_gazebo_GetForce";
    state_ << '\n';
    contacts_ << "iteration,sim_sec,sim_nsec,contact_sec,contact_nsec,collision1,collision2,point,"
                 "x,y,z,nx,ny,nz,depth,f1x,f1y,f1z,f2x,f2y,f2z\n";
    created_ = event::Events::ConnectWorldCreated([this](const std::string& name) { attach(name); });
  }

private:
  void attach(const std::string& name)
  {
    world_ = physics::get_world(name);
    // A real transport subscriber requests contact reporting. It does not change
    // the physics constraints; samples themselves come directly from the manager.
    node_.reset(new transport::Node());
    node_->Init(name);
    contact_subscription_ = node_->Subscribe("~/physics/contacts", &BaseLinkObserver::ignoreContacts, this);
    updated_ = event::Events::ConnectWorldUpdateEnd([this]() { sample(); });
  }

  void ignoreContacts(ConstContactsPtr&) {}

  void linkState(const physics::LinkPtr& link)
  {
    const auto pose = link->WorldPose();
    // WorldLinearVel() without an offset is the link-origin velocity.
    const auto velocity = link->WorldLinearVel();
    const auto omega = link->WorldAngularVel();
    state_ << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z()
           << ',' << pose.Rot().W() << ',' << pose.Rot().X() << ',' << pose.Rot().Y() << ',' << pose.Rot().Z()
           << ',' << velocity.X() << ',' << velocity.Y() << ',' << velocity.Z()
           << ',' << omega.X() << ',' << omega.Y() << ',' << omega.Z();
  }

  void sample()
  {
    if (!model_)
    {
      model_ = world_->ModelByName(model_name_);
      if (!model_)
        return;
      links_ = { model_->GetLink("base_link"), model_->GetLink("left_wheel"), model_->GetLink("right_wheel") };
      for (std::size_t i = 0; i < joints_.size(); ++i)
        joints_[i] = model_->GetJoint(joint_names_[i]);
      for (const auto& link : links_)
        if (!link)
          throw std::runtime_error("Observer cannot find required model link");
      for (const auto& joint : joints_)
        if (!joint)
          throw std::runtime_error("Observer cannot find required drive joint");
    }
    const auto time = world_->SimTime();
    const auto iteration = world_->Iterations();
    state_ << iteration << ',' << time.sec << ',' << time.nsec;
    for (const auto& link : links_)
      linkState(link);
    for (const auto& joint : joints_)
      state_ << ',' << joint->Position(0) << ',' << joint->GetVelocity(0) << ',' << joint->GetForce(0);
    state_ << '\n';

    auto* manager = world_->Physics()->GetContactManager();
    for (unsigned int index = 0; index < manager->GetContactCount(); ++index)
    {
      const auto* contact = manager->GetContact(index);
      if (!contact || !contact->collision1 || !contact->collision2)
        continue;
      const std::string first = contact->collision1->GetScopedName();
      const std::string second = contact->collision2->GetScopedName();
      const std::string prefix = model_name_ + "::";
      if (first.compare(0, prefix.size(), prefix) != 0 && second.compare(0, prefix.size(), prefix) != 0)
        continue;
      for (int point = 0; point < contact->count; ++point)
      {
        const auto& p = contact->positions[point];
        const auto& n = contact->normals[point];
        const auto& f1 = contact->wrench[point].body1Force;
        const auto& f2 = contact->wrench[point].body2Force;
        contacts_ << iteration << ',' << time.sec << ',' << time.nsec << ',' << contact->time.sec << ','
                  << contact->time.nsec << ',' << std::quoted(first) << ',' << std::quoted(second) << ',' << point
                  << ',' << p.X() << ',' << p.Y() << ',' << p.Z() << ',' << n.X() << ',' << n.Y() << ',' << n.Z()
                  << ',' << contact->depths[point] << ',' << f1.X() << ',' << f1.Y() << ',' << f1.Z()
                  << ',' << f2.X() << ',' << f2.Y() << ',' << f2.Z() << '\n';
      }
    }
    if (iteration % 250 == 0)
    {
      state_.flush();
      contacts_.flush();
    }
  }

  std::string directory_, model_name_;
  std::ofstream state_, contacts_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  std::array<physics::LinkPtr, 3> links_;
  std::array<physics::JointPtr, 6> joints_;
  const std::array<std::string, 6> joint_names_{ "left_hip_joint", "left_knee_joint", "right_hip_joint",
                                               "right_knee_joint", "left_wheel_joint", "right_wheel_joint" };
  transport::NodePtr node_;
  transport::SubscriberPtr contact_subscription_;
  event::ConnectionPtr created_, updated_;
};
GZ_REGISTER_SYSTEM_PLUGIN(BaseLinkObserver)
}  // namespace gazebo
