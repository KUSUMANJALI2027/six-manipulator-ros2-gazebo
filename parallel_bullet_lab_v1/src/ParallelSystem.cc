#include "ParallelCore.hh"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>
#include <chrono>
#include <iostream>
namespace parallel_lab {
class ParallelSystem:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemUpdate {
 std::unique_ptr<Core> core;std::vector<gz::sim::Entity> entities;gz::sim::Entity model=gz::sim::kNullEntity;
 std::ofstream csv;double nextRecord=0;bool failed=false;
 public:
 void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&sdf,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
  try{
   model=e;std::string robot=sdf->Get<std::string>("robot");core=std::make_unique<Core>(robot);
   csv.open(sdf->Get<std::string>("csv_path"));if(!csv)throw std::runtime_error("Cannot open output CSV");core->header(csv);core->record(csv);csv.flush();
   std::cout<<"ParallelBulletSystem initialized for "<<robot<<". Native Bullet rigid-body constraints; standard Gazebo Physics system is intentionally absent.\n";
  }catch(const std::exception&ex){failed=true;std::cerr<<"PARALLEL_ERROR: "<<ex.what()<<std::endl;}
 }
 void Update(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
  if(failed||!core||info.paused)return;
  try{
   if(entities.empty()){
    gz::sim::Model m(model);for(auto&b:core->bodies){auto e=m.LinkByName(ecm,b.name);if(e==gz::sim::kNullEntity)throw std::runtime_error("Missing body link: "+b.name);entities.push_back(e);}
   }
   double dt=std::chrono::duration<double>(info.dt).count();if(dt==0)return;
   if(dt<0 || dt>.05)throw std::runtime_error("Time reset/large timestep: restart this run");
   int n=int(std::ceil(dt/.001));for(int i=0;i<n;i++)core->step(dt/n);
   for(size_t i=0;i<core->bodies.size();i++){
    auto t=core->bodies[i].rigid->getWorldTransform();auto p=t.getOrigin();auto q=t.getRotation();
    gz::math::Pose3d pose(gz::math::Vector3d(p.x(),p.y(),p.z()),gz::math::Quaterniond(q.w(),q.x(),q.y(),q.z()));
    ecm.SetComponentData<gz::sim::components::Pose>(entities[i],pose);
   }
   if(core->time>=nextRecord){core->record(csv);csv.flush();nextRecord=core->time+.02;}
   if(core->closure()>.01)throw std::runtime_error("Anchor gap exceeds 10 mm; run stopped for investigation");
  }catch(const std::exception&ex){failed=true;std::cerr<<"PARALLEL_ERROR: "<<ex.what()<<std::endl;}
 }
};
}
GZ_ADD_PLUGIN(parallel_lab::ParallelSystem,gz::sim::System,parallel_lab::ParallelSystem::ISystemConfigure,parallel_lab::ParallelSystem::ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(parallel_lab::ParallelSystem,"parallel_lab::ParallelSystem")
