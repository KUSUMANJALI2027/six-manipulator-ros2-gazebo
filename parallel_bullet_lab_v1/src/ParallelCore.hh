#pragma once
#include <btBulletDynamicsCommon.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace parallel_lab {
constexpr double pi=3.14159265358979323846;
inline double blend(double u){u=std::clamp(u,0.,1.);return u*u*u*(10-15*u+6*u*u);}
struct Body {std::string name;btRigidBody* rigid;double radius,length;};
struct Closure {btRigidBody *a,*b;btVector3 pa,pb;};
class Core {
  std::unique_ptr<btDefaultCollisionConfiguration> config;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btDbvtBroadphase> broad;
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
  std::unique_ptr<btDiscreteDynamicsWorld> world;
  std::vector<std::unique_ptr<btCollisionShape>> shapes;
  std::vector<std::unique_ptr<btRigidBody>> ownedBodies;
  std::vector<std::unique_ptr<btTypedConstraint>> constraints;
  std::vector<Closure> closures;
  std::vector<btVector3> lowerAnchor,upperAnchor;
  std::vector<btSliderConstraint*> sliders;
  std::vector<btHingeConstraint*> hinges;
  btRigidBody* platform=nullptr;
  std::vector<btRigidBody*> upper,lower;
  std::vector<btVector3> base,rad,attach;
  std::vector<double> integral;
  btRigidBody* fixed=nullptr;
 public:
  std::string robot;std::vector<Body> bodies;std::vector<double> home,command;
  double time=0;bool drive=true;
  explicit Core(std::string name):robot(std::move(name)){
    config=std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher=std::make_unique<btCollisionDispatcher>(config.get());broad=std::make_unique<btDbvtBroadphase>();solver=std::make_unique<btSequentialImpulseConstraintSolver>();
    world=std::make_unique<btDiscreteDynamicsWorld>(dispatcher.get(),broad.get(),solver.get(),config.get());
    world->setGravity({0,0,-9.81});world->getSolverInfo().m_numIterations=120;world->getSolverInfo().m_splitImpulse=true;
    fixed=body("fixed_base",{0,0,0},btQuaternion::getIdentity(),0,.035,.04);
    if(robot=="Delta")delta();else if(robot=="Stewart")stewart();else throw std::runtime_error("Choose Delta or Stewart");
    command=home;integral.assign(home.size(),0);
  }
  ~Core(){for(auto &c:constraints)world->removeConstraint(c.get());for(auto &b:ownedBodies)world->removeRigidBody(b.get());}
  btRigidBody* body(std::string name,btVector3 pos,btQuaternion rot,double mass,double radius,double length){
    shapes.push_back(std::make_unique<btCylinderShapeZ>(btVector3(radius,radius,length/2)));
    btVector3 inertia(0,0,0);if(mass>0)shapes.back()->calculateLocalInertia(mass,inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass,nullptr,shapes.back().get(),inertia);
    auto p=std::make_unique<btRigidBody>(ci);p->setWorldTransform(btTransform(rot,pos));p->setDamping(.025,.025);p->setActivationState(DISABLE_DEACTIVATION);
    auto* result=p.get();world->addRigidBody(result,1,0); // Contacts excluded; constrained educational mechanism only.
    ownedBodies.push_back(std::move(p));bodies.push_back({name,result,radius,length});return result;
  }
  btRigidBody* rod(std::string name,btVector3 a,btVector3 b,double mass,double radius=.012){
    btVector3 d=b-a;return body(name,(a+b)/2,shortestArcQuat(btVector3(0,0,1),d.normalized()),mass,radius,d.length());
  }
  template<class T> T* add(std::unique_ptr<T> c){auto*p=c.get();world->addConstraint(p,true);constraints.push_back(std::move(c));return p;}
  void ball(btRigidBody*a,btRigidBody*b,btVector3 p){
    auto pa=a->getWorldTransform().inverse()*p,pb=b->getWorldTransform().inverse()*p;
    add(std::make_unique<btPoint2PointConstraint>(*a,*b,pa,pb));closures.push_back({a,b,pa,pb});
  }
  void delta(){
    home.assign(3,.3);const double rb=.28,rp=.075,L=.2,l=.48,top=.85;
    double rho=rb+L*cos(.3)-rp;double h=top-L*sin(.3)-sqrt(l*l-rho*rho);
    platform=body("platform",btVector3(0,0,btScalar(h)),btQuaternion::getIdentity(),.30,rp,.018);
    for(int i=0;i<3;i++){
      double a=i*2*pi/3;btVector3 r(cos(a),sin(a),0),axis(-sin(a),cos(a),0),b=rb*r+btVector3(0,0,top),e=b+L*(cos(.3)*r-btVector3(0,0,sin(.3)));
      rad.push_back(r);base.push_back(b);attach.push_back(rp*r);
      auto*u=rod("upper_"+std::to_string(i),b,e,.20,.02);upper.push_back(u);
      auto invA=fixed->getWorldTransform().inverse(),invB=u->getWorldTransform().inverse();
      btTransform hingeFrame(shortestArcQuat(btVector3(0,0,1),axis),b);
      auto*hinge=add(std::make_unique<btHingeConstraint>(*fixed,*u,invA*hingeFrame,invB*hingeFrame,true));
      // Determine Bullet angle sign with a reversible 1 mrad orientation probe, before stepping.
      auto original=u->getWorldTransform();double initial=hinge->getHingeAngle();
      btQuaternion perturb(axis,btScalar(.001));auto changed=original;changed.setRotation(perturb*original.getRotation());u->setWorldTransform(changed);
      double sign=hinge->getHingeAngle()>initial?1:-1;u->setWorldTransform(original);
      double lo=initial+sign*(-.35-.3),hi=initial+sign*(1.15-.3);hinge->setLimit(std::min(lo,hi),std::max(lo,hi));hinges.push_back(hinge);
      closures.push_back({fixed,u,invA*b,invB*b});
      for(int side:{-1,1}){
        auto off=axis*(.018*side);auto p=btVector3(0,0,h)+rp*r+off;
        auto*v=rod("forearm_"+std::to_string(i)+"_"+std::to_string(side+1),e+off,p,.06,.008);
        ball(u,v,e+off);ball(v,platform,p);
      }
    }
  }
  void stewart(){
    platform=body("platform",{0,0,.48},btQuaternion::getIdentity(),1.,.20,.025);
    double ba[]={10,110,130,230,250,350},pa[]={50,70,170,190,290,310};
    for(int i=0;i<6;i++){
      auto b=btVector3(.32*cos(ba[i]*pi/180),.32*sin(ba[i]*pi/180),0);
      auto p=btVector3(.20*cos(pa[i]*pi/180),.20*sin(pa[i]*pi/180),.48);auto d=p-b;double L=d.length();
      base.push_back(b);attach.push_back(p-btVector3(0,0,.48));home.push_back(L);
      auto*lo=rod("lower_"+std::to_string(i),b,b+d*.55,.15,.014);auto*up=rod("upper_"+std::to_string(i),b+d*.45,p,.15,.011);
      lower.push_back(lo);upper.push_back(up);lowerAnchor.push_back(lo->getWorldTransform().inverse()*b);upperAnchor.push_back(up->getWorldTransform().inverse()*p);ball(fixed,lo,b);ball(up,platform,p);
      btTransform frame(shortestArcQuat(btVector3(1,0,0),d.normalized()),(b+p)/2);
      auto fA=lo->getWorldTransform().inverse()*frame,fB=up->getWorldTransform().inverse()*frame;
      auto*s=add(std::make_unique<btSliderConstraint>(*lo,*up,fA,fB,true));
      sliders.push_back(s);s->setLowerLinLimit(.30-L);s->setUpperLinLimit(.75-L);s->setLowerAngLimit(0);s->setUpperAngLimit(0);
    }
  }
  std::vector<double> measured()const{
    std::vector<double> q;
    for(size_t i=0;i<home.size();i++){
      if(robot=="Delta"){
        auto e=upper[i]->getWorldTransform()*btVector3(0,0,.10);auto d=e-base[i];
        q.push_back(atan2(-d.z(),d.dot(rad[i])));
      } else q.push_back((upper[i]->getWorldTransform()*upperAnchor[i]-lower[i]->getWorldTransform()*lowerAnchor[i]).length());
    }
    return q;
  }
  void demo(){
    command=home;if(time<5)return;
    double phase=time-5;int joint=int(phase/12);if(joint>=int(home.size()))return;
    double u=fmod(phase,12),step=robot=="Delta"?.04:.004;
    double scale=u<3?blend(u/3):u<6?1:u<9?1-blend((u-6)/3):0;
    command[joint]+=step*scale;
  }
  double duration()const{return 5+12*home.size()+10;}
  void step(double dt){
    if(!(dt>0 && dt<=.002))throw std::runtime_error("Core dt must be >0 and <= 0.002 s");
    demo();auto q=measured();
    for(size_t i=0;i<q.size();i++){
      double err=command[i]-q[i];integral[i]=std::clamp(integral[i]+dt*err,-.2,.2);
      if(!drive)continue;
      if(robot=="Delta"){
        btVector3 axis(-rad[i].y(),rad[i].x(),0);double vel=upper[i]->getAngularVelocity().dot(axis);
        double torque=std::clamp(50*err+15*integral[i]-2.5*vel,-15.,15.);upper[i]->applyTorque(axis*torque);
      }else{
        auto dir=upper[i]->getWorldTransform().getBasis()*btVector3(0,0,1);
        auto rel=upper[i]->getLinearVelocity()-lower[i]->getLinearVelocity();double vel=rel.dot(dir);
        double force=std::clamp(2500*err+600*integral[i]-60*vel,-100.,100.);
        upper[i]->applyCentralForce(dir*force);lower[i]->applyCentralForce(-dir*force);
      }
    }
    world->stepSimulation(dt,0,dt);time+=dt;
    for(auto &b:bodies){auto p=b.rigid->getWorldTransform().getOrigin();if(!std::isfinite(p.length2()) || p.length()>10)throw std::runtime_error("Simulation diverged");}
  }
  double closure()const{double e=0;for(auto&c:closures)e=std::max(e,double((c.a->getWorldTransform()*c.pa-c.b->getWorldTransform()*c.pb).length()));for(auto*s:sliders){auto a=s->getRigidBodyA().getWorldTransform()*s->getFrameOffsetA();auto b=s->getRigidBodyB().getWorldTransform()*s->getFrameOffsetB();auto d=b.getOrigin()-a.getOrigin();auto axis=a.getBasis().getColumn(0);e=std::max(e,double((d-axis*d.dot(axis)).length()));}return e;}
  const btTransform& pose()const{return platform->getWorldTransform();}
  void header(std::ostream&o)const{
    o<<"simulation_time_s";for(size_t i=0;i<home.size();i++)o<<",cmd_"<<i+1<<(robot=="Delta"?"_rad":"_m");for(size_t i=0;i<home.size();i++)o<<",measured_"<<i+1<<(robot=="Delta"?"_rad":"_m");o<<",tool_x,tool_y,tool_z,qx,qy,qz,qw,max_anchor_gap_m\n";
  }
  void record(std::ostream&o)const{
    o<<std::setprecision(10)<<time;for(auto q:command)o<<','<<q;for(auto q:measured())o<<','<<q;auto p=pose().getOrigin();auto q=pose().getRotation();o<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<closure()<<'\n';
  }
};
}
