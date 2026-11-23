#include "acs/simulation/SensorFusion.h"
#include "acs/math/Quaternion.h"
#include "acs/physics/Gravity.h"
#include <algorithm>
#include <cmath>

namespace acs::simulation {
namespace { double sq(double v) { return v * v; } }

SensorFusion::SensorFusion(const FusionConfig& cfg, const SensorConfig& sensors)
    : cfg_(cfg), sensor_cfg_(sensors) {}

void SensorFusion::reset(const acs::math::Vector3& pos, const acs::math::Vector3& vel,
                         const acs::math::Vector3& euler) {
  x_.fill(0.0); x_[0]=pos.x; x_[1]=pos.y; x_[2]=pos.z; x_[3]=vel.x; x_[4]=vel.y; x_[5]=vel.z;
  for (auto& row : p_) row.fill(0.0);
  for (int i=0;i<3;++i) p_[i][i]=sq(cfg_.initial_position_std_m);
  for (int i=3;i<6;++i) p_[i][i]=sq(cfg_.initial_velocity_std_m_s);
  for (int i=6;i<9;++i) p_[i][i]=sq(cfg_.initial_accel_bias_std_m_s2);
  output_={}; output_.roll_rad=euler.x; output_.pitch_rad=euler.y; output_.yaw_rad=euler.z;
  output_.initialized=true; refresh();
}

void SensorFusion::propagate_attitude(const ImuSample& imu, double dt) {
  const double phi=output_.roll_rad, theta=output_.pitch_rad, sp=std::sin(phi), cp=std::cos(phi);
  double ct=std::cos(theta); if (std::abs(ct)<1e-6) ct=std::copysign(1e-6,ct);
  const double coupled=imu.gyro_body_rad_s.y*sp+imu.gyro_body_rad_s.z*cp;
  const double rg=phi+(imu.gyro_body_rad_s.x+std::tan(theta)*coupled)*dt;
  const double pg=theta+(imu.gyro_body_rad_s.y*cp-imu.gyro_body_rad_s.z*sp)*dt;
  output_.yaw_rad += coupled/ct*dt;
  const auto& a=imu.accel_body_m_s2; const double n=a.norm();
  if (n>4.9 && n<14.7) {
    const double k=cfg_.attitude_time_constant_s/(cfg_.attitude_time_constant_s+dt);
    output_.roll_rad=k*rg+(1-k)*std::atan2(-a.y,-a.z);
    output_.pitch_rad=k*pg+(1-k)*std::atan2(a.x,std::sqrt(a.y*a.y+a.z*a.z));
  } else { output_.roll_rad=rg; output_.pitch_rad=pg; }
}

void SensorFusion::predict(const ImuSample& imu, double dt) {
  const auto q=acs::math::Quaternion::from_euler321(output_.roll_rad,output_.pitch_rad,output_.yaw_rad);
  const acs::math::Vector3 bias(x_[6],x_[7],x_[8]);
  const auto an=q.rotate(imu.accel_body_m_s2-bias)+acs::physics::Gravity::ned(-x_[2]);
  const double h=.5*dt*dt;
  x_[0]+=x_[3]*dt+an.x*h; x_[1]+=x_[4]*dt+an.y*h; x_[2]+=x_[5]*dt+an.z*h;
  x_[3]+=an.x*dt; x_[4]+=an.y*dt; x_[5]+=an.z*dt;
  double f[N][N]{}; for(int i=0;i<N;++i) f[i][i]=1; const auto c=q.to_dcm();
  for(int i=0;i<3;++i){ f[i][i+3]=dt; for(int b=0;b<3;++b){f[i][b+6]=-.5*c.m[i][b]*dt*dt; f[i+3][b+6]=-c.m[i][b]*dt;}}
  Covariance fp{}, next{};
  for(int i=0;i<N;++i)for(int j=0;j<N;++j)for(int k=0;k<N;++k)fp[i][j]+=f[i][k]*p_[k][j];
  for(int i=0;i<N;++i)for(int j=0;j<N;++j)for(int k=0;k<N;++k)next[i][j]+=fp[i][k]*f[j][k];
  const double qa=sq(cfg_.accel_process_noise_m_s2), qb=sq(cfg_.accel_bias_process_noise_m_s2);
  for(int i=0;i<3;++i){next[i][i]+=.25*qa*dt*dt*dt*dt;next[i+3][i+3]+=qa*dt*dt;next[i+6][i+6]+=qb*dt;}
  p_=next;
}

double SensorFusion::correct(const std::array<double,N>& h,double residual,double variance){
  std::array<double,N> ph{}; for(int i=0;i<N;++i)for(int j=0;j<N;++j)ph[i]+=p_[i][j]*h[j];
  double s=variance; for(int i=0;i<N;++i)s+=h[i]*ph[i]; if(!(s>1e-12))return 0;
  std::array<double,N> k{}; for(int i=0;i<N;++i){k[i]=ph[i]/s;x_[i]+=k[i]*residual;}
  Covariance n=p_; for(int i=0;i<N;++i)for(int j=0;j<N;++j)n[i][j]-=k[i]*ph[j];
  for(int i=0;i<N;++i){for(int j=i+1;j<N;++j)n[i][j]=n[j][i]=.5*(n[i][j]+n[j][i]);n[i][i]=std::max(n[i][i],1e-12);} p_=n; return residual;
}

const FilteredState& SensorFusion::update(const ImuSample& imu,const GpsSample& gps,
 const AirDataSample& air,const acs::math::Vector3& wind,double dt){
  if(!output_.initialized||!(dt>0)) return output_;
  output_.gps_aiding=false; output_.air_data_aiding=false;
  propagate_attitude(imu,dt);predict(imu,dt);
  if(gps.valid){double sum=0;for(int i=0;i<3;++i){std::array<double,N> h{};h[i]=1;double r=gps.position_ned_m[i]-x_[i];sum+=r*r;correct(h,r,sq(sensor_cfg_.gps_pos_noise_std_m*(i==2?1.5:1)));}
    for(int i=0;i<3;++i){std::array<double,N> h{};h[i+3]=1;correct(h,gps.velocity_ned_m_s[i]-x_[i+3],sq(sensor_cfg_.gps_vel_noise_std_m_s));}
    output_.gps_innovation_m=std::sqrt(sum);output_.gps_aiding=true;}
  if(air.valid){std::array<double,N> ah{};ah[2]=-1;correct(ah,air.altitude_m+x_[2],sq(sensor_cfg_.baro_alt_noise_std_m));
    const acs::math::Vector3 rel(x_[3]-wind.x,x_[4]-wind.y,x_[5]-wind.z);const double pred=rel.norm();
    if(pred>.5){std::array<double,N> h{};h[3]=rel.x/pred;h[4]=rel.y/pred;h[5]=rel.z/pred;output_.air_data_innovation_m_s=correct(h,air.airspeed_m_s-pred,sq(sensor_cfg_.airspeed_noise_std_m_s));}
    output_.airspeed_m_s=air.airspeed_m_s;output_.air_data_aiding=true;
  } else output_.airspeed_m_s=acs::math::Vector3(x_[3]-wind.x,x_[4]-wind.y,x_[5]-wind.z).norm();
  refresh();return output_;
}

void SensorFusion::refresh(){output_.position_ned_m={x_[0],x_[1],x_[2]};output_.velocity_ned_m_s={x_[3],x_[4],x_[5]};output_.accel_bias_body_m_s2={x_[6],x_[7],x_[8]};output_.altitude_m=-x_[2];output_.position_std_m=std::sqrt((p_[0][0]+p_[1][1]+p_[2][2])/3);output_.velocity_std_m_s=std::sqrt((p_[3][3]+p_[4][4]+p_[5][5])/3);}
}  // namespace acs::simulation
