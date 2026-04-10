#include <trajectory_generators/trajectory_generator_theory.h>
#include <trajectory_generators/dd_simple_trajectory_generator_limits.h>
#include <trajectory_generators/dd_simple_trajectory_generator_params.h>
#include <trajectory_generators/velocity_iterator.h>

#include <pcl/common/common.h>

namespace trajectory_generators
{

class AckermannSimpleTrajectoryGeneratorTheory: public TrajectoryGeneratorTheory{

  public:

    AckermannSimpleTrajectoryGeneratorTheory();

    virtual bool hasMoreTrajectories();
    virtual bool nextTrajectory(base_trajectory::Trajectory& _traj);

  private:
    void initialise();
    bool generateTrajectory(
        Eigen::Vector3f sample_target_vel,
        base_trajectory::Trajectory& traj);

    Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
        const Eigen::Vector3f& vel, double dt);

    double getMaxYawRateForSpeed(double linear_x) const;

  protected:

    virtual void onInitialize();

    std::shared_ptr<trajectory_generators::DDTrajectoryGeneratorLimits> limits_;
    std::shared_ptr<trajectory_generators::DDTrajectoryGeneratorParams> params_;

    unsigned int next_sample_index_;
    std::vector<Eigen::Vector3f> sample_params_;

    double wheelbase_;
    double max_steer_;
};

}//end of name space
