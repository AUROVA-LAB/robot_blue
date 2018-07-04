#ifndef _reactive_hokuyo_node_h_
#define _reactive_hokuyo_node_h_

class ReactiveHokuyoNode
{
private:

	// Constant robot hardware constraints
	const float DISTANCE_FROM_SENSOR_TO_FRONT_ =  0.200;
	const float SENSOR_PITCH_DEG_ANGLE_        = 15.000;
	const float SENSOR_HEIGHT_                 =  0.650;

	const float VEHICLE_WIDTH_                 =  0.800;
	const float ABS_MAX_STEERING_DEG_ANGLE     = 30.000;
	const float MIN_OBSTACLE_HEIGHT_           =  0.075;

	// Configurable safety parameters
	float time_to_reach_min_allowed_distance_;
	float safety_distance_to_stop_vehicle_;

	float abs_lateral_safety_margin_;
	float abs_front_safety_margin_;

	float z_threshold_;
	float safety_width_;

	float euclidean_association_threshold_;
	float min_obstacle_radius_;

	// Values to compute output
	float closer_obstacle_point_;

	// Node output
	float max_velocity_recommendation_;

public:

	ReactiveHokuyoNode(void);

	~ReactiveHokuyoNode(void);

};



#endif
