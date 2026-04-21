#pragma once
// mobile robot simulation class

const int NS_MAX = 20; // maximum number of state variables

class robot {
	
	public:
	
	int N; // number of state variables
	
	double t; // current time (seconds)
	double x[NS_MAX+1];  // state vector x
	double xd[NS_MAX+1]; // derivative vector at time t
	
	double vl, vr; // left and right wheel velocity magnitude / speed
	// (velocity direction is assumed perpedicular to shaft)
		
	// reference / desired value of alpha
	double alpha_ref;
		
	// maximum wheel speed (pixels/s)
	double v_max;
		
	double D; // distance between wheels (ie shaft length) (pixels)
	
	// position of laser in local robot coordinates
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	double Lx, Ly;
	
	// position of robot axis of rotation (half way between wheels)
	// and the robot image center (pixels) in local coord
	double Ax, Ay;
	
	// max range of laser / gripper (rad)
	double alpha_max;
	
	// position of gripper center (outputs) (pixels) in global coord
	double xg, yg; 
	
	// position of robot axis with respect in global coord (pixels)
	double xa, ya; 
	
	// robot laser state (0 - off, 1 - on)
	int laser;
	
	robot(double x0, double y0, double theta0, double vmax);

	void sim_step(double dt);

	void set_inputs(int pw_l, int pw_r, int pw_laser, int laser);

	void calculate_outputs();
	
	// ============ COLLISION DETECTION METHODS ============
	void setCollisionData(int N_obs, double* x_obs, double* y_obs, double* size_obs);
	void setCollisionEnabled(bool enabled);
	void setCollisionBox(double width, double length);
	bool checkCollision(double test_x, double test_y, double test_theta) const;
	
	// NEW: Collision recovery
	bool isStuck() const;                  // Check if robot is stuck
	void resetCollisionCounter();          // Reset stuck counter
	int getCollisionCount() const;         // Get collision count
	// =====================================================
	
	private:
	// ============ COLLISION DETECTION MEMBERS ============
	int collision_N_obs;
	double* collision_x_obs;
	double* collision_y_obs;
	double* collision_size_obs;
	bool collision_enabled;
	double collision_robot_width;
	double collision_robot_length;
	
	// NEW: Collision recovery members
	int collision_count;              // Count consecutive collisions
	double last_collision_time;       // Time of last collision
	static const int STUCK_THRESHOLD = 50;  // Collisions before "stuck"
	// =====================================================
};

