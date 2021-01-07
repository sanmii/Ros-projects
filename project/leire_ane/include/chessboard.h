#include <string>
#include <map>
#include <geometry_msgs/PoseStamped.h>

namespace chesslab_setup
{

class Chessboard
{
public:
	Chessboard()
	{
		// call private method in constructor to build the map
		createMap();
	}

	// create a public method to interface with the map
	// you can pass the argument by copy
	geometry_msgs::PoseStamped getPose(std::string __cell_name)
	{
		if( internal_storage__.find(__cell_name) == internal_storage__.end() )
		{
			std::cout << "This cell does not exist, return a default pose" << std::endl;
			geometry_msgs::PoseStamped p;
			p.pose.orientation.w = 1; // just to avoid the troubles for who uses it
			return p;
		}

		return internal_storage__[__cell_name];
	}

private:

	// class member that holds the map
	std::map<std::string, geometry_msgs::PoseStamped> internal_storage__;

	// private method.. can be called only by this class
	void createMap()
	{
		geometry_msgs::PoseStamped p;

		p.header.frame_id = "world";
		p.pose.position.x = -0.175;
		p.pose.position.y = 0.175;
		p.pose.position.z = 0;
		p.pose.orientation.x = 0;
		p.pose.orientation.y = 0;
		p.pose.orientation.z = 0;
		p.pose.orientation.w = 1;

		//values that are not settled are 0; ones settled the values it isn’t necessary to repeat if they don’t change.
		internal_storage__["A1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = 0.175;
		internal_storage__["A2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = 0.175;
		internal_storage__["A3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = 0.175;
		internal_storage__["A4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = 0.175;
		internal_storage__["A8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = 0.175;
		internal_storage__["A7"] =p;

		p.pose.position.x = 0.075;
		p.pose.position.y = 0.175;
		internal_storage__["A6"] =p;

		p.pose.position.x = 0.025;
		p.pose.position.y = 0.175;
		internal_storage__["A5"] = p;

		p.pose.position.x = -0.175;
		p.pose.position.y = 0.125;
		internal_storage__["B1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = 0.125;
		internal_storage__["B2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = 0.125;
		internal_storage__["B3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = 0.125;
		internal_storage__["B4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = 0.125;
		internal_storage__["B8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = 0.125;
		internal_storage__["B7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = 0.125;
		internal_storage__["B6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = 0.125;
		internal_storage__["B5"] = p;

		p.pose.position.x = -0.175;
		p.pose.position.y = 0.075;
		internal_storage__["C1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = 0.075;
		internal_storage__["C2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = 0.075;
		internal_storage__["C3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = 0.075;
		internal_storage__["C4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = 0.075;
		internal_storage__["C8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = 0.075;
		internal_storage__["C7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = 0.075;
		internal_storage__["C6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = 0.075;
		internal_storage__["C5"] = p;

		p.pose.position.x = -0.175;
		p.pose.position.y = 0.025;
		internal_storage__["D1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = 0.025;
		internal_storage__["D2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = 0.025;
		internal_storage__["D3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = 0.025;
		internal_storage__["D4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = 0.025;
		internal_storage__["D8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = 0.025;
		internal_storage__["D7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = 0.025;
		internal_storage__["D6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = 0.025;
		internal_storage__["D5"] = p;

		p.pose.position.x = -0.175;
		p.pose.position.y = -0.025;
		internal_storage__["E1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = -0.025;
		internal_storage__["E2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = -0.025;
		internal_storage__["E3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = -0.025;
		internal_storage__["E4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = -0.025;
		internal_storage__["E8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = -0.025;
		internal_storage__["E7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = -0.025;
		internal_storage__["E6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = -0.025;
		internal_storage__["E5"] = p;


		p.pose.position.x = -0.175;
		p.pose.position.y = -0.075;
		internal_storage__["F1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = -0.075;
		internal_storage__["F2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = -0.075;
		internal_storage__["F3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = -0.075;
		internal_storage__["F4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = -0.075;
		internal_storage__["F8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = -0.075;
		internal_storage__["F7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = -0.075;
		internal_storage__["F6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = -0.075;
		internal_storage__["F5"] = p;

		p.pose.position.x = -0.175;
		p.pose.position.y = -0.125;
		internal_storage__["G1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = -0.125;
		internal_storage__["G2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = -0.125;
		internal_storage__["G3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = -0.125;
		internal_storage__["G4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = -0.125;
		internal_storage__["G8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = -0.125;
		internal_storage__["G7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = -0.125;
		internal_storage__["G6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = -0.125;
		internal_storage__["G5"] = p;

		p.pose.position.x = -0.175;
		p.pose.position.y = -0.175;
		internal_storage__["H1"] = p;

		p.pose.position.x = -0.125;
		p.pose.position.y = -0.175;
		internal_storage__["H2"] = p;

		p.pose.position.x = -0.075;
		p.pose.position.y = -0.175;
		internal_storage__["H3"] = p;

		p.pose.position.x = -0.025;
		p.pose.position.y = -0.175;
		internal_storage__["H4"] = p;

		p.pose.position.x = 0.175;
		p.pose.position.y = -0.175;
		internal_storage__["H8"] = p;

		p.pose.position.x = 0.125;
		p.pose.position.y = -0.175;
		internal_storage__["H7"] = p;

		p.pose.position.x = 0.075;
		p.pose.position.y = -0.175;
		internal_storage__["H6"] = p;

		p.pose.position.x = 0.025;
		p.pose.position.y = -0.175;
		internal_storage__["H5"] = p;
	}
};

}
